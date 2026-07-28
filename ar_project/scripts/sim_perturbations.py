#!/usr/bin/env python3
"""Sensor-level camera perturbation injector for the simulated benchmark.

SIM-ONLY NODE. It must never be started on the Pi: on the real robot the camera
already carries all the noise we could ever want, and deliberately degrading the
only exteroceptive sensor of a moving vehicle is a safety hazard. The launch
files that start this node live on the simulation side only.

The node is a pure ADD-ON: it subscribes to the clean image topic and publishes a
degraded copy on a NEW topic (default /camera/camera/color/image_perturbed).
Nothing in the existing stack changes unless a launch file points a consumer at
the new topic, so the same package can be run with or without perturbations.

WHAT EACH PERTURBATION MODELS PHYSICALLY
----------------------------------------
dropout_prob   - the frame never reaches the consumer: USB/CSI bandwidth
                 starvation, a dropped DDS sample over Wi-Fi, or a sensor node
                 that missed its exposure slot. The detector then has to cope
                 with an irregular, lower effective frame rate.
                 Overlay key: camera.dropout_prob.
motion_blur_px - finite exposure time while the base is yawing. A horizontal box
                 kernel is the correct first-order model for a constant angular
                 velocity during the shutter interval of a forward-looking camera
                 on a differential-drive base (the image shifts along +/-x).
                 Overlay key: camera.motion_blur_px.
blur_sigma     - defocus / a cheap plastic lens that never resolves the MTF the
                 detector was trained on. Isotropic Gaussian PSF.
                 Overlay key: camera.blur_sigma.
smudge         - a fingerprint or a grease film on the lens cover. THE realistic
                 one, and the one that actually breaks a detector: a greasy lens
                 does NOT darken the image uniformly, it LOCALLY destroys high
                 frequency detail inside a few soft blobs while leaving the rest
                 of the frame perfectly sharp. Text on a sign inside such a blob
                 becomes unreadable even though the global image statistics look
                 completely healthy, which is exactly the failure the benchmark
                 wants to provoke. Overlay key: camera.smudge.
darkness       - "lights off" as seen by the sensor: a multiplicative exposure
                 gain plus a small black-level lift, because a real sensor in the
                 dark returns a noisy dark-grey floor, not pure black.
                 Overlay key: camera.darkness (used together with the world-level
                 lights_off list, which kills the gz point lights).
noise_sigma    - photon shot noise + read noise, i.e. what the ISP produces once
                 it pushes the analogue gain up in a dark room. Always pair it
                 with darkness < 1 for a physically plausible low-light frame.
                 Overlay key: camera.noise_sigma.
jpeg_quality   - the lossy link between the camera and the detector (the tracker
                 bridge on the real robot ships JPEG over Wi-Fi). Blocking and
                 ringing artefacts around exactly the high-contrast edges the
                 detector keys on. Not part of the overlay `camera:` schema; it
                 is available as a ROS parameter / profile key for link-quality
                 experiments. 0 disables it.

The perturbation overlays live in config/scenarios/perturbations/<pid>.yaml and
their `camera:` block is forwarded verbatim as the JSON profile below, with the
episode `seed` injected by the runner (the overlay says WHAT kind of degradation,
the seed decides which particular smudge mask and dropped frames realise it).

This node runs for EVERY episode, including the neutral overlay p_none, so the
perturbed topic always exists and the detector/planner subscribe to the same
topic name whatever the overlay is. p_none needs no special handling: its profile
is all-neutral and every effect is skipped when its parameter is zero, so a frame
is republished untouched. The `passthrough` parameter is a manual debugging
override (bypass the pipeline entirely regardless of profile) -- house_sim never
sets it.

PROFILE JSON (std_msgs/String on profile_topic, latched TRANSIENT_LOCAL)
------------------------------------------------------------------------
    {"smudge": 0.0, "blur_sigma": 0.0, "darkness": 1.0, "noise_sigma": 0.0,
     "dropout_prob": 0.0, "motion_blur_px": 0, "jpeg_quality": 0, "seed": 0}
Every key is optional. Keys that are NOT present are reset to their default
(the values shown above), so one profile message fully describes the camera
state - an episode never inherits a stale effect from the previous one. The
optional "seed" key re-seeds the episode. Unknown keys are ignored with a
warning. The same seven keys are also declared as ROS parameters with the same
names and defaults, so the node is usable without a runner.

The active profile is echoed as JSON on state_topic (latched) whenever it
changes, so the episode trace records exactly what was applied.
"""

import json
import math
import random
import sys

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Imported defensively so the node can die with ONE readable message at startup
# instead of an import traceback buried in the launch log.
try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge, CvBridgeError
    _IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - depends on the target rootfs
    cv2 = None
    np = None
    CvBridge = None
    CvBridgeError = Exception
    _IMPORT_ERROR = exc

# Only these are safe to touch pixel-wise while preserving the declared encoding.
SUPPORTED_ENCODINGS = ('rgb8', 'bgr8', 'mono8')

# name -> (default, minimum, maximum, caster)
PROFILE_SPEC = {
    'smudge': (0.0, 0.0, 1.0, float),
    'blur_sigma': (0.0, 0.0, 32.0, float),
    'darkness': (1.0, 0.0, 1.0, float),
    'noise_sigma': (0.0, 0.0, 255.0, float),
    'dropout_prob': (0.0, 0.0, 1.0, float),
    'motion_blur_px': (0, 0, 64, int),
    'jpeg_quality': (0, 0, 100, int),
}
PROFILE_KEYS = ('smudge', 'blur_sigma', 'darkness', 'noise_sigma',
                'dropout_prob', 'motion_blur_px', 'jpeg_quality')
PROFILE_DEFAULTS = {k: PROFILE_SPEC[k][0] for k in PROFILE_KEYS}

# Smudge look. Tuned on the 320x240 sim camera: the smear must be strong enough
# to make 1024x576 sign text unreadable at 2 m, which is the point of the test.
SMUDGE_SMEAR_SIGMA_FRAC = 0.035   # blur sigma of the smeared copy, in image widths
SMUDGE_GLARE_GAIN = 1.05          # grease scatters light -> slight contrast loss
SMUDGE_GLARE_LIFT = 14.0          # ... and lifts the black level inside the blob
SMUDGE_MASK_BLUR_FRAC = 0.10      # blob edges are soft; a smear has no outline

# A darkened sensor floors at its black level, not at zero, and that residual
# offset is what makes low-light frames look washed out instead of crushed.
BLACK_LEVEL_LIFT = 6.0

WARN_PERIOD_S = 10.0


class SimPerturbations(Node):
    def __init__(self):
        super().__init__('sim_perturbations')

        self.declare_parameter('image_in', '/camera/camera/color/image_raw')
        self.declare_parameter('image_out', '/camera/camera/color/image_perturbed')
        self.declare_parameter('profile_topic', '/sim_perturbation/profile')
        self.declare_parameter('state_topic', '/sim_perturbation/state')
        self.declare_parameter('seed', 0)
        self.declare_parameter('passthrough', False)
        self.declare_parameter('input_reliability', 'best_effort')
        for key in PROFILE_KEYS:
            self.declare_parameter(key, PROFILE_SPEC[key][0])

        self.image_in = self.get_parameter('image_in').value
        self.image_out = self.get_parameter('image_out').value
        self.profile_topic = self.get_parameter('profile_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.passthrough = bool(self.get_parameter('passthrough').value)

        self._bridge = CvBridge()
        self._seed = int(self.get_parameter('seed').value)
        self._profile = dict(PROFILE_DEFAULTS)
        self._pixel_effect = False
        self._state_seq = 0
        self._frames_in = 0
        self._frames_out = 0
        self._frames_dropped = 0
        self._warn_stamps = {}
        self._motion_kernels = {}
        self._mask_key = None
        self._mask = None
        self._smear_sigma = 0.0
        self._drop_rng = random.Random(self._seed)
        self._noise_rng = np.random.default_rng(self._seed)

        # The output mirrors the input reliability so a consumer that was written
        # against the raw camera topic keeps working when it is re-pointed here.
        reliability = self._reliability_from_param(
            self.get_parameter('input_reliability').value)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Latched: the runner may publish the episode profile before this node is
        # up, and a late subscriber of the state must still see what was applied.
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(Image, self.image_out, image_qos)
        self._state_pub = self.create_publisher(String, self.state_topic, latched_qos)
        self._profile_sub = self.create_subscription(
            String, self.profile_topic, self._on_profile, latched_qos)
        self._sub = self.create_subscription(
            Image, self.image_in, self._on_image, image_qos)

        initial = {key: self.get_parameter(key).value for key in PROFILE_KEYS}
        self._activate(self._sanitise(initial), self._seed, 'initial parameters')
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            'sim_perturbations (SIM ONLY) %s -> %s | profile<-%s state->%s | '
            'seed=%d passthrough=%s reliability=%s'
            % (self.image_in, self.image_out, self.profile_topic, self.state_topic,
               self._seed, self.passthrough,
               'reliable' if reliability == ReliabilityPolicy.RELIABLE else 'best_effort'))

    # ---------------------------------------------------------------- config

    def _reliability_from_param(self, value):
        text = str(value).strip().lower()
        if text == 'reliable':
            return ReliabilityPolicy.RELIABLE
        if text != 'best_effort':
            self.get_logger().warn(
                'Unknown input_reliability "%s"; falling back to best_effort.' % (value,))
        return ReliabilityPolicy.BEST_EFFORT

    def _sanitise(self, raw, base=None):
        """Clamp a raw dict onto a full profile. Missing keys take `base`."""
        profile = dict(PROFILE_DEFAULTS if base is None else base)
        for key, value in raw.items():
            if key == 'seed':
                continue
            if key not in PROFILE_SPEC:
                self._warn_throttled('key:' + str(key),
                                     'Ignoring unknown profile key "%s".' % (key,))
                continue
            _default, low, high, cast = PROFILE_SPEC[key]
            try:
                casted = cast(value)
            except (TypeError, ValueError):
                self._warn_throttled(
                    'cast:' + key,
                    'Profile key "%s" has a non-numeric value %r; keeping %r.'
                    % (key, value, profile[key]))
                continue
            if casted < low or casted > high:
                self._warn_throttled(
                    'range:' + key,
                    'Profile key "%s"=%r is outside [%r, %r]; clamping.'
                    % (key, casted, low, high))
                casted = cast(min(max(casted, low), high))
            profile[key] = casted
        return profile

    def _activate(self, profile, seed, source):
        """Install a profile, restart the RNG streams and echo the state."""
        changed = (profile != self._profile) or (seed != self._seed)
        if seed != self._seed:
            self._seed = seed
            self._mask_key = None  # the smudge pattern is a function of the seed
        self._profile = profile
        # A new profile marks the start of an episode: restart both random
        # streams so replaying the same seed replays the same dropped frames and
        # the same noise field.
        self._drop_rng = random.Random(self._seed)
        self._noise_rng = np.random.default_rng(self._seed)
        self._pixel_effect = (
            profile['smudge'] > 0.0
            or profile['blur_sigma'] > 0.0
            or profile['motion_blur_px'] > 1
            or profile['darkness'] != 1.0
            or profile['noise_sigma'] > 0.0
            or profile['jpeg_quality'] > 0)
        self._publish_state()
        self.get_logger().info(
            'Profile from %s%s: %s (seed=%d, pixel_effects=%s, passthrough=%s)'
            % (source, '' if changed else ' (unchanged, RNG restarted)',
               json.dumps(profile, sort_keys=True), self._seed,
               self._pixel_effect, self.passthrough))

    def _publish_state(self):
        self._state_seq += 1
        payload = dict(self._profile)
        payload['seed'] = self._seed
        payload['passthrough'] = self.passthrough
        payload['active'] = bool(self._pixel_effect or self._profile['dropout_prob'] > 0.0)
        payload['seq'] = self._state_seq
        payload['stamp'] = self.get_clock().now().nanoseconds * 1e-9
        payload['image_in'] = self.image_in
        payload['image_out'] = self.image_out
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._state_pub.publish(msg)

    def _on_profile(self, msg):
        text = (msg.data or '').strip()
        if not text:
            return
        try:
            raw = json.loads(text)
        except ValueError as exc:
            self.get_logger().error('Ignoring malformed profile JSON: %s' % (exc,))
            return
        if not isinstance(raw, dict):
            self.get_logger().error('Ignoring profile JSON: expected an object.')
            return
        seed = self._seed
        if 'seed' in raw:
            try:
                seed = int(raw['seed'])
            except (TypeError, ValueError):
                self.get_logger().warn('Ignoring non-integer seed %r.' % (raw['seed'],))
        self._activate(self._sanitise(raw), seed, self.profile_topic)

    def _on_set_parameters(self, params):
        """`ros2 param set` support: a partial merge onto the active profile."""
        raw = {}
        seed = self._seed
        passthrough = self.passthrough
        for param in params:
            if param.name in PROFILE_SPEC:
                raw[param.name] = param.value
            elif param.name == 'seed':
                seed = int(param.value)
            elif param.name == 'passthrough':
                passthrough = bool(param.value)
        if raw or seed != self._seed or passthrough != self.passthrough:
            self.passthrough = passthrough
            self._activate(self._sanitise(raw, base=self._profile), seed, 'parameters')
        return SetParametersResult(successful=True)

    # ----------------------------------------------------------------- image

    def _on_image(self, msg):
        self._frames_in += 1
        if self.passthrough:
            # p_none: keep the topic alive with byte-identical frames.
            self._pub.publish(msg)
            self._frames_out += 1
            return

        profile = self._profile

        # 1. dropout - decided before anything else, and independently of the
        #    encoding, because losing a frame is a transport event, not a pixel
        #    operation.
        if profile['dropout_prob'] > 0.0 and self._drop_rng.random() < profile['dropout_prob']:
            self._frames_dropped += 1
            return

        if msg.encoding not in SUPPORTED_ENCODINGS:
            self._warn_throttled(
                'encoding',
                'Encoding "%s" is not one of %s; republishing untouched.'
                % (msg.encoding, ', '.join(SUPPORTED_ENCODINGS)))
            self._pub.publish(msg)
            self._frames_out += 1
            return

        if not self._pixel_effect:
            self._pub.publish(msg)
            self._frames_out += 1
            return

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self._warn_throttled('to_cv2', 'imgmsg_to_cv2 failed: %s' % (exc,))
            return

        degraded = self._apply(image, profile)

        try:
            out = self._bridge.cv2_to_imgmsg(degraded, encoding=msg.encoding)
        except CvBridgeError as exc:
            self._warn_throttled('to_msg', 'cv2_to_imgmsg failed: %s' % (exc,))
            return
        # The whole downstream stack (tf lookups, detector<->depth association)
        # keys on the stamp: copy the header verbatim, never restamp.
        out.header = msg.header
        self._pub.publish(out)
        self._frames_out += 1

        if self._frames_out == 1 or self._frames_out % 300 == 0:
            self.get_logger().info(
                'Perturbed %d frame(s), dropped %d of %d received.'
                % (self._frames_out, self._frames_dropped, self._frames_in))

    def _apply(self, image, profile):
        """Effects in the documented order. Every stage is skipped when off."""
        work = image

        # 2. motion blur: horizontal box kernel (1 px wide kernel is identity).
        motion_px = int(profile['motion_blur_px'])
        if motion_px > 1:
            work = cv2.filter2D(work, -1, self._motion_kernel(motion_px))

        # 3. defocus.
        blur_sigma = float(profile['blur_sigma'])
        if blur_sigma > 0.0:
            work = cv2.GaussianBlur(work, (0, 0), blur_sigma)

        smudge = float(profile['smudge'])
        gain = float(profile['darkness'])
        noise_sigma = float(profile['noise_sigma'])
        if smudge > 0.0 or gain != 1.0 or noise_sigma > 0.0:
            # One uint8->float32->uint8 round trip for all three stages.
            buf = work.astype(np.float32)

            # 4. smudge: alpha-blend the frame with a heavily blurred, slightly
            #    brightened copy of itself inside the cached blob mask.
            if smudge > 0.0:
                alpha = self._smudge_mask(work.shape)
                if buf.ndim == 3:
                    alpha = alpha[:, :, None]
                smear = cv2.GaussianBlur(buf, (0, 0), self._smear_sigma)
                smear = smear * SMUDGE_GLARE_GAIN + SMUDGE_GLARE_LIFT
                alpha = alpha * smudge
                buf = buf * (1.0 - alpha) + smear * alpha

            # 5. exposure loss + black-level lift.
            if gain != 1.0:
                buf = buf * gain + BLACK_LEVEL_LIFT * (1.0 - gain)

            # 6. sensor noise.
            if noise_sigma > 0.0:
                buf = buf + self._noise_rng.normal(
                    0.0, noise_sigma, buf.shape).astype(np.float32)

            work = np.clip(buf, 0.0, 255.0).astype(np.uint8)

        # 7. lossy link.
        quality = int(profile['jpeg_quality'])
        if quality > 0:
            work = self._jpeg_roundtrip(work, quality)

        return work

    def _motion_kernel(self, width):
        kernel = self._motion_kernels.get(width)
        if kernel is None:
            kernel = np.full((1, width), 1.0 / float(width), dtype=np.float32)
            self._motion_kernels[width] = kernel
        return kernel

    def _smudge_mask(self, shape):
        """Soft elliptical blob mask, built once per (size, seed) and cached.

        WHY a mask and not a global blur: a greasy lens does not darken the image
        uniformly, it locally destroys high-frequency detail. Sharp background +
        a few unreadable patches is what actually breaks the detector, and it is
        also what a global blur completely fails to reproduce.
        """
        height, width = int(shape[0]), int(shape[1])
        key = (height, width, self._seed)
        if self._mask_key == key and self._mask is not None:
            return self._mask

        rng = random.Random((self._seed * 2654435761) & 0xFFFFFFFF)
        mask = np.zeros((height, width), dtype=np.float32)
        blobs = rng.randint(3, 5)
        for index in range(blobs):
            if index == 0:
                # The finger-print one, roughly on the optical axis.
                cx = width * (0.5 + rng.uniform(-0.10, 0.10))
                cy = height * (0.5 + rng.uniform(-0.10, 0.10))
                rx = width * rng.uniform(0.18, 0.30)
                ry = height * rng.uniform(0.16, 0.28)
            else:
                # The rest sit out towards the rim, where dust and grease collect.
                angle = rng.uniform(0.0, 2.0 * math.pi)
                radius = rng.uniform(0.55, 0.85)
                cx = width * (0.5 + 0.5 * radius * math.cos(angle))
                cy = height * (0.5 + 0.5 * radius * math.sin(angle))
                rx = width * rng.uniform(0.10, 0.22)
                ry = height * rng.uniform(0.10, 0.22)
            cv2.ellipse(
                mask,
                (int(round(cx)), int(round(cy))),
                (max(2, int(rx)), max(2, int(ry))),
                rng.uniform(0.0, 180.0), 0.0, 360.0, 1.0, -1)

        # Feather the outlines: a smear fades out, it has no boundary.
        ksize = max(3, int(min(height, width) * SMUDGE_MASK_BLUR_FRAC) | 1)
        mask = cv2.GaussianBlur(mask, (ksize, ksize), 0)
        peak = float(mask.max())
        if peak > 1e-6:
            mask = mask * (1.0 / peak)

        self._mask = np.ascontiguousarray(mask, dtype=np.float32)
        self._mask_key = key
        self._smear_sigma = max(2.0, SMUDGE_SMEAR_SIGMA_FRAC * max(height, width))
        self.get_logger().info(
            'Built smudge mask %dx%d from seed %d: %d blob(s), smear sigma %.1f px.'
            % (width, height, self._seed, blobs, self._smear_sigma))
        return self._mask

    def _jpeg_roundtrip(self, image, quality):
        # cv2 assumes BGR here; for an rgb8 frame the luma/chroma split is swapped
        # but the round trip is still a valid JPEG degradation and the channel
        # order comes back unchanged, which is all we need.
        gray = (image.ndim == 2)
        ok, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            self._warn_throttled('jpeg_enc', 'JPEG encode failed; passing the frame on.')
            return image
        decoded = cv2.imdecode(
            buffer, cv2.IMREAD_GRAYSCALE if gray else cv2.IMREAD_COLOR)
        if decoded is None:
            self._warn_throttled('jpeg_dec', 'JPEG decode failed; passing the frame on.')
            return image
        return decoded

    # ------------------------------------------------------------------ misc

    def _warn_throttled(self, key, text, period_s=WARN_PERIOD_S):
        # Node clock, not wall clock: under use_sim_time the throttle has to
        # follow simulated time, and it must survive a /clock reset going back.
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._warn_stamps.get(key)
        if last is not None and last <= now < (last + period_s):
            return
        self._warn_stamps[key] = now
        self.get_logger().warn(text)


def main(args=None):
    if _IMPORT_ERROR is not None:
        message = ('sim_perturbations needs cv_bridge + opencv + numpy: %s. '
                   'Install ros-jazzy-cv-bridge and python3-opencv.' % (_IMPORT_ERROR,))
        sys.stderr.write(message + '\n')
        rclpy.logging.get_logger('sim_perturbations').fatal(message)
        raise SystemExit(1)

    rclpy.init(args=args)
    node = SimPerturbations()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
