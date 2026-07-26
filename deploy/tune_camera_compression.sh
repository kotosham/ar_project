#!/usr/bin/env bash
# Tune the RealSense image_transport compression on the Pi (Level-0 perf fix).
#
# WHY: the ONE compressed camera stream that crosses Wi-Fi is encoded by the
# image_transport publisher plugins INSIDE the RealSense node on the Pi. Two
# publisher-side parameters dominate Pi CPU and link bytes:
#   * compressedDepth.format = rvl   (vs the default png). PNG-encoding a 16-bit
#     depth frame is the single most expensive CPU op in the whole camera path
#     on the Pi; RVL is purpose-built for depth and far cheaper. If the running
#     plugin build does not accept 'rvl', we fall back to png_level=1 (fast PNG).
#   * compressed.jpeg_quality = 75   (vs the default 95). ~half the color bytes
#     over Wi-Fi; invisible to YOLOE/DINO/VLM at 640x480.
#
# These are image_transport parameters, NOT RealSense parameters, so they are
# set at RUNTIME here rather than baked into rs_launch (rs_launch does not pass
# arbitrary parameter names through). Changing them takes effect on the next
# published frame and is instantly reversible -- see `revert` below.
#
# Usage (on the Pi, after the RealSense node is up and ROS is sourced):
#   bash tune_camera_compression.sh                 # apply (node /camera/camera)
#   bash tune_camera_compression.sh /camera/camera  # apply to an explicit node
#   bash tune_camera_compression.sh /camera/camera revert   # back to defaults
#
# The script DISCOVERS the exact parameter names from `ros2 param list` instead
# of hard-coding a prefix, so it works regardless of the camera namespace.

set -u

NODE="${1:-/camera/camera}"
ACTION="${2:-apply}"

if [ "$ACTION" = "apply" ]; then
  DEPTH_FMT="rvl"; JPEG_Q="75"; PNG_LVL="1"
elif [ "$ACTION" = "revert" ]; then
  DEPTH_FMT="png"; JPEG_Q="95"; PNG_LVL="9"
else
  echo "usage: $0 [NODE] [apply|revert]" >&2
  exit 2
fi

echo "camera compression $ACTION on node '$NODE'"

PARAMS="$(ros2 param list "$NODE" 2>/dev/null)"
if [ -z "$PARAMS" ]; then
  echo "  ERROR: no parameters from '$NODE' -- is the RealSense node up and ROS sourced?" >&2
  echo "  Discover the node name with: ros2 node list | grep -i camera" >&2
  exit 1
fi

set_param() {  # <param-name> <value>
  local name="$1" val="$2"
  if ros2 param set "$NODE" "$name" "$val" >/dev/null 2>&1; then
    echo "  set $name = $val"
  else
    echo "  skip $name (set failed or not settable at runtime)"
  fi
}

# 1) depth: prefer RVL; if this plugin build rejects it, a fast PNG level still
#    cuts CPU vs the default png_level 9.
depth_fmt_params="$(echo "$PARAMS" | grep -E 'compressedDepth\.format$' || true)"
depth_lvl_params="$(echo "$PARAMS" | grep -E 'compressedDepth\.png_level$' || true)"
if [ -z "$depth_fmt_params" ]; then
  echo "  note: no compressedDepth.format param found (no compressedDepth subscriber yet?)."
  echo "        The plugin declares these lazily once the edge relay subscribes."
fi
for p in $depth_fmt_params; do set_param "$p" "$DEPTH_FMT"; done
for p in $depth_lvl_params; do set_param "$p" "$PNG_LVL"; done

# 2) color: JPEG quality.
jpeg_params="$(echo "$PARAMS" | grep -E 'compressed\.jpeg_quality$' || true)"
for p in $jpeg_params; do set_param "$p" "$JPEG_Q"; done

echo "done. verify with: ros2 param get $NODE <one of the params above>"
