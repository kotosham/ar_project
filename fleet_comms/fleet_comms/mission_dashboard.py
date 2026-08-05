"""Mission dashboard: human-readable web monitor for the VLM robot stack.

One small rclpy node + a stdlib HTTP server (no external assets, works offline)
that shows, in plain language:

  * per-component health (RealSense, EKF, EPOS4/CAN, /scan, Nav2, SLAM,
    detector, VLM orchestrator, executive FSM) from /robot_health +
    /heartbeat — what each element receives/executes and what failed;
  * the executive FSM mission state from /mission/status;
  * what the VLM sees / thinks / decided / what actually happened, from
    /vlm/activity (+ Set-of-Mark frame on /vlm/setofmark and the top-down
    map the VLM was shown on /vlm/map_view);

Run it on the EDGE machine (next to the orchestrator/detector) so the heavy
views stay link-free; only /robot_health (1 Hz, ~KB), /mission/status
(latched) and /heartbeat cross the Wi-Fi — per DATA_CONTRACTS this adds no
load the reactive loop could ever feel.

    ros2 run fleet_comms mission_dashboard
    # then open http://<edge-host>:8088
"""
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from ar_project_msgs.msg import Heartbeat

from fleet_comms.qos import liveliness_status


def _latched_sub_qos(depth=1):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


_HB_STATUS = {0: 'OK', 1: 'DEGRADED', 2: 'DOWN'}


class MissionDashboard(Node):
    HEARTBEAT_PERIOD_S = 0.5

    def __init__(self):
        super().__init__('mission_dashboard')
        self.declare_parameter('port', 8088)
        self.declare_parameter('bind', '0.0.0.0')
        self.port = int(self.get_parameter('port').value)
        self.bind = str(self.get_parameter('bind').value)

        self._lock = threading.Lock()
        self._components = {}            # name -> {level, message, values, rx}
        self._health_rx = 0.0            # monotonic of last /robot_health
        self._heartbeats = {}            # node_name -> {status, cpu, lat, rx}
        self._mission = None             # parsed /mission/status JSON
        self._activity = deque(maxlen=200)
        self._setofmark = None           # (jpeg bytes, monotonic)
        self._map_view = None

        self.create_subscription(DiagnosticArray, '/robot_health',
                                 self._on_health, 5)
        self.create_subscription(Heartbeat, '/heartbeat', self._on_heartbeat,
                                 liveliness_status(self.HEARTBEAT_PERIOD_S))
        self.create_subscription(String, '/mission/status', self._on_mission,
                                 _latched_sub_qos())
        self.create_subscription(String, '/vlm/activity', self._on_activity,
                                 _latched_sub_qos(depth=50))
        self.create_subscription(CompressedImage, '/vlm/setofmark',
                                 self._on_setofmark, 1)
        self.create_subscription(CompressedImage, '/vlm/map_view',
                                 self._on_map_view, 1)

        self._httpd = ThreadingHTTPServer((self.bind, self.port),
                                          self._make_handler())
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self.get_logger().info(
            'mission dashboard: open http://%s:%d  (components: /robot_health; '
            'VLM trace: /vlm/activity)' % (self.bind or 'localhost', self.port))

    # -- ROS callbacks --------------------------------------------------------

    def _on_health(self, msg):
        now = time.monotonic()
        with self._lock:
            self._health_rx = now
            for st in msg.status:
                level = st.level
                if isinstance(level, bytes):           # byte field in ROS2
                    level = level[0] if level else 0
                self._components[st.name] = {
                    'level': int(level),
                    'message': st.message,
                    'values': {kv.key: kv.value for kv in st.values},
                    'rx': now,
                }

    def _on_heartbeat(self, msg):
        with self._lock:
            self._heartbeats[msg.node_name] = {
                'status': _HB_STATUS.get(int(msg.status), '?'),
                'cpu_load': round(float(msg.cpu_load), 2),
                'latency_ms': round(float(msg.last_latency_ms), 1),
                'epoch': int(msg.mission_epoch),
                'rx': time.monotonic(),
            }

    def _on_mission(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            data = {'state': msg.data}
        with self._lock:
            self._mission = data

    def _on_activity(self, msg):
        try:
            event = json.loads(msg.data)
        except (ValueError, TypeError):
            event = {'event': 'raw', 'detail': msg.data}
        event['rx'] = time.time()
        with self._lock:
            self._activity.append(event)

    def _on_setofmark(self, msg):
        with self._lock:
            self._setofmark = (bytes(msg.data), time.monotonic())

    def _on_map_view(self, msg):
        with self._lock:
            self._map_view = (bytes(msg.data), time.monotonic())

    # -- state snapshot for the web page -------------------------------------

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            comps = [
                {'name': name, **{k: v for k, v in c.items() if k != 'rx'},
                 'age_s': round(now - c['rx'], 1)}
                for name, c in sorted(self._components.items())
            ]
            beats = {
                name: {**{k: v for k, v in b.items() if k != 'rx'},
                       'age_s': round(now - b['rx'], 1)}
                for name, b in self._heartbeats.items()
            }
            return {
                'mission': self._mission,
                'link_age_s': (round(now - self._health_rx, 1)
                               if self._health_rx else None),
                'components': comps,
                'heartbeats': beats,
                'activity': list(self._activity)[-80:],
                'setofmark_age_s': (round(now - self._setofmark[1], 1)
                                    if self._setofmark else None),
                'map_age_s': (round(now - self._map_view[1], 1)
                              if self._map_view else None),
                'server_time': time.time(),
            }

    def _image(self, which):
        with self._lock:
            item = self._setofmark if which == 'setofmark' else self._map_view
        return item[0] if item else None

    # -- HTTP -----------------------------------------------------------------

    def _make_handler(dashboard):
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_a):            # silence per-request logs
                pass

            def _send(self, code, ctype, body, extra=None):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Cache-Control', 'no-store')
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split('?', 1)[0]
                if path in ('/', '/index.html'):
                    self._send(200, 'text/html; charset=utf-8',
                               _PAGE.encode('utf-8'))
                elif path == '/state':
                    self._send(200, 'application/json; charset=utf-8',
                               json.dumps(dashboard.snapshot(),
                                          ensure_ascii=False).encode('utf-8'))
                elif path == '/events':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    try:
                        while True:
                            payload = json.dumps(dashboard.snapshot(),
                                                 ensure_ascii=False)
                            self.wfile.write(
                                ('data: %s\n\n' % payload).encode('utf-8'))
                            self.wfile.flush()
                            time.sleep(1.0)
                    except (BrokenPipeError, ConnectionError, OSError):
                        return
                elif path in ('/setofmark.jpg', '/map.jpg'):
                    img = dashboard._image(
                        'setofmark' if path == '/setofmark.jpg' else 'map')
                    if img is None:
                        self._send(404, 'text/plain', b'no image yet')
                    else:
                        self._send(200, 'image/jpeg', img)
                else:
                    self._send(404, 'text/plain', b'not found')
        return Handler

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


_PAGE = r"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>VLM Robot Monitor</title>
<style>
:root{--bg:#101418;--card:#1a2026;--line:#2a333c;--fg:#dbe4ec;--dim:#8b98a5;
 --ok:#33c06e;--warn:#e2b93b;--stale:#e28f3b;--err:#e25555;color-scheme:dark}
*{box-sizing:border-box;margin:0}
body{background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif;padding:14px}
h1{font-size:18px;margin-bottom:8px}
h2{font-size:14px;color:var(--dim);text-transform:uppercase;letter-spacing:.06em;margin:18px 0 8px}
.top{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.chip{padding:4px 10px;border-radius:20px;background:var(--card);border:1px solid var(--line);font-weight:600}
.chip small{color:var(--dim);font-weight:400}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));gap:8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:8px 10px}
.card .nm{font-weight:600;display:flex;align-items:center;gap:6px}
.dot{width:10px;height:10px;border-radius:50%;flex:0 0 auto}
.lv0{background:var(--ok)}.lv1{background:var(--warn)}.lv2{background:var(--err)}.lv3{background:var(--stale)}
.card .msg{color:var(--dim);font-size:12px;margin-top:2px;word-break:break-word}
.card .kv{color:var(--dim);font-size:11px;margin-top:2px}
.cols{display:grid;grid-template-columns:1.2fr .8fr;gap:14px}
@media(max-width:900px){.cols{grid-template-columns:1fr}}
#feed{background:var(--card);border:1px solid var(--line);border-radius:10px;
 padding:8px 10px;max-height:420px;overflow-y:auto;font-size:13px}
#feed .ev{padding:3px 0;border-bottom:1px dashed var(--line)}
#feed .t{color:var(--dim);font-size:11px;margin-right:6px}
#feed .tag{display:inline-block;min-width:74px;font-weight:600}
.tag.observe{color:#6ab7ff}.tag.plan{color:#b087f5}.tag.step{color:#5fd0a5}
.tag.fail{color:var(--err)}.tag.deg{color:var(--warn)}.tag.note{color:var(--dim)}
.imgs{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.imgs figure{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:6px}
.imgs img{width:100%;border-radius:6px;display:block;min-height:60px;background:#000}
.imgs figcaption{color:var(--dim);font-size:12px;padding:4px 2px 0}
#link.bad{color:var(--err);font-weight:700}
.mono{font-family:ui-monospace,Consolas,monospace}
</style></head><body>
<div class="top">
  <h1>VLM Robot Monitor</h1>
  <span class="chip" id="mstate">FSM: -</span>
  <span class="chip" id="mtask">mission: -</span>
  <span class="chip"><small>Pi link:</small> <span id="link">-</span></span>
</div>

<div class="cols">
<div>
  <h2>Robot Component Status</h2>
  <div class="grid" id="comps"></div>
  <h2>Process Heartbeats</h2>
  <div class="grid" id="beats"></div>
</div>
<div>
  <h2>VLM Thoughts and Actions</h2>
  <div id="feed"></div>
  <h2>Robot View</h2>
  <div class="imgs">
    <figure><img id="som" alt="no frame"><figcaption id="somc">Set-of-Mark: no frame</figcaption></figure>
    <figure><img id="map" alt="no map"><figcaption id="mapc">VLM map: no data</figcaption></figure>
  </div>
</div>
</div>

<script>
const LABELS={realsense:"RealSense Camera",ekf_odometry:"EKF Odometry",
 scan:"Laser Scan (/scan)",control_epos4:"EPOS4/CAN Drives",
 wheel_odometry:"Wheel Odometry",slam_correction:"SLAM map->odom Correction",
 slam_map:"SLAM Map (/map)",detection_stream:"Detections (/target_pixel)",
 cmd_vel:"Motion Commands",search_coordinator:"Executive FSM (Pi)",
 planner_orchestrator:"VLM Orchestrator",detector:"YOLOE Detector",
 nav2:"Nav2 Navigation",twist_mux:"Twist mux",collision_monitor:"Collision Monitor",
 cmd_vel_watchdog:"cmd_vel Watchdog",slam_rtabmap:"RTAB-Map SLAM Process"};
const LVL={0:"OK",1:"WARN",2:"FAIL",3:"NO DATA"};
function el(id){return document.getElementById(id)}
function esc(s){return String(s??"").replace(/[&<>]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;"}[c]))}
function fmtT(ts){const d=new Date(ts*1000);return d.toLocaleTimeString('en-US')}
function evLine(e){
 const t=`<span class="t">${e.stamp?fmtT(e.stamp):""}</span>`;
 switch(e.event){
  case"mission_start":return t+`<span class="tag step">START</span> target "${esc(e.target)}", client ${esc(e.client)} (creds: ${esc(e.creds)})`;
  case"mission_end":return t+`<span class="tag step">FINISH</span> ${e.steps} step(s)${e.degraded?' - <b class="tag deg">DEGRADED (FLAT fallback)</b>':""}`;
  case"observe":{const d=(e.detections||[]).map(x=>`${esc(x.label)} ${x.score}@${x.distance_m}m`).join(", ");
   return t+`<span class="tag observe">OBSERVE</span> step ${e.step}: ${e.n_detections} object(s)${d?" - "+d:""}; map: ${e.map==="yes"?"yes":"no"} -> asking ${esc(e.client)}`}
  case"plan":{const a=(e.actions||[]).map(x=>`<b>${esc(x.action)}</b>${x.rationale?" <i>("+esc(x.rationale)+")</i>":""}`).join("; ");
   return t+`<span class="tag plan">PLAN</span> step ${e.step} (${e.latency_ms} ms): ${a||"- empty"}`}
  case"plan_failed":return t+`<span class="tag fail">VLM ERROR</span> step ${e.step}: ${esc(e.error)} (breaker ${e.cb_open?"OPEN":"closed"})`;
  case"step_start":return t+`<span class="tag step">DOING</span> step ${e.step}: <b>${esc(e.action)}</b>${e.rationale?" - "+esc(e.rationale):""}`;
  case"step_result":return t+`<span class="tag ${e.result==="ok"?"step":"fail"}">${e.result==="ok"?"DONE":"FAILED"}</span> step ${e.step}: ${esc(e.action)} in ${e.duration_s}s`;
  case"detect_all":{const o=(e.objects||[]).map(x=>`${esc(x.label)}(${x.score})`).join(", ");
   return t+`<span class="tag observe">SCAN</span> frame objects: ${o||"none"}`}
  case"degraded":return t+`<span class="tag deg">DEGRADED</span> ${esc(e.detail)}`;
  case"notes":return t+`<span class="tag note">MEMORY</span> ${esc(e.summary||"")} <span class="mono">[${(e.facts||[]).length} facts, ~${e.token_estimate} tok.]</span>`;
  default:return t+`<span class="tag note">${esc(e.event)}</span> ${esc(JSON.stringify(e))}`}
}
let lastSeq=-1;
function render(s){
 const m=s.mission||{};
 el("mstate").innerHTML=`FSM: <b>${esc(m.state||"-")}</b>`+(m.active_subtask?` <small>(${esc(m.active_subtask)})</small>`:"");
 el("mtask").innerHTML=`mission: <b>${esc(m.instruction||"-")}</b>`+(m.outcome?` <small>-> ${esc(m.outcome)}</small>`:"")+(m.progress!=null?` <small>${Math.round(m.progress*100)}%</small>`:"");
 const link=el("link");
 if(s.link_age_s==null){link.textContent="no data";link.className="bad"}
 else{link.textContent=s.link_age_s+"s ago";link.className=s.link_age_s>5?"bad":""}
 el("comps").innerHTML=(s.components||[]).map(c=>{
  const kv=Object.entries(c.values||{}).filter(([k])=>k!=="detail")
   .map(([k,v])=>`${esc(k)}=${esc(v)}`).join(" · ");
  const det=(c.values||{}).detail||"";
  return `<div class="card"><div class="nm"><span class="dot lv${c.level}"></span>${esc(LABELS[c.name]||c.name)} <small style="color:var(--dim)">${LVL[c.level]||c.level}</small></div>
  <div class="msg">${esc(c.message)}</div>${det?`<div class="kv">${esc(det)}</div>`:""}${kv?`<div class="kv mono">${kv}</div>`:""}</div>`}).join("");
 el("beats").innerHTML=Object.entries(s.heartbeats||{}).map(([n,b])=>{
  const lv=b.status==="OK"?0:(b.status==="DEGRADED"?1:2);
  const stale=b.age_s>3;
  return `<div class="card"><div class="nm"><span class="dot lv${stale?3:lv}"></span>${esc(LABELS[n]||n)} <small style="color:var(--dim)">${stale?"STALE "+b.age_s+"s":esc(b.status)}</small></div>
  <div class="kv mono">cpu=${b.cpu_load} · latency=${b.latency_ms}ms · epoch=${b.epoch}</div></div>`}).join("");
 const feed=el("feed");
 const act=s.activity||[];
 const last=act.length?act[act.length-1].seq:-1;
 if(last!==lastSeq){
  lastSeq=last;
  feed.innerHTML=act.slice().reverse().map(e=>`<div class="ev">${evLine(e)}</div>`).join("");
 }
 if(s.setofmark_age_s!=null){el("som").src="/setofmark.jpg?t="+Date.now();
  el("somc").textContent=`Set-of-Mark frame for VLM, ${s.setofmark_age_s}s ago`}
 if(s.map_age_s!=null){el("map").src="/map.jpg?t="+Date.now();
  el("mapc").textContent=`Map sent to VLM, ${s.map_age_s}s ago`}
}
function connect(){
 const es=new EventSource("/events");
 es.onmessage=e=>{try{render(JSON.parse(e.data))}catch(_){}};
 es.onerror=()=>{es.close();setTimeout(connect,2000)};
}
connect();
</script></body></html>
"""


def main():
    rclpy.init()
    node = MissionDashboard()
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
