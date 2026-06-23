#!/usr/bin/env bash
# Single-host smoke test for the Phase 1.1 zenoh transport.
# Confirms that with multicast OFF, pub->sub delivery works THROUGH the local
# router (i.e. discovery does not depend on multicast). Run in WSL:
#   bash deploy/transport/smoke_test_zenoh.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
RUN="$(mktemp -d)"
sed 's/EDGE_HOST/localhost/' "$HERE/zenoh_session_config.json5" > "$RUN/session.json5"

echo "=== start router (multicast off) ==="
RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CONFIG_URI="$HERE/zenoh_router_config.json5" \
  nohup ros2 run rmw_zenoh_cpp rmw_zenohd > "$RUN/router.log" 2>&1 &
RPID=$!
sleep 6
if ss -ltn 2>/dev/null | grep -q ':7447'; then echo "router listening on 7447"; else echo "WARN: no 7447 listener"; fi

echo "=== subscriber (peer, multicast off, via router) ==="
RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_SESSION_CONFIG_URI="$RUN/session.json5" \
  nohup ros2 topic echo /zenoh_smoke std_msgs/msg/String --once > "$RUN/sub.log" 2>&1 &
SPID=$!
sleep 3

echo "=== publisher ==="
RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_SESSION_CONFIG_URI="$RUN/session.json5" \
  timeout 8 ros2 topic pub -r 5 /zenoh_smoke std_msgs/msg/String '{data: zenoh-router-ok}' > "$RUN/pub.log" 2>&1 &
sleep 6

echo "=== RESULT ==="
if grep -q 'zenoh-router-ok' "$RUN/sub.log"; then
  echo "PASS: message delivered through router with multicast off"
  cat "$RUN/sub.log"
else
  echo "FAIL: no delivery"
  echo "-- sub.log --"; cat "$RUN/sub.log"
  echo "-- router.log tail --"; tail -15 "$RUN/router.log"
fi

kill $SPID $RPID 2>/dev/null; pkill -f rmw_zenohd 2>/dev/null; sleep 1
echo "=== DONE ==="
