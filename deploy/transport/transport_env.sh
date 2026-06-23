# ROS 2 transport environment — ROADMAP Phase 1.1.
#
# Source on EVERY host, AFTER the ROS overlays:
#   source /opt/ros/jazzy/setup.bash
#   source ~/ros2_ws/install/setup.bash
#   source <repo>/deploy/transport/transport_env.sh

# --- PRIMARY: rmw_zenoh, single edge router, multicast OFF ---
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="${ZENOH_SESSION_CONFIG_URI:-/etc/zenoh/zenoh_session_config.json5}"
# Point at the edge router without editing the json5 (overrides connect/endpoints):
#   export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.1.10:7447"]'
# Block node startup until the router is reachable, instead of running blind:
export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-10}"

# --- FALLBACK: Fast DDS LARGE_DATA + Discovery Server ---
# Use ONLY if rmw_zenoh is unavailable. Comment out the zenoh block above and
# uncomment the following on every host (run the discovery server on edge — see
# fastdds-discovery-server.service):
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA        # large msgs over TCP, off multicast
#   export ROS_DISCOVERY_SERVER="192.168.1.10:11811"    # single edge discovery server
