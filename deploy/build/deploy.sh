#!/usr/bin/env bash
# One-command build/deploy for the Pi(robot)+edge(GPU) split. See README.md.
#
#   ./deploy.sh edge      build the edge package set locally (detector + orchestrator)
#   ./deploy.sh pi        rsync source -> Pi + remote colcon build (executive + HW)
#   ./deploy.sh all       edge, then pi
#   ./deploy.sh shell     ssh into the Pi with the workspace sourced
#   ./deploy.sh doctor    check ssh reachability + ROS on both ends
#
# Tiers (deps resolved by colcon --packages-up-to):
#   Pi   = search_coordinator + ar_project   (-> ar_project_msgs, object_tracking_msgs, fleet_comms)
#   edge = planner_orchestrator + object_tracking (-> both msgs + fleet_comms)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# repo root = parent of the ar_project repo (ar_project/deploy/build -> up 3)
DEFAULT_REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

[ -f "$SCRIPT_DIR/deploy.env" ] && . "$SCRIPT_DIR/deploy.env"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REPO_ROOT="${REPO_ROOT:-$DEFAULT_REPO_ROOT}"
EDGE_WS="${EDGE_WS:-$HOME/ros2_ws}"

PI_TARGETS="search_coordinator ar_project"
EDGE_TARGETS="planner_orchestrator object_tracking"
# array (not a string) so globs like *.pt are passed literally to rsync, not expanded
RSYNC_EXCLUDES=(--exclude=build/ --exclude=install/ --exclude=log/ --exclude=.git/
  --exclude=__pycache__/ --exclude='*.pyc' --exclude=model_weights/ --exclude='*.pt'
  --exclude='*.ts' --exclude='*.env' --exclude='*.db')

die() { echo "ERROR: $*" >&2; exit 1; }
need_pi() { [ -n "${PI_HOST:-}" ] && [ -n "${PI_USER:-}" ] || \
  die "PI_HOST/PI_USER unset — copy deploy.env.example to deploy.env and fill them in."; }

build_edge() {
  echo ">> edge build ($EDGE_TARGETS) in $EDGE_WS"
  [ -d "$REPO_ROOT/ar_project" ] || die "repos not found under $REPO_ROOT (set REPO_ROOT in deploy.env)"
  # link the two repos' packages into the edge workspace if not already there
  mkdir -p "$EDGE_WS/src"
  for r in ar_project object_tracking; do
    [ -e "$EDGE_WS/src/$r" ] || ln -s "$REPO_ROOT/$r" "$EDGE_WS/src/$r"
  done
  # shellcheck disable=SC1090
  . "/opt/ros/$ROS_DISTRO/setup.bash"
  ( cd "$EDGE_WS" && colcon build --symlink-install --packages-up-to $EDGE_TARGETS )
  echo ">> edge done. source $EDGE_WS/install/setup.bash"
}

sync_pi() {
  need_pi
  echo ">> rsync source -> $PI_USER@$PI_HOST:$PI_WS/src"
  ssh "$PI_USER@$PI_HOST" "mkdir -p '$PI_WS/src'"
  rsync -az --delete "${RSYNC_EXCLUDES[@]}" \
    "$REPO_ROOT/ar_project" "$REPO_ROOT/object_tracking" \
    "$PI_USER@$PI_HOST:$PI_WS/src/"
}

build_pi() {
  need_pi
  echo ">> remote colcon build on Pi ($PI_TARGETS)"
  ssh "$PI_USER@$PI_HOST" "PI_WS='$PI_WS' ROS_DISTRO='$ROS_DISTRO' TARGETS='$PI_TARGETS' bash -s" <<'REMOTE'
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${PI_WS}"
# resolve any missing system deps once (no-op if already satisfied)
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
colcon build --symlink-install --packages-up-to ${TARGETS} \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
echo ">> Pi build done. source ${PI_WS}/install/setup.bash"
REMOTE
}

doctor() {
  echo "REPO_ROOT = $REPO_ROOT"; echo "ROS_DISTRO = $ROS_DISTRO"; echo "EDGE_WS = $EDGE_WS"
  [ -d "$REPO_ROOT/ar_project" ] && echo "repos: OK" || echo "repos: MISSING under REPO_ROOT"
  if [ -n "${PI_HOST:-}" ]; then
    echo -n "Pi ssh ($PI_USER@$PI_HOST): "
    ssh -o ConnectTimeout=5 "$PI_USER@$PI_HOST" \
      "source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null && echo reachable, ROS \$ROS_DISTRO ok" \
      || echo "UNREACHABLE (check PI_HOST/keys)"
  else
    echo "Pi: PI_HOST unset (set deploy.env)"
  fi
}

case "${1:-}" in
  edge)  build_edge ;;
  pi)    sync_pi && build_pi ;;
  all)   build_edge && sync_pi && build_pi ;;
  shell) need_pi; ssh -t "$PI_USER@$PI_HOST" \
           "cd '$PI_WS' && exec bash --rcfile <(echo 'source /opt/ros/$ROS_DISTRO/setup.bash; source install/setup.bash 2>/dev/null')" ;;
  doctor) doctor ;;
  *) sed -n '2,16p' "$0"; exit 1 ;;
esac
