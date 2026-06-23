#!/usr/bin/env bash
# Phase 1.2 [FMEA] clock-offset proof. Run on the Pi (and edge) AFTER chrony has
# synced to the edge master. Asserts the measured offset is well under the
# tightest matching window.
#
#   TF transform_tolerance 0.2 s | depth-match 0.35 s | pixel-age 1.5 s
#
# We require |offset| and RMS <= 0.02 s (10% of the tightest 0.2 s window) as a
# healthy margin. Run:  bash deploy/time_sync/check_offset.sh
set -u
THRESH=0.02   # seconds; 10% of the 0.2 s TF window

echo "=== chronyc tracking ==="
chronyc tracking
echo "=== chronyc sources -v ==="
chronyc sources -v

LAST=$(chronyc tracking | awk -F: '/Last offset/{gsub(/[^0-9.eE+-]/,"",$2); print $2}')
RMS=$(chronyc tracking  | awk -F: '/RMS offset/ {gsub(/[^0-9.eE+-]/,"",$2); print $2}')
REF=$(chronyc tracking  | awk -F: '/Reference ID/{print $2}')

echo "=== verdict (threshold ${THRESH}s = 10% of the 0.2s TF window) ==="
echo "reference=${REF} last_offset=${LAST}s rms_offset=${RMS}s"
awk -v l="$LAST" -v r="$RMS" -v t="$THRESH" 'BEGIN{
  al=(l<0)?-l:l; ar=(r<0)?-r:r;
  if (l=="" || r=="") { print "FAIL: chrony not tracking yet (no offset)"; exit 1 }
  if (al<=t && ar<=t) { printf "PASS: offset %.6fs, RMS %.6fs within %.3fs (<< 0.2/0.35/1.5s windows)\n", al, ar, t; exit 0 }
  printf "FAIL: offset %.6fs or RMS %.6fs exceeds %.3fs margin\n", al, ar, t; exit 1
}'
