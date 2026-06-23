# Frontier list published by the frontier extractor (ROADMAP 2.3) and consumed by
# the ExploreFrontier skill server (2.4). header.frame_id = map.
std_msgs/Header header
Frontier[] frontiers           # sorted best-first (hysteresis-aware ranking)
int32 committed_id             # currently committed frontier id; -1 = none committed
bool source_has_unknown        # false => the map source carries NO unknown cells, so
                               # no frontiers can exist (misconfigured source / not a
                               # SLAM grid). The extractor logs loudly when this is false.
