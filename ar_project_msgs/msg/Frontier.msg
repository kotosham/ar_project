# A single frontier cluster: free cells of the SLAM occupancy grid that border
# unknown space. Produced by the frontier extractor (ROADMAP 2.3), selected by
# the ExploreFrontier skill (2.4). All geometry is in the `map` frame.
int32 id                       # stable id (coarse-quantized centroid); >=0.
                               # matches ExploreFrontier goal.frontier_id (-1 = best)
geometry_msgs/Point centroid   # cluster centroid in the map frame (z = 0)
uint32 size                    # number of frontier cells in the cluster
float32 score                  # ranking score, higher = better (size vs distance)
float32 distance_m             # straight-line distance from the robot at publish time
