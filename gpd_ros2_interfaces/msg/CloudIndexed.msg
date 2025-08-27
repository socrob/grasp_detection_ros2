# This message holds a point cloud and a list of indices into the point cloud 
# at which to sample grasp candidates.

# The point cloud.
gpd_ros2_interfaces/CloudSources cloud_sources

# The indices into the point cloud at which to sample grasp candidates.
int64[] indices
