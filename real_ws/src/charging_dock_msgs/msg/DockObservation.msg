std_msgs/Header header
# dock_reference is the physical charging face, projected from the observed
# lidar face using face_to_dock; x points out towards the vehicle.
# pose is only meaningful when valid AND center_available.
bool valid
bool center_available
geometry_msgs/Pose pose
float32 quality
# Width/residual describe the observed lidar face (currently the back plate).
float32 visible_width
float32 fit_residual
string reason
