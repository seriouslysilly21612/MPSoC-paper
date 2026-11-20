std_msgs/Header header

# Original COLOR capture time, carried end-to-end for latency metrics. Kept
# SEPARATE from header.stamp (which stays the depth-frame stamp; header.frame_id
# = camera_depth_optical_frame is what the base node uses for the TF lookup).
# That collision is why PickTarget carries capture_stamp too — rationale there.
builtin_interfaces/Time capture_stamp

bool target_valid
bool depth_valid

int32 class_id
string class_name
float32 confidence

float32 center_x
float32 center_y
float32 width
float32 height

float32 x
float32 y
float32 z

# the inference mode of the detections this target came from (DetectionArray.mode)
uint8 mode

# the tool tag (AprilTag on the flange, D-22) in the same color image as the object, its centre in the frame
# of x, y, z: the camera frame of header.frame_id after pick_target_3d_node, base_link after
# pick_target_base_node. tag_valid false: the tag was not looked for (tool_tag:=false), the color image of
# the capture was not there, or the tag was not found in it.
bool tag_valid
float32 tag_x
float32 tag_y
float32 tag_z
