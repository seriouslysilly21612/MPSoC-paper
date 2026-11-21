# The tool tag (D-22) in the color image of one capture, as vitis_ai_detector_node found it in the image
# that it ran (I-122): one message for every image it runs, while tool_tag:=true. pick_target_3d_node pairs
# it with the object of the same capture (header.stamp).
std_msgs/Header header

bool found
# why not, when found is false: no_camera_info, not_found, not_searched (the whole image searched again
# only after a wait), other_id, several, pnp_failed, busy (the search was behind), search_error
string reason

# [m] the centre of the tag in the color optical frame, from its four corners (IPPE_SQUARE)
float64 x
float64 y
float64 z
# found in the window around its last place; the mean reprojection error of the corners [px]; the error
# of the other IPPE solution over that of this one (> 2: clear)
bool window
float32 reproj_px
float32 ambiguity
# [ms] the search itself, in the thread of the detector
float32 search_ms
