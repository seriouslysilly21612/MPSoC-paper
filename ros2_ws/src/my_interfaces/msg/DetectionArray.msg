std_msgs/Header header
Detection[] detections

# the inference mode that produced these detections: 0 = input 416, 1 = input 320 (the mode numbers of the
# mailbox, obs_mailbox.h). The detector sets it; the nodes after it carry it to the bridge.
uint8 mode
