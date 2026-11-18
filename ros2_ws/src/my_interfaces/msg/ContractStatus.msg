# What the control task wrote into its status in one period (obs_status.h of
# the control application). The bridge of the control application publishes
# it for the admission rule of the detector (vitis_ai_detector_pkg/admission.py),
# with every time moved to the clock of the image stamps.

# release of the period that wrote the status
builtin_interfaces/Time stamp

# tolerance of that period: delta of the contract [m]
float32 tolerance
# period of the control task, which is the deadline of its response [s]
float32 control_period
# longest response of the control task, release to torque, of the last one to two seconds [s]
float32 response_max
# check value of the contract constants of the control application
uint32 contract_crc
# method of the control application: M1, M2, A2, A3 or FULL
string method

# that period had a camera result to use
bool result_in_use
# no camera result for too long
bool target_lost

# the camera result used last (result_n 0: none so far, and the fields below are 0)
uint32 result_n
builtin_interfaces/Time result_capture
builtin_interfaces/Time result_valid_until
# eq. (7) of that result in the last period that used it [m]
float32 result_error_bound
# 0 = 416, 1 = 320
uint8 result_mode
# the target the result belongs to
uint32 target_seq
