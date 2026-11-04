# Pick & Place System — Implementation Progress

Last updated: 2026-06-22

---

## System Summary

> **This Overview is a 2026-06-22 snapshot** (SSD stand-in detector, ~17 Hz, "robot control not implemented"). For the current facts see the **2026-09-04** section at the end of this file.

A ROS2-based pick-and-place perception pipeline running on Kria KV260 (APU, Ubuntu 22.04 + ROS2 Humble). **Current pipeline**: RealSense → Vitis-AI DPU detection (long-running worker process) → 2D pick-logic filtering → **single-point reverse-projection 3D** (raw depth, `align_depth` OFF) → `base_link` target, **~17 Hz, z verified**. Detector is an SSD ADAS stand-in (car/bicycle/person), not the final pick model. Robot control (RPU/Indy7) not yet implemented.

> This file is a **chronological history**. For the current architecture at a glance, see `workflow.md`; the newest dated section at the bottom is the current state. Earlier "Current Status Snapshot" sections are **superseded** snapshots (they still describe aligned-depth, which has since been replaced by reverse projection).

---

## Initial Implemented Workflow (Historical Baseline)

The following was the initial mock-based end-to-end pipeline. It is retained as historical context. This document is chronological — the **current** architecture is the last dated section (`Pipelining + CPU-contention investigation + single-point depth`) plus `workflow.md`; the intermediate "Current Status Snapshot — 2026-06-22" section is an older, now-superseded snapshot (aligned depth, before reverse projection).

```
[USB Camera (V4L2, 640x480 BGR8)]
         |
         | cv2.VideoCapture → CvBridge → sensor_msgs/Image
         ↓
[Node: camera_publisher]                         package: camera_source_pkg
  - Publishes: /camera/image_raw
  - Params: device_id, fps (10.0), width (640), height (480), frame_id
         |
         | sensor_msgs/Image
         ↓
[Node: mock_detector]                            package: mock_detection_pkg
  - Subscribes: /camera/image_raw
  - Publishes: /detections (my_interfaces/Detection)
  - Fixed dummy output: class='object', confidence=0.90
  - BBox center = image center; size = 1/4 of image
         |
         | my_interfaces/Detection
         ↓
[Node: pick_logic_node]                          package: pick_logic_pkg
  - Subscribes: /detections
  - Publishes: /pick_target (my_interfaces/PickTarget)
  - Current logic: pass-through (Detection → PickTarget, target_valid=True)
         |
         | my_interfaces/PickTarget
         |
         |  +── /camera/camera/aligned_depth_to_color/image_raw  (16UC1 or 32FC1)
         |  +── /camera/camera/aligned_depth_to_color/camera_info (fx, fy, cx, cy)
         ↓
[Node: pick_target_3d_node]                      package: target_3d_pkg
  - Subscribes: /pick_target, depth image, camera_info
  - Publishes: /pick_target_3d (my_interfaces/PickTarget3D)
  - Depth extraction: 5x5 median patch around bbox center
  - 3D projection: x=(u-cx)*z/fx, y=(v-cy)*z/fy
  - Valid depth range: 0.05m – 2.00m
  - Depth scale (16UC1): 0.001 m/unit
         |
         | my_interfaces/PickTarget3D  (x, y, z in camera frame)
         ↓
[Robot Control / Grasping]                       ← NOT IMPLEMENTED
```

---

## Custom Message Types (my_interfaces)

```
Detection.msg
  int32   class_id
  string  class_name
  float32 confidence
  float32 center_x, center_y
  float32 width, height

DetectionArray.msg
  std_msgs/Header header
  Detection[] detections

PickTarget.msg
  bool    target_valid
  + all Detection fields

PickTarget3D.msg
  std_msgs/Header header
  bool    target_valid
  bool    depth_valid
  + all Detection fields
  float32 x, y, z          # 3D position in camera frame (meters)
```

---

## Initial Package Status (Historical)

This table describes the project before the later RealSense, bringup, filtering, TF, and Vitis-AI work.

| Package | Node | Status | Description |
|---|---|---|---|
| `my_interfaces` | — | Done | Custom message definitions |
| `camera_source_pkg` | `camera_publisher` | Done | USB V4L2 camera → `/camera/image_raw` |
| `mock_detection_pkg` | `mock_detector` | Done (stub) | Fixed dummy bbox, no real ML model |
| `pick_logic_pkg` | `pick_logic_node` | Done (stub) | Pass-through, no selection logic |
| `target_3d_pkg` | `pick_target_3d_node` | Done | 2D+depth → 3D coordinate via pinhole model |
| `apu_rpu_bridge_pkg` | — | Empty placeholder | APU→RPU bridge (OpenAMP/AXI), not started |
| `system_bringup_pkg` | — | Empty placeholder | Launch files and YAML config, not started |

---

## Initial Gaps (Historical)

### High Priority
- **Real ML detection model**: Replace `mock_detector` with SSD-MobileNet v2 or tiny YOLOv3 via Vitis AI or CPU inference.
- **Unified launch file + YAML parameter config**: `system_bringup_pkg` is empty. All nodes must be started manually in separate terminals.
- **pick_logic filtering**: Current implementation accepts every detection unconditionally. No confidence threshold, no multi-detection disambiguation, no class filtering.

### Medium Priority
- **RealSense D435i real integration test**: `target_3d_pkg` is designed for D435i topic layout but has not been tested with the physical camera.
- **Calibration file persistence**: Camera intrinsics are received live from `camera_info` topic but not saved to disk.

### Low Priority (Future Phases)
- **APU → RPU bridge** (`apu_rpu_bridge_pkg`): Transfer pick target from ROS2 to FreeRTOS via OpenAMP or AXI.
- **RPU firmware + Indy7 Ethernet control**: FreeRTOS firmware and robot arm trajectory protocol not started.
- **FPGA PL hardware acceleration**: Vision pipeline acceleration on PL not designed or implemented.

---

## Initial Mock Run Commands (Historical)

```bash
# Terminal 1 — USB camera
ros2 run camera_source_pkg camera_publisher

# Terminal 2 — Mock detector
ros2 run mock_detection_pkg mock_detector

# Terminal 3 — Pick logic
ros2 run pick_logic_pkg pick_logic_node

# Terminal 4 — 3D localization (requires RealSense D435i running)
ros2 run target_3d_pkg pick_target_3d_node

# Verify output
ros2 topic echo /pick_target_3d
```

---

## Initial Known Limitations (Historical)

- `mock_detector` always outputs the same class and position regardless of actual scene content.
- `pick_logic_node` blindly forwards the first detection with `target_valid=True`; no filtering.
- `camera_publisher` FPS is controlled by ROS timer only (`CAP_PROP_FPS` is commented out).
- The 3D output coordinate frame is the depth camera frame; no transform to robot base frame exists yet.

---

## Progress Update — 2026-05-07

### Step 1 — RealSense End-to-End Pipeline Validation

Status: **Completed**

The manually launched RealSense-based pipeline was validated successfully.

Validated flow:

```
[RealSense D435i via realsense2_camera]
         |
         | /camera/camera/color/image_raw
         | /camera/camera/aligned_depth_to_color/image_raw
         | /camera/camera/aligned_depth_to_color/camera_info
         ↓
[Node: mock_detector]
  - Subscribes to RealSense color image
  - Publishes /detections
         ↓
[Node: pick_logic_node]
  - Subscribes to /detections
  - Publishes /pick_target
         ↓
[Node: pick_target_3d_node]
  - Subscribes to /pick_target
  - Subscribes to aligned depth image and camera_info
  - Publishes /pick_target_3d
```

The following topics were confirmed as part of the active pipeline:

```
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/detections
/pick_target
/pick_target_3d
```

Manual Step 1 command set:

```bash
# Terminal 1 — RealSense
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=false \
  align_depth.enable:=true \
  enable_rgbd:=false \
  pointcloud.enable:=false \
  colorizer.enable:=false \
  enable_infra:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  depth_module.depth_profile:=848x480x30 \
  rgb_camera.color_profile:=848x480x30

# Terminal 2 — Mock detector
ros2 run mock_detection_pkg mock_detector \
  --ros-args --remap /camera/image_raw:=/camera/camera/color/image_raw

# Terminal 3 — Pick logic
ros2 run pick_logic_pkg pick_logic

# Terminal 4 — 3D target localization
ros2 run target_3d_pkg pick_target_3d_node --ros-args \
  -p patch_radius:=4 \
  -p max_depth_m:=3.5

# Terminal 5 — Verify output
ros2 topic echo /pick_target_3d --once
```

Important observation:

- `pick_target_3d_node` only publishes when `/pick_target` messages arrive.
- If `/detections` is not produced, `/pick_target` and `/pick_target_3d` will also remain silent.

---

### Step 2 — Unified Bringup Launch and YAML Config

Status: **Completed**

The previously manual multi-terminal pipeline was converted into a single bringup launch flow.

Added files:

```
src/system_bringup_pkg/launch/pick_place_apu.launch.py
src/system_bringup_pkg/config/realsense_pick_place.yaml
src/system_bringup_pkg/config/target_3d.yaml
```

Updated files:

```
src/system_bringup_pkg/setup.py
src/system_bringup_pkg/package.xml
```

Current launch command:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch system_bringup_pkg pick_place_apu.launch.py
```

The launch file currently starts:

```
realsense2_camera rs_launch.py
mock_detection_pkg/mock_detector
pick_logic_pkg/pick_logic
target_3d_pkg/pick_target_3d_node
```

The launch file includes the required remapping for the mock detector:

```python
remappings=[
    ('/camera/image_raw', '/camera/camera/color/image_raw'),
]
```

This remapping is required because:

```
mock_detector default subscription: /camera/image_raw
RealSense color topic:             /camera/camera/color/image_raw
```

Without this remap, `/detections` is not published and the downstream `/pick_target_3d` topic remains silent.

Step 2 validation commands:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /detections --once
ros2 topic echo /pick_target --once
ros2 topic echo /pick_target_3d --once
```

Expected active nodes:

```
/camera/camera
/mock_detector
/pick_logic_node
/pick_target_3d_node
```

Expected active topics:

```
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/detections
/pick_target
/pick_target_3d
/tf_static
```

---

### Current System Status After Step 2

The APU-side ROS2 perception baseline is now repeatable through a single launch command.

Current working pipeline:

```
[RealSense D435i]
  color image + aligned depth + camera_info
         ↓
[mock_detector]
  fixed dummy bbox at image center
         ↓
[pick_logic_node]
  pass-through target selection
         ↓
[pick_target_3d_node]
  median depth patch + pinhole projection
         ↓
[/pick_target_3d]
  target x/y/z in camera frame
```

The system still uses a mock detector and pass-through pick logic. It is not yet a real object-aware pick system.

---

## Step 3 Plan — TF and Calibration Baseline

Status: **Not started**

Step 3 should establish the minimum TF/calibration structure needed to connect camera-frame target coordinates to the robot coordinate system.

### Step 3 Goal

Create a TF path from the robot base frame to the RealSense camera frame:

```
base_link
  ↓
camera_link
  ↓
camera_color_optical_frame or camera_depth_optical_frame
```

This is required because `/pick_target_3d` currently reports `x`, `y`, `z` in the camera frame. Indy7 control will eventually need target coordinates in a robot base frame.

### Step 3.1 — Confirm Current Output Frame

Run:

```bash
ros2 topic echo /pick_target_3d --once
```

Check:

```yaml
header:
  frame_id: ...
```

Expected frame is likely one of:

```
camera_color_optical_frame
camera_depth_optical_frame
```

Because aligned depth is being used, `camera_color_optical_frame` is the likely output frame, but this must be confirmed from the actual message.

### Step 3.2 — Confirm RealSense Internal TF

Run:

```bash
ros2 topic list | grep tf
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

Expected result:

- `/tf_static` exists.
- `tf2_echo` prints a transform from `camera_link` to the optical frame.

If this fails, RealSense TF publication or frame naming must be checked before adding robot-base transforms.

### Step 3.3 — Add Temporary `base_link -> camera_link` Static Transform

Use an identity transform first to verify TF connectivity:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 \
  --y 0.0 \
  --z 0.0 \
  --roll 0.0 \
  --pitch 0.0 \
  --yaw 0.0 \
  --frame-id base_link \
  --child-frame-id camera_link
```

Then verify:

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

This confirms that the global TF chain is connected.

### Step 3.4 — Replace Temporary Values With Measured Camera Mount Values

Measure the physical RealSense pose relative to the intended robot base frame.

Use:

```
x: forward offset from base_link to camera_link, meters
y: left offset from base_link to camera_link, meters
z: upward offset from base_link to camera_link, meters
roll: rotation around x-axis, radians
pitch: rotation around y-axis, radians
yaw: rotation around z-axis, radians
```

Initial values can be approximate, but they must be recorded clearly. Precision calibration can come later.

Example placeholder only:

```
x = 0.45
y = 0.10
z = 0.70
roll = 0.0
pitch = 0.0
yaw = 0.0
```

These are not verified project values.

### Step 3.5 — Add Static TF Node to Bringup Launch

Add a `tf2_ros/static_transform_publisher` node to:

```
src/system_bringup_pkg/launch/pick_place_apu.launch.py
```

Example:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_camera_tf',
    arguments=[
        '--x', '0.45',
        '--y', '0.10',
        '--z', '0.70',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'camera_link',
    ],
    output='screen',
),
```

Also add this dependency to `src/system_bringup_pkg/package.xml`:

```xml
<exec_depend>tf2_ros</exec_depend>
```

Then rebuild:

```bash
colcon build --packages-select system_bringup_pkg
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Step 3.6 — Validate TF in RViz2

Run:

```bash
rviz2
```

Suggested RViz setup:

```
Fixed Frame: base_link
Add -> TF
Add -> Image
Add -> PointCloud2 or depth image
```

Check:

- `base_link` exists.
- `camera_link` is connected below `base_link`.
- RealSense optical frames are connected.
- Camera direction is plausible and not obviously inverted.

### Step 3 Completion Criteria

Step 3 is complete when:

```
/pick_target_3d header.frame_id is confirmed
/tf_static exists
base_link -> camera_link transform exists
base_link -> camera optical frame tf2_echo succeeds
RViz Fixed Frame = base_link works without TF errors
Measured or placeholder camera mount values are recorded
```

Important note:

Step 3 only creates the TF tree. It does not yet transform `/pick_target_3d` into `base_link` coordinates. A later step should add a node that consumes `/pick_target_3d`, applies TF2, and publishes a robot-base-frame target message.

---

## Immediate Next Work

1. Execute Step 3.1 and record the actual `/pick_target_3d.header.frame_id`.
2. Test RealSense internal TF with `tf2_echo`.
3. Add a temporary `base_link -> camera_link` static transform.
4. Move the static transform into `pick_place_apu.launch.py`.
5. Rebuild and validate the TF tree in RViz2.

---

## Progress Update — 2026-05-17

### APU ROS2 Perception Pipeline Status

Status: **RealSense + mock pipeline completed through base-frame target output**

The baseline ROS2 pipeline has moved beyond the old Step 3 plan.

Implemented/validated:

```
RealSense D435i
→ /camera/camera/color/image_raw
→ /detections
→ /pick_target
→ /pick_target_3d
→ /pick_target_base
```

Current important packages/files:

```
src/my_interfaces/msg/Detection.msg
src/my_interfaces/msg/DetectionArray.msg
src/my_interfaces/msg/PickTarget.msg
src/my_interfaces/msg/PickTarget3D.msg

src/mock_detection_pkg/mock_detection_pkg/mock_detector.py
src/pick_logic_pkg/pick_logic_pkg/pick_logic.py
src/target_3d_pkg/target_3d_pkg/pick_target_3d_node.py
src/target_3d_pkg/target_3d_pkg/pick_target_base_node.py

src/system_bringup_pkg/launch/pick_place_apu.launch.py
src/system_bringup_pkg/config/pick_logic.yaml
src/system_bringup_pkg/config/target_3d.yaml
src/system_bringup_pkg/config/target_base.yaml
src/system_bringup_pkg/config/vitis_ai_detector.yaml
```

Completed since the previous update:

```
Step 3:
  Static TF base_link -> camera_link added to bringup launch.
  RealSense camera optical frames are connected through TF.
  Current base_to_camera values are placeholders and must later be replaced by measured calibration.

Step 4:
  target_transform / base-frame conversion implemented.
  /pick_target_3d is transformed into /pick_target_base.

Step 5A:
  pick_logic_node now subscribes to DetectionArray.
  2D filters implemented:
    confidence threshold
    allowed class filtering
    bbox edge reject
    bbox size reject
  Logging throttled to reduce repeated spam.

Step 5B-min:
  Depth/3D/TF filtering path added around target-base publishing.
  Invalid depth/TF cases are handled before publishing usable base-frame targets.

DetectionArray:
  DetectionArray.msg added.
  mock_detector publishes DetectionArray on /detections.
  pick_logic_node consumes DetectionArray and selects the first acceptable detection.
```

### KV260 Vitis-AI / DPU Status

Status: **DPU runtime and standalone SSD postprocess validated**

Current board/runtime:

```
Board: Kria KV260
Active app: kv260-smartcam
Vitis-AI Runtime/Library: 2.5.0
DPU arch: DPUCZDX8G_ISA1_B3136
DPU fingerprint: 0x101000016010406
Valid DPU CU: DPUCZDX8G:DPUCZDX8G_1
```

Current `xdputil query` state:

```
kv260-smartcam active slot: 0
k26-starter-kits active slot: -1
DPU Core 0:
  DPU Arch: DPUCZDX8G_ISA1_B3136
  fingerprint: 0x101000016010406
  frequency: 300 MHz

pp_pipeline_accel warning / fingerprint 0x0 still appears, but DPU Core 0 is usable.
```

Model used for DPU pipeline validation:

```
~/vitis_ai_work/smartcam_models/models/ssd_adas_pruned_0_95/ssd_adas_pruned_0_95.xmodel
~/vitis_ai_work/smartcam_models/models/ssd_adas_pruned_0_95/ssd_adas_pruned_0_95.prototxt
~/vitis_ai_work/smartcam_models/models/ssd_adas_pruned_0_95/label.json
```

Model details:

```
model: ssd_vehicle_v3_480x360
type: SSD
input: (1, 360, 480, 3)
classes:
  0 background
  1 car
  2 bicycle
  3 person

outputs:
  mbox_conf_reshape_fix: (1, 16436, 4), fix_point=4
  mbox_loc_fixed:        (1, 65744),    fix_point=3

input:
  data_fixed: (1, 360, 480, 3), fix_point=-1
```

Important limitation:

```
ssd_adas_pruned_0_95 is NOT a YCB detector.
It is only being used to validate the DPU detector pipeline shape.
Final pick-place detection still requires a YCB/custom detector compiled for B3136.
```

### Standalone SSD ADAS Test Status

Status: **Completed / successful**

Script:

```
~/vitis_ai_work/scripts/ssd_adas_image_test.py
```

Implemented in standalone script:

```
image load
BGR resize to 480x360
mean subtraction: 104, 117, 123
input fix_point handling
VART runner execute_async()
conf/loc dequantization
softmax for conf logits
prior box generation from prototxt values
bbox decode
class thresholds
NMS
terminal detection output
optional overlay image save
```

Validated:

```
External test.jpg:
  DPU inference succeeded.
  detections produced.
  overlay bbox visually correct.

Saved RealSense frame:
  /home/ubuntu/vitis_ai_work/realsense_frames/realsense_0000.jpg
  standalone inference succeeded even after source ~/ros2_ws/install/setup.bash.
  detections: 0 for that frame, but DPU execution and postprocess completed.
  output saved:
    /tmp/realsense_standalone_check.jpg
```

Conclusion:

```
VART, xmodel, DPU app, SSD decode, and RealSense-frame standalone execution are all valid.
The detector correctness test is sufficient for pipeline integration work.
```

### ROS2 Vitis-AI Detector Integration Status

Status: **Package created, but live ROS integration blocked at VART execute_async()**

Created package:

```
src/vitis_ai_detector_pkg/
```

Current files:

```
src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_detector_node.py
src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_worker.py
src/system_bringup_pkg/config/vitis_ai_detector.yaml
```

Intended ROS detector flow:

```
/camera/camera/color/image_raw
→ vitis_ai_detector_node
→ /detections (my_interfaces/DetectionArray)
→ pick_logic_node
→ /pick_target
→ pick_target_3d_node
→ /pick_target_3d
→ pick_target_base_node
→ /pick_target_base
```

Current `vitis_ai_detector.yaml` role:

```
model_path: ssd_adas_pruned_0_95.xmodel
input_topic: /camera/camera/color/image_raw
output_topic: /detections
softmax_mode: auto
process_period_sec: 0.2
debug_trace: true
worker_log_path: /tmp/vitis_ai_worker.log
```

Observed failure path 1 — direct VART inside rclpy node:

```
ROS image callback received.
Image conversion succeeded.
Preprocess succeeded.
Output buffers allocated.
Crash occurs at:
  runner.execute_async(...)

Observed error:
  Segmentation fault / Bus error
```

Observed failure path 2 — subprocess worker using stdin/stdout IPC:

```
Worker starts successfully.
Worker loads xmodel successfully.
Worker creates VART runner successfully.
Worker receives image payload successfully.
Worker preprocess succeeds.
Worker allocates output buffers.
Worker crashes/hangs at:
  runner.execute_async(...)
```

Worker log endpoint:

```
/tmp/vitis_ai_worker.log
```

Latest worker log ending:

```
worker log start
worker init
priors generated
load_model start: .../ssd_adas_pruned_0_95.xmodel
create_runner start
create_runner done
load_model done: input_shape=(1, 360, 480, 3)
ready sent
request header received
request data received: width=1280, height=720, channels=3, data_len=2764800
image reshaped
detect start: image_shape=(720, 1280, 3), dtype=uint8
preprocess done: input_shape=(1, 360, 480, 3)
output buffers allocated
execute_async start
```

Direct worker protocol test result:

```
ready: {"status": "ready", "input_shape": [1, 360, 480, 3]}
response: <empty>
returncode: -11
log ends at: execute_async start
```

Important comparison:

```
Standalone script with same xmodel and same RealSense jpg succeeds.
Worker process fed through stdin/stdout crashes at execute_async.
Therefore the model, DPU app, and postprocess are valid.
Current blocker is the live/worker integration execution mode around VART execute_async().
```

### Current Hypothesis / Blocker

The likely issue is not ROS message format or SSD postprocess.

Current blocker:

```
VART runner.execute_async() crashes when invoked from the ROS detector integration path.
This happens both in-process and in the current stdin/stdout worker process.
Standalone image script remains successful.
```

Most likely next investigation areas:

```
1. Avoid stdin/stdout pipe IPC and test a file-based one-shot worker.
2. Test a standalone long-running worker outside ROS that reads images from disk.
3. Test whether RealSense + DPU concurrent runtime causes XRT/VART instability.
4. Consider C++ Vitis-AI detector node or Vitis-AI Library API if Python VART live integration remains unstable.
5. Keep ROS detector integration separate from final YCB/custom model work.
```

### Immediate Next Work

Do not debug model accuracy.
Do not tune ADAS thresholds deeply.

Next technical task:

```
Create a minimal one-shot/file-based VART worker test:
  input: image file path
  process: run existing standalone-style VART inference in a fresh process
  output: JSON detections file

Then call that from the ROS node or a small wrapper.
```

Purpose:

```
Determine whether VART is failing because of:
  A. long-running subprocess + pipe-fed image buffer
  B. ROS/RealSense concurrent runtime
  C. Python VART live integration in general
```

If file-based one-shot worker succeeds:

```
Use it as a temporary bridge for /detections publication.
It will be slower but enough to validate the full ROS pipeline.
```

If file-based one-shot worker also fails only while RealSense/ROS is running:

```
Investigate XRT/VART concurrency or move toward C++/Vitis-AI Library integration.
```

---

## Progress Update — 2026-05-18

### Integrated Vitis-AI Launch Validation

Status: **Completed / end-to-end perception pipeline validated**

Created and validated a Vitis-AI based integrated launch:

```
src/system_bringup_pkg/launch/pick_place_vitis_ai.launch.py
```

Validated launch command:

```bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
```

Validated flow:

```
RealSense D435i
→ vitis_ai_detector_node
→ /detections
→ pick_logic_node
→ /pick_target
→ pick_target_3d_node
→ /pick_target_3d
→ static TF base_link -> camera_link
→ pick_target_base_node
→ /pick_target_base
```

Successful observed outputs with a car image shown to the camera:

```
/detections:
  count=1
  class_name=car

/pick_target:
  target_valid=true
  class_name=car

/pick_target_3d:
  target_valid=true
  depth_valid=true
  frame_id=camera_color_optical_frame
  example z ~= 0.35 m

/pick_target_base:
  target_valid=true
  depth_valid=true
  frame_id=base_link
```

Important config alignment:

```
The manual successful detector test used 1280x720 color images.
The integrated launch initially used 848x480 and produced count=0.
realsense_pick_place.yaml was updated to:

depth_module.depth_profile: 848x480x30
rgb_camera.color_profile: 1280x720x30

pick_logic.yaml is also set for:
image_width: 1280
image_height: 720
```

### Historical One-Shot Detector Implementation

Status: **Superseded by long-running worker mode / retained as fallback**

Former detector structure:

```
vitis_ai_detector_node
  subscribes: /camera/camera/color/image_raw
  writes:     /tmp/vitis_ai_latest.jpg
  calls:      ~/vitis_ai_work/scripts/ssd_adas_oneshot_json.py
  reads:      /tmp/vitis_ai_detections.json
  publishes: /detections
  publishes: /vitis_ai_detector/overlay
```

This one-shot file-based detector was introduced because:

```
1. Direct VART execute_async() inside rclpy node crashed.
2. Long-running stdin/stdout Python worker also crashed at execute_async().
3. Standalone image-based script succeeded reliably.
```

Known limitation at that stage:

```
The one-shot implementation is slow and can timeout because every processed frame:
  writes a JPEG file
  starts a Python process
  loads the xmodel
  runs DPU inference
  writes JSON
  returns detections to ROS
```

This was acceptable only as a validation bridge and is now retained as the fallback path.

### Streaming Direction Identified at That Stage

The final/standard detector architecture was identified as streaming-oriented. This direction was subsequently implemented using the long-running worker mode documented in the 2026-05-21 and 2026-06-22 sections.

```
RealSense image stream
→ long-running detector process/node
→ image frames processed in memory
→ xmodel loaded once
→ repeated DPU inference
→ /detections publish
```

Target optimized structures, in preferred order:

```
1. Separate long-running detector process outside rclpy, using stable IPC.
   Candidate IPC: Unix domain socket, ZeroMQ, shared memory, or file path queue.

2. C++ VART / Vitis-AI Library detector process.
   This is likely more stable and closer to Xilinx/Vitis-AI examples than Python VART inside ROS2.

3. ROS2 wrapper node only handles:
   image subscription
   detector process communication
   DetectionArray publishing
```

Do not spend time optimizing the ADAS model itself:

```
ssd_adas_pruned_0_95 detects car / bicycle / person only.
It is not the final YCB pick-place detector.
It is currently used to prove the KV260 DPU + ROS2 perception pipeline.
```

### Next Work

Immediate next priority:

```
Stabilize and document the integrated launch state.
Reduce timeout noise if needed by increasing timeout_sec or lowering process rate.
```

Next engineering priority:

```
Replace the one-shot file-based detector with a streaming detector architecture:
  xmodel load once
  repeated inference
  no per-frame Python process startup
  no per-frame JPEG/JSON disk roundtrip
```

Later project priorities:

```
1. Replace placeholder base_link -> camera_link TF with measured calibration.
2. Prepare/compile final YCB/custom detector for B3136.
3. Replace ADAS classes in pick_logic.yaml with actual pickable object classes.
4. Add robot command layer after perception is stable.
```

---

## Issue Record — 2026-05-19

### ROS2 + Python VART Runtime Crash During Live Detector Integration

Status: **Resolved on 2026-05-21 / one-shot detector retained only as fallback**

Yes. The first attempt to integrate the Vitis-AI detector directly with ROS2 failed at runtime.
The failure was not caused by the xmodel itself, the DPU overlay, or the SSD post-processing logic.
The failure happened when VART inference was invoked from the live ROS2 detector integration path.

Working standalone path:

```
saved image file
→ ssd_adas_image_test.py
→ load xmodel
→ create VART runner
→ execute_async()
→ wait()
→ SSD postprocess
→ detections / overlay output
```

This path succeeds with:

```
~/vitis_ai_work/scripts/ssd_adas_image_test.py
~/vitis_ai_work/scripts/ssd_adas_oneshot_json.py
```

Failing integration path 1 — VART directly inside the ROS2 Python node:

```
/camera/camera/color/image_raw
→ rclpy image callback
→ image conversion
→ preprocess to int8 input tensor
→ allocate output buffers
→ runner.execute_async()
→ crash
```

Observed runtime failures:

```
Segmentation fault
Bus error
```

The trace showed the node reached:

```
[trace] execute_async start
```

and then the process crashed before returning from VART.

Failing integration path 2 — long-running Python worker using stdin/stdout raw image IPC:

```
ROS2 wrapper node
→ send raw image payload to worker through stdin
→ worker loads xmodel
→ worker creates VART runner
→ worker receives image
→ worker preprocesses image
→ worker allocates output buffers
→ runner.execute_async()
→ worker crashes
```

Observed worker result:

```
returncode: -11
response: <empty>
```

Worker log ended at:

```
worker log start
worker init
priors generated
load_model start: .../ssd_adas_pruned_0_95.xmodel
create_runner start
create_runner done
load_model done: input_shape=(1, 360, 480, 3)
ready sent
request header received
request data received: width=1280, height=720, channels=3, data_len=2764800
image reshaped
detect start: image_shape=(720, 1280, 3), dtype=uint8
preprocess done: input_shape=(1, 360, 480, 3)
output buffers allocated
execute_async start
```

Important conclusion:

```
Same board
same kv260-smartcam DPU app
same B3136-compatible xmodel
same image content
same SSD postprocess
standalone script succeeds
ROS2 in-process / long-running pipe worker crashes at execute_async
```

Therefore the validated interpretation is:

```
The model and DPU runtime are valid.
The crash is tied to the live integration/runtime boundary around Python VART execute_async().
```

Historical workaround used before the root-cause fix:

```
vitis_ai_detector_node
→ subscribe RealSense image
→ save latest frame to /tmp/vitis_ai_latest.jpg
→ run standalone-style one-shot process
→ read /tmp/vitis_ai_detections.json
→ publish /detections
→ publish /vitis_ai_detector/overlay
```

This workaround was functionally validated:

```
/detections
→ /pick_target
→ /pick_target_3d
→ /pick_target_base
```

works when a car image is detected by `ssd_adas_pruned_0_95`.

Known limitation of the historical workaround:

```
slow latency
occasional timeout
per-frame JPEG write
per-frame Python process startup
per-frame xmodel load
per-frame JSON file roundtrip
```

Architecture originally recommended before the root-cause fix:

```
Keep ROS2 and VART separated by process boundary.
Use a long-running detector process that loads xmodel once.
Use lower-overhead IPC instead of per-frame file/process startup.
Preferred next prototype:
  ROS2 wrapper process
  + shared memory input tensor
  + lightweight signal/request mechanism
  + detector worker process without rclpy
```

Resource-efficient target:

```
ROS2 wrapper:
  subscribe image
  keep latest frame only
  resize/preprocess to int8[1,360,480,3]
  write preprocessed tensor to shared memory
  signal worker
  publish DetectionArray

Vitis-AI worker:
  no rclpy
  load xmodel once
  create VART runner once
  wait for latest tensor
  execute_async repeatedly
  write compact detection result back
```

The later root-cause investigation showed that shared memory was not required to fix the crash. The actual cause was the XIR graph/subgraph Python objects going out of scope while the VART runner remained in use. Keeping those objects alive fixed repeated `execute_async()` calls. If another runtime stability problem appears in the future, the fallback options remain:

```
C++ VART
or
Vitis-AI Library based detector process
```

---

## Progress Update — 2026-05-21

### VART `execute_async()` Crash Root Cause

Status: **Root cause identified / long-running worker validated**

The previous ROS2 + Python VART crash was narrowed down and fixed.

Confirmed successful tests:

```
ssd_adas_repeat_infer_test.py
  runner created once
  same JPG image inferred 1000 times
  output buffers newly allocated each iteration
  result: success

ssd_adas_repeat_infer_test.py --reuse-output-buffers
  runner created once
  same JPG image inferred 1000 times
  output buffers reused
  result: success

vitis_ai_worker_direct_image_test.py
  imports VitisAiWorker directly
  calls detect() repeatedly
  result after fix: success

vitis_ai_worker_protocol_test.py
  starts vitis_ai_worker.py as long-running subprocess
  sends raw image payload through stdin/stdout protocol
  100 iterations
  result after fix: success
  observed throughput: ~12.9 FPS on 299x168 test image
```

Root cause:

```
The VART runner was created from XIR graph/subgraph objects,
but the graph/subgraph Python wrapper objects were allowed to go out of scope.

The runner object alone was not enough to keep the underlying XIR objects alive
on this Vitis-AI Python runtime.

When runner.execute_async() was called later, the process segfaulted.
```

Failing pattern:

```python
def load_runner():
    graph = xir.Graph.deserialize(model_path)
    dpu_subgraphs = get_dpu_subgraphs(graph)
    runner = vart.Runner.create_runner(dpu_subgraphs[0], "run")
    return runner
```

Fixed pattern:

```python
self.graph = xir.Graph.deserialize(model_path)
self.dpu_subgraphs = get_dpu_subgraphs(self.graph)
self.runner = vart.Runner.create_runner(self.dpu_subgraphs[0], "run")
```

Files updated:

```
src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_worker.py
  Keeps self.graph and self.dpu_subgraphs alive for the runner lifetime.

src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_detector_node.py
  Adds detector_mode=worker.
  Starts a long-running vitis_ai_worker.py subprocess.
  Sends image payloads to the worker instead of launching one Python process per frame.
  Keeps one-shot mode as fallback via detector_mode=oneshot.

src/system_bringup_pkg/config/vitis_ai_detector.yaml
  detector_mode set to worker.
  worker_script_path, worker_log_path, worker_softmax added.

~/vitis_ai_work/scripts/ssd_adas_repeat_infer_test.py
~/vitis_ai_work/scripts/vitis_ai_worker_protocol_test.py
~/vitis_ai_work/scripts/vitis_ai_worker_direct_image_test.py
~/vitis_ai_work/scripts/ssd_adas_path_worker.py
  Diagnostic scripts added during root-cause isolation.
```

Build validation:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vitis_ai_detector_pkg system_bringup_pkg
```

Result:

```
vitis_ai_detector_pkg: success
system_bringup_pkg: success
```

Current detector architecture:

```
vitis_ai_detector_node
  subscribes: /camera/camera/color/image_raw
  converts ROS Image -> BGR numpy array
  sends raw BGR frame to long-running vitis_ai_worker.py
  receives JSON detections
  publishes: /detections
  publishes: /vitis_ai_detector/overlay

vitis_ai_worker.py
  no rclpy
  loads xmodel once
  creates VART runner once
  keeps XIR graph/subgraph alive
  executes repeated VART inference
  returns DetectionArray-compatible JSON items
```

Immediate next validation:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
```

Then verify:

```bash
ros2 topic echo /detections --once
ros2 topic echo /pick_target --once
ros2 topic echo /pick_target_3d --once
ros2 topic echo /pick_target_base --once
```

Expected result:

```
The integrated RealSense -> Vitis-AI worker -> /detections -> /pick_target_base
pipeline should run without the previous execute_async segmentation fault.
```

Launch validation result:

```
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
```

Observed:

```
RealSense D435i started successfully.
vitis_ai_detector_node started in worker mode.
Long-running Vitis-AI worker became ready with input_shape=[1, 360, 480, 3].
/detections was published repeatedly without execute_async crash.
Downstream /pick_target, /pick_target_3d, and /pick_target_base also published.
```

Observed sample with no ADAS object in view:

```
/detections: []
/pick_target: target_valid=false
/pick_target_3d: target_valid=false, depth_valid=false
/pick_target_base: target_valid=false, depth_valid=false, frame_id=base_link
```

Interpretation:

```
This is a valid no-detection pipeline state, not a runtime failure.
The current SSD ADAS model only detects car, bicycle, and person.
Use a visible car/person/bicycle image to validate positive detections.
```

---

## Current Status Snapshot — 2026-06-22

### Current Detector Architecture

Status: **Long-running worker mode implemented and selected by default**

Actual configuration:

```yaml
detector_mode: "worker"
process_period_sec: 0.1
input_topic: "/camera/camera/color/image_raw"
output_topic: "/detections"
overlay_topic: "/vitis_ai_detector/overlay"
```

Current execution flow:

```text
RealSense color image
→ vitis_ai_detector_node (ROS2 wrapper)
→ raw BGR frame sent to long-running vitis_ai_worker.py
→ xmodel and VART runner reused
→ SSD post-processing
→ DetectionArray-compatible JSON response
→ /detections
→ pick_logic_node
→ /pick_target
→ pick_target_3d_node
→ /pick_target_3d
→ pick_target_base_node
→ /pick_target_base
```

The worker currently uses stdin/stdout IPC. It does not use shared memory yet.
Shared memory remains an optional optimization if IPC copying or CPU usage becomes a measured bottleneck; it is no longer required as a crash workaround.

### Resolved Runtime Failure

The earlier `Segmentation fault` / `Bus error` at `runner.execute_async()` is resolved.

Root cause:

```text
XIR graph/subgraph Python wrapper objects were released after runner creation.
The VART runner later referenced invalid underlying XIR state.
```

Fix:

```python
self.graph = xir.Graph.deserialize(self.model_path)
self.dpu_subgraphs = get_dpu_subgraphs(self.graph)
self.runner = vart.Runner.create_runner(self.dpu_subgraphs[0], "run")
```

The worker retains `self.graph` and `self.dpu_subgraphs` for the full runner lifetime.
Repeated inference and the worker protocol were validated after this fix.

### Verified Functional Scope

Validated:

```text
KV260 SmartCam DPU overlay active
B3136-compatible SSD ADAS xmodel load
standalone VART inference
SSD confidence/location dequantization
softmax, prior decode, thresholds, and NMS
bbox overlay generation
long-running worker inference without the previous crash
RealSense → /detections
/detections → /pick_target
/pick_target + aligned depth → /pick_target_3d
camera optical frame → base_link → /pick_target_base
```

Current model limitation:

```text
ssd_adas_pruned_0_95 classes:
  0 background
  1 car
  2 bicycle
  3 person

This model validates the DPU and ROS2 perception pipeline.
It is not the final YCB/custom pick-object detector.
```

The current `base_link → camera_link` static transform still uses placeholder values. Therefore `/pick_target_base` proves the TF pipeline works, but its coordinates must not be treated as final robot coordinates until camera-to-base calibration is completed.

### Multi-Image Regression Check

Status: **Planned / no completed result has been recorded yet**

The next low-risk verification is to run several still images through the standalone one-shot inference path. This checks preprocessing, DPU inference, SSD post-processing, class output, and bbox coordinates independently of ROS2 and depth.

Recommended image set:

```text
clear car image
clear bicycle image
clear person image
multiple supported objects in one image
image without car/bicycle/person
RealSense-captured color image
```

Recommended directories:

```text
~/vitis_ai_work/test_batch/input
~/vitis_ai_work/test_batch/output
```

Per-image command pattern:

```bash
python3 ~/vitis_ai_work/scripts/ssd_adas_oneshot_json.py \
  --model ~/vitis_ai_work/smartcam_models/models/ssd_adas_pruned_0_95/ssd_adas_pruned_0_95.xmodel \
  --image ~/vitis_ai_work/test_batch/input/car_01.jpg \
  --output ~/vitis_ai_work/test_batch/output/car_01_overlay.jpg \
  --json-output ~/vitis_ai_work/test_batch/output/car_01.json
```

Acceptance criteria:

```text
bbox covers the correct object
bbox remains inside the source image coordinates
class is correct for car/bicycle/person
confidence is plausible and stable
empty image does not produce excessive false positives
script exits normally without VART runtime errors
```

This is a correctness regression check, not a full model accuracy or mAP evaluation.
External JPG files cannot validate `/pick_target_3d` because they do not provide synchronized depth and camera intrinsics. The full 3D pipeline must still be checked with the live RealSense topics.

### Current Next Priorities

```text
1. Complete and record the multi-image standalone regression check.
2. Re-run the integrated worker-mode launch with a visible supported object.
3. Measure real 1280x720 live latency/FPS and CPU/memory usage before changing IPC.
4. Optimize IPC with latest-frame-only/shared memory only if measurements justify it.
5. Replace placeholder camera-to-base TF with measured calibration.
6. Prepare and compile the final YCB/custom detector for the B3136 DPU.
7. Add robot reachability, safe workspace, and robot command layers afterward.
```

---

## Low-Latency Streaming Optimization — 2026-06-22

Status: **Implemented and built / live DPU benchmark pending**

Goal:

```text
Use the RealSense video stream continuously without accumulating stale frames,
while minimizing CPU, memory, disk I/O, and end-to-end detection latency.
```

The existing long-running worker architecture was retained. The one-shot fallback was not removed.

Implemented changes:

```text
Camera subscription QoS:
  KEEP_LAST
  depth=1
  BEST_EFFORT

Frame policy:
  process the newest available frame
  do not accumulate an image queue
  process_period_sec=0.0, so the DPU runs again as soon as it is available

Worker IPC:
  read the ROS image payload through a zero-copy NumPy view
  perform only the required RGB-to-BGR output allocation
  resize 1280x720 BGR image to the xmodel input size before pipe transfer
  current SSD input: 480x360x3
  preserve source_width/source_height in the request
  return bbox coordinates in the original 1280x720 image coordinate system

Payload reduction:
  previous raw frame: 1280x720x3 = 2,764,800 bytes
  current worker payload: 480x360x3 = 518,400 bytes
  reduction: approximately 81 percent

VART worker:
  xmodel and runner remain loaded once
  XIR graph/subgraph lifetime fix remains in place
  output tensor buffers are allocated once and reused
  redundant worker-side image copy removed

Optional work configuration:
  publish_overlay=false
  publish_compressed_overlay=true  # temporary visual validation mode
  overlay size=480x360
  overlay JPEG quality=70
  worker_log_path="/tmp/vitis_ai_worker.log"  # temporary diagnosis
  log_period_sec=1.0

Low-rate performance log:
  processing_ms: detector callback processing time
  frame_age_ms: camera timestamp to detection publication age
```

Files updated:

```text
src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_detector_node.py
src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_worker.py
src/system_bringup_pkg/config/vitis_ai_detector.yaml
```

Validation completed:

```text
Python syntax compilation: success
colcon build --packages-select vitis_ai_detector_pkg system_bringup_pkg --symlink-install: success
```

Hardware repetition test could not run in the current session because the DPU device was unavailable:

```text
/dev/dri does not exist
xdputil query aborts while opening /dev/dri/by-path
VART runner creation fails before inference
```

This is an accelerator/app activation issue, not a Python syntax or ROS package build failure. Before live validation, restore the SmartCam app and verify:

```bash
sudo xmutil listapps
sudo xmutil unloadapp
sudo xmutil loadapp kv260-smartcam
xdputil query
```

Expected active accelerator:

```text
kv260-smartcam: Active_slot 0
DPUCZDX8G_ISA1_B3136
fingerprint: 0x101000016010406
```

Then run:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
```

Validation topics:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /detections
ros2 topic echo /detections --once
ros2 topic echo /pick_target_3d --once
ros2 topic echo /pick_target_base --once
```

Compressed overlay is currently enabled for visual bbox validation:

```yaml
# src/system_bringup_pkg/config/vitis_ai_detector.yaml
publish_overlay: false
publish_compressed_overlay: true
overlay_width: 480
overlay_height: 360
overlay_jpeg_quality: 70
```

After diagnosis, restore `worker_log_path: ""` to remove unnecessary file I/O.

Shared memory is not implemented at this stage. The current pipe payload has first been reduced by approximately 5.3 times. Shared memory should be added only if live CPU/memory measurements show that the remaining IPC copy is a meaningful bottleneck.

### Worker startup timeout handling — 2026-06-22

Observed failure:

```text
Starting Vitis-AI worker process
subprocess.TimeoutExpired: worker ready timed out after 10.0 seconds
```

The timeout occurs before frame inference: the worker did not finish XIR/VART runner initialization and send its `ready` response within the inference timeout. Possible triggers include temporary DPU startup delay or an old worker still owning runtime resources.

Implemented:

```text
worker_startup_timeout_sec=30.0  # startup only
timeout_sec=10.0                 # inference remains unchanged
cleanup partially started worker when initialization fails
safe node cleanup when constructor/startup fails
```

Python syntax validation and package rebuild completed successfully.

### Live worker stall and overlay bottleneck diagnosis — 2026-06-22

Observed behavior:

```text
worker starts and reports ready
several detections are published successfully
ROS launch output later becomes silent
worker log ends at response sent
```

Process inspection while apparently stalled showed both processes still alive:

```text
vitis_ai_detector_node: alive
vitis_ai_worker.py: alive
/vitis_ai_detector_node: present in ROS graph
camera image input: approximately 6-7 Hz during the check
```

Therefore, absence of console output did not by itself mean process death. The worker trace established that the complete accelerator path succeeded:

```text
execute_async start
execute_async submitted
runner wait done
raw outputs selected
postprocess done
detections done
response sent
```

This excludes DPU execution, `runner.wait()`, SSD decoding, and worker response generation as the location of that stall. The remaining path was the ROS wrapper after receiving the worker response, especially overlay construction/serialization/publication.

Temporary stage timing was added to the detector log:

```text
detect_ms
detection_publish_ms
overlay_ms
frame_age_ms
```

Measured example:

```text
detect_ms:             103-343 ms
detection_publish_ms:  0.8-1.5 ms
raw overlay_ms:        656-975 ms
frame_age_ms:          approximately 1.0-2.2 s and increasing
```

Conclusion: raw ROS overlay publication was 5-9 times slower than inference and dominated callback latency. `/detections` publication itself was not the bottleneck.

Implemented overlay changes:

```text
raw /vitis_ai_detector/overlay disabled
480x360 overlay generated from the source frame
bbox scaled only for visualization
DetectionArray coordinates remain in the original 1280x720 source coordinate system
JPEG CompressedImage published on /vitis_ai_detector/overlay/compressed
JPEG quality set to 70
```

This preserves the existing 2D/3D target pipeline while reducing overlay network payload and Python/DDS serialization pressure.

An unrelated launch failure was also found and fixed. The YAML contained an extra quote:

```text
invalid: worker_log_path: "/tmp/vitis_ai_worker.log""
fixed:   worker_log_path: "/tmp/vitis_ai_worker.log"
```

The corrected YAML was parsed successfully and both packages rebuilt successfully:

```text
vitis_ai_detector_pkg
system_bringup_pkg
```

Current validation command:

```bash
pkill -f vitis_ai_worker.py
pkill -f vitis_ai_detector_node

cd ~/ros2_ws
source install/setup.bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
```

Check topics separately because `ros2 topic hz` runs continuously until stopped:

```bash
timeout 15s ros2 topic hz /detections
timeout 15s ros2 topic hz /vitis_ai_detector/overlay/compressed
```

Desktop visualization target:

```text
base image topic: /vitis_ai_detector/overlay
image transport: compressed
wire topic: /vitis_ai_detector/overlay/compressed
```

Pending validation:

```text
confirm compressed overlay continues publishing without the previous silent stall
compare new overlay_ms and frame_age_ms with the raw-overlay measurements
confirm /detections, /pick_target_3d, and /pick_target_base remain continuous
disable worker file logging after diagnosis
```

---

## Camera Stall Root Cause + Throughput Profiling Session — 2026-06-22

### 1. Silent Stall Root Cause: RealSense camera node busy-spin

Symptom: after ~70 s of normal operation the whole pipeline went silent
(`/detections` stopped) while every process stayed alive — the "silent stall".

Diagnosis (from live logs + `ps`/`top`):

```text
worker:    265 requests / 265 responses, last line "response sent" -> idle, healthy
detector:  stopped publishing at the same instant
realsense2_camera_node: burned 235 -> 310 CPU-seconds in ~75 s wall time
           = pegging one A53 core in a 100% busy-spin, delivering ZERO frames
```

Conclusion: the freeze originates at the **RealSense camera node**, not the
Vitis-AI/DPU path. The detector only processes the newest camera frame, so when
the camera stops delivering frames the entire downstream pipeline goes silent.
The earlier overlay bottleneck (chased in previous sections) was already fixed.

Likely cause: on the KV260 APU, `1280x720x30 RGB + 848x480x30 depth + on-CPU
depth-to-color alignment` saturates the USB3/CPU budget and librealsense wedges.
(`dmesg` UVC logs not readable without sudo, but frame=0 + 100% busy-spin is the
classic signature.)

### 2. Fix: lower color resolution (keep FPS), validated 2 min no-stall

```text
src/system_bringup_pkg/config/realsense_pick_place.yaml
  rgb_camera.color_profile: 1280x720x30 -> 848x480x30   (depth already 848x480x30)
  Rationale: detector downsizes every frame to 480x360 before inference anyway
  (send_resized_input), so detection quality is unchanged, but USB bandwidth and
  the on-CPU alignment cost drop ~2.3x. Both profiles are 16:9 -> same 480x360
  resize aspect.

src/system_bringup_pkg/config/pick_logic.yaml
  image_width/height: 1280/720 -> 848/480
  (detector returns bbox in SOURCE coords, so edge/area filters need new dims)
```

Result: 2-minute continuous run with no recurrence. Camera ~17-20 Hz stable,
realsense CPU ~90% (no longer wedging).

### 3. Detector throughput optimizations (overlay + worker file log off)

```text
src/system_bringup_pkg/config/vitis_ai_detector.yaml
  publish_compressed_overlay: true -> false   (overlay was ~70 ms/frame, debug-only)
  worker_log_path: "/tmp/vitis_ai_worker.log" -> ""  (worker did ~18 file
                                                       open/write/flush/close per frame)
```

Effect: processing_ms ~180 -> ~85 ms, /detections ~5.7 -> ~7.5 Hz.

### 4. Per-stage timing instrumentation

Added timers so detect_ms is fully decomposed. Worker reports pure compute
sub-times in its JSON response; the node splits the rest:

```text
src/vitis_ai_detector_pkg/.../vitis_ai_worker.py
  detect() measures pre_ms / dpu_ms / post_ms / worker_ms, returned as
  response["timing"]. (worker file logging disabled, no per-frame I/O.)

src/vitis_ai_detector_pkg/.../vitis_ai_detector_node.py
  image_callback logs the full breakdown and adds age_in_ms (frame age at
  callback entry = camera capture -> our callback).
```

Timing model (nested totals, leaves are the real work):

```text
frame_age_ms (capture -> publish)
├─ age_in_ms                         (camera/USB/realsense-node/DDS/queue; one lump, not splittable from our side)
└─ processing_ms (whole callback)
   ├─ detect_ms = img_ms + worker_call_ms
   │              worker_call_ms = ipc_overhead_ms + worker_ms
   │                               worker_ms = pre_ms + dpu_ms + post_ms
   ├─ detection_publish_ms
   └─ overlay_ms
```

### 5. CSV metrics collection feature

```text
src/vitis_ai_detector_pkg/.../vitis_ai_detector_node.py
  New param metrics_csv_path (config default /tmp/vitis_ai_metrics.csv).
  When set: every frame's metrics are buffered in memory (no per-frame terminal
  spam, just a throttled "Collecting metrics: N rows" heartbeat) and written to
  CSV once on shutdown. SIGTERM handler added so pkill also flushes the CSV
  (Ctrl-C / SIGTERM save; kill -9 does not).
  18 columns: stamp_sec, stamp_nanosec, count, image_w, image_h, processing_ms,
  detect_ms, img_ms, worker_call_ms, ipc_overhead_ms, pre_ms, dpu_ms, post_ms,
  worker_ms, detection_publish_ms, overlay_ms, age_in_ms, frame_age_ms.
```

### 6. Profiling result (546 frames / 63 s, ~/vitis_ai_metrics.csv)

Measurement integrity confirmed: sum(leaf means)=88.7=mean(processing_ms);
mean(age_in)+mean(processing)=349.4=mean(frame_age_ms).

```text
publish rate ~ 8.6 Hz

processing_ms breakdown (mean 88.7 ms):
  pre_ms              42.5 ms   48%   <-- dominant bottleneck
  post_ms             20.1 ms   23%
  dpu_ms              12.7 ms   14%   (pure DPU; very stable 12-20)
  ipc_overhead_ms     10.2 ms   12%
  img_ms               2.3 ms    3%
  detection_publish_ms 0.9 ms    1%
  overlay_ms           0.0 ms    0%

latency frame_age_ms (mean 349 ms):
  age_in_ms          260.7 ms   75%   <-- camera/realsense-node side
  processing_ms       88.7 ms   25%
```

### 7. Conclusions

```text
THROUGHPUT bottleneck = pre_ms (Python preprocessing), NOT DPU/IPC/alignment.
  DPU is only ~13 ms (chip could do ~75 Hz). shared memory (ipc ~10 ms) and
  alignment removal do NOT raise the detector rate.

LATENCY bottleneck = age_in_ms ~260 ms (realsense node ~90% CPU publishes frames
  long after capture). Reducing realsense load (alignment) helps latency; our
  processing optimization only helps it by ~50 ms.
```

### 8. Next steps (priority for the 30 Hz throughput goal)

```text
1. LUT preprocessing in vitis_ai_worker.preprocess_image():
   precompute per-channel 256-entry int8 tables (mean/fix are constant, input is
   uint8) -> replace per-frame float math with a table lookup.
   Expected pre_ms 42 -> ~5 ms  => processing ~51 ms => ~17-19 Hz.
2. post_ms pre-threshold: filter priors on raw logit before softmax/decode/NMS.
   Expected post_ms 20 -> ~8 ms => processing ~39 ms => ~22-25 Hz.
3. (optional, latency only) reduce realsense alignment load.
4. shared memory IPC only if ipc_overhead becomes relatively significant after 1-2.

Validation method: re-run with metrics_csv_path enabled, compare before/after
column means from the CSV.
```

---

## Detector Optimization Results — LUT + post-process pre-filter — 2026-06-22

### Metrics file layout (~/ros2_ws/metrics/)

```text
vitis_ai_metrics.csv  = metrics_csv_path target (latest run; overwritten each run)
vitis_ai_metrics1.csv = stage 1 baseline (before optimization, 546 frames)
vitis_ai_metrics2.csv = stage 2 after LUT (749 frames)
                        (stage 3 result currently in vitis_ai_metrics.csv, 788 frames)
metrics_csv_path config default: /home/ubuntu/ros2_ws/metrics/vitis_ai_metrics.csv
save_metrics_csv() does os.makedirs(exist_ok=True), so the dir is auto-created.
```

### Optimization 1: LUT preprocessing (DONE, verified bit-identical)

```text
src/vitis_ai_detector_pkg/.../vitis_ai_worker.py
  build_input_lut(): precompute per-channel 256-entry int8 tables once at load
    (mean/scale/round/clip are constant, input is uint8 -> only 256 values).
  preprocess_image(): replace per-frame float math with 3 LUT lookups.
  Verified identical to the old formula bit-for-bit (input_fix -1/0/1/2,
  random images, 100% match).
Effect: pre_ms 42.5 -> 12.7 ms.
```

### Optimization 2: post-process background pre-filter (DONE, verified identical)

```text
src/vitis_ai_detector_pkg/.../vitis_ai_worker.py
  postprocess(): most of the 16436 priors are background, so filter them out
    BEFORE softmax/decode/NMS, then process only the small candidate set.
    - softmax case (logits): keep prior if z_fg - z_background >= logit(threshold)
      (safe necessary condition; epsilon margin so FP borderline never dropped).
    - already-probabilities case: keep prior if any fg prob >= min class threshold.
    loc dequant + decode_boxes run only on candidates.
  Verified identical to full-scan path (30 random trials, softmax + non-softmax).
Effect: post_ms 19.4 -> 7.4 ms.
```

### Stage-by-stage measurement (each ~1 min, car in view)

```text
metric              1 baseline     2 +LUT     3 +LUT+post
publish Hz                8.61      12.17           13.00     (+51% total)
pre_ms                   42.5       12.7            12.5
post_ms                  20.1       19.4             7.4
dpu_ms                   12.7       13.0            12.8      (hardware floor)
ipc_overhead_ms          10.2       10.1            10.0
processing_ms            88.7       58.2            45.8      (-43, ~half)
frame_age_ms            349.4      306.9           292.1
age_in_ms               260.7      248.7           245.9
```

Integrity held every stage (sum of leaf means == processing_ms;
age_in + processing == frame_age).

### Key finding: throughput now limited by camera delivery / callback scheduling, not compute

```text
Stage 3 processing_ms = 45.8 ms  => arithmetically ~21.8 Hz capable,
but actual publish rate is only 13.0 Hz.
The ~31 ms/frame gap = the callback waiting for the next camera frame
(camera ~17-20 Hz, capped by on-CPU depth-to-color alignment) + executor/busy-skip.

Remaining processing_ms breakdown (45.8 ms):
  dpu_ms  12.8 (28%, hardware floor, not reducible in Python)
  pre_ms  12.5 (27%)
  ipc     10.0 (22%)
  post_ms  7.4 (16%)
```

Conclusion: easy compute wins (pre, post) are now spent. Going past ~13 Hz
needs the camera/scheduling side, not more post-process shaving.

### Remaining levers toward 30 Hz (priority)

```text
1. Callback pipelining: preprocess frame N+1 while the worker runs DPU on frame N
   (overlap the ~23 ms dpu+ipc wait). Closes the 13 -> ~21.8 Hz (compute-limit) gap
   WITHOUT touching the camera. Recommended next.
2. Raise camera FPS by removing full-frame alignment (see note below). Needed to go
   above ~21 Hz, and also cuts latency (age_in ~246 ms is 75% of frame_age).
3. (small) shared memory IPC (ipc 10 -> ~6 ms); further pre_ms shaving.
   dpu_ms is fixed by the chip.
```

### Architectural note: depth for x/y/z without full-frame alignment

```text
pick_target_3d_node only samples depth at ONE pixel (bbox center), but align_depth
reprojects all ~407k pixels. The plan is NOT to toggle alignment on/off, but to
REPLACE full-frame alignment with single-point reverse projection:
  - disable align_depth, subscribe to RAW depth + depth/color intrinsics +
    depth->color extrinsics (/camera/camera/extrinsics/depth_to_color).
  - compute depth for the target pixel only, via rs2_project_color_pixel_to_depth_pixel
    style epipolar line search (cheap per detection).
This keeps accurate z while removing the alignment CPU cost (helps camera FPS AND
latency). Do this when latency/FPS demands it; current alignment path works and is
left ON for now.
```

### Status at end of session

```text
DONE:   camera stall fix (resolution), overlay/log off, timing instrumentation,
        CSV metrics collection, LUT preprocessing, post-process pre-filter.
RESULT: 8.6 -> 13.0 Hz, processing 88.7 -> 45.8 ms, frame_age 349 -> 292 ms,
        detection accuracy unchanged (both optimizations verified bit-identical).
NEXT:   (1) callback pipelining, (2) single-point depth / alignment replacement.
```

---

## Pipelining + CPU-contention investigation + single-point depth — 2026-06-22

### Auto-measurement + boot helpers (tooling)

```text
vitis_ai_detector_node.py
  metrics_duration_sec param (config default 60.0): auto-collect N seconds from
    the FIRST processed frame, then write the CSV automatically and log
    "Metrics window complete" (no manual stopwatch). 0 = until shutdown.
  SIGTERM handler so pkill also flushes the CSV.
Metrics CSVs now live in ~/ros2_ws/metrics/ (save_metrics_csv does makedirs).
  vitis_ai_metrics1.csv=baseline, 2=+LUT, 3=+LUT+post, (4=+pipe).

Boot auto-load of the accelerator (run once, with sudo):
  /etc/systemd/system/kv260-smartcam.service (oneshot, After=dfx-mgr.service,
  ExecStartPre=-/usr/bin/xmutil unloadapp, ExecStart=/usr/bin/xmutil loadapp
  kv260-smartcam) + systemctl enable. Removes the manual loadapp every boot.
```

### Stage 4: callback pipelining (DONE)

```text
vitis_ai_detector_node.py
  image_callback now only stores the newest frame (lightweight, never blocks).
  A dedicated worker thread (worker_loop -> process_frame) consumes frames
  back-to-back so the DPU worker is never idle waiting for the next callback.
  (Python threads are fine here: the heavy DPU work is a separate process, and
   the node thread mostly waits on pipe I/O where the GIL is released.)

Result: throughput stayed ~13 Hz (12.7), but latency dropped
  frame_age 292 -> 261 ms, age_in 246 -> 214 ms.
Interpretation: pipelining did NOT raise throughput -> the inter-callback gap
  was NOT the bottleneck. The system is camera/upstream-limited, not compute.
```

### CPU-contention investigation (key finding)

```text
Camera color rate measured directly with `ros2 topic hz`:

  camera ALONE (no consumers):           ~25 Hz  (align on == align off)
  full pipeline, align_depth ON:         camera ~12 Hz, /detections ~10 Hz
  full pipeline, align_depth OFF:        camera ~20 Hz, /detections ~20 Hz

top under full load:
  align ON:  realsense2_camera_node = ~100% (a full A53 core), ~40% idle overall
  align OFF: realsense ~35%, detector ~50%, worker ~40%, ~52% idle

ROOT CAUSE: realsense computes depth-to-color alignment ONLY when something
subscribes to the aligned topic. Camera-alone has no aligned subscriber, so no
alignment ran -> 25 Hz both ways (this misled an earlier A/B test). Under the
real pipeline, pick_target_3d subscribed to aligned depth -> alignment ran ->
its single thread pegged one core at 100% and throttled the camera to ~12 Hz.
Alignment ~= 65% of a core, and it HALVED detection throughput.
```

### Single-point reverse projection (DONE + validated)

Replaced full-frame alignment with per-target depth lookup so align_depth can
stay OFF permanently while still producing z.

```text
src/system_bringup_pkg/config/realsense_pick_place.yaml
  align_depth.enable: true -> false   (permanent)

src/target_3d_pkg/target_3d_pkg/pick_target_3d_node.py  (rewritten)
  subscribes raw depth (/camera/camera/depth/image_rect_raw, 16UC1) +
  depth/color camera_info + /camera/camera/extrinsics/depth_to_color.
  For the bbox-center COLOR pixel, runs rs2_project_color_pixel_to_depth_pixel
  (epipolar line search) to find the matching DEPTH pixel, patch-median z, then
  deprojects in camera_depth_optical_frame. Output frame changed
  camera_color_optical_frame -> camera_depth_optical_frame (still TF-connected
  to base_link). NOTE: rs2 extrinsics rotation is COLUMN-major (reshape order='F').

src/target_3d_pkg/package.xml: added realsense2_camera_msgs dependency.
src/system_bringup_pkg/config/target_3d.yaml: raw-depth/extrinsics topics.
Rebuilt: colcon build --packages-select target_3d_pkg --symlink-install.

Board intrinsics (848x480):
  depth: fx=fy=419.803, cx=423.589, cy=236.568, distortion 0
  color: fx=601.064, fy=599.600, cx=424.407, cy=230.548, distortion 0
  extrinsics depth->color t ~= [15.09mm, 0.16mm, ~0]  (baseline)
```

Validation:

```text
- Geometry sim with real intrinsics/extrinsics: 5 known points recovered the
  exact depth pixel (0.0 px error). Confirms algorithm + column-major rotation.
  (color->depth disparity is LARGE, e.g. 60-90 px, so naive same-pixel is wrong.)
- Live: /pick_target_3d depth_valid=true, z=0.324 m; tape-measure CONFIRMED.
- TF self-consistent: x_base=0.45+z_opt, y_base=0.10-x_opt, z_base=0.70-y_opt
  all matched /pick_target_base exactly.
- Throughput: /detections ~13 -> ~17 Hz (+30%); freed ~65% of an A53 core
  (headroom for future EtherCAT/RT robot control).
```

### Perception status

```text
DONE: RealSense -> Vitis-AI detect -> /detections -> pick_logic -> /pick_target
      -> single-point 3D (reverse projection) -> /pick_target_base.
      ~17 Hz, z accuracy verified, detector accuracy unchanged through all
      optimizations (LUT + post-filter were bit-identical; alignment removal
      validated geometrically + on hardware).
STILL PLACEHOLDER: base_link->camera_link static TF (needs measured calibration
      before /pick_target_base is a real robot coordinate).
STILL STANDIN: ssd_adas (car/bicycle/person) is not the final pick object model.
```

### Next options (not yet started)

```text
1. Camera-to-base calibration: replace the placeholder base_link->camera_link
   TF with measured/calibrated values so /pick_target_base is a real robot frame.
2. Final YCB/custom detector for B3136 (swap model + classes; the worker JSON
   contract is model-agnostic, so node/pipeline/3D/downstream are unchanged).
3. Robot control layer: APU -> RPU (FreeRTOS) -> Indy7 (Ethernet/EtherCAT).
   Plan CPU core isolation so RT control does not starve the vision pipeline.
```

## YOLOv3-tiny 7-class Model Swap: train → quantize → compile → board deploy → Gate 5 — 2026-07-07

Replaced the SSD ADAS stand-in with a custom **YOLOv3-tiny 7-class** detector
(`0 apple, 1 peach, 2 orange, 3 banana, 4 tennis_ball, 5 mustard_bottle, 6 person`).
Full command/gate reference: `yolov3_tiny_execution_plan.md`; decision logic:
`yolo_v3_process.md`. Model is done and on the board; live real-object validation
(Gate 5) is partially failing and the work is paused (see end of section).

### 1. Model & framework choice (why)

```text
- Structure = YOLOv3-tiny: uses only DPU-supported ops (conv/BN/LeakyReLU,
  maxpool, nearest upsample, concat per UG1414 v2.5 Table 20) -> whole net on
  DPU, single subgraph feasible.
- Trained via ultralytics yolov5 v7.0 repo (models/hub/yolov3-tiny.yaml, classic
  anchor head) because vai_q_pytorch requires a PyTorch nn.Module. NOT darknet
  (.weights needs conversion + lacks mosaic), NOT v8 "tinyu" (anchor-free head
  would break the board decode).
- Decode = yolov5 style: xy=(2*sig-0.5+grid)*stride, wh=(2*sig)^2*anchor,
  conf=sig(obj)*sig(cls). Board worker + decode_meta.json written to this and
  unit-tested bit-identical to the image-test path.
```

### 2. Dataset (no real training images — decision D2)

Synthetic (BlenderProc + YCB meshes 006/011/013/015/017/056, camera sampled
around the D10 top-down 0.8 m / gray optical-table geometry) + COCO
(apple/orange/banana/person) + Open Images (peach, tennis_ball) + BOP ycbv
(mustard, banana). Real captures allowed for validation only.

```text
Dataset: train 15,799 / val 2,093  (Gate 1 passed, contact-sheet reviewed)
```

### 3. Training + the SiLU->Hardswish pivot (Gate 2) [decision D11]

First training used yolov5's default Conv activation = **SiLU**. SiLU is **not a
DPU-supported op**; the quantizer left it as a float op (`VAIQ_WARN ... aten::silu_`)
and it would fragment the compile into per-conv CPU subgraphs. Fixed by switching
activation to **Hardswish** (closest DPU-supported op to SiLU; quantizer maps
PyTorch Hardswish -> XIR hardswish, compiler supports it) and **retraining**
(activation is baked into the trained weights; there is no exact SiLU->Hardswish
weight transform). Added a pre-train **Inspector gate (Gate 0, `12a_inspect_docker.sh`)**
so DPU-mapping is verified before spending 2 h on training.

```text
activation change cost ~ 0 (Hardswish curve ~ SiLU):
                     SiLU(1st)   Hardswish(2nd, final)
  all  mAP@0.5        0.758        0.766
  banana                .694        .738  (P 0.72->0.80)
Final per-class mAP@0.5 (run pickplace_v3tiny_hswish, 150 ep, ~2 h RTX 4060):
  apple .625 | peach .955 | orange .734 | banana .738 |
  tennis_ball .978 | mustard_bottle .823 | person .509
```

Confusion diagnosis: only real class-confusion was mustard->banana (0.39). Split
val by source: synthetic-val mAP 0.993 (mustard diag 0.98, banana cell 0.01),
ycbv-val banana P 0.434 -> the confusion is confined to ycbv occlusion/clutter,
not our domain. Gate 2 passed.

### 4. Quantize + two VAI 2.5 packaging bugs (Gate 3)

Two crashes came from Vitis-AI 2.5's own `pytorch_nndct/nn/modules/hardswish.py`
(not our code), confirmed against the v2.5/v3.0 GitHub source:
```text
(a) __init__ inits undefined symbol FixNeuronWithBackward (dead line; v3.0 deletes it)
(b) forward calls fake_quantize_per_tensor() missing required args method/inplace
Fix: 12/12a docker wrappers sed-patch both at container start (--rm image).
```

```text
Gate 3 (hswish weights, patched): calib 500 imgs OK, VAIQ_WARN 0,
  cosine head[0] 0.9923 / head[1] 0.9849  (incl. DPU hardswish fixed-point approx)
  -> DeployModel_int.xmodel + decode_meta.json exported
Lesson recorded: read VAIQ_WARN / unknown-op BEFORE trusting the cosine number.
```

### 5. Compile + board structure check (Gate 4)

```text
vai_c_xir: "Total device subgraph number 4, DPU subgraph number 1"  (PASS)
  ("4" = input-feed + 1 DPU compute block + 2 output tensors; all conv compute
   is in the single DPU subgraph, matching Inspector's "all ops on DPU".)
xmodel -> board ~/vitis_ai_work/models/  (md5 e2ca87c2466f715e9ecc00c43b599cc4)
xdputil verify: fingerprint 0x101000016010406 (B3136),
  input [1,416,416,3] fixpos 6, outputs [1,26,26,36] & [1,13,13,36] fixpos 2
  (416/16=26, 416/32=13, ch 36 = 3 anchors x (5+7)) -> matches decode_meta.json
```

### 6. Board deploy: config switch + a latent config bug

Config YAMLs (install->build->src all symlinks, live, no rebuild):
```text
vitis_ai_detector.yaml: model_path -> yolov3_tiny_7class.xmodel,
  worker_script_path -> vitis_ai_worker_yolo.py, send_resized_input -> false
  (worker does letterbox internally). SSD revert values kept in a top comment.
pick_logic.yaml: allowed_classes -> 6 pickable (person detected but not pickable = safety).
FIXED latent bug: metrics_duration_sec 0 -> 0.0 (node declares it double; an
  integer raises InvalidParameterType and kills the detector on any launch).
```

Live pipeline came up clean:
```text
detector worker mode, 848x480: dpu_ms ~17-23, processing ~50-70 ms
  (faster than the SSD stand-in). /detections publishing, Gate 6 plumbing OK.
Smoke on 12 old frames: person generalizes (0.32-0.50, all caught at thr 0.30),
  ZERO false positives on office clutter.
```

### 7. Gate 5: real-object validation — sim-to-real gap (partial FAIL, paused)

apple/orange/banana detect well on real objects (0.76-0.90, no FP), but 3 of 6
classes fail. This is a class-specific domain gap, **not** a threshold or
preprocessing issue (same frame: 3 classes perfect, 3 at ~0). Rearrange test
separated the causes:

```text
peach        -> misclassified as apple (peach score ~0.02, apple confident 0.54)
                = appearance confusion (plastic peach looks apple-like). Hardest.
tennis_ball  -> 0.004 clustered -> 0.14-0.26 separated (still < 0.5). Marginal.
mustard      -> lying-down 0.001 -> UPRIGHT 0.66-0.80. = pose sensitivity; the
                shape IS learned, synthetic just under-covered lying/top-down poses.
Pattern: COCO-rich classes (apple/orange/banana) transferred; synthetic-dependent
  classes (peach/tennis/mustard) did not. Threshold tuning cannot help (scores ~0).
CONFOUND: the test camera was low/oblique, NOT the real top-down D10 geometry.
```

### 8. Status at end of session (PAUSED 2026-07-07)

```text
DONE (no rework): train (Gate2 0.766) / quantize (Gate3) / compile (Gate4);
  xmodel on board; board inference healthy; apple/orange/banana real-detect OK.
CONFIG NOW = YOLO (not yet fully validated). Revert to SSD via the comment in
  vitis_ai_detector.yaml if the stand-in is needed during other board work.
OPEN (Gate 5): peach/tennis_ball/mustard real-object detection.
Agreed resume order (synthetic re-render first, D2 preserved):
  1. re-validate in the REAL top-down geometry (this test wasn't) to separate
     geometry artifact from true gap;
  2. re-render synthetic targeting the observed failures (lying bottle top-down,
     tennis lighting/clutter variety, peach texture vs apple hard-negatives);
  3. retrain -> requantize -> recompile -> re-verify on board;
  4. only if peach still fails, revisit D2 (small real training set) — user's call.
New session onboarding entry point: inst_claude.md.
Board cleaned up (DPU/camera released) so the parallel RT-kernel work is unaffected.
```

### 9. Gate 5 re-validation PASS (D13 6-class) + apple retrain (D14) — 2026-07-09

The D13 6-class model (peach dropped, YCB real scans added) was retrained,
quantized, compiled, and deployed to the board; then re-validated on real
top-down objects.

```text
Gate2 mAP50 0.748 (mustard .933 / tennis .950 / orange·banana .736 /
  apple .629 (weakest) / person .503).
Gate3 cosine .9757/.9615 (< prior .99 but > the 0.95 fast-finetune line -> pass).
Gate4 DPU subgraph 1 (md5 d925c711..., outputs 33ch=(6+5)*3). Old 7-class -> models/*.OLD7.*
Real top-down: mustard 0.02 -> 0.814 (the payoff), tennis 0.491 -> 0.677,
  orange 0.850, banana 0.777. => 5/6 classes solid.
apple only: 0.489 / 0.549 / 0.549 over 3 frames = straddles the 0.50 cut.
```

Two findings on apple:
- The dropped **peach** (out-of-distribution) was detected AS apple at 0.462 and
  suppressed neighbors; removing it from the table raised the real apple
  0.216 -> 0.549 and lifted the others too. Deployment has no peach, so this
  false positive is moot in production.
- The real apple is genuinely marginal — a **color domain gap** (real apple is
  lighter than the YCB apple) plus apple being the hardest class (round, confuses
  orange/tennis). NOT a quantity problem (already ~6371 instances).

**D14 (user decision):** stabilize apple with REAL captures from the deployment
camera — apple alone on the table so single-object auto-label is trivial. Chosen
over "just more data" because apple's issue is the color gap, not count.
Done: added `--manual` (Enter=1 frame) capture mode; new `autolabel_single_object.py`
(DPU top-1 box, forced class-id); ~54 apple frames captured+labeled (1 misdetect
dropped) -> `datasets/real_apple_yolo` (train-only); `hyp hsv_v` 0.40->0.50 (hsv_h
kept to avoid apple<->orange confusion). Retraining running on the desktop.

Next on resume: judge new apple mAP -> quantize/compile/deploy -> board Gate5
(apple stable at 0.7+?) -> **revert the `apple:0.40` threshold hack to 0.50** (it
was added under the wrong premise that the 0.462 box was the real apple — it was
the peach) -> Gate 6 pipeline / Gate 7 live.

### 10. Desktop GPU driver hard-freeze during D14 retrain — 2026-07-09 evening

The D14 retrain (`11_train.sh`) hard-froze the whole desktop TWICE, both times
before finishing epoch 0 (GPU ~50°C, so not thermal). Post-reboot forensics
(`journalctl -b -1`, kern.log) showed NO OOM, NO NVIDIA Xid, NO MCE — a silent
hard hang. (The "XID 641" lines were the RealTek NIC chip id, a red herring;
NVRM was just the boot load banner.)

Root cause: the NVIDIA driver had been switched to **nvidia-driver-595-OPEN**
(open kernel module, 595.71.05). apt history showed `apt install
nvidia-driver-595-open` plus a background upgrade 595.58.03 -> 595.71.05 that only
takes effect after a reboot (a loaded .ko is not swapped until reboot). The open
module forces GSP firmware, and open+GSP on this GeForce card hard-hangs under
CUDA load with no logged Xid. This explains "trained fine this afternoon (old
module still in memory), froze this evening (595.71.05 loaded after the reboot)."

Fix: rolled back to the proprietary driver —
  sudo apt purge nvidia-driver-595-open nvidia-dkms-595-open nvidia-kernel-source-595-open
  sudo apt install nvidia-driver-580 && sudo apt autoremove && sudo reboot
Verified via `/proc/driver/nvidia/version` (no "Open Kernel Module"). Recommended
`apt-mark hold` on all installed nvidia packages (optionally an
unattended-upgrades blacklist for "nvidia-"/"libnvidia-") to stop silent
auto-updates.

Status: driver fixed; D14 retrain then completed cleanly on 580 (see §11 below).
If it hard-freezes again on the proprietary driver, escalate to hardware
(memtest86, PSU, PCIe reseat).

### 11. D14 complete — Gate 5 PASS, apple fixed (model swap done) — 2026-07-10

After the driver rollback, `11_train.sh` completed cleanly (hswish5, 2.528h, no
freeze — confirming the 595-open driver was the cause). Quantize/compile passed
(md5 9bc6520c, DPU subgraph 1).

Board Gate 5 on the same real top-down frames used for D13:
```text
apple   0.489/0.549/0.549 (D13) -> 0.876/0.899/0.875 (D14)   <-- fixed
orange 0.85-0.88, banana 0.83-0.85, mustard 0.81-0.87, tennis 0.82-0.85
=> all 6 classes solid on real objects. Model swap (Gates 2-5) complete.
```

Note: val Gate 2 apple stayed ~0.625 (D13 0.629) because `real_apple_yolo` is
TRAIN-ONLY, so the val set cannot see the real-apple gain — the board Gate 5 is
the decisive test, and it jumped 0.5 -> 0.88.

Cleanup: reverted the `apple:0.40` threshold hack to 0.50 in both the worker
(`vitis_ai_worker_yolo.py`) and `yolov3_tiny_image_test.py` (apple now 0.88, and
the 0.40 was based on the wrong premise that the 0.462 box was the real apple — it
was the peach). A weak orange (0.19) fires on the yellow mustard bottle but sits
far below the 0.50 deployment threshold (harmless).

Deployed xmodel = D14 (md5 9bc6520c); it overwrote D13 on the board without a
backup — D13 is regenerable from the desktop hswish2 best.pt via 12->13 if needed.
Old 7-class remains at models/*.OLD7.*.

Remaining: Gate 6 (full pipeline) / Gate 7 (live) — mind the RT-kernel isolcpus
2-core load.

### 12. Gate 6 full-pipeline performance measurement — 2026-07-10

Measured the full pipeline live for 3 min on 4 cores (isolcpus currently OFF —
/proc/cmdline has no isolcpus). Harness under ~/vitis_ai_work/perf/: perf_probe.py
(topic Hz + capture->detection E2E latency via /detections header.stamp + per-core
& per-node CPU from /proc) + the detector node's built-in per-frame metrics CSV
(dpu_ms/pre/post). Orchestrator run_gate6_perf.sh enables the node metrics CSV via
the symlink-live YAML, launches, runs a 180 s probe, stops, restores the YAML,
summarizes. Outputs: vision_metrics.csv, pipeline_timeseries.csv, cpu_timeseries.csv,
gate6_summary.csv.

```text
Vision (per-frame, n=2784): dpu_ms 18.0 (med 17.6, p95 20.5)  pre 17.9  post 6.4
  worker_ms 42.3  detect_ms 57.5  => DPU not the bottleneck; CPU preprocess caps it.
Pipeline (1s, n=180): camera 30.0 Hz | det/pick/3d/base all 15.0 Hz (chain 1:1)
  | capture->detection E2E 137 ms (p95 149) | ~5 objects/frame.
  Detection is compute-bound ~15 Hz (YOLO worker heavier than old SSD; the earlier
  "camera-limited ~17Hz" note no longer holds — camera supplies 30fps).
CPU (1s, n=180), 4 cores: total 79% of 4 (~3.17 cores). Per-node (%/one core):
  target_3d 68.8 (TOP) | camera 56.9 | detector 49.9 | worker_dpu 36.2 |
  target_base 13.2 | pick_logic 8.8.
```

Gate 6 = PASS (end-to-end works, all stages synced at 15 Hz, E2E 137 ms); Gate 7
perf CSV captured (live checklist still TODO).

Key findings: (1) pick_target_3d_node is the #1 CPU consumer (69% of a core) — its
depth_callback cv_bridge-converts the 848x480x16 depth image at 30 Hz, but the 3D
compute (pick_target_callback) only runs ~15 Hz. This confirms the integrated_progress
prediction that target_3d is the prerequisite to optimize for the future 3+1 EtherCAT
isolation (vision must fit in 3 cores; it currently needs ~3.17). Fix levers: lower
realsense depth 30->15 fps (config, biggest), defer cv_bridge to pick_target_callback,
scalarize the reverse-projection loop. (2) The YOLO worker ALREADY carries the SSD
pre/post optimizations — build_input_lut/letterbox_lut (LUT preprocess) and
decode_head int8 objectness pre-filter — so pre_ms/post_ms are already
post-optimization; the remaining CPU lever is the 3D node, not pre/post.

### 13. Camera FW fix + vision CPU optimization phase 1+2 + strategic pivot to RT — 2026-07-14

Three things happened this session, on the **stock kernel** (5.15.0-1070; RT kernel is
blocked for vision by the zocl crash below). Full tables live in
`integrated_progress.md §4.2`; this is the chronological summary.

**(a) D435i camera FW wedge — fixed.** After reconnecting the RealSense, RGB frames
froze seconds after librealsense start (dmesg `GET_CUR ... -32 (exp. 1024)`, HWM XU
stall). Survived hardware_reset / physical replug / port change / reboot; V4L2-direct
capture was unaffected, which layer-split the fault to the librealsense/FW path, not the
kernel UVC driver. **Fixed by flashing FW 5.16.0.1 → 5.17.0.10** (`rs-fw-update`, official
`D4XX_FW_Image-5.17.0.10.bin`). Config gained `initial_reset: true` as a wedge defense.
Diagnostic method in memory `d435i-fw-rgb-wedge-fix`.

**(b) zocl RT-kernel crash — discovered, handed off.** First attempt to run the DPU
pipeline on the RT kernel (kv260b) crashed the board ~30 s in: SLUB freelist corruption
in the zocl KDS path (`___slab_alloc` ← `kds_alloc_command[zocl]` ← `zocl_execbuf_ioctl`),
register-fingerprint-confirmed, kernel Oops → freeze → hard reboot. Not memory/HW/radix —
it's a memory-safety bug in the zocl **vendor** driver, separate from the radix-tree
crash we fixed on 07-13. **This is why vision optimization runs on the stock kernel.**
Handed to the RT-dedicated session; details in `rt_kernel_postmortem.md §12`, resume
point in memory `zocl-dpu-rt-kernel-crash`.

**(c) Vision CPU optimization phase 1+2 — the three levers predicted in §12 were
executed, plus more.** Baseline re-measured on stock kernel + new FW: **total 76.8%**
of 4 cores (target_3d 68.9 TOP). Result after both phases: **total ~44%**
(≈3.07 → 1.8 cores, **-1.25 cores**), e2e 124 → 81 ms, det held at 15 Hz.

- Phase 1 (76.8 → 53.1%): epipolar reverse-projection **vectorized** (the "scalarize the
  loop" lever — replaced the per-candidate Python loop with a single (3,N) matmul;
  original kept as `_loop` reference, A/B-verified over 4500 live picks, mismatch 0);
  static camera_info/extrinsics subscriptions **pruned** after first receipt; letterbox
  on the cv2 SIMD path (bit-identical); realsense **depth 30 → 15 fps** (the "biggest"
  lever from §12); detection cap `process_period_sec: 0.045` (trap: 0.062 drops to a
  3-frame cadence / 13.8 Hz — must sit inside the 33.3–66.6 ms window).
- Phase 2 (53.1 → ~44%): the "defer cv_bridge" lever landed as **lazy depth** — the
  callback stores the msg, and pick time takes a zero-copy `np.frombuffer` view
  (bit-identical, ~26× vs cv_bridge); base node **static-TF cache** (cache (R,t) once,
  one matmul per point; error 0.000e+00 vs tf2); worker decode constant hoisting. The
  **largest single lever was transport**: `.bashrc` was silently forcing CycloneDDS, so
  1.22 MB images were doing UDP-loopback copies within one board — switching to
  **FastDDS + shared-memory** (`~/ros2_ws/fastdds_shm_profile.xml`, 16 MB segment) took
  camera 40 → 29 and detector 39 → 31, -6.6 %p in one move.
- **Measured-then-rejected**: merging pick_logic+t3d+base into one rclpy process
  (`pick_post_stack`) cost **+5.4 pt**, not the predicted saving — an rclpy
  SingleThreadedExecutor rebuilds the whole wait-set per callback, so more entities in
  one process raises every callback's dispatch cost. Rolled back; the code is kept with
  the rejection documented. Lesson: don't merge nodes to save CPU in rclpy — only rclcpp
  composition wins here.
- Also freed the DP display interrupt storm (11k irq/s → 0) by unbinding `zynqmp-display`;
  persistence method now confirmed = blacklist the `zynqmp_dpsub` module + `update-initramfs`.

**(d) Strategic decision — RT kernel first, vision performance later.** With vision down
to ~1.8 cores, there are **~2.2 cores free**; IgH EtherCAT (~0.1–0.3) + control (~0.1–0.4)
fit with 3×+ margin, so **CPU need not be reduced further** for integration. The next
priority is therefore **finishing the RT kernel** (production rev-6 + resolving the zocl
crash), not chasing fps/latency — the zocl crash is the real prerequisite for RT+DPU
integration, and performance work is better re-baselined on top of RT (expect +5–10 %p
overhead there). A phase-3 performance lever catalog (POSIX-shm IPC as the one CPU/latency
win-win, worker 3-stage pipelining, cap tuning, 60 fps color, the YOLOv3-tiny swap, rclcpp
composition) is parked in `integrated_progress.md §4.2` for after RT is done.

## Real-time tracking ('o'): the app widens from approach-only to approach + track — 2026-08-03

Until this day the robot application did one thing: the operator picked an
object from the `p` menu, `v` approached it, and the arm stopped ~15 cm above
it. 08-03 added the second half of the application — **the TCP follows the
object while a person moves it by hand** — and the day's work was rewritten
three times, each rewrite forced by a live run. Commits below are in
`~/RAON-RT-Revision` (branch `kv260-merge`); the decision/bug ledger account is
`docs/RAON-RT/merge.md` (D15, E36, E37).

**v1 (`9204372` then `4efb05d`) — backstop first, servo second.** `9204372`
lands what must exist before any new motion source: a NaN gate, torque
saturation against the URDF effort limits (`{431.97, 431.97, 197.23,
79.79×3}`), and a CTC snapshot. `4efb05d` adds the tracker: a seqlock sample
pipe from the ROS bridge into the 1 kHz RT loop (`PeekTrack`), a 0.10 m jump
reject, `eTrackingServo` (rewiring the dormant `SetTargetPose_Jacobian`), a key
whitelist while tracking, TCP freeze on loss, and a 2 s OFF latch. New cfg
keys: `TRACK_VMAX_MPS 0.20`, `TRACK_EMA_ALPHA 0.4`, `TRACK_LOST_MS 500`.

**v2 (`2993804`, same day, decided on the robot) — the gate chain was the
bug.** Live verdict: "more often not following than following". The v1 gates
formed a self-defeating chain — one vision glitch trips the jump reject, the
now-stale reference makes the *next*, legitimate sample look like another jump,
the track goes LOST, re-acquisition demands proximity, and the 2 s latch turns
the whole thing OFF. Deleted: jump gate, 'v'-first requirement, proximity gate,
OFF latch. What remains is deliberately **one real safety device — the TCP
linear speed cap** — with the workspace box, TCP-freeze-on-loss, the entry
gates, the key whitelist and the torque backstop arranged around it. 'o' now
follows the selected object from wherever the arm is, re-acquires
unconditionally after an occlusion, and accepts `p`+digit retargeting mid-track.

**E36 (`8656327`) — +y was followed, −y was not.** The soft joint limit was
evaluated on the raw counter frame; J1's counter reads ≈ −5.8 rad where the arm
is physically near 0, so one direction hit a limit that does not exist (the
same counter-vs-physical split as E20). Fixed with fold-toward-zero before the
comparison. Standing rule: **any new code that compares `m_Q` against a limit
must fold first.**

**FF re-enable (`3ceeb9f`) and E37 (`e5edfa1`) — the one that ended in an
E-stop.** In the last few cm J3/J4 froze and the TCP stopped short of the
object centre: the tracking servo had inherited `KfScale = 0` plus the
approach-era **position-space** lag gate, but the servo's reference is
non-accumulating, so the lag it measures is ~1e-4 and the friction feed-forward
never wakes. The first fix set `KfScale = 1` on entry **and bypassed the lag
gate** — and that bypass was the accident. Un-gated FF is bang-bang (visible
squirm) and, on a descent, runaway: the speed cap limits the *commanded*
velocity only, and once breakaway occurs kinetic friction is far below static,
so Kd alone could not hold ~6 Nm — the arm dove at the table and the operator
hit E-stop. Three corrections: (1) move the FF gate into **velocity space**
(`(qd_ref − qd)·sign / 0.01` — full assist at standstill, self-balancing on
arrival, zero while overrunning); (2) a 10 mm rest deadband with continuous
shaping; (3) a goal-z floor `TRACK_GOAL_ZMIN_M = 0.25`, since the workspace
box's `z_min` is the table surface itself. Lesson recorded: the lag gate's
principle — assist only while lagging — is load-bearing. Never bypass it; only
move it into the measurement space that matches the mode.

**Post-E37 cleanup (`40aaa43`).** J3 idling, creep after arrival and general
sluggishness traced to an active z-alignment restoration term (rotation and
position see-sawing, and FF waking inside the deadband). Deleted — commanding
zero angular velocity and letting the DLS solve hold the posture softly is
enough — and `Kp_task_pos` 1 → 2 as the time-constant knob.

**v3 hybrid (`8d64868`, `25bc044`, `38a787a`) — the servo is a local law.**
Large moves under the servo alone showed everything a local linearization
shows: the branch is never chosen, the wrist wanders, J4 chatters, the dive
retries. So large moves stop being the servo's problem — beyond
`TRACK_SERVO_RANGE_M = 0.15` the tracker auto-fires the **already-verified
approach stack** (`StartTrackLeg`: deterministic ready-seed IK + quintic, a
lean twin of the direct approach leg, without staging/refine/logging), the
servo runs only inside 0.15 m, and an object escaping past
`TRACK_LEG_TRIGGER_M = 0.25` brakes (STAGE) and re-legs. Chatter fixes:
velocity gate ramp 0.01 → 0.04, saturation ramp onto the same 0.04 band
(`25bc044`, J4 residual tremor; approach/refine paths unchanged), then
`38a787a` for the residual axis-3 tremor — a 50 ms LPF on the FF command
velocity, because jitter is a spectrum problem and widening a ramp only
attenuates it. `38a787a` also adds the null-space guard: near a singularity
axes 2/3 can wind a full turn while the TCP barely moves, so the far-check
never fires and the arm folds onto the table; a 1.5 rad joint-drift budget
(`TRACK_SERVO_DQ_MAX_RAD`) now brakes and re-plans the leg.

**Also 08-03, cfg only: `KF_2 = 4.0`.** In tracking the wrist worked while the
elbow sat still — axis-2 stiction was never compensated (the "j2 lag" the
approach era had already flagged as the next lever). Reverting to `KF_2 = 0.0`
restores the exact configuration the 07-31 one-shot approach runs (#52–58) were
verified with.

**Verification status.** The approach half ('v') is live-verified: 07-31
one-shot 7/7, and `App/Indy7/approach_results/approach_log.csv` still records
4.21–5.93 mm finals with 0 refine iterations as late as 08-05. **v3 hybrid
tracking is implemented but NOT live-verified** — the 8-step ladder (static
hover → loss → low speed → *speed-cap proof* → boundary → fault drill →
hand-held) is still open in `merge.md`, and the standing rule until the cap is
proven is: do not lift the object by hand. Tracking also produces no
quantitative log — `EmitApproachReport` fires only on the approach-complete
edge, so there is no tracking counterpart to `approach_results/` or
`rt_log_results/`.

**What the application is now — and is not.** vision → `p` menu → digit → `v`
approach to object-top + 15 cm → `o` follow that object while a person moves
it. **There is no grasp, transport or release**: no gripper hardware, and no
gripper code in the app (`grep -i "gripper\|grasp"` over `App/Indy7` returns
only unrelated English words). The `pick_*` spellings throughout the perception
side — the launch file, `pick_logic_pkg`, `/pick_target`, `/pick_target_base`,
`PickTarget3D` — are **legacy names from the initial pick&place framing** and
are deliberately left unrenamed.

## Moving-object pivot: depth/color time alignment + throughput 15.3 → 21.7 Hz — 2026-08-04/05

Premise change: the pipeline's target moved from static pick scenes to MOVING
objects (real-time tracking), which re-opened two decisions that were correct
for static scenes.

**Depth/color time alignment (08-04).** With free-running streams (color 30 /
depth 15, `enable_sync: false`) the depth frame used for a pick was captured
p50 **+52.5 ms** away from the color frame the bbox came from — harmless when
nothing moves, but on a mover the bbox pixel can leave the object and the node
silently reads the background (all fail-safes pass; `depth_valid` stays true).
Fixed as two halves, both required, measured in one run via the new
`skew_if_latest_ms` control column:
- `enable_sync: true` + depth 15→30 fps: every color frame gets a
  same-timestamp depth partner (the ROS wrapper stamps a whole frameset with
  ONE time `t` — which also re-attributes the 07-31 align A/B: align's
  "skew 0" was the bundled sync's doing, align itself is spatial only).
- `pick_target_3d_node`: 800 ms depth history (age-pruned deque) + nearest-
  by-capture-stamp selection replaces "newest depth" — the detection arrives
  ~90 ms late, so the newest frame is ~3 frames PAST the partner (sync alone
  measured p50 33.3 ms). Result: skew |mean| 52.5 → **4.3 ms**, 89% exactly
  0.0, cost +0.18 cores / E2E +1.8 ms / throughput 0
  (`evidence/metrics/runs/sync30_compare_20260804.md`). Also fixed: `_emit`
  re-read `latest_depth_msg` at emit time, so the skew column measured a frame
  the pick never used. Viewer + operator GUI now draw every color frame
  (~30 fps) with the newest boxes (0.5 s age gate); the exact stamp-join lives
  on behind `--sync` for bbox↔frame verification.

**Throughput (08-05, `docs/vision/throughput.md` is the canonical account).**
The binding constraint was neither the camera (supplies 29.4 Hz) nor the DPU
(73% idle) but `process_period_sec: 0.045` — a DELIBERATE 15 Hz cap from the
07-14 CPU phase (its comment even documents the 33.3..66.6 ms window and the
0.062 jitter trap), premised on static objects. Re-decided to 0.030. Measured
same-scene back-to-back: the gate alone is a rate↔freshness trade, NOT free
(+2.6 Hz but E2E p50 +14 ms — an ungated worker grabs the frame that SAT in
latest_msg up to 33 ms instead of discarding and taking a fresh arrival; the
first attempt at this A/B was invalidated by a scene change, 5→1 objects,
which shifted processing across the gate and mimicked a config effect).
The trade was then dissolved by landing the IPC lever parked in the 07-14
phase-3 catalog: frames now cross to the worker via a **/dev/shm mmap**
(`shm_frame.py`, one memcpy, read-only view, growth keeps the inode; fallback
`worker_shm: false`) instead of 3-4 copies through the stdin pipe. Same scene:
ipc 13.4 → **4.65 ms**, processing 53.0 → **44.9 ms**, **15.3 → 21.7 Hz**,
E2E p50/p95 −10/−11 ms, freshness p50/p95 −14/−16 ms, **CPU unchanged 1.60**,
zero worker restarts/timeouts. 0.030+shm now dominates the old 0.045 on every
metric (`evidence/metrics/runs/gate_ipc_20260805.md`). Next lever, only if
tracking proves 21.7 Hz insufficient: in-worker pipelining (pre ‖ DPU) to push
processing under 33.3 ms, where the worker always waits for a fresh arrival
and the freshness/rate trade disappears structurally.

## Operator console: remote key/jog/gain channels + stops that actually stop — 2026-08-06

`cfdaa03` (ros2_ws, 08-05) adds `raon_operator_gui`, a **C++ Qt** operator
console with an embedded RViz, replacing the PyQt5 skeleton (`378f059`) — the
rewrite was forced, not chosen: embedding RViz is C++ only (ament_python →
ament_cmake, Qt 5.15.3 Widgets, OpenCV 4.5.4, Indy7 STLs vendored). It runs on
the **desktop PC**, never on the board, for the same CPU-budget reason that
removed board-side overlay drawing.

`4b2113a` (RAON-RT-Revision, edited 08-05, committed 08-06) is the robot half:
three inbound channels — `/operator_key` (`std_msgs/String`, one character,
RELIABLE depth 1), `/operator_gains` (JSON kp/kd/kf), `/operator_jog` (JSON
axis/q) — and two outbound: `/robot_state` (20 Hz JSON: joints, mode, gain
readback, `menu_items`, and the very booleans that `DoInput`'s reject guards
read) and `/joint_states`, plus `/operator_msg` mirroring the operator printf.

The design rule is that the console's gating is a **mirror** of the robot's
`DoInput` guards and must never run ahead of it — the last line of defence is
always the robot. Two consequences: the `p` menu is published by the robot as
`menu_items` and the console sends *array position + 1* as the digit (a console
that inferred numbers from `/detections` would send the arm to a different
object), and `q`/`z` get no buttons at all (`q` starts a 10-minute ISO cube
run). The HOME-record button is permanently disabled, because re-recording HOME
would pollute the ready-seed anchor.

The same commit fixed two stop keys that did not actually stop:

```text
e  E-STOP     was: the CiA402 cyclic state machine re-armed QUICK_STOP by itself
              now: KillMotionSources() + servo-off (brake) on every axis, latched
x  soft STOP  was: a profile-mode halt bit — meaningless in CST
              now: KillMotionSources(), servo stays on, holds in gravity comp
```

`KillMotionSources()` is the single root kill for tracking, approach, ISO,
RECT, refine and jog.

**Not live-verified.** The commit message ends "Built and linked. Not yet
exercised on the robot", and the GUI README marks RViz render and
robot-connected operation as unverified (harness/offscreen PASS only). A binary
containing the console code did run on 08-05 (`bin/Indy7Ctrl.out` mtime 21:55,
an approach logged at 22:20), but whether the console channels were exercised
in that session is **unconfirmed**.

## RPU track: FreeRTOS boots on R5F-0 and runs its LED tasks (Gate 1) — 2026-09-01/02

The first real milestone on the RPU side: **a FreeRTOS ELF boots on Cortex-R5F
core 0 and runs its LED tasks** — the boot is confirmed by the kernel log
(2026-09-02, both boots; 09-01 reached DT enumeration only), while the physical
blink is a **user observation** (see below). Kernel log, 2026-09-02:

```text
15:01:46 remoteproc remoteproc0: powering up ff9a0000.rf5ss:r5f_0
15:01:46 remoteproc remoteproc0: Booting fw image r5_freertos.elf, size 505088
15:01:46 remoteproc remoteproc0: remote processor ff9a0000.rf5ss:r5f_0 is now up
(an earlier boot the same day, 11:02:31, also reached "is now up", size 486340)
```

**The pass criterion was LED, not UART.** `nm r5_freertos.elf` shows
`led35_task` / `led36_task`, and the session's shell history sets the Linux
heartbeat trigger to `none` first so Linux releases the LED. The physical blink
is a **user observation** — it cannot be re-verified mechanically from the
board. Every RPU plan document had named UART as the Gate 1 evidence; in
practice UART routing was never exercised.

**Memory layout is 0x36000000, not the 0x3ed00000 the plans called
"standard".** Live device tree: `reserved-memory/rproc@36000000`, length
`0x400000` (4 MiB), `no-map`. The ELF links `.text` at 0x36000000 (FileSiz
0x5b3c / MemSiz 0x13600 ≈ 78 KB used) plus a second LOAD at 0x00000000
(`.vectors` + `.bootdata`, ATCM). The documented 0x3ed00000 sits **inside this
board's CMA window** (`cma: Reserved 1000 MiB at 0x36400000`) — the carve-out
that was actually used tucks in immediately below CMA instead.

**The overlay went in at boot time, not through configfs.** U-Boot's
`boot.scr.uimg` already supports a `user-override.dtb` hook (verified with
`strings`), so no `flash-kernel` / FIT rebuild was needed — contrary to the
plan documents' "path 2". The reproducible sequence (shell history + kernel log,
artifacts live in `~/rpu_work/`, outside git):

```bash
# 1) base DTB with __symbols__ (so fdtoverlay works)
cp /usr/lib/linux-image-$(uname -r)/xilinx/smk-k26-revA-sck-kv-g-revB.dtb base.dtb
# 2) compile overlay, merge OFFLINE (always from base.dtb — merges do not stack)
dtc -@ -O dtb -o rpu_v2.dtbo rpu_v2.dtso
fdtoverlay -i base.dtb -o user-override.dtb rpu_v2.dtbo
# 3) safety gate + deploy + reboot once
systemctl is-active ethercat            # must be inactive: not during a live session
sudo cp user-override.dtb /boot/firmware/user-override.dtb && sync && sudo reboot
# 4) after reboot: no overlay, no modprobe — the node appears by itself
cat /sys/class/remoteproc/remoteproc0/name        # ff9a0000.rf5ss:r5f_0
# 5) load + start (the firmware NAME resets to default on every boot)
sudo cp ~/rpu_work/r5_freertos.elf /lib/firmware/
echo r5_freertos.elf | sudo tee /sys/class/remoteproc/remoteproc0/firmware
echo start          | sudo tee /sys/class/remoteproc/remoteproc0/state
```

A v1 configfs overlay (`rpu_v1.dtso`) had already enumerated the core on 09-01
and even booted firmware on 09-02 11:02, so v1 was not abandoned for failing —
it was superseded because it carried **no `reserved-memory`**, i.e. the
firmware was running on DDR the kernel had never been told to keep. The
principle was already written down correctly in the guide: reserved-memory can
only be reserved at the memblock stage, so adding it at runtime is not the same
thing.

**Also settled: the build environment exists.** `strings` on the ELF shows
Vitis **2022.1 on Windows** (`d:\xilinx\vitis\2022.1`, GCC 11.2.0
arm-none-eabi), XSA `design_1_wrapper`, domain `psu_cortexr5_0`, BSP
`freertos10_xilinx_v1_11` with `gpiops_v3_9`, `scugic_v4_7`, `ttcps_v3_15`
(the µs clock material the later phases need), `standalone_v7_7`. The plan
documents' "Vitis not installed — install it in Phase 0" is closed.

**What the current boot configuration cannot do yet.** The firmware has no
`.resource_table` (`nm | grep -ci 'rpmsg\|openamp\|resource_table'` = 0), and
the kernel accordingly logs `no resource table found.` — a **warning only**, the
core still comes up, but there is no rpmsg and no trace buffer. The device tree
likewise has no `mboxes`/`mbox-names` on `r5f_0`, no vring/buffer
reserved-memory regions, and no RPU-side IPI mailbox node.

The remaining work is exactly three items:

```text
(1) APU <-> RPU communication (rpmsg)
    needs: DT rework (IPI mailbox + mboxes + vring/buffer regions) -> one reboot,
           AND a firmware rebuild carrying a resource table.
    The 4 MiB carve-out uses only ~78 KB, so vring/buffer fit INSIDE the
    0x36000000 window without touching CMA. Do NOT resurrect 0x3ed00000.
(2) Porting the RAON-RT real-time control task to the RPU
    open question: how much of it ports (full control vs interpolation/CiA402
    only) given a single R5F core, VFP and 128 KB TCM. Decide the scope, and
    re-size the carve-out, before starting.
(3) SOEM (EtherCAT) on the RPU
    source import + osal/oshw FreeRTOS stubs + a clean build need no NIC;
    PHY link-up / raw frames / OP need GEM3, which APU IgH currently owns for
    the live robot -> blocked pending the user's re-decision.
```

Items (1) and (2) need no GEM3 and can proceed now; the only scheduling
constraint is that (1)'s DT change costs a reboot, so it must not collide with a
live robot session (`systemctl is-active ethercat` first, and keep a known-good
`/boot/firmware/user-override.dtb` backup — with the file absent, boot.scr falls
back to the FIT's DTB). Board-side load for RPU work stays on CPU0-2; CPU3 is
isolated for the 1 kHz control pair.

## Documentation: the app identity is recorded as "approach + track" — 2026-09-04

A documentation-only pass, no code touched. Two facts had drifted far enough
from the documents to mislead:

1. **The application is approach + tracking, not pick & place.** The TCP
   approaches an object on the table and then follows it while a person moves
   it; grasp/transport/release are out of scope because there is no gripper
   hardware. "Pick & place" was the **initial framing** and stopped being
   accurate on 2026-08-03 (see that section). Documents now say so; **code,
   launch, package, node, topic and message names keep their `pick_*`
   spellings** — renaming them would touch the live system for no benefit — and
   the documents label those names as legacy where they appear.
2. **The RPU track is past Gate 1** (FreeRTOS + LED, 09-02) and the robot is
   already controlled at 1 kHz over EtherCAT from the **APU** (IgH + RAON-RT).
   Several documents still read "robot control not yet implemented", which was
   true when written and is not now.

Reminder for readers of this file: it is **append-only and chronological**.
The `System Summary` at the top is a 2026-06-22 snapshot (SSD stand-in
detector, ~17 Hz, "robot control not implemented") and is superseded by the
dated sections below it; the newest dated section is the current state.

## pick_logic v3: operator-named class, sticky single-instance track — 2026-09-04

Pipeline review after the application change (approach + tracking, no
gripper). Detector, `pick_target_3d_node` and `pick_target_base_node` needed no
change — their moving-object work landed on 2026-08-04/05 and their remaining
parameters are sensor limits or workspace-derived. `pick_logic` did:

- **Removed the v2 scoring and hysteresis switch.** The score
  (0.5·conf + 0.3·centre-proximity + 0.2·area) and the "a better-scoring
  object takes over after 3 frames" rule were the pick-and-place idea of
  "pick the best object". While TRACKING, moving the held object toward the
  image edge let a static object near the centre win by >0.08, and the v2
  bridge (no jump gate, AUTO = any class matches) would then glide the TCP to
  it at the speed cap. The area term was also dead: it normalised against half
  the image, so every real object contributed <0.003.
- **`desired_class` is now required.** '' publishes invalid with reason
  `no_class_selected`; the operator names a class ('p' menu → live param).
  AUTO mode is gone on the pipeline side: the bridge menu entry, the GUI AUTO
  button and `test_gate2b_bridge.py` C1 still reference it and are to be
  removed on the robot side (harmless meanwhile — no LOCK, 'v'/'o' refused).
- **One instance, sticky.** Adoption = highest confidence of the class;
  continuation = nearest candidate; no switch until the track has been
  missing `lost_timeout_frames` in a row. The `stability_match_px` distance
  gate applies only when the class is visible 2+ times, so a single instance
  is re-acquired on the first frame after an occluded move (was: wait for
  lost_timeout, then 3 warm-up frames). The 2+ rule is kept although the
  experiments place one object per class: that is a scene assumption, not a
  detector guarantee (a split banana box past NMS, a same-class false
  positive) — reasoning recorded in the `_update_track` docstring.
- Launch arg `desired_class` (default '') for standalone metrics runs; the
  earlier A_production run relied on AUTO picking apple.
- `tools/vision/test_pick_logic_v2.py` rewritten: S1 no selection, S2/S3
  desired class, S4 max-confidence adoption, S5 two same-class instances
  stay locked, S6 re-acquire after occluded move — 6/6 PASS. Not yet verified
  on the robot.

Same day, documentation: the APU→RPU bridge is not a ros2_ws package.
`CROS2PickBridge` (RAON-RT) already subscribes to `/pick_target_base`; when
the RT task moves to the RPU only its output changes (seqlock → rpmsg).
`apu_rpu_bridge_pkg` (deleted 2026-07-15) will not be recreated.
