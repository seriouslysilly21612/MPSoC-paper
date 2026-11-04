# Kria KV260 Pick & Place System

## Project Overview

This is an **object approach + tracking system** running on **Kria KV260** (Zynq UltraScale+ MPSoC). The vision pipeline localizes an object on the table, the Indy7 TCP moves to a hover point above it (approach), and it keeps following the object when a person moves it (tracking). **"Pick & Place" was the initial framing and is no longer what the app does** — grasp / transport / place do not exist (no gripper hardware). The title of this file and every `pick_*` launch/node/topic name are legacy from that framing and are deliberately NOT renamed. Architecture:

- **Hardware**: Kria KV260 starter kit + Intel RealSense D435i depth camera (active RGB-D source; a legacy USB camera path in `camera_source_pkg` exists but is unused)
- **Target Robot**: Neuromeka Indy7 (6-axis collaborative arm). **No gripper hardware** — grasping is out of scope
- **SoC Processors**:
  - APU (ARM): Ubuntu 22.04 + ROS2 Humble
  - RPU (Real-Time): FreeRTOS — **boots standalone on R5F-0 (firmware boot verified 2026-09-02, two boots; 09-01 reached DT enumeration only)**. The LED tasks run; the blink itself is a user observation, not re-verifiable from the board. Remaining: ① APU↔RPU messaging ② porting the RAON-RT real-time control task ③ SOEM EtherCAT on the RPU
- **Task**: Detect objects via vision → compute the 3D object position in `base_link` → move the Indy7 TCP to a hover point above the object, and follow the object while a person moves it. No grasp, no place.

## System Architecture

The main pipeline separates into three distinct communication boundaries:

1. **PL ↔ APU**: the DPU (vision accelerator in PL, via the `kv260-smartcam` overlay) is **now active** — the APU feeds frames to the DPU with VART and gets detections back. (The RealSense camera itself enters over USB directly to the APU, not through PL.)
2. **APU → RPU**: target/trajectory handoff to the RPU — **not yet implemented** (RPU remaining task ①). Today the robot is driven from the **APU** instead: the RAON-RT app subscribes to `/pick_target_base` directly.
3. **RPU → External Arm**: EtherCAT from the RPU to the Indy7 drives — **not yet implemented** (RPU remaining task ③, SOEM). Today the **APU** does this with IgH EtherCAT at 1 kHz.

Current status: RealSense → DPU detection → 2D filtering → single-point 3D (reverse projection) → base-frame target on APU via ROS2 (**21.7 Hz, capture→base E2E p50 ~118 ms, depth-color skew ≈0** — 2026-08-05: moving-object pivot, gate 0.030 + /dev/shm worker IPC + sync'd 30/30 + nearest-stamp depth; z verified). The detector runs the **final pick-object model** — YOLOv3-tiny 6-class (apple / orange / banana / tennis_ball / mustard_bottle / person), INT8, trained and compiled for this DPU (the old SSD ADAS stand-in is retired). An RT kernel (`5.15.199-rt91-rt-kv260c`) is built and verified.

**Robot control is live on the APU** (repo `~/RAON-RT-Revision`, branch `kv260-merge` — not this repo): RAON-RT drives the Indy7 at 1 kHz over IgH EtherCAT on the RT kernel and consumes `/pick_target_base`. Approach (operator key `'v'`, hover = object top +15 cm) is **field-verified** (one-shot: 2026-08-05, 4 runs, 4.21–5.93 mm; the 2026-07-31 session was 6–12 mm); tracking (key `'o'`, implemented 2026-08-03, v3 hybrid) and the operator console (2026-08-06) are **implemented but not yet field-verified**. Canonical doc: `docs/RAON-RT/merge.md`.

**RPU track** (separate, `docs/rpu/`): FreeRTOS boots standalone on R5F-0 via remoteproc and runs its LED tasks (work 2026-09-01~02; firmware boot verified 2026-09-02, 11:02 and 15:01 — the blink itself is a user observation). Remaining: ① APU↔RPU messaging ② porting the RAON-RT real-time control task to the RPU ③ SOEM EtherCAT on the RPU.

## Current Implementation Snapshot — Perception (APU)

Run the whole perception pipeline:

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py
```

To watch detections as a bbox overlay on the desktop (verified 2026-07-16;
launches merged 2026-07-20), keep this same launch running and run
`detection_viewer_pkg/detection_viewer_node` on the desktop — there is no
separate viewing launch. The board only JPEG-compresses; the desktop draws.
By default the viewer renders EVERY color frame (~30 fps) with the newest boxes
(which update at the detection rate, ~15–22 Hz, so they lag the picture ~100 ms); `--sync` restores the
old 15 fps frame-exact stamp join, which is the mode to use when verifying
bbox↔frame alignment. The compressed topic encodes LAZILY (only while the
viewer subscribes), so it adds nothing to the board when nobody is watching.
Board-side drawing does not exist — it cost 44 ms/frame on top of a 37.6 ms
detect and was removed (2026-09-03).
Full procedure: `docs/vision/desktop_viewer_plan.md`

Active nodes / files / output topics (camera → 3D pick target):

| Node | File | Output topic |
|---|---|---|
| `camera` (realsense2_camera) | config: `system_bringup_pkg/config/realsense_pick_place.yaml` | color / depth / camera_info / extrinsics |
| `vitis_ai_detector_node` (+ `vitis_ai_worker_yolo.py`) | `vitis_ai_detector_pkg/vitis_ai_detector_pkg/` | `/detections` (`DetectionArray`) |
| `pick_logic_node` | `pick_logic_pkg/pick_logic_pkg/pick_logic.py` | `/pick_target` (`PickTarget`) |
| `pick_target_3d_node` | `target_3d_pkg/target_3d_pkg/pick_target_3d_node.py` | `/pick_target_3d` (`camera_depth_optical_frame`) |
| `base_to_camera_tf` (static TF, **calibrated** 2026-07-27) | launch argument | `/tf_static` |
| `pick_target_base_node` | `target_3d_pkg/target_3d_pkg/pick_target_base_node.py` | `/pick_target_base` (`base_link`) |

Node parameters live in `system_bringup_pkg/config/*.yaml` (one per node). Custom messages are in `my_interfaces` (`Detection`, `DetectionArray`, `PickTarget`, `PickTarget3D`).

**Detailed docs (read these before changing perception):**
- `docs/STATUS.md` — **start here.** Integration hub + the canonical routing table for every other doc.
- `docs/vision/workflow.md` — node-by-node walkthrough, each parameter's value + rationale, the preprocessing/postprocessing techniques, and the pipeline architecture.
- `docs/vision/vision_final.md` — the whole vision track end to end: SSD→YOLO swap, training, the Vitis-AI quantize/compile flow onto the DPU, and every pipeline optimization with its measured delta.
- `docs/history.md` — full chronological history (decisions, measurements, root-cause fixes).
- `docs/vision/yolov3_tiny_execution_plan.md` — the model-swap plan as executed (dataset sourcing, UG1414 v2.5-grounded quantize/compile commands, phase gates). Historical: it says "7-class" and assumes synthetic-only data; both changed during execution (peach dropped → 6-class; real images added at D12/D14). `vision_final.md` is the accurate account.
- `docs/vision/yolov5_migration_plan.md` — **the next vision task (decided 2026-09-08).** Replace the mainline detector with a YOLOv5-family model (YOLOv5n first choice) and keep YOLOv3-tiny 6-class as a **legacy artifact for A/B comparison, not deleted**. Covers the dataset rebuild (BlenderProc synthetic dropped, deployment-camera captures extended to all 5 classes), the gates, and the three code changes co-existence needs.

**Key perception facts (see `docs/vision/workflow.md` for detail):**
- Detection runs in a **long-running worker process** (`vitis_ai_worker_yolo.py`); process isolation fixed a VART `execute_async` segfault. The detector↔worker boundary is a **model-agnostic JSON contract** (swapping the model touches only the worker's preprocessing constants + decode). Since 2026-08-05 the frame payload rides a **/dev/shm mmap** (one memcpy; `shm_frame.py`, fallback `worker_shm: false`) — only the JSON control lines stay on the pipe. Never unlink the shm file mid-run (inode trap — see `shm_frame.py` docstring).
- **Preprocess = LUT**, **postprocess = background pre-filter** — both verified **bit-identical** to the naive version, large speedups.
- **Callback pipelining**: the subscription callback only stores the newest frame; a worker thread consumes frames back-to-back.
- **3D = single-point reverse projection** on raw depth (`align_depth` OFF); only the bbox-center pixel is matched to its depth pixel.
- **Throughput is set by ONE place: the detector's worker loop** — `max(camera supply 33.3 ms, processing time, the process_period_sec gate)` rounded UP to the next color-frame arrival. 2026-08-05 state: gate relaxed 0.045 → **0.030** (the old 15 Hz cap was deliberate but premised on static objects) and worker IPC moved from the stdin pipe to a **/dev/shm mmap** (`shm_frame.py`, ipc 13.4 → 4.65 ms) → **processing ~45 ms = 21.7 Hz**, CPU unchanged (1.60 cores), camera still supplies 29.4 Hz and the DPU is ~60% idle. Next lever: in-worker pipelining (pre ‖ DPU) to push processing under 33.3 ms — below that the freshness/rate trade-off disappears entirely. **If asked to improve throughput, read `docs/vision/throughput.md` first and propose its §4 ladder in order** — starting with the camera or the DPU is almost always wrong.
- Build note: `vitis_ai_detector_pkg` is editable-installed (egg-link → src is live); `target_3d_pkg` needs `colcon build --packages-select target_3d_pkg --symlink-install` after edits. Config YAMLs are symlinked (live, no rebuild).

## User Level & Assumptions

- Assume undergraduate level with **limited FPGA experience**
- Explain advanced topics (device tree, Vitis AI flows, AXI/OpenAMP) from first principles
- Do not skip setup, integration, or verification steps
- Keep answers technically correct but beginner-aware

## How Claude Should Help

When answering implementation questions, **always**:
1. Verify feasibility on KV260 hardware and the specific OS/runtime stack
2. Reference official Xilinx/AMD docs and board-specific examples first
3. Separate PL, APU, and RPU concerns explicitly—do not collapse them into one problem
4. Provide procedures, validation steps, and debugging order, not abstract explanations
5. State assumptions about board revision, software versions, and device tree when relevant

### Response Priority for Vision & AI Tasks

When selecting models or approaches for the vision pipeline:

1. **Output utility for pick & place first**: Does it give object class, 2D location, and depth? That is the minimum.
2. **KV260 deployability second**: Is there a clear Vitis AI path or realistic inference path on the board?
3. **Latency third**: Does end-to-end time (capture → detect → 3D compute → ROS2 publish) fit the cycle?
4. **Model choice fourth**: Paper accuracy is less important than practical deployment.

**Recommended vision models for KV260 Pick & Place (in priority order)**:
- SSD / SSDLite-MobileNet v2 (most conservative, best official support)
- tiny/pruned YOLOv3 (faster but more tuning required)
- Instance segmentation (only if objects overlap heavily and contours are essential)
- Pose estimation (only if orientation control is critical to the manipulation task)

**Do not assume** a newer model is better. Prefer models with clear Vitis AI support and proven KV260 deployment paths.

## Key Constraints & Decisions

- **FPGA / DPU**: KV260 `kv260-smartcam` overlay active; detection runs on the DPU (DPUCZDX8G_ISA1_B3136) via a long-running Python VART worker process.
- **RPU firmware**: FreeRTOS boots standalone on R5F-0 (remoteproc, boot-time DT via `/boot/firmware/user-override.dtb`, carve-out `rproc@36000000` 4 MiB) and runs its LED tasks — firmware boot verified 2026-09-02 (11:02 and 15:01, kernel log `is now up`; 09-01 reached DT enumeration only). The blink itself is a **user observation** and cannot be re-verified from the board (`/sys/class/leds/` does not expose led35/36). The firmware has **no resource table** and the DT has **no vring/IPI mailbox**, so APU↔RPU rpmsg needs both a DT rework and a firmware rebuild. Robot control itself does not wait on this: it runs on the APU (see Project Overview).
- **RealSense D435i depth**: raw (unaligned) depth, 16UC1, 0.001 scale. `align_depth.enable` is OFF — full-frame alignment was removed (it pegged one A53 core and halved throughput). 3D uses **single-point reverse projection**: the color bbox-center pixel is matched to its depth pixel via depth/color intrinsics + depth→color extrinsics (rs2_project_color_pixel_to_depth_pixel). Output is in `camera_depth_optical_frame`.
- **Depth/color time alignment (2026-08-04, for MOVING objects)**: `enable_sync: true` + depth at 30 fps (same as color) so every color frame has a same-timestamp depth partner, AND `pick_target_3d_node` keeps an 800 ms depth history and picks the frame **nearest the color capture stamp** (`nearest_by_stamp`). Both halves are required — the detection arrives ~90 ms late, so the *newest* depth is ~3 frames past the partner. Measured: skew |mean| 28.5 → **4.3 ms**, 89% exactly 0.0, for +0.18 cores and no latency/throughput change (`evidence/metrics/runs/sync30_compare_20260804.md`). Do NOT "simplify" the history back to a single latest frame — that alone puts skew back to 36 ms.
- **ROS2 message protocol**: Custom `Detection`, `DetectionArray`, `PickTarget`, `PickTarget3D` types in `my_interfaces`. The detector↔worker boundary is a model-agnostic JSON contract, so swapping the model leaves the node/pipeline/3D/downstream unchanged.
- **Operator level**: Unified bringup launch `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py` starts the full perception pipeline.

## Source Priority & Topic-Based Reference Strategy

When answering user questions, **always**:

1. **Check topic-specific markdown files first**: Before researching externally, look for a matching topic markdown file in the `docs/reference/` directory
2. **Read the markdown file**: Extract all reference URLs and documentation links from the relevant topic file
3. **Consult the referenced sites**: Use the links in the markdown to find specific, up-to-date information
4. **Ground your answer in those sources**: When answering, cite the markdown file and the official sources it references

**Topic file lookup strategy**:
- Identify the dominant topic in the user's question
- Search for a matching markdown file in `docs/reference/`:
  - `reference_01_kria_core_architecture.md` — KV260, Zynq UltraScale+, APU/RPU/PL architecture
  - `reference_02_openamp_freertos_ethernet.md` — OpenAMP, FreeRTOS, RPU firmware, Ethernet control
  - `reference_03_vitis_ai_vision.md` — Vitis AI, vision models, inference deployment, quantization
  - `reference_04_vivado_axi_pl.md` — Vivado, AXI, PL (programmable logic), bitstream generation
  - `reference_05_vitis_platform_software.md` — Vitis unified software platform, build flows, board support packages
  - `reference_06_ros2_camera_pipeline.md` — ROS2 Humble, camera drivers, sensor integration, message pipelines
- If found, read it and use its curated links for research
- If not found, proceed to the general source priority below

**General source priority when no topic file exists**:

1. **AMD/Xilinx official documentation** (UG, PG, release notes)
2. **Kria board-specific docs** (KV260 quick start, Kria Apps examples)
3. **Xilinx wiki** and design hubs
4. **Official GitHub repositories** (Xilinx-AI, Vitis-Tutorials)
5. **Official AMD forums**
6. **Community sources** only if higher-priority sources are insufficient

If uncertain or sources disagree: state the disagreement, cite both sources, and recommend official documentation confirmation before committing to implementation.

## Version Baseline

- **Board**: Kria KV260 (latest revision preferred)
- **OS**: Ubuntu 22.04 LTS (or PetaLinux if required)
- **ROS2**: Humble
- **Accelerator**: `kv260-smartcam` overlay; DPU `DPUCZDX8G_ISA1_B3136` (fingerprint `0x101000016010406`), Vitis-AI runtime/library 2.5.0. Boot auto-load via a `kv260-smartcam.service` systemd unit.
- **Camera**: realsense2_camera v4.57.7 / librealsense 2.57.7; D435i FW **5.17.0.10**; color & depth both 848×480×30, `enable_sync` **ON**, `align_depth` OFF.
  - ⚠️ **Do not use FW 5.16.0.1** — RGB frames stop after tens of seconds of streaming (`docs/history.md` §13 has the split-diagnosis method and the fix).
- **Current model**: YOLOv3-tiny 6-class INT8 — apple / orange / banana / tennis_ball / mustard_bottle / person. Ships inside the package at `src/vitis_ai_detector_pkg/models/yolov3_tiny_7class.xmodel` (the `7class` in the filename is a leftover from the initial 7-class run; peach was dropped at D13. `decode_meta.json`, which must stay in the same directory, is authoritative).
- **RT kernel**: `5.15.199-rt91-rt-kv260c` — production, DEBUG off, radix + zocl fixes applied. Needed only for the EtherCAT track; the vision pipeline runs fine on the stock kernel.
- **Tools**: Vivado/Vitis/Vitis AI → match target task & official support matrix.
- **Avoid breaking changes**: Prefer stable, board-validated versions over "latest" unless specifically required.

## Known Gaps & TODOs

- [x] Unified bringup launch + YAML config (`pick_place_vitis_ai.launch.py`)
- [x] Real DPU detector replacing the mock detector
- [x] 2D pick-logic filtering (confidence / class / edge / bbox-size)
- [x] 3D localization via single-point reverse projection (`align_depth` off)
- [x] Final pickable-object detector for B3136 — YOLOv3-tiny 6-class INT8, mAP@0.5 0.766 (`docs/vision/vision_final.md`)
- [x] Pipeline CPU optimization — 76.8% → ~44% (−1.25 core), lossless
- [x] RT kernel `5.15.199-rt91-rt-kv260c` built + verified (`docs/rt/rt_final.md`)
- [x] Camera-to-base calibration (`base_link → camera_link`) — eye-to-hand 2026-07-27, 16 poses, residual 1.8 cm / 1.7°, + E21 base re-baseline
- [x] Robot control on the APU — RAON-RT + IgH EtherCAT 1 kHz; approach field-verified, tracking implemented 2026-08-03 (field verification pending)
- [x] RPU FreeRTOS boot + LED task (firmware boot verified 2026-09-02; blink = user observation)
- [ ] **YOLOv5 becomes the mainline detector; YOLOv3-tiny 6-class stays as a legacy A/B baseline** ← next vision task, decided 2026-09-08 (`docs/vision/yolov5_migration_plan.md`)
- [ ] APU↔RPU messaging (rpmsg — needs resource table + vring/IPI in DT) ← RPU remaining ①
- [ ] Port the RAON-RT real-time control task to the RPU ← RPU remaining ②
- [ ] SOEM EtherCAT on the RPU ← RPU remaining ③ (takes GEM3 from the APU — **do not start without the user's re-decision**)
- [ ] Custom PL acceleration beyond the vendor DPU (e.g. pre/post-processing or a custom bitstream). NOTE: the `kv260-smartcam` DPU already accelerates the NN inference in PL.

## Communication Style

- Answer in **Korean**, keep technical terms (e.g., "calibration," "depth scale," "device tree") in English.
- Be precise, calm, and procedure-oriented.
- Prefer validation steps and debugging order over speculation.
- Do not make strong claims without high-priority source support.
- State dependencies on board revision, software version, or configuration when relevant.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
