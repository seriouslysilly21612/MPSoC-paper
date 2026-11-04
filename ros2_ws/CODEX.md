# Kria KV260 Pick & Place System

## Project Overview

This is a **Pick & Place system** running on **Kria KV260** (Zynq UltraScale+ MPSoC) with the following architecture:

- **Hardware**: Kria KV260 starter kit + Intel RealSense D435i depth camera + USB RGB camera
- **Target Robot**: Neuromeka Indy7 (6-axis collaborative arm) + 3-finger gripper
- **SoC Processors**:
  - APU (ARM): Ubuntu 22.04 + ROS2 Humble
  - RPU (Real-Time): FreeRTOS (future: hardware acceleration, Ethernet control interface)
- **Task**: Detect objects via vision → compute 3D pick position → generate robot trajectory → execute grasp and place

## System Architecture

The main pipeline separates into three distinct communication boundaries:

1. **PL → APU**: Vision accelerator (if used) or raw sensor data from PL to ROS2
2. **APU → RPU**: ROS2 pick decision and trajectory sent to RPU for robot control
3. **RPU → External Arm**: Ethernet protocol to Neuromeka Indy7 for trajectory execution

Current status: Camera → detection → 3D localization on APU via ROS2. Robot control layers (RPU bridge, Ethernet trajectory protocol) not yet implemented.

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

- **No FPGA bitstream yet**: Current implementation uses APU only (CPU inference or no inference).
- **No RPU firmware**: Placeholder exists; FreeRTOS and robot control not yet implemented.
- **RealSense D435i format**: Depth aligned to color, 16UC1 encoding, 0.001 scale factor.
- **ROS2 message protocol**: Custom `Detection`, `PickTarget`, `PickTarget3D` types defined in `my_interfaces`.
- **Operator level**: System startup via command-line ROS2 node launchers (no unified launch file yet).

## Source Priority & Topic-Based Reference Strategy

When answering user questions, **always**:

1. **Check topic-specific markdown files first**: Before researching externally, look for a matching topic markdown file in `/home/ubuntu/ros2_ws/site_md/` directory
2. **Read the markdown file**: Extract all reference URLs and documentation links from the relevant topic file
3. **Consult the referenced sites**: Use the links in the markdown to find specific, up-to-date information
4. **Ground your answer in those sources**: When answering, cite the markdown file and the official sources it references

**Topic file lookup strategy**:
- Identify the dominant topic in the user's question
- Search for a matching markdown file in `site_md/`:
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
- **Tools**: Vivado/Vitis/Vitis AI → choose version matching target task and official support matrix
- **Avoid breaking changes**: Prefer stable, board-validated versions over "latest" unless specifically required.

## Known Gaps & TODOs

- [ ] Real ML model integration (mock detector only)
- [ ] RPU firmware + Ethernet robot control
- [ ] APU↔RPU bridge (`apu_rpu_bridge_pkg`)
- [ ] Unified launch file and YAML parameter config
- [ ] Calibration file persistence
- [ ] Pick logic filtering (multiple detection disambiguation)
- [ ] Hardware acceleration (FPGA PL logic for vision pipeline)

## Communication Style

- Answer in **English**, keep technical terms (e.g., "calibration," "depth scale," "device tree") in English.
- Be precise, calm, and procedure-oriented.
- Prefer validation steps and debugging order over speculation.
- Do not make strong claims without high-priority source support.
- State dependencies on board revision, software version, or configuration when relevant.

