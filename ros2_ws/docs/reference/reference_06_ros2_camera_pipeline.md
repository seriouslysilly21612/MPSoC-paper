# Reference 06 — ROS2, KRS, Camera Integration, and the End-to-End Robotics Pipeline

Use this file when the question is primarily about:
- ROS2 on Kria,
- Kria Robotics Stack (KRS),
- camera integration at system level,
- how perception output flows into robot-arm control logic,
- orchestrating the end-to-end robotics pipeline.

---

## What This File Covers

This file is the primary lookup target for:
- using ROS2 on the APU side,
- understanding KRS and accelerated ROS2 examples,
- mapping perception output into motion/control logic,
- reasoning about camera-to-APU/PL-to-robot dataflow at a system level.

This file is especially useful when the user is asking:
- how the whole project should be organized,
- whether a vision result can be consumed by ROS2,
- how to stage implementation milestones,
- how to combine camera input, inference, and robot control.

---

## Scope Boundary

This file is about **application-level orchestration** and the **end-to-end robotics pipeline**.

It is not the primary source for:
- Vitis AI version support details,
- custom AXI block design,
- APU↔RPU communication internals.

Use the specialized files when the question narrows into those subproblems.

---

## Recommended Lookup Order

1. ROS official documentation
2. Kria Robotics Stack docs
3. KRS feature and example pages
4. Board/platform docs only for integration constraints
5. Adjacent-topic files for the specific subproblem

---

## Recommended System View

A practical system view for this project is:

1. **Camera input stage**
   - acquire RGB-D or video stream,
   - identify host-side driver and SDK constraints,
   - determine whether the first landing point is Ubuntu user space, ROS2 node, or an accelerated pipeline.

2. **Perception stage**
   - run inference or preprocessing/postprocessing,
   - decide whether the path is CPU/GPU-like, Vitis AI/DPU-like, or custom PL acceleration.

3. **Decision stage**
   - use ROS2 nodes or application logic to interpret perception output,
   - determine the target object or target pose.

4. **Control-transfer stage**
   - hand off control data from APU to RPU if real-time Ethernet output is required.

5. **Actuation stage**
   - transmit the command from the RPU to the robot arm over Ethernet.

This staged model is useful when the user asks for implementation order.

---

## Implementation-Planning Hint

When the user asks how to build the whole project, recommend a staged validation order such as:
1. verify basic OS and board bring-up,
2. verify camera input path,
3. verify perception path independently,
4. verify ROS2 consumption of the result,
5. verify APU→RPU communication,
6. verify RPU Ethernet output,
7. only then integrate the full closed loop.

---

## Primary References

### ROS2 and KRS references
- ROS official documentation  
  `https://docs.ros.org/`
- Kria Robotics Stack landing page  
  `https://xilinx.github.io/KRS/sphinx/build/html/index.html`
- KRS install guide  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/install.html`
- KRS hardware guide  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/hardware.html`

### Feature references
- ROS 2-centric development  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/features/ros2centric.html`
- Real-time ROS 2  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/features/realtime_ros2.html`
- Accelerated ROS 2 applications  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/features/accelerated_apps_ros2.html`
- Contributing ROS 2 packages  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/features/contributing_ros2.html`
- Definitions  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/other/definitions.html`

### Example references
- Example: ROS 2 publisher  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/0_ros2_publisher.html`
- Example: Hello Xilinx  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/1_hello_xilinx.html`
- Example: HLS ROS 2  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/2_hls_ros2.html`
- Example: Offloading ROS 2 publisher  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/3_offloading_ros2_publisher.html`
- Example: Accelerated ROS 2 publisher  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/4_accelerated_ros2_publisher.html`
- Example: Faster ROS 2 publisher  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/5_faster_ros2_publisher.html`
- Example: Perception  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/6_perception.html`
- Example: Vitis accelerated function  
  `https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/vitis_accelerated_function.html`

### Supporting board and system references
- AMD KV260 Starter Kit User Guide  
  `https://docs.amd.com/r/en-US/ug1089-kv260-starter-kit`
- Kria Apps Docs: Creating Applications  
  `https://xilinx.github.io/kria-apps-docs/creating_applications/2022.1/build/html/index.html`

---

## Answering Rules for This Topic

When the user asks how to architect the whole robotics pipeline:
1. separate perception, decision, control-transfer, and actuation stages,
2. identify where ROS2 belongs,
3. identify which boundaries require other topic files,
4. recommend a validation sequence rather than a giant one-shot integration.

When the user asks about a camera such as ZED 2i:
1. separate camera acquisition from AI inference support,
2. separate SDK/driver feasibility from robotics-application design,
3. state clearly which parts are official Kria support and which are general Linux/ROS2 integration assumptions.

---

## Use Adjacent Files Next

Use other files when the subproblem becomes more specific:
- **Vitis AI and inference support** → `reference_03_vitis_ai_vision_en.md`
- **APU ↔ RPU handoff and RPU Ethernet** → `reference_02_openamp_freertos_ethernet_en.md`
- **PL-side custom acceleration or AXI transport** → `reference_04_vivado_axi_pl_en.md`
- **Vitis platform and software build** → `reference_05_vitis_platform_software_en.md`