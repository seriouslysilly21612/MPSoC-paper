# Reference 01 — Kria KV260 Core Architecture and Board Context

Use this file when the question is primarily about:
- overall project architecture,
- board capabilities and limitations,
- Kria KV260 board-specific flow,
- Zynq UltraScale+ MPSoC fundamentals,
- deciding whether a feature belongs in PL, APU, or RPU.

---

## What This File Covers

This file defines the baseline architecture and the first references to consult when the user asks broad project questions.

Typical questions:
- Can KV260 run Ubuntu and FreeRTOS in a heterogeneous design?
- Which subsystem should handle vision, control, or Ethernet?
- Is a requested feature possible at board level?
- Which references should be checked before going into tool-specific details?

---

## Default Project Model

Assume the system has three layers unless the user states otherwise:

### Layer 1 — PL / accelerator domain
- custom logic,
- preprocessing/postprocessing,
- vision acceleration,
- AXI-based hardware interfaces,
- DMA or stream-oriented movement into PS-side software.

### Layer 2 — APU / high-level application domain
- Ubuntu,
- ROS2,
- application logic,
- perception orchestration,
- motion planning and decision logic.

### Layer 3 — RPU / real-time output domain
- FreeRTOS,
- bounded-latency handling,
- deterministic data transfer,
- Ethernet-side output to the robot arm.

---

## Communication Boundaries to Keep Separate

Always analyze these as separate interfaces:
1. **PL → APU**
2. **APU → RPU**
3. **RPU → external robot arm**

Do not assume that one mechanism solves all three boundaries.

Examples:
- **AXI / DMA** mainly helps with **PL ↔ PS/APU** style movement.
- **OpenAMP / RPMsg / shared memory** mainly helps with **APU ↔ RPU**.
- **lwIP / Ethernet MAC / protocol stack choices** mainly help with **RPU ↔ external device**.

---

## First Questions to Ask Internally

When the user asks a broad architecture question, evaluate in this order:
1. What is the dominant boundary? PL→APU, APU→RPU, or RPU→Ethernet?
2. Is the question about hardware possibility, software support, or tool flow?
3. Does the answer depend on Ubuntu vs PetaLinux?
4. Does the answer depend on tool version or board-specific firmware flow?
5. Is a Kria-specific page enough, or is a lower-level MPSoC reference required?

---

## Recommended Lookup Order

### For board-level feasibility
1. Kria KV260 board docs
2. Kria Apps Docs board-flow pages
3. Zynq UltraScale+ MPSoC architecture docs
4. Xilinx Wiki board or MPSoC pages
5. Official GitHub examples

### For lower-level hardware capability
1. Zynq UltraScale+ MPSoC TRM and Processing System docs
2. Embedded Design Tutorials
3. Kria board docs only as board-specific context
4. Wiki or GitHub examples for supplemental detail

---

## Primary References

### Kria KV260 board and app-flow references
- AMD KV260 Starter Kit User Guide  
  `https://docs.amd.com/r/en-US/ug1089-kv260-starter-kit`
- Kria Apps Docs: Creating Applications  
  `https://xilinx.github.io/kria-apps-docs/creating_applications/2022.1/build/html/index.html`
- Bitstream Management  
  `https://xilinx.github.io/kria-apps-docs/creating_applications/2022.1/build/html/docs/bitstream_management.html`
- DFX Landing Page  
  `https://xilinx.github.io/kria-apps-docs/dfx/build/html/docs/DFX_Landing_Page.html`
- Kria DFX K26  
  `https://xilinx.github.io/kria-apps-docs/dfx/build/html/docs/Kria_DFX_K26.html`
- Accelerators on K26  
  `https://xilinx.github.io/kria-apps-docs/dfx/build/html/docs/Accelerators_On_K26.html`
- Building the design  
  `https://xilinx.github.io/kria-apps-docs/kv260/2022.1/build/html/docs/building_the_design.html`
- Build Vivado design  
  `https://xilinx.github.io/kria-apps-docs/kv260/2022.1/build/html/docs/build_vivado_design.html`
- Build Vitis platform  
  `https://xilinx.github.io/kria-apps-docs/kv260/2022.1/build/html/docs/build_vitis_platform.html`
- Build accelerator  
  `https://xilinx.github.io/kria-apps-docs/kv260/2022.1/build/html/docs/build_accel.html`
- Generate custom firmware  
  `https://xilinx.github.io/kria-apps-docs/kv260/2022.1/build/html/docs/generating_custom_firmware.html`
- Build application Docker container  
  `https://xilinx.github.io/kria-apps-docs/kv260/2022.1/build/html/docs/build_application_docker_container.html`

### Zynq UltraScale+ MPSoC architecture references
- Zynq UltraScale+ MPSoC product page  
  `https://www.amd.com/en/products/adaptive-socs-and-fpgas/soc/zynq-ultrascale-plus-mpsoc.html`
- DS891 overview  
  `https://docs.amd.com/v/u/en-US/ds891-zynq-ultrascale-plus-overview`
- UG1085 TRM  
  `https://docs.amd.com/v/u/en-US/ug1085-zynq-ultrascale-trm`
- PG201 Processing System  
  `https://docs.amd.com/r/en-US/pg201-zynq-ultrascale-plus-processing-system`
- UG1209 Embedded Design Tutorial  
  `https://docs.amd.com/r/en-US/ug1209-embedded-design-tutorial`
- AMD knowledge hub document  
  `https://docs.amd.com/api/khub/documents/PQWk4s3EaVsCJQ8xtNe3JQ/content`
- AMD knowledge hub document  
  `https://docs.amd.com/api/khub/documents/PxXD1BvQdOi91ImBBaA_lg/content`

### Supplemental MPSoC references
- Zynq UltraScale+ MPSoC wiki  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/444006775/Zynq%2BUltraScale%2BMPSoC`
- Zynq UltraScale+ MPSoC example designs  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/189595724/Zynq%2BUltraScale%2BMPSoC%2BExample%2BDesigns`
- Embedded-Design-Tutorials repository  
  `https://github.com/Xilinx/Embedded-Design-Tutorials`
- system-device-tree-xlnx  
  `https://github.com/Xilinx/system-device-tree-xlnx`
- libdfx  
  `https://github.com/Xilinx/libdfx`

---

## Use Adjacent Files Next

If the dominant topic becomes more specific, switch to the matching reference file:
- **OpenAMP / RPU / Ethernet** → `reference_02_openamp_freertos_ethernet_en.md`
- **Vitis AI / inference / model flow** → `reference_03_vitis_ai_vision_en.md`
- **Vivado / AXI / Block Design / PL interfaces** → `reference_04_vivado_axi_pl_en.md`
- **Vitis platform creation / embeddedsw / debug** → `reference_05_vitis_platform_software_en.md`
- **ROS2 / KRS / camera-to-robot pipeline** → `reference_06_ros2_camera_pipeline_en.md`