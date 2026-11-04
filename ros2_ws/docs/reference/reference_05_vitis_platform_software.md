# Reference 05 — Vitis Unified Software Platform, Platform Creation, embeddedsw, and Software Debug

Use this file when the question is primarily about:
- Vitis Unified Software Platform,
- platform creation,
- software build flow,
- BSP or standalone/embedded software support,
- embeddedsw drivers,
- debugging and software bring-up.

---

## What This File Covers

This file is the primary lookup target for:
- building or selecting a Vitis platform,
- understanding Vitis software structure around Kria or MPSoC projects,
- using embeddedsw components,
- software-side integration of hardware-generated platforms,
- software debugging flow.

---

## Scope Boundary

This file is about **software platform creation and software-side integration**.

Do not start here if the real question is mainly:
- custom AXI or hardware block design → use the Vivado/AXI file,
- AI runtime support → use the Vitis AI file,
- APU↔RPU inter-processor communication → use the OpenAMP file.

---

## Recommended Lookup Order

1. Vitis official docs
2. Vitis Tutorials
3. Kria board-specific Vitis platform pages
4. embeddedsw repositories and documentation
5. Vitis-related wiki pages
6. official forum
7. adjacent-topic fallback only if needed

---

## Decision Hints

### When the user asks "Do I need Vitis here?"
Answer by separating:
- hardware design in Vivado,
- exported platform / XSA or equivalent handoff,
- software project creation and build,
- runtime or OS-side deployment.

### When the user asks about version choice
Do not choose the newest version by default. Compare:
- compatibility with KV260 flow,
- compatibility with target OS path,
- compatibility with Vitis AI if needed,
- tutorial and board-specific documentation availability,
- driver and BSP maturity.

### When the user asks about a software bug
Separate:
- build-time issue,
- platform generation issue,
- BSP/driver mismatch,
- runtime issue,
- OS-specific issue.

---

## Primary References

### Official Vitis references
- AMD Vitis product page  
  `https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vitis.html`
- UG1416 Vitis documentation  
  `https://docs.amd.com/v/u/en-US/ug1416-vitis-documentation`
- UG1400 Vitis Embedded overview  
  `https://docs.amd.com/r/en-US/ug1400-vitis-embedded/Vitis-Unified-Software-Platform-Overview`
- UG1605 Vitis tutorials  
  `https://docs.amd.com/v/u/en-US/UG1605-vitis-tutorials`
- UG1701 Vitis accelerated embedded getting started  
  `https://docs.amd.com/r/en-US/ug1701-vitis-accelerated-embedded/Getting-Started-with-Vitis-Unified-Software-Platform`
- UG1137 Zynq UltraScale+ MPSoC software development: Vitis Unified Software Platform  
  `https://docs.amd.com/r/en-US/ug1137-zynq-ultrascale-mpsoc-swdev/Vitis-Unified-Software-Platform`

### Tutorial references
- Vitis Tutorials 2021.2  
  `https://xilinx.github.io/Vitis-Tutorials/2021-2/build/html/index.html`
- Vitis Tutorials 2021.1  
  `https://xilinx.github.io/Vitis-Tutorials/2021-1/build/html/index.html`
- Getting Started with Vitis Part 2  
  `https://xilinx.github.io/Vitis-Tutorials/2021-2/build/html/docs/Getting_Started/Vitis/Part2.html`
- Vitis Platform Creation step 3 (VCK190)  
  `https://xilinx.github.io/Vitis-Tutorials/2021-2/build/html/docs/Vitis_Platform_Creation/Introduction/03_Edge_VCK190/step3.html`
- Edge AI ZCU104 step 1  
  `https://xilinx.github.io/Vitis-Tutorials/2022-1/build/html/docs/Vitis_Platform_Creation/Design_Tutorials/02-Edge-AI-ZCU104/step1.html`
- Software profiling tutorial  
  `https://xilinx.github.io/Embedded-Design-Tutorials/docs/2021.1/build/html/docs/Feature_Tutorials/sw-profiling/sw-profiling.html`
- Vitis Embedded Software Debugging overview  
  `https://xilinx.github.io/Embedded-Design-Tutorials/docs/2020.2/build/html/docs/Vitis-Embedded-Software-Debugging/docs/1-xilinx-debug-solution-overview/README.html`
- VMK180 TRD Vitis platform build  
  `https://xilinx.github.io/vmk180-trd/2021.1/build/html/docs/build_vitis_platform.html`
- KV260 2022.1 build Vitis platform  
  `https://xilinx.github.io/kria-apps-docs/kv260/2022.1/build/html/docs/build_vitis_platform.html`
- Kria creating applications: Vitis platform flow  
  `https://xilinx.github.io/kria-apps-docs/creating_applications/2022.1/build/html/docs/vitis_platform_flow.html`
- Alveo and Versal platform references  
  `https://xilinx.github.io/Alveo-Versal-Platforms/`

### Wiki references
- Vitis Unified Software Platform wiki  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/297009243/Vitis%2BUnified%2BSoftware%2BPlatform`
- Xilinx Wiki page  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/18841916/`
- Xilinx Wiki page  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/2945679365`
- 2025.1 Release page  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/3281321985/2025.1%2BRelease`

### Official GitHub references
- Xilinx/Vitis-Tutorials repository  
  `https://github.com/Xilinx/Vitis-Tutorials`
- Xilinx/embeddedsw repository  
  `https://github.com/Xilinx/embeddedsw`
- embeddedsw GitHub Pages source  
  `https://github.com/Xilinx/embeddedsw.github.io`
- Vitis Platform Creation README  
  `https://github.com/Xilinx/Vitis-Tutorials/blob/master/Vitis_Platform_Creation/Design_Tutorials/03_Edge_VCK190/README.md`

---

## Answering Rules for This Topic

When the user asks about creating or exporting a platform:
1. identify the hardware handoff artifact,
2. identify the target software environment,
3. explain the tool-chain boundary between Vivado and Vitis,
4. explain where board-specific flow differs from generic flow.

When the user asks about drivers or BSP behavior:
1. identify whether the issue is standalone, FreeRTOS, Linux, or user-space,
2. identify which embeddedsw component actually applies,
3. state version-sensitive assumptions clearly.

---

## Use Adjacent Files Next

Use other files only when the problem clearly shifts:
- **Vivado or AXI hardware side** → `reference_04_vivado_axi_pl_en.md`
- **Vitis AI runtime / model flow** → `reference_03_vitis_ai_vision_en.md`
- **Board-wide feasibility or MPSoC architecture** → `reference_01_kria_core_architecture_en.md`