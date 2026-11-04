# Reference 02 — OpenAMP, FreeRTOS, APU↔RPU, and RPU Ethernet

Use this file when the question is primarily about:
- APU↔RPU communication,
- OpenAMP or RPMsg,
- FreeRTOS on the RPU,
- deterministic control partitioning,
- sending data from the RPU over Ethernet.

---

## What This File Covers

This file is the primary lookup target for:
- Ubuntu on APU + FreeRTOS on RPU coexistence,
- how to pass data from Linux/ROS2 logic to real-time control code,
- how RPU software can output data to an external robot arm,
- when to use OpenAMP versus other mechanisms.

---

## Scope Boundary

This file is about **APU ↔ RPU** and **RPU ↔ external Ethernet**.

Do **not** start from AXI or Vivado references unless the question is actually about a separate **PL ↔ PS** boundary.

### Conceptual split
- **OpenAMP / RPMsg / shared memory**: inter-processor communication between APU and RPU.
- **lwIP / Ethernet stack / MAC driver**: network output from RPU.
- **AXI / DMA**: usually not the first answer for pure APU↔RPU application messaging.

---

## Recommended Lookup Order

### For APU↔RPU communication
1. Kria OpenAMP page
2. Zynq OpenAMP Getting Started Guide
3. Xilinx Wiki OpenAMP page
4. MPSoC TRM only if lower-level clarification is needed
5. Official forum only after the above are insufficient

### For RPU FreeRTOS + Ethernet
1. Kria FreeRTOS page
2. Kria FreeRTOS lwIP example
3. embeddedsw / lwIP-related references if needed
4. MPSoC and Ethernet driver references only as supporting material

---

## Decision Hints

### When OpenAMP is usually appropriate
OpenAMP is usually the first candidate when:
- Linux on APU must exchange structured messages with FreeRTOS on RPU,
- the user needs a clear control-message path rather than raw shared-memory design,
- the system is heterogeneous and software-managed rather than PL-stream-managed.

### When a lower-level mechanism may also matter
A lower-level shared-memory or device-tree detail may matter when:
- the official OpenAMP example does not match the required boot flow,
- memory reservation or interrupt routing becomes the real problem,
- kernel or firmware version differences affect the path.

### When Ethernet on the RPU becomes its own subproblem
Treat RPU Ethernet as a separate subproblem when:
- protocol details with the robot arm matter,
- latency or determinism requirements are strict,
- socket-style behavior is not enough and a specific transport must be implemented.

---

## Primary References

### OpenAMP references
- Kria OpenAMP landing page  
  `https://xilinx.github.io/kria-apps-docs/openamp/build/html/openamp_landing.html`
- Zynq OpenAMP Getting Started Guide  
  `https://docs.amd.com/r/en-US/ug1186-zynq-openamp-gsg`
- Xilinx Wiki OpenAMP page  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/18841718/OpenAMP`

### FreeRTOS references
- FreeRTOS on Kria  
  `https://xilinx.github.io/kria-apps-docs/freertos/build/html/docs/freertos_kria.html`
- FreeRTOS lwIP TCP performance server example  
  `https://xilinx.github.io/kria-apps-docs/freertos/build/html/docs/freertos_kria_lwip_tcpperfserver.html`

### Supporting MPSoC references
- UG1085 TRM  
  `https://docs.amd.com/v/u/en-US/ug1085-zynq-ultrascale-trm`
- PG201 Processing System  
  `https://docs.amd.com/r/en-US/pg201-zynq-ultrascale-plus-processing-system`
- UG1209 Embedded Design Tutorial  
  `https://docs.amd.com/r/en-US/ug1209-embedded-design-tutorial`

### Supporting driver / software references
- Xilinx/embeddedsw repository  
  `https://github.com/Xilinx/embeddedsw`
- embeddedsw GitHub Pages source  
  `https://github.com/Xilinx/embeddedsw.github.io`

---

## Answering Rules for This Topic

When the user asks how to connect Linux/ROS2 logic to the RPU:
1. first state whether the boundary is really **APU → RPU**,
2. explain why OpenAMP is or is not the correct first candidate,
3. separate message transport from network output,
4. state version-sensitive assumptions clearly.

When the user asks about Ethernet from the RPU:
1. ask or identify the required protocol,
2. separate driver support from application protocol design,
3. state whether the answer depends on bare-metal style examples versus FreeRTOS integration.

---

## Use Adjacent Files Next

Use other files only when the problem clearly extends beyond this boundary:
- **PL data source feeding the APU** → `reference_04_vivado_axi_pl_en.md`
- **General board or architecture feasibility** → `reference_01_kria_core_architecture_en.md`
- **Ubuntu / Vitis software-build issues** → `reference_05_vitis_platform_software_en.md`