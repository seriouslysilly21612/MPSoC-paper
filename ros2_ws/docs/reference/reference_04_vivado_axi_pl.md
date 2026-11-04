# Reference 04 — Vivado, Block Design, AXI, and PL↔APU Data Movement

Use this file when the question is primarily about:
- Vivado Block Design,
- custom PL logic,
- AXI4-Lite / AXI4 / AXI4-Stream interfaces,
- DMA / VDMA / GPIO / Interconnect,
- PL↔APU data transport,
- integrating custom IP into a Kria or Zynq design.

---

## What This File Covers

This file is the primary lookup target for:
- custom AXI interface design,
- how PL data reaches the PS/APU side,
- which AXI IP blocks to use,
- how to reason about DMA versus stream infrastructure,
- how to build or debug Block Design-based integration.

---

## Scope Boundary

This file is about **PL ↔ APU / PS-side software**.

Do not start from OpenAMP unless the real problem is **APU ↔ RPU**.

Do not start from Vitis AI docs unless the user is asking about a Vitis AI runtime flow rather than custom hardware interface design.

---

## Recommended Lookup Order

### For Block Design and IP integration
1. Vivado official user guides
2. Board-specific build guides
3. Vivado tutorials
4. Official GitHub tutorial repositories
5. Wiki pages

### For AXI or DMA data movement
1. AXI reference and product guides
2. Driver API docs
3. Wiki pages
4. embeddedsw GitHub repositories
5. Broader Vivado or Vitis references only if necessary

---

## Decision Hints

### When to think AXI4-Lite
Use AXI4-Lite when the problem is mainly:
- register access,
- control/status,
- low-bandwidth configuration from software.

### When to think AXI4 / DMA
Use AXI4 or DMA-oriented thinking when the problem is mainly:
- bulk memory movement,
- frame or buffer transport,
- software-visible memory transfer between PL and PS-side software.

### When to think AXI4-Stream
Use AXI4-Stream when the problem is mainly:
- pipelined streaming data,
- image/video path between hardware blocks,
- throughput-oriented accelerator chains.

### When the user is confused between communication layers
State clearly:
- AXI is usually for **PL ↔ PS-side hardware/software data path**,
- OpenAMP is usually for **APU ↔ RPU software messaging**,
- Ethernet is usually for **RPU ↔ external device networking**.

---

## Primary References

### Vivado references
- AMD Vivado product page  
  `https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vivado.html`
- UG910 Vivado getting started  
  `https://docs.amd.com/r/en-US/ug910-vivado-getting-started`
- UG896 Vivado IP documentation  
  `https://docs.amd.com/r/en-US/ug896-vivado-ip/Vivado-Design-Suite-Documentation`
- UG949 Vivado design methodology  
  `https://docs.amd.com/r/en-US/ug949-vivado-design-methodology/Vivado-Design-Suite-User-and-Reference-Guides`
- UG892 design flows overview: Working with IP  
  `https://docs.amd.com/r/en-US/ug892-vivado-design-flows-overview/Working-with-IP`
- UG896 IP-centric design flow  
  `https://docs.amd.com/r/en-US/ug896-vivado-ip/IP-Centric-Design-Flow`

### Board-flow and tutorial references
- XUP FPGA Vivado flow  
  `https://xilinx.github.io/xup_fpga_vivado_flow/`
- XUP FPGA Vivado flow Lab 1  
  `https://xilinx.github.io/xup_fpga_vivado_flow/lab1.html`
- XUP FPGA Vivado flow Lab 3  
  `https://xilinx.github.io/xup_fpga_vivado_flow/lab3.html`
- XUP FPGA Vivado flow Lab 4  
  `https://xilinx.github.io/xup_fpga_vivado_flow/lab4.html`
- XUP FPGA Vivado flow Lab 5  
  `https://xilinx.github.io/xup_fpga_vivado_flow/lab5.html`
- XUP FPGA Vivado flow presentations  
  `https://xilinx.github.io/xup_fpga_vivado_flow/presentations.html`
- XUP embedded system design flow Lab 1  
  `https://xilinx.github.io/xup_embedded_system_design_flow/lab1.html`
- KV260 build Vivado design  
  `https://xilinx.github.io/kria-apps-docs/kv260/2022.1/build/html/docs/build_vivado_design.html`
- KD240 build Vivado design  
  `https://xilinx.github.io/kria-apps-docs/kd240/build/html/docs/build_vivado_design.html`
- VMK180 TRD build Vivado design  
  `https://xilinx.github.io/vmk180-trd/2022.1/build/html/docs/build_vivado_design.html`
- VCK190 Ethernet TRD build Vivado design  
  `https://xilinx.github.io/vck190-ethernet-trd/2021.1/build/html/docs/build_vivado_design.html`
- Versal restart TRD build hardware  
  `https://xilinx.github.io/versal-restart-trd/2023.1/pages/build-hw.html`
- RTL kernel workflow: Vivado IP  
  `https://xilinx.github.io/Vitis-Tutorials/2020-2/docs/build/html/docs/Hardware_Accelerators/Feature_Tutorials/01-rtl_kernel_workflow/vivado_ip.html`

### AXI references
- UG1037 Vivado AXI Reference Guide  
  `https://docs.amd.com/v/u/en-US/ug1037-vivado-axi-reference-guide`
- PG021 AXI DMA  
  `https://docs.amd.com/r/en-US/pg021_axi_dma`
- PG059 AXI Interconnect  
  `https://docs.amd.com/r/en-US/pg059-axi-interconnect`
- PG085 AXI4-Stream Infrastructure  
  `https://docs.amd.com/r/en-US/pg085-axi4stream-infrastructure`
- PG144 AXI GPIO  
  `https://docs.amd.com/r/en-US/pg144-axi-gpio`
- UG1399 Vitis HLS: How AXI4-Stream Works  
  `https://docs.amd.com/r/en-US/ug1399-vitis-hls/How-AXI4-Stream-Works`

### Driver API references
- AXI DMA driver API  
  `https://xilinx.github.io/embeddedsw.github.io/axidma/doc/html/api/index.html`
- AXI DMA driver group  
  `https://xilinx.github.io/embeddedsw.github.io/axidma/doc/html/api/group___a_x_i_d_m_a.html`
- xaxidma.h  
  `https://xilinx.github.io/embeddedsw.github.io/axidma/doc/html/api/xaxidma_8h.html`
- AXI CDMA API  
  `https://xilinx.github.io/embeddedsw.github.io/axicdma/doc/html/api/index.html`
- AXI VDMA API  
  `https://xilinx.github.io/embeddedsw.github.io/axivdma/doc/html/api/index.html`
- AXI Ethernet API  
  `https://xilinx.github.io/embeddedsw.github.io/axiethernet/doc/html/api/index.html`
- GPIO API files  
  `https://xilinx.github.io/embeddedsw.github.io/gpio/doc/html/api/files.html`
- AXI PCIe API  
  `https://xilinx.github.io/embeddedsw.github.io/axipcie/doc/html/api/index.html`

### Supplemental wiki and GitHub references
- AXI DMA standalone driver  
  `https://xilinx-wiki.atlassian.net/wiki/display/A/AXI%2BDMA%2BStandalone%2BDriver`
- AXI GPIO  
  `https://xilinx-wiki.atlassian.net/wiki/display/A/AXI%2BGPIO`
- DMA Drivers - Soft IPs  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/18842337/DMA%2BDrivers%2B-%2BSoft%2BIPs`
- Linux DMA from user space 2.0  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/1027702787/Linux%2BDMA%2BFrom%2BUser%2BSpace%2B2.0`
- AXI Ethernet standalone driver  
  `https://xilinx-wiki.atlassian.net/wiki/display/A/AXI%2BEthernet%2BStandalone%2BDriver`
- Linux AXI Ethernet driver  
  `https://xilinx-wiki.atlassian.net/wiki/display/A/Linux%2BAXI%2BEthernet%2Bdriver`
- Validating a master AXI4 interface using the Verification IP as a slave  
  `https://xilinx-wiki.atlassian.net/wiki/display/A/Validating%2Ba%2Bmaster%2BAXI4%2Binterface%2Busing%2Bthe%2BVerification%2BIP%2Bas%2Ba%2Bslave`
- Axi-Quad SPI  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/575209708/Axi-Quad%2BSPI`
- Xilinx/embeddedsw repository  
  `https://github.com/Xilinx/embeddedsw`
- axidma driver header  
  `https://github.com/Xilinx/embeddedsw/blob/master/XilinxProcessorIPLib/drivers/axidma/src/xaxidma.h`
- axidma driver directory  
  `https://github.com/Xilinx/embeddedsw/tree/master/XilinxProcessorIPLib/drivers/axidma`
- axiethernet driver directory  
  `https://github.com/Xilinx/embeddedsw/tree/master/XilinxProcessorIPLib/drivers/axiethernet`
- axipcie driver header  
  `https://github.com/Xilinx/embeddedsw/blob/master/XilinxProcessorIPLib/drivers/axipcie/src/xaxipcie.h`
- mcdma API docs  
  `https://github.com/Xilinx/embeddedsw/blob/master/XilinxProcessorIPLib/drivers/mcdma/doc/html/api/index.html`

---

## Answering Rules for This Topic

When the user asks how to connect PL output to software:
1. identify whether the data is register-like, frame-like, or stream-like,
2. identify whether the software endpoint is Linux user space, kernel space, or bare-metal style code,
3. match the transport candidate to the data shape,
4. state what must be configured in hardware, address map, driver, and software.

When the user asks about a custom AXI IP:
1. separate control plane from data plane,
2. state whether AXI4-Lite, AXI4, or AXI4-Stream is appropriate,
3. explain how it will be accessed from PS-side software.

---

## Use Adjacent Files Next

Use other files only when the problem clearly shifts:
- **APU ↔ RPU messaging** → `reference_02_openamp_freertos_ethernet_en.md`
- **Vitis software packaging or platform creation** → `reference_05_vitis_platform_software_en.md`
- **AI runtime support rather than custom hardware interface** → `reference_03_vitis_ai_vision_en.md`