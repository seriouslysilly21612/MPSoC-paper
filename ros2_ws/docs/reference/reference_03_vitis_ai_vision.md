# Reference 03 — Vitis AI, Vision Inference, and Accelerator-Oriented Flow

Use this file when the question is primarily about:
- Vitis AI,
- DPU or accelerator-oriented inference flow,
- model support and deployment,
- Ubuntu vs PetaLinux feasibility for AI workflows,
- choosing Vitis AI versions,
- AI runtime examples and model zoo references.

---

## What This File Covers

This file is the primary lookup target for:
- whether a vision model can run on KV260 through an AMD/Xilinx-supported path,
- which Vitis AI version is most appropriate,
- which runtime or tutorial flow best matches the board and OS,
- what official AI examples exist,
- whether Ubuntu-based usage is practical or if PetaLinux is required.

---

## Scope Boundary

This file is about **AI/inference workflow**.

Do not start from generic Vivado or AXI references unless the question is explicitly about:
- building custom PL around the AI block,
- integrating a custom IP around the data path,
- PS↔PL transport separate from the AI runtime itself.

---

## Recommended Lookup Order

1. Vitis AI official docs portal and UG1414
2. Vitis AI workflow / release docs
3. Vitis AI GitHub repositories
4. Vitis AI wiki pages
5. AMD official forum discussions
6. Adjacent-topic Vitis or MPSoC references only if the AI docs are insufficient

---

## Decision Hints

### When the user asks "Can I use Ubuntu instead of PetaLinux?"
Evaluate in this order:
1. Is there an official Ubuntu-oriented flow or sample for the requested AI use case?
2. Does the target example depend on a certified Ubuntu image or a PetaLinux-specific runtime stack?
3. Is the user asking about a proof-of-concept, or a supported deployment path?
4. Does the requested model depend on a DPU/TRD path that is board- and version-sensitive?

### When the user asks "Which Vitis AI version should I use?"
Do not choose the newest version by default. Compare:
- board support,
- runtime support,
- OS support path,
- example/tutorial alignment,
- documentation completeness,
- integration burden with the user's actual project.

### When the user asks about model choice
Separate:
- **what Vitis AI supports officially**,
- **what a specific board image or TRD supports**,
- **what is practical for the user's camera, frame rate, and robotics pipeline**.

---

## Primary References

### Official Vitis AI references
- AMD developer resource page  
  `https://www.amd.com/en/developer/resources/vitis-ai.html`
- Vitis AI documentation portal  
  `https://vitisai.docs.amd.com/`
- UG1414 Vitis AI  
  `https://docs.amd.com/r/en-US/ug1414-vitis-ai`
- AMD Vitis AI product page  
  `https://www.amd.com/en/products/software/vitis-ai.html`

### Workflow and version references
- Vitis AI main site  
  `https://xilinx.github.io/Vitis-AI/`
- Vitis AI 3.0 docs  
  `https://xilinx.github.io/Vitis-AI/3.0/html/index.html`
- Vitis AI 3.5 workflow  
  `https://xilinx.github.io/Vitis-AI/3.5/html/docs/workflow.html`
- Vitis AI 3.0 release documentation  
  `https://xilinx.github.io/Vitis-AI/3.0/html/docs/reference/release_documentation.html`

### Wiki references
- Vitis Unified Software Platform wiki page  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/297009243/Vitis%2BUnified%2BSoftware%2BPlatform`
- Building Vitis AI sample applications on certified Ubuntu 20.04 LTS  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/2072838191/Building%2BVitis-AI%2BSample%2BApplications%2Bon%2BCertified%2BUbuntu%2B20.04%2BLTS%2Bfor%2BXilinx%2BDevices`
- Snaps - xlnx-vai-lib-samples snap for certified Ubuntu  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/2068283453/Snaps%2B-%2Bxlnx-vai-lib-samples%2BSnap%2Bfor%2BCertified%2BUbuntu%2Bon%2BXilinx%2BDevices`
- Vitis-AI 3.0 DPU TRD for QNX 7.1  
  `https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/2710831105/Vitis-AI%2B3.0%2BDPU%2BTRD%2Bfor%2BQNX%2B7.1`

### Official GitHub references
- Xilinx/Vitis-AI repository  
  `https://github.com/Xilinx/Vitis-AI`
- Vitis-AI releases  
  `https://github.com/Xilinx/Vitis-AI/releases`
- Vitis-AI-Tutorials  
  `https://github.com/Xilinx/Vitis-AI-Tutorials`
- VAI runtime examples README  
  `https://github.com/Xilinx/Vitis-AI/blob/master/examples/vai_runtime/README.md`
- AI Model Zoo  
  `https://github.com/xilinx/ai-model-zoo`
- Vitis-AI Copyleft Model Zoo  
  `https://github.com/Xilinx/Vitis-AI-Copyleft-Model-Zoo`

---

## Answering Rules for This Topic

When the user asks whether a specific AI deployment path is supported:
1. identify the target OS,
2. identify the target version family,
3. identify whether the user needs official support, a workaround, or only a proof-of-concept,
4. state the difference between runtime feasibility and officially documented support.

When the user asks for a practical recommendation:
1. compare the official path first,
2. explain what is easier to integrate with the user’s robotics pipeline,
3. state what is lost if a less-official path is chosen.

---

## Use Adjacent Files Next

Use other files only when the question clearly shifts:
- **Vivado / AXI / PL-side custom integration** → `reference_04_vivado_axi_pl_en.md`
- **Vitis platform creation / software packaging** → `reference_05_vitis_platform_software_en.md`
- **ROS2 and camera pipeline integration** → `reference_06_ros2_camera_pipeline_en.md`
- **General board feasibility** → `reference_01_kria_core_architecture_en.md`