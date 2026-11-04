# YOLOv8(계열) on 현재 KV260 보드 via Vitis-AI 3.5 — 적합성 검사

- 작성: 2026-07-09
- 근거 문서: `~/ug1414-vitis-ai-en-us-3.5.pdf` (UG1414 v3.5, 2023-09-28) + 보드 실사
- 목적: **현재 KV260 보드 상태에서 Vitis-AI 3.5를 사용해 YOLOv8 계열(특히 OBB)을 DPU에 올릴 수 있는가**를 판정.
- 결론 요약을 먼저 보고, 근거는 아래 각 절 참조.

---

## 0. 결론 (verdict)

| 층위 | 판정 | 요지 |
|---|---|---|
| **모델/연산자 적합성** | 🟢 **적합** | YOLOv8의 op(SiLU·C2f·head)는 3.5 front-end에서 DPUCZDX8G로 매핑 가능. SiLU→Hardswish **자동 변환**(2.5의 수동 패치 불필요). |
| **호스트 툴체인(데스크톱)** | 🟢 **적합** | 3.5 vai_q_pytorch가 PyTorch 1.4~2.0 지원 → 최신 YOLOv8 학습·양자화 가능. 컴파일은 우리 B3136 fingerprint 타깃 가능. |
| **현재 보드에 바로 배포** | 🔴 **불가 (버전 mismatch)** | 보드 런타임이 **전부 2.5.0**. 3.5로 컴파일한 xmodel은 3.5 런타임을 요구 → 2.5 VART에서 로드 불가. |
| **보드를 3.5로 올려서 배포** | 🟠 **가능하나 대공사 + 리스크** | 보드 Vitis-AI 런타임(VART/XIR/library) + DPU overlay/driver를 3.5로 마이그레이션 필요. **현재 RT-PREEMPT 커널과의 zocl/xrt 호환**까지 얽힘. |

**한 줄 결론**: YOLOv8 자체는 3.5에서 오히려 YOLOv3보다 **깔끔하게** 올라간다(SiLU 자동). 문제는 모델이 아니라 **보드 플랫폼 전체를 2.5→3.5로 이관**해야 한다는 것이며, 이는 RT 커널 작업과 DPU overlay/driver 층에서 충돌하는 별도 프로젝트다. **현재 마일스톤(그리퍼 없음, 물체 위 정지)에는 정당화되지 않음.**

---

## 1. 현재 보드 실사 (측정값)

```
DPU        : DPUCZDX8G_ISA1_B3136, fingerprint 0x101000016010406, IP version v4.0.0
런타임     : libvart-runner 2.5.0 (2022-07-20), target_factory 2.5.0
패키지     : vitis-ai-runtime 2.5.0 / vitis-ai-library 2.5.0
커널       : 5.15.199-rt91-rt-kria (RT-PREEMPT, /sys/kernel/realtime=1)
             ※ 2026-07-09 조사 시점의 첫 RT 빌드. 이후 폐기되고 현재는 -rt-kv260c #11.
               이 문서의 결론(Vitis-AI 2.5 런타임이 3.5 xmodel의 blocker)은 커널과 무관하게 유효.
overlay    : kv260-smartcam (systemd 자동로드), DPU IP v4.0.0
```
→ **보드의 Vitis-AI 스택은 전부 2.5.0.** 이것이 3.5 모델 배포의 근본 제약.

---

## 2. YOLOv8 아키텍처 vs DPUCZDX8G op 지원 (3.5 Table 28)

UG1414 v3.5 **Table 28 (Currently Supported Operators)** — 열 `DPUCZDX8G_ISA1_B4096 (ZCU102/104)` = 우리 B3136과 **동일 ISA1**:

| YOLOv8 구성요소 | 필요 op | DPUCZDX8G_ISA1 지원 | 비고 |
|---|---|---|---|
| Conv+BN+**SiLU** | conv2d + activation | 🟢 (activation: ReLU, LeakyReLU, ReLU6, **Hard-Swish**, Hard-Sigmoid) | SiLU 네이티브 미지원이지만 **3.5가 Hardswish로 자동 변환**(§3) |
| **C2f** 모듈 | Split / Concat / Add / Conv | 🟢 | 표준 op, 전부 지원 |
| Upsample(nearest) / Concat (neck) | resize / concat | 🟢 | |
| Detect head (anchor-free) | conv 출력 | 🟢 (raw conv) | |
| **DFL decode** (softmax 16-bin + box) | softmax | 🟠 | **off-DPU 후처리 권장**(우리 YOLOv3 decode와 동일 패턴). 필요 시 `--replace_softmax`로 hard-softmax 치환 옵션도 존재 |
| **OBB angle** (회전각) | conv 출력 1채널 | 🟢 (raw conv) | 각도 decode는 off-DPU(CPU) 후처리 |

**핵심**: YOLOv8도 우리 YOLOv3처럼 **backbone+neck+head conv만 DPU, decode(DFL/angle)는 ARM 후처리** 패턴이면 op 매핑이 성립한다. 즉 연산자 차원의 blocker는 없다. (활성화 op 집합은 2.5와 3.5가 동일 — DPU 하드웨어 능력은 그대로고, 3.5는 front-end 변환이 더 좋아진 것.)

---

## 3. 3.5 툴체인 적합성 (호스트/데스크톱) — 오히려 유리

2.5에서 우리가 겪은 SiLU 문제(수동 cfg 교체 + `hardswish.py` 배포버그 2건 sed 패치, D11)가 **3.5에선 자동화**된다:

- **SiLU→Hardswish 자동 변환**: 양자화 config에 `"convert_silu_to_hswish"` 존재, 메시지 `QUANTIZER_TORCH_REPLACE_SILU: "SiLU has been replaced by Hardswish"`. sigmoid→hardsigmoid, relu6→relu도 자동. → **YOLOv8의 SiLU를 손 안 대고 처리**.
- **PyTorch 지원 폭**: 3.5 vai_q_pytorch = **PyTorch 1.4~1.13, 2.0** 지원(2.5는 1.2~1.10.2). YOLOv8(ultralytics)은 torch ≥1.8 → **3.5와 호환**(1.13 또는 2.0 핀). 2.5로는 최신 YOLOv8 학습 자체가 어려움.
- **컴파일러**: 여전히 `vai_c_xir -a arch.json`(fingerprint) 방식. `/opt/vitis_ai/compiler/arch`에 arch.json 제공. **우리 B3136 fingerprint를 타깃으로 컴파일 가능**.

→ 데스크톱에서 "YOLOv8 학습 → 3.5 docker에서 양자화 → B3136용 xmodel 컴파일"까지는 **기술적으로 성립하고, 2.5 YOLOv3 경로보다 깔끔**하다.

---

## 4. 배포 blocker — 버전 호환 (이 검사의 핵심)

**3.5로 컴파일한 xmodel은 3.5 런타임을 요구하며, 보드의 2.5 VART에서 로드된다고 볼 수 없다.**

근거·논리:
- xmodel은 **XIR로 직렬화**된 그래프. 보드의 `libxir`/`libvart`(2.5)가 이를 파싱·실행. XIR 포맷/opcode는 메이저 버전 간 변한다.
- 문서에 `QUANTIZER_TORCH_XIR_MISMATCH: "XIR version does not match ..."` 에러 클래스 존재 → XIR 버전 일치가 전제.
- AMD Vitis-AI는 **컴파일러와 런타임을 같은 릴리스로 묶어 배포**하며 교차 버전 혼용을 지원하지 않음(정책). DPU **하드웨어**(ISA1)는 같아 DPU subgraph 명령은 실행 가능할 수 있으나, **xmodel 로딩 계층(VART/XIR)과 CPU subgraph/graph 메타데이터**가 버전에 종속 → 2.5 런타임이 3.5 xmodel을 못 읽을 가능성이 매우 높음.

→ **결론: 현재 보드(2.5)에 3.5 YOLOv8 xmodel을 그대로 올리는 것은 불가.** 보드 런타임을 3.5로 올려야 함.

---

## 5. 보드를 3.5로 올리려면 (필요 작업 + 리스크)

| 항목 | 내용 | 리스크 |
|---|---|---|
| VAI 런타임 교체 | 보드의 `vitis-ai-runtime/library`, `libvart`, `libxir`, `target-factory`를 3.5로 | 중 (패키지/의존성) |
| DPU overlay/IP | 3.5 런타임이 기대하는 DPUCZDX8G IP와 현 overlay(IP v4.0.0)의 호환 확인. 불일치 시 **DPU bitstream(overlay) 재빌드/교체** | **높음** (Vivado/Vitis TRD 재빌드 가능성) |
| **RT 커널 상호작용** | DPU 접근은 `zocl`/`xrt` 커널 모듈 경유 → 이들이 **현재 RT-PREEMPT 커널(5.15.199-rt91)** + VAI 3.5 양쪽과 호환돼야 함 | **높음** (커널 버전 민감. RPU/EtherCAT 트랙과 overlay/driver 층에서 충돌) |
| KV260 3.5 board image | UG1414는 DPUCZDX8G 레퍼런스로 ZCU102/104만 명시(KV260 직접 언급 없음). KV260용 3.5 board setup은 **Kria apps + Vitis-AI board-setup 문서(별도)** 확인 필요 | 중 (문서/이미지 확보) |

→ 이는 "모델 교체"가 아니라 **보드 플랫폼 이관 프로젝트**다. 특히 **RT 커널이 이미 커스텀 빌드**되어 있어 DPU driver(zocl/xrt) 층에서 RT 커널 ↔ VAI 3.5 동시 호환을 새로 잡아야 한다(현재 2.5 스택은 이 RT 커널에서 검증됨).

---

## 6. OBB(회전) 특이사항

- YOLOv8-OBB의 부가 출력은 **각도 1채널(conv)** — DPU엔 raw conv로 올라가고 **각도 decode는 CPU 후처리**. op 자체 blocker는 아님.
- 단 AMD model zoo에 edge DPU용 OBB 기성 예제가 없을 가능성이 커, **decode를 우리가 직접 구현**(YOLOv3에서 한 것과 동종 작업)해야 함.
- **회전정보 대안**: 현재 축정렬 detector로도 긴 물체(banana/mustard)의 방향은 **depth point cloud PCA**로 후단에서 산출 가능. OBB가 회전정보의 유일 경로는 아님.

---

## 7. 노력/효용 판단 & 권고

- **효용**: YOLOv8은 정확도·OBB(회전) 이점. 회전은 grasp planning 단계에서 필요.
- **비용**: 위 §5의 보드 플랫폼 이관(2.5→3.5) + RT 커널 재호환 + (가능성 있는) DPU overlay 재빌드. 현재 검증된 2.5 파이프라인을 리셋.
- **타이밍**: 현재 마일스톤은 "말단부 물체 위 정지"(그리퍼 없음) → **회전각 불필요**. 지금 전환은 (a) 현 문제(도메인 갭)를 안 풀고 (b) 검증 기반을 흔들며 (c) RT 트랙과 충돌.

**권고**:
1. **지금은 전환하지 않음.** YOLOv3-tiny(2.5) 경로로 검출을 완성.
2. OBB/YOLOv8은 **grasp planning 착수 시점**에 재검토. 그때 (a) 회전이 정말 필요한지(PCA로 충분한지) 판단하고, (b) 필요하면 **보드 3.5 이관을 독립 프로젝트로** 계획(RT 커널/overlay 호환 선검증 포함).
3. 만약 조기에 검증만 해보고 싶다면 **저비용 선검증**: 데스크톱 3.5 docker에서 YOLOv8을 우리 B3136 fingerprint로 **컴파일만** 시도 → single DPU subgraph로 떨어지는지 확인(보드 배포 없이 op 매핑만 검증). 실제 실행은 별도 3.5 보드 이관 후.

---

## 8. 검증 필요 항목 (본 문서에서 단정 못 한 것)

1. **[핵심] 3.5 xmodel의 2.5 런타임 로드 실패**를 실측으로 확인(예상: 실패). 저비용: 3.5로 컴파일한 임의 xmodel을 보드에서 `xdputil`/VART로 로드 시도.
2. **DPU IP v4.0.0(현 overlay) ↔ VAI 3.5 기대 IP** 호환 여부 — 3.5 DPUCZDX8G TRD의 IP 버전 확인.
3. **KV260용 Vitis-AI 3.5 board image/overlay** 존재 및 절차 — Kria apps + Vitis-AI 3.5 board-setup 문서(UG1414 범위 밖).
4. **RT-PREEMPT 5.15.199 + VAI 3.5 zocl/xrt** 동시 호환 — 커널 모듈 버전 매트릭스.
5. YOLOv8-OBB의 DFL/angle **off-DPU decode** 실제 single-DPU-subgraph 컴파일 결과(§7-3 선검증).

---

## 9. 근거 (UG1414 v3.5)

- Table 28 "Currently Supported Operators" (p.138 부근): DPUCZDX8G_ISA1 activation = ReLU/LeakyReLU/ReLU6/**Hard-Swish**/Hard-Sigmoid (2.5와 동일).
- 양자화 config `convert_silu_to_hswish`, 메시지 `QUANTIZER_TORCH_REPLACE_SILU`(SiLU→Hardswish 자동), `QUANTIZER_TORCH_XIR_MISMATCH`(XIR 버전 일치 요구).
- PyTorch 지원: vai_q_pytorch 1.2~2.0(본문), 지원표 1.4~1.13/2.0.
- 컴파일러: `vai_c_xir -a arch.json`(fingerprint), arch.json 위치 `/opt/vitis_ai/compiler/arch`.
- `--replace_softmax`(softmax→hard-softmax) 옵션.
- 문서판: UG1414 v3.5, 2023-09-28.
- 보드 실사: `xdputil query`(fingerprint 0x101000016010406, IP v4.0.0), `dpkg`(vitis-ai-runtime/library 2.5.0), `uname -r`(5.15.199-rt91-rt-kria).
