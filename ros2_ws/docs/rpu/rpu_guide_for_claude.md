# RPU 작업 가이드 — Claude 세션용 운영 문서

> **대상 독자: Claude (Opus 포함 모든 모델의 새 세션).**
> 목적: 이전 대화 기록 없이 이 문서만으로 RPU FreeRTOS + SOEM 작업을 막힘없이 이어가게 하는 것.
> 작성: 2026-07-08 (Fable, 보드 실사 기반) / **최종 재검증·갱신: 2026-09-04, 커널 `5.15.199-rt91-rt-kv260c` 기준.** 아래 "검증된 사실"은 전부 이 보드에서 명령으로 직접 확인된 것.
> **현재 트랙 상태 (2026-09-04)**: **Gate 1 통과 (작업 2026-09-01~02, 펌웨어 부팅 검증 2026-09-02)** — R5F-0에 FreeRTOS elf 부팅 성공(`remote processor ff9a0000.rf5ss:r5f_0 is now up`, 2026-09-02 11:02 / 15:01 두 차례), **판정 수단은 UART/trace가 아니라 LED 점멸**(`led35_task`/`led36_task`, 사용자 육안 확인). 적용 방식은 configfs가 아니라 **부팅 DTB 병합(`/boot/firmware/user-override.dtb`)**.
> **남은 3과제**: ① APU↔RPU 통신(rpmsg) ② RAON-RT 실시간 제어 task의 RPU 이식 ③ EtherCAT SOEM의 RPU 설치. ①②는 GEM3와 무관해 **지금 진행 가능**, ③은 빌드까지만 가능하고 NIC 실동작(Gate 2/3)은 **GEM3 이관 = 사용자 재결정 전 금지** — APU IgH가 Indy7 실운용에 사용 중.
> **앱 전환 반영**: 이 트랙의 최종 목표는 pick & place가 아니라 **(a) 테이블 위 물체 위로 TCP 접근 + (b) 사람이 물체를 옮기면 TCP가 추종**이다. 파지·이송·놓기는 범위 밖(그리퍼 하드웨어 미보유). 코드/토픽/패키지 이름의 `pick_*`은 pick&place 시절 legacy.

---

## 0. 필독 순서

1. **이 파일 전체** — 특히 §1(세션 환경)과 §4(금지 사항)는 작업 전 반드시 숙지
2. `~/ros2_ws/docs/rpu/rpu_plan.md` — 전체 논리(왜 이 순서인가, 포팅 난제 분석)
3. `~/ros2_ws/docs/rpu/rpu_freertos_soem_execution_plan.md` — Gate 기준·리스크 표
4. `~/ros2_ws/CLAUDE.md` — 프로젝트 전체 규칙 (한국어 답변, 학부 수준 설명, source priority)
5. 질문 주제별 공식 문서 링크: `~/ros2_ws/docs/reference/reference_02_openamp_freertos_ethernet.md`
6. 지속 메모리: `kria-rt-preempt-project.md` (APU IgH 트랙과의 관계 포함)
7. APU 쪽(RAON-RT/IgH) 맥락이 필요하면: `~/ros2_ws/docs/RAON-RT/merge.md` (정본 로그 — RPU+SOEM은 §9 백로그 항목)

---

## 1. 세션 환경 — 가장 먼저 인지할 것

**Claude Code는 KV260 보드 그 자체에서 실행 중이다.** (`uname -r` → `5.15.199-rt91-rt-kv260c`, PREEMPT_RT, hostname `kria`)

이것이 의미하는 것:
- 보드 상태를 추측하지 말 것. `/proc/device-tree`, `/sys`, `dmesg`로 직접 확인 가능하다.
- **원격 = AX88179B USB Ethernet NIC(`enx…`, 드라이버 `ax_usb_nic` v4.1.0, `192.168.120.50/24`) 경유 SSH. Tailscale은 제거됨.**
  eth0(GEM3)은 더 이상 원격 생명줄이 아니다 — 대신 **IgH EtherCAT 예약 포트**다(아래 참조). USB NIC를 죽이는 명령(`ip link set enx… down`, `ax_usb_nic` 언로드, USB 리셋)이 이제 세션 즉사 경로다.
- **eth0(GEM3)의 현재 상태**: 평시엔 macb 소유·IP 없음(유휴). RAON-RT 실기 세션 때 IgH(`ethercat.service` 기동)가 점유해 Indy7을 제어한다. **RPU가 GEM3를 가져가는 Phase 2+는 사용자 재결정 전 착수 금지.**
- **로봇 실기 세션 중 금지(E34)**: RAON-RT 실기 구동 중에는 보드에서 무거운 작업(빌드, 대량 IO, 에이전트 다중 실행) 금지 — 1kHz 제어 지터를 오염시킨 전례가 있다. RPU 작업은 실기 세션 밖에서.
- **sudo는 비밀번호 필요** (passwordless 아님). sudo가 필요한 단계(dmesg, iomem, overlay 적용, modprobe)는 사용자에게 실행을 요청하거나 비밀번호 입력을 받는다.
- Vitis는 이 보드에서 실행 불가(aarch64). **모든 R5 elf 빌드는 사용자의 x86 PC에서** 이루어진다. Claude는 소스/링커스크립트/DT를 작성해 주고, 빌드는 사용자에게 절차를 안내하는 분업 구조다. elf 전송은 USB NIC 경유 scp — GEM3 상태와 무관하다.
- 사용자 작업 스타일: 일일이 허락 묻지 말고 자율 진행하되, **연결 단절·비가역 작업만은 사전 고지**. 답변은 한국어(기술용어는 영어), 학부생 수준에서 원리부터 설명.

**원격 작업 온보딩 (새 세션 첫 5분)**:
```bash
uname -r                                   # 5.15.199-rt91-rt-kv260c 인지 확인 (다르면 §3 전체 재검증)
ip -br link; ip route | head -3            # 원격 경로가 enx…(USB NIC)인지, eth0 상태 확인
systemctl is-active ethercat               # active면 로봇 세션 가능성 → 사용자에게 실기 여부 확인 후 작업
ls /sys/class/remoteproc/                  # remoteproc0 = ff9a0000.rf5ss:r5f_0 (부팅 DTB에 병합됨 — 항상 있어야 정상)
cat /sys/class/remoteproc/remoteproc0/state  # offline/running. firmware 이름은 재부팅마다 기본값으로 리셋됨(§6-C)
ls /sys/kernel/config/device-tree/overlays/  # kv260-smartcam_image_1 뿐 — RPU는 configfs를 쓰지 않는다
cat /sys/devices/system/cpu/isolated       # 3 — CPU3는 RAON-RT 전용, 침범 금지
```

---

## 2. 확정 결정사항 — 재논의 금지

| 결정 | 내용 | 근거 |
|---|---|---|
| 실행 코어 | R5F-**0**, **split** 모드 | lockstep 불필요, 코어1은 추후 여유 |
| RTOS | FreeRTOS 10 (Vitis BSP `freertos10_xilinx`) | Kria 공식 지원 경로 |
| EtherCAT master | **SOEM** v1.4.x (RPU) / IgH는 APU 전용 | SOEM만이 RTOS 포팅 가능 구조 |
| Vitis 버전 | **2022.1** | kria-apps-docs 튜토리얼 기준, 커널 5.15 세대 일치. **설치 완료 (Phase 0 종료)** — 사용자 x86 PC의 **Windows** 설치(`d:\xilinx\vitis\2022.1`), GCC 11.2.0 arm-none-eabi. 근거: `strings ~/rpu_work/r5_freertos.elf` |
| 빌드 워크스페이스 | XSA `design_1_wrapper`(자체 Vivado BD export), 도메인 `psu_cortexr5_0`, OS `freertos10_xilinx_v1_11` | 위 elf strings에서 확인. BSP 라이브러리: `gpiops_v3_9`, `scugic_v4_7`, `ttcps_v3_15`(Phase 4 µs 시계 재료), `standalone_v7_7`, `xiltimer_v1_0` |
| RPU 로드 | Linux **remoteproc** (+ 안정화 후 systemd 자동화) | Kria는 BOOT.BIN 수정이 비표준(부트펌웨어 QSPI 고정) |
| 코드 배치 | DDR 예약 영역 링크, 지연민감 코드만 TCM | SOEM+제어 코드가 TCM 128KB 초과 |
| DMA 버퍼 | 처음엔 **non-cacheable** (R5 MPU) | 캐시 일관성 버그 원천 차단, 최적화는 동작 후 |
| SOEM 수신 | 처음엔 **폴링**, 이후 인터럽트+큐 | 단순한 것부터 검증 |
| 작업 순서 | Phase 0~1 완료. **다음은 GEM3 무관한 ①rpmsg(Gate 1.5) → ②제어 task 이식(Gate 4a) → ③SOEM 빌드(Gate 3-0)** (2026-09-04). Phase 2+(GEM3 이관)는 여전히 별도 사용자 결정 필요 | APU IgH는 "완료"가 아니라 **실운용 중**(RAON-RT가 Indy7 제어) — GEM3를 가져가면 그동안 로봇 운용 중단. 이관 시 APU 제어 중단 합의 + macb rebind 복구 절차 필수 |
| 대상 로봇 | Indy7, STEP 우회 드라이브 직결, **CST(`DRIVE_MODE=10`)** — merge.md D1 확정(옛 CiA402+CSP 서술은 낡음) | APU에서 IgH+RAON-RT로 실제 제어 성공·운용 중 → Phase 4는 **IgH→SOEM 번역 이식** (`~/RAON-RT/EMasterApp/Device/EcatCommon.h`가 IgH 타입 사용) |
| 테스트 슬레이브 | LS Mecapion **L7N** 서보 + 모터 + 전원 보유 | 문서 PDF는 사용자에게 요청 → `~/ros2_ws/docs/l7n/` (2026-09-04 현재 미수령) |
| 범위 | 그리퍼 제외(하드웨어 미보유). **말단부가 물체 위에서 정지 + 사람이 물체를 옮기면 추종**. **주기 1kHz 확정** | 앱 전환(2026-08-03, RAON-RT `'o'` 추종 구현). RAON-RT APU 트랙이 1kHz로 실운용하며 실증 (2026-07~08) |

---

## 3. 검증된 보드 사실 + 재검증 명령

작업 시작 시(특히 재부팅/커널 업데이트 후) 아래를 다시 실행해 상태가 유지되는지 확인하라.
**2026-08-03에 RT 커널(`5.15.199-rt91-rt-kv260c`) 기준으로 전 항목 재검증 완료** — 7/8의 우려(커스텀 커널에서 remoteproc 지원 소실)는 발생하지 않았다. 커널 재빌드 불필요.
**2026-09-04 재실사**: Gate 1 통과로 DT·remoteproc·예약 메모리 3개 행이 바뀌었다(아래 ★ 표시). 나머지는 불변.

| 사실 | 검증 명령 | 결과 (RT 커널, 2026-08-03 / ★는 2026-09-04) |
|---|---|---|
| remoteproc 드라이버 모듈 존재, vermagic 일치, **binding = `xlnx,zynqmp-r5-remoteproc`** (Xilinx 5.15 벤더 binding — upstream 6.x의 `zynqmp-r5fss`와 다름!) | `modinfo /lib/modules/$(uname -r)/kernel/drivers/remoteproc/zynqmp_r5_remoteproc.ko \| grep alias` | 존재·alias 확인. 커널 config: `ZYNQMP_R5_REMOTEPROC=m`, `REMOTEPROC=y`, `ZYNQMP_IPI_MBOX=y` |
| rpmsg 스택 완비 | `ls /lib/modules/$(uname -r)/kernel/drivers/rpmsg/` | rpmsg_core/char/ns/virtio 전부 존재 (`RPMSG*=m`) |
| ★ **DT에 R5 노드 존재** — `rf5ss@ff9a0000`(+`r5f_0`), `tcm@ffe00000`, `tcm@ffe20000`가 **부팅 DTB에 병합**되어 있다 | `ls /proc/device-tree/rf5ss@ff9a0000/`; `ls /sys/class/remoteproc/` | `remoteproc0` = **`ff9a0000.rf5ss:r5f_0`**, state `offline`(펌웨어 기동 전 정상). `lsmod \| grep zynqmp_r5` → 자동 로드·자동 probe (**`modprobe` 수동 불필요**) |
| ★ RPU child에 **`mboxes`/`mbox-names` 없음** — rpmsg 선행조건 미충족 | `ls /proc/device-tree/rf5ss@ff9a0000/r5f_0/` | `compatible, memory-region, power-domain, ranges, sram, #a/#s-cells`만. **Gate 1.5의 DT 재작업 항목** |
| configfs overlay 동작 (`OF_OVERLAY=y`, `OF_CONFIGFS=y`) | `ls /sys/kernel/config/device-tree/overlays/` | `kv260-smartcam_image_1` applied — RT 커널에서 실증. **단 RPU는 이 경로를 쓰지 않는다**(§6-B) |
| GEM3만 활성 (ff0e0000, macb, eth0). **평시 유휴(IP 없음), IgH 세션 시 점유** | `for g in ff0b ff0c ff0d ff0e; do cat /proc/device-tree/axi/ethernet@${g}0000/status; done`; `ip -br addr show eth0` | disabled×3/okay. eth0 UP·IP 없음. IgH kmod는 `/lib/modules/$(uname -r)/ethercat/`에 이 커널용으로 빌드되어 있고 `ethercat.service`는 필요 시 기동 |
| 원격 = USB NIC | `ip route \| head -3` | default via 192.168.120.1 dev `enx…` (ax_usb_nic v4.1.0) |
| UART0(ff000000) **disabled**, UART1(ff010000)=Linux 콘솔(ttyPS1) | `cat /proc/device-tree/axi/serial@ff000000/status` | disabled (변화 없음) |
| 기존 IPI: mailbox@ff9905c0, `xlnx,ipi-id = <4>` (충돌 금지) | `od -An -tx1 /proc/device-tree/zynqmp-ipi/mailbox@ff9905c0/xlnx,ipi-id` | 00 00 00 04 |
| ★ reserved-memory = `pmu@7ff00000` + **`rproc@36000000` (0x36000000 + 0x400000 = 4 MiB, `no-map`)** | `ls /proc/device-tree/reserved-memory/`; `od -An -tx1 /proc/device-tree/reserved-memory/rproc@36000000/reg` | `00000000 36000000 00000000 00400000`. **vring/buffer 노드는 없음** — Gate 1.5에서 이 4 MiB 창 안에 추가 |
| RAM 3.8GiB, CMA 1000M = **0x36400000–0x74C00000** (carve-out 바로 위에 안착 — 밀려난 것이 아님) | `journalctl -k \| grep cma`; `grep -i cma /proc/meminfo` | `cma: Reserved 1000 MiB at 0x0000000036400000`. CmaFree는 시점에 따라 변동 폭이 매우 크다(수 MB ~ 수백 MB 수준 — 08-03엔 ~340-400MB, 09-04 재측정 시엔 수 MB대. 특정 수치는 측정 시각과 함께 적을 것) |
| cmdline: `clk_ignore_unused`, `cma=1000M` + **RT 격리 파라미터 추가됨** | `cat /proc/cmdline` | `skew_tick=1 isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2` — **CPU3는 RAON-RT 1kHz 전용, RPU 작업이 CPU3에 부하 금지** |
| ★ 부팅: U-Boot + flash-kernel `image.fit`, **그리고 boot script가 `user-override.dtb`를 지원** — 이 훅이 RPU DT의 실제 적용 경로다 | `strings /boot/firmware/boot.scr.uimg \| grep -A3 user-override` | `if test -e … /user-override.dtb; then … load … ${dtb_addr} user-override.dtb`. **파일이 없으면 FIT의 순정 DTB로 폴백** = 복구 경로 |
| 모듈 서명 강제 없음 (OOT 모듈 로드 자유) | `cat /sys/kernel/security/lockdown` | `[none]` |

**RPU 예약 메모리 — 확정값 (2026-09-02 적용, 실측)**: **`0x36000000` + 4 MiB, `no-map`**. 펌웨어 ELF는 `.text @ 0x36000000`(LOAD 0x36000000, MemSiz 0x13600 ≈ 78 KB 사용) + 두 번째 LOAD `@0x00000000`(0x628, `.vectors`+`.bootdata` = TCM ATCM).

⚠️ **문서가 오래 "표준 관례"로 적어 둔 0x3ed00000은 이 보드에서 쓰면 안 된다** — CMA 창(0x36400000–0x74C00000) **한복판**이다. runtime configfs 경로로 그 주소를 잡았다면 살아 있는 CMA를 침범했을 것이다. 확장이 필요하면 4 MiB carve-out **안에서** 해결하거나(현재 78 KB만 사용 — vring/buffer를 넣을 여유 충분) **아래쪽으로**(0x35C00000 등) 늘리고 재검증할 것. 위로 늘리면 0x36400000에서 CMA와 충돌한다.

---

## 4. 금지·주의 사항 (Opus가 세션을 망치는 지름길들)

1. **USB NIC(`enx…`)를 건드리는 명령 = 원격 세션 즉사** — 절대 금지. **eth0 관련 명령도 사전 고지 없이 실행 금지** (IgH EtherCAT 예약 포트 — 로봇 트랙과 충돌). unbind 복구는 `echo ff0e0000.ethernet > /sys/bus/platform/drivers/macb/bind` 또는 재부팅.
1-b. **RAON-RT 실기 세션 중에는 RPU 작업 자체를 보류** (E34 — 보드 부하가 1kHz 지터 오염). `systemctl is-active ethercat`이 active면 사용자에게 실기 여부부터 확인.
2. **QSPI 부트펌웨어(`xmutil bootfw_*`)를 건드리지 말 것.** 이 프로젝트에 불필요하며 잘못되면 복구가 어렵다.
3. **kv260-smartcam overlay를 내리지 말 것** (`xmutil unloadapp` 금지). perception 파이프라인(DPU)이 그 위에서 돈다. RPU 작업과 충돌하지 않는다.
4. `/boot/firmware/image.fit` 직접 수정 금지. DT를 boot-time에 바꿀 땐 §6-B의 `user-override.dtb` 절차만 사용. **교체 전 현행본 백업 필수**(`sudo cp /boot/firmware/user-override.dtb /boot/firmware/user-override.dtb.ok`) — 잘못 만들면 부팅 실패다. 최후 복구는 SD를 PC에 꽂아 `user-override.dtb`를 지우는 것(boot script가 FIT의 순정 DTB로 폴백).
4-b. **`user-override.dtb`는 누적되지 않는다.** DT를 고칠 때마다 항상 `base.dtb`에서 다시 `fdtoverlay` 병합할 것 — 기존 `user-override.dtb` 위에 덧씌우면 안 된다.
4-c. **DT 변경 = 재부팅 1회 = 리스크.** RAON-RT 실기 일정과 반드시 분리하고, `systemctl is-active ethercat`으로 세션 중이 아님을 먼저 확인.
5. **RPU reserved-memory를 runtime configfs overlay로만 잡은 채 실사용 금지** — 이 원칙은 지켜졌다(2026-09-02에 boot-time `user-override.dtb`로 전환 완료). 향후 DT 실험을 configfs로 빠르게 돌리는 것은 가능하나, 본 운용은 반드시 boot-time 반영 상태에서.
6. 이 커널의 remoteproc binding은 **Xilinx 5.15 벤더 형식**이다. 웹에서 찾은 upstream 6.x용 DT 예제(`xlnx,zynqmp-r5fss`, `compatible = "xlnx,zynqmp-r5f"`)를 그대로 쓰면 probe되지 않는다. 반드시 §6-A의 방법으로 이 커널 소스의 예제를 기준으로 삼을 것.
7. FreeRTOS elf에 **resource table 섹션이 없으면 rpmsg·trace buffer를 못 쓴다.** 실측 정정(2026-09-02): 로드가 **실패하지는 않는다** — 커널이 `no resource table found.` 경고만 내고 `is now up`, LED도 정상 동작했다(현 `r5_freertos.elf`가 정확히 그 상태: `readelf -S`에 `.resource_table` 없음, `nm | grep -ci 'rpmsg\|openamp'` = 0). 다만 **잔여 과제 ①(rpmsg)의 선결조건**이므로 Gate 1.5에서는 OpenAMP echo-test 베이스로 재빌드해야 한다.
8. 사용자에게 물어봐도 되는 것: L7N PDF 요청, **GEM3 이관 시점 합의(Phase 2 전제 — 로봇 운용 중단 수반)**, 실기 세션 일정, DT 변경용 재부팅 시점. 그 외 기술 판단은 자율 진행. (master 종류=IgH, 주기=1kHz, Vitis 설치 완료는 이미 확정 — §2 참조, 다시 묻지 말 것)

---

## 5. 문서·파일 지도

| 경로 | 내용 |
|---|---|
| `~/ros2_ws/docs/rpu/rpu_plan.md` | 마스터 계획 (논리 전개) |
| `~/ros2_ws/docs/rpu/rpu_freertos_soem_execution_plan.md` | 실행 절차·Gate·리스크 |
| `~/ros2_ws/docs/rpu/rpu_guide_for_claude.md` | 이 문서 |
| `~/rpu_work/` | ★ **Gate 1에서 실제로 쓰인 산출물이 전부 여기 있다** — `rpu_v1.dtso`(configfs 시험용), **`rpu_v2.dtso`(적용본)**, `base.dtb`, `user-override.dtb`, `r5_freertos.elf`. **git 미추적**(저장소 밖) — 정본화 여부는 미결 |
| `~/ros2_ws/tools/rpu/` | `rpu_rproc.dtso`, `rpu_overlay_apply.sh` — **작성만 되고 적용되지 않았다**(주소 0x3ed00000·configfs 전제). vring 이름 규칙(§6-A 계약 3)과 IPI 노드 구성은 Gate 1.5에서 **주소만 0x36000000 창으로 바꿔** 재사용 가치가 있다 |
| `/boot/firmware/user-override.dtb` | ★ 실제 적용된 RPU device tree (boot script가 로드). 백업은 `.ok` 접미로 |
| `/lib/firmware/r5_freertos.elf` | ★ 현재 RPU 펌웨어 (505,088 B, `~/rpu_work/` 사본과 md5 동일) |
| `~/ros2_ws/docs/l7n/` | L7N 드라이브 문서 (Phase 3 때 사용자로부터 수령 — 아직 없음) |
| `~/ros2_ws/src/apu_rpu_bridge_pkg/` | **삭제됨(2026-07-15 정리, `0313586`) — 재생성 안 함(2026-09-04)**: APU 측 브리지는 RAON-RT `CROS2PickBridge` |
| `~/ros2_ws/docs/RAON-RT/merge.md` | APU 트랙(IgH+RAON-RT) 정본 로그 — Phase 4 이식 원본 코드는 `~/RAON-RT/` |
| `/lib/firmware/` | RPU elf 배치 위치 |
| `/sys/kernel/config/device-tree/overlays/` | runtime overlay 적용 지점 — **smartcam 전용, RPU는 미사용** |
| `~/ros2_ws/docs/reference/reference_02_*.md` | Kria FreeRTOS/OpenAMP 공식 링크 모음 |

**진행 기록 규칙**: Phase/Gate 달성·중요 발견·결정 변경 시 ①execution_plan의 해당 섹션 갱신 ②지속 메모리 `kria-rt-preempt-project.md` 갱신. 사용자 확인 사항은 날짜와 함께 기록.

---

## 6. Phase 1 상세 런북 — remoteproc + FreeRTOS

✅ **Gate 1 통과 (2026-09-02)**

**아래는 실제로 통했던 절차다** (bash history + `journalctl` 교차 검증). DT를 다시 손댈 때(= 잔여 과제 ① rpmsg) 그대로 재사용한다.

```bash
# [1] 베이스 DTB 확보 (__symbols__ 보유 = fdtoverlay 병합 가능)
dtc -I dtb -O dts /usr/lib/linux-image-$(uname -r)/xilinx/smk-k26-revA-sck-kv-g-revB.dtb | grep -c __symbols__
cd ~/rpu_work && cp /usr/lib/linux-image-$(uname -r)/xilinx/smk-k26-revA-sck-kv-g-revB.dtb base.dtb

# [2] overlay 소스 작성 → 컴파일 → 오프라인 병합 (항상 base.dtb에서 새로 — §4-4b)
dtc -@ -O dtb -o rpu_v2.dtbo rpu_v2.dtso
fdtoverlay -i base.dtb -o user-override.dtb rpu_v2.dtbo          # exit 0 확인

# [3] 병합 결과 검증
dtc -I dtb -O dts user-override.dtb | grep -nE 'reserved-memory|rproc@36000000|no-map'
dtc -I dtb -O dts user-override.dtb | grep -nA14 'zynqmp-r5-remoteproc'
for p in 'zynqmp-r5-remoteproc' '"xilinx,r5f"' 'rproc@36000000'; do \
  echo -n "$p : "; dtc -I dtb -O dts user-override.dtb | grep -c "$p"; done   # 셋 다 1

# [4] 안전 게이트 + 배포 + 재부팅
systemctl is-active ethercat                                      # inactive 확인 (실기 세션 아님)
sudo cp /boot/firmware/user-override.dtb /boot/firmware/user-override.dtb.ok  # 있으면 백업
sudo cp user-override.dtb /boot/firmware/user-override.dtb && sync
sudo reboot

# [5] 재부팅 후 — configfs overlay도 modprobe도 불필요, 자동 등장
cat /sys/class/remoteproc/remoteproc0/name                        # ff9a0000.rf5ss:r5f_0
```

이후 펌웨어 로드는 §6-C.

### 6-A. DT overlay — **작성 완료 (2026-08-03), 파일 준비됨**

⚠️ **정정 (2026-09-04)**: 실제로 적용된 것은 여기 준비돼 있던 `tools/rpu/rpu_rproc.dtso`가 아니라 **`~/rpu_work/rpu_v2.dtso`**이고, 방식도 configfs가 아니라 **부팅 DTB 병합**이다.

| 파일 | 상태 |
|---|---|
| `~/rpu_work/rpu_v2.dtso` | ★ **실제 적용본.** rf5ss/r5f_0 + TCM 2개 + `reserved-memory/rproc@36000000`(4 MiB, no-map). **mboxes·vring·buffer·IPI 노드는 없다** |
| `~/rpu_work/rpu_v1.dtso` | configfs 시험용 초기본. reserved-memory 없음(= 예약 안 된 DDR에 펌웨어를 얹는 상태) → 그래서 v2+부팅 DTB로 승격. **실패해서 버린 것이 아니다** — v1로도 enumeration(09-01 12:57)과 펌웨어 부팅(09-02 11:02) 모두 성공했다 |
| `~/ros2_ws/tools/rpu/rpu_rproc.dtso` | 미적용(0x3ed00000·configfs 전제). vring 이름 규칙·IPI 노드 구성만 Gate 1.5에서 주소 바꿔 재사용 |

*(추정, 단정 불가)*: 09-01 12:19의 첫 probe 실패(`could not get R5 core platform device`)는 overlay 적용과 모듈 로드 **순서** 문제로 보이나 로그만으로는 확정 불가.

overlay 구조는 **이 커널의 `zynqmp_r5_remoteproc.ko` 바이너리 strings + linux-xlnx `xlnx_rebase_v5.15` 드라이버 소스 대조로 확정** (2026-08-03). 수정할 일이 생기면 아래 드라이버 계약을 지킬 것:

1. **cluster 노드**: `compatible = "xlnx,zynqmp-r5-remoteproc"`, `xlnx,cluster-mode = <1>`(u32, dtso 원값. **1이 split인지 lockstep인지는 이 보드에서 미확인** — 보드에 커널 소스가 없어 대조 불가이고, 상류 바인딩 관례는 오히려 0=split이다. 값만 그대로 재사용할 것). reg 불필요. 드라이버가 `devm_of_platform_populate`로 child를 platform device화하므로 simple-bus 불필요.
2. **child(r5f_0)에 실리는 property**: `sram`(TCM phandle 목록), `memory-region`, **`power-domain`(단수형! 표준 `power-domains` 아님)** = `<&zynqmp_firmware 7>`(7=NODE_RPU_0), `mboxes`+`mbox-names = "tx","rx"`.
3. **reserved-memory child 이름 함정**: 드라이버가 `strstr(name,"vdev0vring")` 후 **`name[14]`에서 vring 번호를 읽는다** — 이름은 반드시 `rpu0vdev0vring0`/`rpu0vdev0vring1`(15자, 마지막이 번호), 버퍼는 `rpu0vdev0buffer`(`compatible="shared-dma-pool"` 필수). 이름을 바꾸면 조용히 고장난다.
4. **TCM**: global 주소(0xffe00000/0xffe20000, 각 64KB)로 기술 — R5-view(0x0/0x20000) 변환·PM 요청은 드라이버가 수행.
5. **IPI**: 새 `xlnx,zynqmp-ipi-mailbox` 노드(APU agent id 7, child mailbox id 1=RPU0, 버퍼 ff990600/ff9900c0 세트) — 기존 base DT의 id 0/4와 충돌 없음.

**메모리 배치 (링커 스크립트와 동기 필수) — 실제값**: 단일 carve-out **0x36000000 + 4 MiB(no-map)**, 그 안에서 펌웨어가 `.text @0x36000000`부터 ~78 KB만 쓰고 있다. 벡터/부트데이터는 TCM ATCM(R5 view 0x0).

**Gate 1.5(rpmsg) 확장안** — 4 MiB 창 안에서 해결되므로 CMA를 건드리지 않는다:
```
0x36000000 + 256KB : 펌웨어 (현재 78KB 사용)
0x36040000 / 0x36044000, 각 16KB : rpu0vdev0vring0 / rpu0vdev0vring1
0x36048000 + 1MB   : rpu0vdev0buffer (shared-dma-pool)
```
⚠️ 옛 배치(0x3ed00000 계열)는 **폐기** — CMA 한복판이다(§3 경고). SOEM 단계에서 펌웨어가 커지면 carve-out을 **아래로** 확장(0x35C00000 등) + lscript 동기 수정 + 재검증.

### 6-B. 적용 — 두 경로의 원리와 사용 시점

**결론 (2026-09-02 확정): boot-time 경로를 채택했다.**

**채택 경로: `user-override.dtb` (U-Boot boot script 훅)** — 위 [1]~[5] 절차. `flash-kernel`도 `image.fit` 재생성도 **필요 없었다**(과거 문서의 flash-kernel 절차는 폐기). boot script가 SD 부트파티션의 `user-override.dtb`를 있으면 로드하고 없으면 FIT의 순정 DTB로 폴백하므로, **파일 하나 두고 빼는 것이 곧 적용/복구**다.

**왜 configfs를 버렸나**: `reserved-memory`는 원래 **부팅 초기(memblock 단계)에만 예약**된다. runtime overlay로 추가한 예약 영역은 커널이 이미 그 페이지를 다른 용도로 쓰고 있을 수 있다 → 짧은 hello 실험은 대개 통과하지만(v1이 실제로 통과했다) **본 운용에서 무작위 메모리 오염 리스크**. 게다가 이 보드에서 옛 관례 주소(0x3ed00000)를 configfs로 잡았다면 살아 있는 CMA를 침범했을 것이다.

**참고(현재 미사용)**: `~/ros2_ws/tools/rpu/rpu_overlay_apply.sh`의 configfs apply/remove/status 경로는 남아 있다. DT 문법·probe를 재부팅 없이 빠르게 반복 검증하고 싶을 때만 쓰고, 실사용은 반드시 boot-time 상태에서.

### 6-C. 펌웨어 로드·실행·로그

```bash
echo stop | sudo tee /sys/class/remoteproc/remoteproc0/state   # running이면 먼저 정지
sudo cp ~/rpu_work/r5_freertos.elf /lib/firmware/
echo r5_freertos.elf | sudo tee /sys/class/remoteproc/remoteproc0/firmware
echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
cat /sys/class/remoteproc/remoteproc0/state   # → running
```
⚠️ **`firmware` 이름은 재부팅마다 기본값(`rproc-…-fw`)으로 리셋된다** — 매 부팅 후 다시 써 줘야 한다.
⚠️ LED로 판정하려면 Linux가 그 LED를 놓아야 한다: `echo none | sudo tee /sys/class/leds/heartbeat/trigger`.

**RPU 로그·판정 수단 (실측 우선순위)**:
1. **LED (현재 유일하게 실증된 수단)**: 펌웨어의 `led35_task`/`led36_task`가 GPIO를 토글한다. Gate 1은 이 점멸을 사용자가 육안 확인해 통과 판정했다(2026-09-02). Claude는 보드에서 **기계적으로 재검증 불가** — 사용자 보고에 의존한다.
2. **커널 로그**: `journalctl -k | grep -i remoteproc` → `Booting fw image r5_freertos.elf, size 505088` / `remote processor … is now up`. 펌웨어가 **올라갔는지**까지는 이걸로 확정된다(태스크가 도는지는 아님).
3. **remoteproc trace buffer**: `sudo cat /sys/kernel/debug/remoteproc/remoteproc0/trace0` — **현 펌웨어에는 resource table이 없어 `trace0`이 존재하지 않는다**(§4-7). Gate 1.5에서 OpenAMP 베이스로 재빌드하면 그제서야 성립한다. (`/sys/kernel/debug/remoteproc/`는 권한 필요 — 미확인)
4. **UART1 공유**: Linux 콘솔(ttyPS1)과 같은 포트라 출력이 섞인다 — 임시용.
5. **UART0**: DT상 disabled이고 **KV260 캐리어에서 물리적으로 어디로 나오는지 미확인** — 쓰려면 KV260 캐리어 회로도(UG1089 계열)에서 PS UART0 핀 라우팅부터 확인할 것. 확인 없이 stdout을 UART0으로 잡으면 "로그가 안 보이는" 헛수고를 하게 된다.

**Gate 1 판정 (2026-09-02 통과)**: FreeRTOS elf 부팅(`is now up`, 커널 로그로 확정) + **LED 점멸은 사용자 육안 확인**(보드에서 재검증 불가 — `/sys/class/leds/`에 led35/36이 노출되지 않는다).
**미확인 잔여**: start/stop **반복** 안정성 — 로그상 각 부팅에서 1회씩만 확인됐다. 다음 RPU 세션에서 반복 검증할 것.

**막힐 때 진단 순서**: ①`dmesg`의 overlay 적용 오류(문법/phandle) ②probe 실패(binding 불일치 — §4-6 재확인, reserved-memory/IPI phandle 오류) ③elf 로드 실패(링커 주소가 reserved 영역 밖, resource table 부재 — §4-7) ④running인데 로그 없음(로그 수단 문제 — trace buffer로 전환).

---

## 7. PC측(Vitis 2022.1) 가이드 — 사용자 안내용

Claude는 보드에서 실행 불가하므로 아래를 사용자에게 절차로 안내한다. **이 분업(사용자 PC에서 빌드 → 보드로 scp)은 계속 유지된다.**

1. **설치**: ✅ **완료.** Vitis 2022.1이 사용자 x86 PC(Windows, `d:\xilinx\vitis\2022.1`)에 설치되어 있고 실제로 R5 elf를 산출했다. (문서가 오래 권장하던 Linux 설치와 다르지만 결과가 나왔으므로 그대로 간다.)
2. **XSA 확보**: ✅ **완료.** 사용자 자체 Vivado 블록디자인 export = **`design_1_wrapper`**. PL은 smartcam이 쓰므로 bitstream 불필요.
3. **플랫폼/앱 생성**: ✅ Gate 1 기준 완료 — 도메인 `psu_cortexr5_0`, OS `freertos10_xilinx_v1_11`, **순정 FreeRTOS + GPIO(LED) 템플릿**(BSP: `gpiops_v3_9`, `scugic_v4_7`, `ttcps_v3_15`, `standalone_v7_7`).
   ⚠️ **Gate 1.5(rpmsg) 착수 시 여기서 갈아탄다** — application 템플릿을 **"OpenAMP echo-test"** 베이스로 바꿔야 `.resource_table`·rpmsg·trace가 들어온다(현 elf에는 셋 다 없다, §4-7).
4. **링커 스크립트**: Gate 1 elf는 **템플릿 기본값 그대로 0x36000000**에 링크됐고(수정 없음) DT carve-out과 일치한다. Gate 1.5에서는 §6-A의 vring/buffer 배치와 lscript를 **동시에** 맞출 것. 벡터/ISR/주기태스크의 TCM(R5 view 0x0) 배치는 SOEM 단계부터. resource table 섹션(`.resource_table`) 유지 확인.
5. **BSP 설정**: stdout/stdin = §6-C에서 정한 UART(또는 무UART + LED/trace만). 이후 Phase용으로 XilPM 라이브러리 포함 권장(§8 클럭 이슈 대응).
6. elf를 보드로 전송: **scp (USB NIC 경유 — GEM3 상태와 무관)**. GEM 이관 후에도 USB NIC가 살아 있으므로 그대로 쓸 수 있다.

---

## 7-B. 남은 3과제와 Gate 재매핑 (2026-09-04)

Gate 1 통과 후 남은 것은 정확히 3가지다: **① APU↔RPU 통신 ② RAON-RT 실시간 제어 task의 RPU 이식 ③ EtherCAT SOEM의 RPU 설치.**
기존 Phase 0~5 / Gate 1~5 번호는 이 문서와 `rpu_plan.md`·`execution_plan`·`STATUS.md`에 박혀 있으므로 **재번호하지 않고 소수점으로 분할**한다.

```
Phase 0   개발환경 ...................... ✅ 완료 (Vitis 2022.1/Windows, XSA design_1_wrapper)
Gate 1    remoteproc + FreeRTOS ......... ✅ 통과 2026-09-02 (판정 = LED)
──── 여기부터 GEM3 무관 = 지금 진행 가능 ────
Gate 1.5  rpmsg 링크 .................... ① APU↔RPU 통신   (기존 Phase 5의 1~3을 앞으로 분리·승격)
Gate 4a   제어 task 이식 ................ ② RT task 이식   (EtherCAT 없는 1kHz dry-run + 지터 측정)
Gate 3-0  SOEM 빌드 통과 ................ ③ 중 GEM3 불요분 (소스 편입 + osal/oshw 스텁 컴파일·링크)
──── 여기부터 GEM3 필요 = 사용자 재결정 전 금지 ────
Gate 2    GEM3 raw L2 왕복 .............. §8
Gate 3    SOEM으로 슬레이브 OP 도달 ...... §9 (③ 나머지)
Gate 4b   1kHz + DC + CiA402 실축 ....... §10 (기존 Gate 4)
Gate 5    end-to-end .................... 물체 위 접근 + 사람이 옮기면 추종
```

**판정 기준**
- **Gate 1.5** — `/dev/rpmsg*` 등장 + APU↔RPU 에코 왕복 N회 무손실 + watchdog 타임아웃 시 RPU 자체 정지 동작.
  *선행 2가지(둘 다 필요)*: ⓐ DT에 IPI mailbox 노드 + r5f_0의 `mboxes`/`mbox-names` + vring/buffer reserved-memory 추가 → `base.dtb`에서 재병합 → **재부팅 1회** ⓑ 펌웨어를 OpenAMP echo-test 베이스로 재빌드(`.resource_table` 포함). 부수효과로 `trace0` 로그 수단도 생긴다.
  *주소*: §6-A의 0x36000000 창 확장안. **0x3ed00000은 폐기.**
- **Gate 4a** — EtherCAT 없이 R5에서 1kHz 주기 태스크가 RAON-RT 제어 연산(궤적/IK 또는 CTC)을 예산 내에 완주 + R5 사이클 카운터로 지터 기록. **이것이 "APU RT-PREEMPT 대비 정량 비교"(이 트랙의 존재 이유)를 GEM3 없이 확보하는 유일한 경로**다.
  *착수 전 판정 필요(미확인)*: RAON-RT 제어 코드의 C++/double 연산·RBDL 의존이 R5F(단일 코어, VFP, TCM 128KB)에서 어떻게 성립하는가 — 이식 범위(전체 vs 보간·CiA402만)를 먼저 정해야 한다. 현 4 MiB carve-out도 재산정 대상.
- **Gate 3-0** — SOEM v1.4.x + `osal/freertos` + `oshw/freertos` 스텁이 Vitis에서 **링크까지 통과**. NIC 실동작은 Gate 2 이후. §9 체크리스트를 "빌드까지"와 "OP 도달"로 나눠 쓴다.

**GEM3 제약과의 관계** — 제약(§4-1, §8 전제)은 **그대로 유지**된다:

| 과제 | GEM3 필요? | 판정 |
|---|---|---|
| ① rpmsg | 불필요 (IPI + DDR 공유메모리만) | **지금 가능.** 단 DT 변경 → 재부팅 1회 → 실기 세션과 분리(§4-4c) |
| ② 제어 task 이식 | 불필요 (Gate 4a는 연산·주기·지터만) | **지금 가능.** 지터 측정 중 APU가 무거우면 DDR 경합으로 오염 → E34 규율 적용 |
| ③ SOEM | **부분 충돌** — 빌드(Gate 3-0)는 불필요, PHY 링크업·raw 프레임·OP 도달은 필수 | **Gate 3-0까지만.** 그 앞은 사용자 재결정 전 금지 |

순서 권고: ① → ② → ③(빌드까지) → 그 시점에 GEM3 이관을 사용자에게 재요청.

---

## 8. Phase 2 상세 — GEM3 이관의 함정 지도

**전제 확인 (2026-08-03 갱신)**: APU IgH는 "완료"를 넘어 **Indy7 실운용 중**이다. Phase 2는 GEM3를 가져가므로 그동안 APU 로봇 제어가 중단된다. 착수 전 필수: ①사용자와 운용 중단 기간 합의 ②macb rebind 복구 절차 준비(스크립트화) ③실기 세션과 겹치지 않는 시간대. 사용자 결정 없이 착수 금지.

### 절차 개요
1. 사용자 고지 + UART/물리 접근 준비 확인 (§1)
2. `echo ff0e0000.ethernet | sudo tee /sys/bus/platform/drivers/macb/unbind`
3. RPU에서 emacps 예제 기반 링크업 → raw frame 왕복 (PC와 직결, Wireshark 검증, EtherType 0x88A4)

### 함정 1 — 클럭 게이팅 (unbind의 숨은 부작용) ★반드시 읽을 것
macb 드라이버는 unbind(remove) 시 자기가 켰던 GEM3 클럭들을 **끈다**. `clk_ignore_unused`는 부팅 시 미사용 클럭을 안 끄는 옵션일 뿐, 드라이버가 명시적으로 끄는 것은 막지 못한다. 그 상태에서 RPU의 emacps 초기화는 클럭이 꺼진 하드웨어를 만지게 되어 실패한다(standalone 드라이버는 클럭이 이미 켜져 있다고 가정).

**확인 방법**: unbind 전후로 CRL_APB의 GEM3 클럭 레지스터를 읽어 비교.
```bash
# GEM3_REF_CTRL은 CRL_APB(0xFF5E0000) 영역. 정확한 오프셋/비트는 UG1085 레지스터 레퍼런스로 확인할 것.
sudo busybox devmem 0xFF5E005C   # busybox 미설치 시: sudo apt install busybox / 또는 devmem2
```
**해결 경로 (둘 중 하나)**:
- (a) RPU 펌웨어 초기화 코드에서 XilPM(`XPm_RequestNode` 등) 또는 CRL_APB 직접 write로 GEM3 클럭/전원을 스스로 켠다 — 자립적이라 권장.
- (b) boot-time DT에서 GEM3를 `status = "disabled"`로 바꿔 Linux가 아예 안 건드리게 한다(§6-B 경로) — psu_init(FSBL)이 켜둔 클럭이 그대로 유지됨. 단 이 경우 SSH 경로가 부팅부터 없다.
개발 초기는 (a)+unbind 조합(재부팅 복구 가능), 최종 운용은 (b) 권장.

### 함정 2 — DMA 캐시 일관성 (최대 난제)
- 증상: 간헐적 프레임 깨짐/옛 데이터, printf 넣으면 증상 소멸.
- 예방: **BD ring + 패킷 버퍼를 R5 MPU로 non-cacheable(Device/Strongly-ordered 아닌 Normal Non-cacheable) 영역에 배치**하고 시작. `Xil_SetTlbAttributes`/MPU 설정 API 사용. 캐시+수동 flush 최적화는 Gate 2 통과 후에만.

### 함정 3 — PHY (TI DP83867)
- embeddedsw emacps 예제에 DP83867 지원 코드 있음. PHY 주소는 MDIO 스캔으로 탐지. RGMII delay 설정이 링크업 실패의 단골 원인.
- 진단: MDIO로 PHY ID 레지스터가 읽히는가 → autoneg 완료 비트 → 링크 파트너 확인.

### 함정 4 — 인터럽트
- GEM3 IRQ(SPI 63, GIC 번호 95)가 RPU GIC(scugic)에 등록되어야 함. xparameters.h의 emacps 인터럽트 ID 사용.
- Linux가 살아있는 동안 같은 IRQ를 양쪽에서 잡으면 안 됨 — unbind가 선행되면 안전.

**Gate 2**: RPU↔PC 임의 L2 프레임 수천 회 무손실 왕복.
**진단 순서**: 클럭(함정1) → PHY 링크(함정3) → 송신만 되고 수신 안 됨(함정4) → 간헐 깨짐(함정2).

---

## 9. Phase 3 — SOEM 포팅 체크리스트

디렉토리 구조 (SOEM v1.4.x 기준):
```
soem/osal/freertos/osal.c            ← 신규 작성
soem/osal/freertos/osal_defs.h
soem/oshw/freertos/nicdrv.c          ← 신규 작성 (Phase 2 emacps 코드 래핑)
soem/oshw/freertos/nicdrv.h
soem/oshw/freertos/oshw.c            ← htons/ntohs 등 (기존 포트 복사 수준)
```

구현 함수 목록:
- `osal`: `osal_current_time`(TTC 기반 µs — FreeRTOS tick 1ms로는 불가), `osal_time_diff`, `osal_usleep`(sub-ms는 TTC 폴링), `osal_timer_start/is_expired`, mutex 계열(FreeRTOS 세마포어), `osal_thread_create`(초기엔 미사용 가능)
- `oshw`/nicdrv: `hw_open/close`, `hw_send`(emacps TX), `hw_receive`(초기 폴링), redundancy 관련은 단일 포트 스텁
- SOEM core는 무수정 원칙. 수정 충동이 들면 포팅 계층이 잘못된 것.

검증: `simple_test` 이식 → L7N 연결 → PREOP→SAFEOP→OP. 이 시점에 사용자에게 L7N PDF 요청(`~/ros2_ws/docs/l7n/`).

**Gate 3**: L7N OP 도달 + PDO read 정상.

---

## 10. Phase 4~5 요점

**Phase 4 (1kHz + DC + CiA402)**:
- TTC 인터럽트 1kHz 태스크에서 send/receive_processdata. DC 활성화, SYNC0 정렬.
- jitter를 R5 사이클 카운터(PMU cycle counter)로 측정·기록 — APU IgH 대비 정량 비교가 이 프로젝트의 성과 지표.
- L7N: CiA402 상태머신(Shutdown→Switch On→Enable Operation), CSP 모드. 랩 가이드(MAN-20241113-LX02H0001)의 드라이브 설정 개념 참조 가능(IgH 기준이지만 CiA402 시퀀스는 동일).
- 이후 **Indy7 제어 코드 이식**: 원본은 `~/RAON-RT/`(EMasterApp) — **IgH 기반 확정**(`EcatCommon.h`가 `ec_slave_config_t` 등 IgH 타입 사용) → SOEM API로 번역 계층 필요. CiA402 시퀀스·PDO 매핑·제어 로직(CTC, 마찰 FF 등)은 재사용, EtherCAT API 호출부만 교체. 상세 맥락은 `docs/RAON-RT/merge.md`.

**Phase 5 (rpmsg + ROS2 통합)** — ⚠️ **이 중 rpmsg 링크(아래 1~2번)는 잔여 과제 ①로 앞당겨졌다 = Gate 1.5(§7-B). GEM3와 무관하므로 Phase 2를 기다리지 않는다.**
- RPU: OpenAMP echo-test 기반 rpmsg endpoint. Linux: `modprobe rpmsg_char` → `/dev/rpmsg*`.
- 메시지: 고정 크기 C struct + 버전 필드. APU→RPU(목표 관절값/궤적 세그먼트 + watchdog 카운터), RPU→APU(관절 상태/EtherCAT 상태/오류).
- **통신 방식 결정(2026-07-08)**: 초기 구현은 RPMsg 단독(현재 데이터량이 페이로드 한계 ~496B/msg 안). 병목 실측 시에만 raw shared memory 데이터면 추가(hybrid) — 그 경우 양측 non-cacheable + seqlock/double-buffer + Linux는 UIO(generic-uio 노드, cmdline에 이미 활성) 사용. cross-core mutex/atomic은 쓰지 말 것(A53↔R5는 캐시 coherency 도메인이 다름). 예약 영역은 처음부터 shm 확장분 포함해 잡아둘 것. 상세 논리: rpu_plan.md Phase 5 설계 노트.
- 안전: rpmsg watchdog timeout → RPU 자체 감속 정지. EtherCAT WKC 오류 → Quick Stop + 보고. **안전 로직은 전부 RPU 쪽에 둔다** (APU 죽어도 로봇은 멈춰야 함).
- APU 측 브리지 = RAON-RT `CROS2PickBridge`(`App/Indy7/ROS2PickBridge.cpp`, 이미 `/pick_target_base` 구독 중) — 출력을 seqlock→rpmsg로 교체. ros2_ws에 별도 패키지 없음(2026-09-04 확정; 토픽 이름은 pick&place 시절 legacy, 실제 내용은 추종 대상의 base_link 좌표). 부팅 자동화는 smartcam.service 패턴의 systemd unit: DT 확인 → 펌웨어 로드 → firmware 이름 재설정 → start → bridge 기동.

**Gate 5 (최종, 앱 전환 반영)**: perception → RPU → **Indy7 말단부가 물체 바로 위에서 정지하고, 사람이 물체를 옮기면 그 물체를 추종**, end-to-end 1회. (파지·이송·놓기는 범위 밖 — 그리퍼 하드웨어 미보유.) 현재 APU 트랙은 이 동작을 이미 수행 중이다 — 접근은 실기 검증 완료, 추종 v3는 실기 미검증(`docs/RAON-RT/merge.md`).

---

## 11. 트러블슈팅 도구 상자 (보드)

```bash
dmesg | tail -50                                  # overlay/remoteproc/드라이버 오류 1차 확인
sudo cat /sys/kernel/debug/remoteproc/remoteproc0/trace0   # RPU 로그 (trace 엔트리 필요)
cat /sys/class/remoteproc/remoteproc0/state       # offline/running
sudo cat /proc/iomem                              # 메모리 예약 실태
ls /sys/kernel/config/device-tree/overlays/       # 적용된 overlay
sudo busybox devmem <addr>                        # 레지스터 직접 확인 (클럭/PHY 진단)
ethtool -i eth0                                   # (이관 전) GEM 드라이버 상태
```
PC측: Vitis XSCT + JTAG (USB 케이블)로 R5 브레이크포인트 디버깅 가능 — trace/printf로 안 잡히는 문제(하드폴트 등)는 JTAG이 최후 수단.

## 12. 미결 사항 (해소되면 이 문서와 메모리 갱신)

- [x] ~~Indy7 제어 코드의 EtherCAT master 종류~~ → **IgH 확정** (2026-08-03, `~/RAON-RT/EMasterApp/Device/EcatCommon.h` 실사). Phase 4는 IgH→SOEM 번역 이식
- [x] ~~제어 주기 확정~~ → **1kHz 확정** (2026-08-03, RAON-RT 실운용 실증)
- [x] ~~Vitis 2022.1 설치~~ → **완료** (사용자 x86 PC/Windows, XSA `design_1_wrapper`. 2026-09-02 elf 산출로 실증)
- [x] ~~RPU reserved-memory 주소~~ → **`0x36000000` + 4 MiB, no-map으로 확정·적용** (2026-09-02, 부팅 DTB 병합). **옛 0x3ed00000 안은 폐기 — 이 보드의 CMA 한복판**
- [x] ~~DT 적용 방식~~ → **부팅 DTB 병합(`/boot/firmware/user-override.dtb`)으로 확정.** configfs·flash-kernel 경로 아님
- [x] ~~Gate 1 실행~~ → **통과 (2026-09-02)**. FreeRTOS elf `is now up`(커널 로그) + LED 점멸은 사용자 육안 확인(재검증 불가)
- [ ] **① APU↔RPU 통신 (Gate 1.5)** — DT에 IPI mailbox + `mboxes` + vring/buffer 추가(재부팅 1회) + 펌웨어를 OpenAMP echo-test 베이스로 재빌드 → `/dev/rpmsg*` 에코 왕복. §7-B
- [ ] **② RAON-RT 제어 task 이식 (Gate 4a)** — EtherCAT 없는 1kHz dry-run + R5 사이클 카운터 지터 기록. 착수 전 이식 범위 판정 필요(C++/double/RBDL vs R5F 제약). §7-B
- [ ] **③ SOEM 설치 — 빌드까지 (Gate 3-0)** — 소스 편입 + osal/oshw 스텁 링크 통과. OP 도달(Gate 3)은 GEM3 이후. §9
- [ ] start/stop **반복** 안정성 검증 (Gate 1에서 각 부팅 1회씩만 확인됨 — 미확인)
- [ ] L7N 문서 PDF 수령 (Phase 3 착수 시 — `docs/l7n/` 아직 없음)
- [ ] KV260 캐리어의 PS UART0 물리 라우팅 여부 (UART0 쓰려는 경우만 — 현 판정 수단은 LED, Gate 1.5 후엔 trace buffer라 급하지 않음)
- [ ] `~/rpu_work/`(git 미추적)의 산출물을 `tools/rpu/`로 정본화할지 결정 — 현재 실제로 쓰인 dtso/elf가 저장소 밖에 있다. ※ `tools/rpu/`도 **아직 git 미추적**(`git status` → `?? tools/rpu/`)이라, 정본화하려면 커밋까지 해야 한다
- [ ] GEM3 이관(Phase 2) 시점 — 사용자 결정 대기 (로봇 운용 중단 수반)
