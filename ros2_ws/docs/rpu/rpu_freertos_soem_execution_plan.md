# RPU FreeRTOS + SOEM EtherCAT Master 실행 계획

작성: 2026-07-07. 보드 실사(이 문서의 "보드 검증 상태" 참조) 기반으로 작성된 실행 계획.
> **상태 갱신 (2026-08-03)**: 이 문서의 "보드 검증 상태" 표는 구커널(5.15.0-1070) 기준 — **현재 사실은 `rpu_guide_for_claude.md` §3이 정본** (RT 커널 재검증 완료, 재빌드 불필요). Gate 1 착수 중.
> **상태 갱신 (2026-09-04)**: **Gate 1 통과 (작업 2026-09-01~02, 펌웨어 부팅 검증 2026-09-02: 11:02·15:01 2회)** — R5F-0에서 FreeRTOS 단독 구동 + **LED 태스크 실행**(점멸 자체는 사용자 육안 확인). Phase 0(Vitis 2022.1 설치)도 완료. 판정 수단·DT 적용 방식·메모리 맵이 이 문서의 원안과 다르다(LED / 부팅 DTB 병합 / 0x36000000) — 아래 Phase 1 절에 실제값으로 갱신해 두었다. **남은 것은 ① APU↔RPU 통신 ② RAON-RT 실시간 제어 task의 RPU 이식 ③ EtherCAT SOEM의 RPU 설치** 3가지이고, Gate 번호 대응(Gate 1.5 / 4a / 3-0)은 `rpu_guide_for_claude.md` §7-B가 정본.
목적: **RPU 계층 구축** — RPU firmware + APU↔RPU bridge + 로봇 EtherCAT 제어. (2026-09-04: 이 중 RPU firmware는 Gate 1로 구동 확인됐고, 로봇 EtherCAT 제어는 **APU에서 IgH+RAON-RT로 이미 실운용 중**이다 — 이 문서의 과제는 그것을 RPU로 옮기는 것이지 "미구현 계층을 처음 만드는 것"이 아니다.)
> 앱 전환 주석(2026-09-04): 최종 목표는 pick & place가 아니라 **(a) 물체 위로 TCP 접근 + (b) 사람이 물체를 옮기면 추종**이다. 파지·이송·놓기는 범위 밖(그리퍼 미보유). `pick_*` 이름은 legacy.
참조: `docs/reference/reference_02_openamp_freertos_ethernet.md` (Kria FreeRTOS/OpenAMP 공식 문서 링크),
랩 가이드 MAN-20241113-LX02H0001 (IgH 기반, 프로토콜 개념 참조용).

## 아키텍처 목표

```
┌────────────────── KV260 (ZynqMP) ──────────────────┐
│  APU (A53×4)                RPU (R5F-0, split)      │
│  Ubuntu 22.04 + ROS2        FreeRTOS 10 (Vitis BSP) │
│  perception 파이프라인       SOEM + 1kHz 제어 루프    │
│  CROS2PickBridge     ⇄⇄⇄   OpenAMP rpmsg           │
│  (WiFi/USB Ethernet)        PS GEM3 ── RJ45 ────────┼── EtherCAT (Indy7 방향)
└─────────────────────────────────────────────────────┘
```

- APU: 기존 perception (RealSense→DPU→pick_target_base) 유지. RAON-RT `CROS2PickBridge`가 rpmsg로 목표점/궤적 전달 — **ros2_ws에 별도 패키지는 두지 않는다**(2026-09-04 확정; 구 `apu_rpu_bridge_pkg`는 2026-07-15 삭제, 재생성 안 함).
- RPU: FreeRTOS + SOEM. GEM3를 전용으로 사용해 raw EtherCAT 프레임 송수신, 1kHz 이상 주기 제어.
- **주의**: kv260-smartcam DPU overlay와 RPU 사용은 독립적 — 충돌 없음 (PL vs RPU).

## 보드 검증 상태 (2026-07-07 실사)

| 항목 | 상태 |
|---|---|
| 커널 remoteproc 지원 | `zynqmp_r5_remoteproc.ko`, `rpmsg_char.ko`, `virtio_rpmsg_bus.ko` 모듈 존재 ✓ |
| device tree R5 노드 | ~~없음~~ → **2026-09-02 해소**: `rf5ss@ff9a0000`(+`r5f_0`)·TCM 2개·`reserved-memory/rproc@36000000`이 **부팅 DTB에 병합**됨. `remoteproc0` = `ff9a0000.rf5ss:r5f_0`, 모듈 자동 probe |
| configfs DT overlay | 동작 확인 ✓ (`/sys/kernel/config/device-tree/overlays/`에 smartcam 로드됨, dfx-mgr active). **단 RPU는 이 경로를 쓰지 않는다** — reserved-memory 때문에 boot-time으로 승격 |
| IPI mailbox | `/proc/device-tree/zynqmp-ipi` 존재 (mailbox@ff9905c0, `xlnx,ipi-id = <4>`). **RPU rpmsg용 IPI 채널은 여전히 없다** — `r5f_0`에 `mboxes`/`mbox-names`도, vring/buffer reserved-memory도 미추가. **잔여 과제 ①의 선결조건** |
| Ethernet | GEM3(`ff0e0000`)만 okay, 드라이버 `macb`, eth0. GEM0-2 disabled |
| cmdline | `clk_ignore_unused` 이미 있음 ✓ (GEM을 Linux에서 떼어도 클럭 유지에 유리), `cma=1000M` |
| 부팅 | U-Boot + flash-kernel `image.fit` (GRUB 없음). boot.scr.uimg 존재 — **이 boot script가 `user-override.dtb`를 지원하며, 그것이 RPU DT의 실제 적용 경로다**(파일 없으면 FIT 순정 DTB로 폴백 = 복구) |
| **원격 접속 리스크** | ~~Tailscale over eth0~~ → **해소**: Tailscale 제거, 원격은 AX88179B **USB NIC**(`enx…`, 192.168.120.50). eth0는 IgH EtherCAT 예약 포트라 이관해도 SSH는 안 끊긴다. **남은 리스크는 네트워크가 아니라 로봇 운용 중단** |

## 핵심 기술 판단 (사전 확정)

1. **SOEM에 공식 FreeRTOS+ZynqMP 포트 없음** → `osal`/`oshw` 계층을 FreeRTOS + `emacps`(Xilinx embeddedsw standalone GEM 드라이버) 위에 직접 포팅. 이것이 전체 개발량의 중심. SOEM은 TCP/IP 스택(lwIP) 불필요 — raw L2 프레임 send/recv만 있으면 됨.
2. **RPU 로드는 remoteproc** (Kria Ubuntu 표준 흐름). BOOT.BIN 수정은 Kria에선 비표준(부트 펌웨어가 QSPI에 고정, xmutil bootfw 관리) — 부팅 자동화는 **systemd 서비스로 remoteproc start** 방식 사용.
3. **R5는 split 모드, R5F-0 사용**. 코드/데이터는 DDR 예약 영역에 링크(SOEM+스택이 TCM 128KB 초과), 주기 태스크/ISR 등 지연 민감 코드만 TCM 배치.
4. **GEM3 이관은 2단계**: 개발 중엔 런타임 driver unbind(`echo ff0e0000.ethernet > /sys/bus/platform/drivers/macb/unbind`, 재부팅으로 복구 가능), 안정화 후 boot-time DT에서 disable.
5. **Vitis 버전: 2022.1** (classic IDE). 근거: kria-apps-docs FreeRTOS/OpenAMP 튜토리얼이 2022.1 기준, 보드 커널 5.15 세대와 일치, docs/reference/ 참조 문서들도 2022.1 흐름. CLAUDE.md 방침("최신보다 board-validated 버전") 부합. 빌드는 x86 PC에서 수행 (보드는 aarch64라 Vitis 실행 불가).

## 확정 사항 (2026-07-08 사용자 확인, 2차 업데이트)

> 아래 항목은 **2026-09-04 실사로 갱신**되었다.

- **작업 순서**: Phase 0~1 완료. **다음은 GEM3와 무관한 ①rpmsg(Gate 1.5) → ②제어 task 이식(Gate 4a) → ③SOEM 빌드(Gate 3-0)**. Phase 2(GEM3 이관)만 남겨 두는 이유는 "IgH 완료 대기"가 아니라 **APU IgH가 실운용 중이라 이관하면 로봇이 서기 때문** — 사용자 재결정 전 착수 금지.
- **Indy7 접점**: STEP 우회, 드라이브 직결 확정. **APU에서 IgH+RAON-RT로 실제 제어·운용 중** → Phase 4는 신규 개발이 아니라 검증된 제어 로직의 RPU 이식이되, master가 **IgH로 확정**되었으므로 **IgH→SOEM 번역 계층이 필요**하다.
- **테스트 드라이브**: **LS Mecapion L7N** (EtherCAT, CiA402) + 모터 + 전원 준비 완료. 관련 문서 PDF 제공 가능 → Phase 3 착수 시 ESI/PDO 매핑 자료로 수령 (2026-09-04 현재 미수령).
- **로그·판정 수단**: UART 시리얼 터미널은 가능하나 **Gate 1 판정은 LED로 이루어졌다**(커널 `is now up`은 기계적 확인, `led35_task`/`led36_task` 점멸은 **사용자 육안 확인** — 보드에서 재검증 불가). 현 펌웨어는 resource table이 없어 `trace0`이 없다 — trace는 Gate 1.5에서 OpenAMP 베이스로 재빌드해야 생긴다.
- **빌드 PC**: x86 PC, **Vitis 2022.1 설치 완료**(Windows `d:\xilinx\vitis\2022.1`, XSA `design_1_wrapper`, GCC 11.2.0 arm-none-eabi). **빌드는 PC에서, elf는 scp로 보드에** — 이 분업 구조 유지.
- **보드 접근**: 손 닿는 곳 ✓. USB NIC가 원격 생명줄이라 GEM3 이관 후에도 SSH 유지.
- **그리퍼**: 미보유 — 범위에서 제외. Phase 5의 end-to-end 목표는 **"말단부가 물체 바로 위에서 정지 + 사람이 물체를 옮기면 추종"**까지.
- **제어 주기**: **1kHz 확정** (RAON-RT APU 트랙이 1kHz로 실운용하며 실증).

---

## Phase 1 — remoteproc + FreeRTOS Hello World

✅ **Gate 1 통과 (작업 2026-09-01~02, 펌웨어 부팅 검증 2026-09-02: 11:02·15:01 2회)**

**목표**: Ubuntu에서 R5F-0에 FreeRTOS elf를 로드/시작, 태스크가 도는 것을 확인. → **달성. 단 확인 수단은 UART가 아니라 LED였다.**
아래는 **실제로 통한 절차**로 갱신한 것이다(원안의 configfs·0x3ed00000 경로는 폐기). 명령 전문·함정은 `rpu_guide_for_claude.md` §6.

### PC 쪽 (x86, Vitis 2022.1)

실제 산출물 기준으로 갱신 (2026-09-04):
1. Vitis 2022.1 설치 ✅ (Windows `d:\xilinx\vitis\2022.1`). XSA는 사용자 자체 Vivado BD export = **`design_1_wrapper`**.
2. Platform: 도메인 `psu_cortexr5_0`, OS `freertos10_xilinx_v1_11`. BSP: `gpiops_v3_9`, `scugic_v4_7`, `ttcps_v3_15`, `standalone_v7_7`.
3. 앱: **순정 FreeRTOS + GPIO(LED) 템플릿** — `led35_task`/`led36_task`. 링커 스크립트는 **무수정**(템플릿 기본값 0x36000000이 carve-out과 일치).
4. 산출물 `r5_freertos.elf`(505,088 B)를 scp로 보드 `/lib/firmware/`에 복사.
   ⚠️ Gate 1.5(rpmsg) 착수 시에는 앱 템플릿을 **OpenAMP echo-test**로 갈아타야 한다 — 현 elf에는 `.resource_table`도 rpmsg 심볼도 없다.

### 메모리 맵 (Xilinx OpenAMP 표준 배치 준수)

**2026-09-02 실적용값으로 갱신** — 원안(0x3ed00000 계열)은 폐기됐다:
```
0x36000000 + 0x400000 (4 MiB, no-map) : rproc carve-out — 현재 펌웨어가 ~78KB만 사용
  ├ 0x36000000 + 256KB   : 펌웨어 (.text 시작점)
  └ (Gate 1.5 확장안) 0x36040000/0x36044000 각 16KB : vring0/1,  0x36048000 + 1MB : rpmsg 버퍼
TCM ATCM (R5 view 0x0, APU view 0xffe00000) : 벡터테이블(.vectors/.bootdata) + 지연민감 코드
```
⚠️ **옛 0x3ed00000 배치는 폐기** — 이 보드의 CMA 창(`0x36400000–0x74C00000`) 한복판이다. carve-out을 4 MiB 위로 늘리면 CMA와 충돌하므로, 확장이 필요하면 **아래쪽으로**(0x35C00000 등) 늘리고 재검증할 것.

### 보드 쪽

**실제로 통한 절차 (2026-09-02)** — 원안의 configfs 경로는 폐기됐다:
1. **R5 DT 소스 작성** (`~/rpu_work/rpu_v2.dtso`): `xlnx,zynqmp-r5-remoteproc` 클러스터(`xlnx,cluster-mode = <1>` — split/lockstep 매핑은 미확인, 상류 관례는 0=split) + TCM 2개 + `reserved-memory/rproc@36000000`. ※ IPI mailbox·vring 노드는 **미포함** — 잔여 과제 ①에서 추가.
2. 컴파일 + **오프라인 병합**(configfs 아님):
   ```bash
   dtc -@ -O dtb -o rpu_v2.dtbo rpu_v2.dtso
   fdtoverlay -i base.dtb -o user-override.dtb rpu_v2.dtbo    # base.dtb = /usr/lib/linux-image-$(uname -r)/xilinx/smk-k26-revA-sck-kv-g-revB.dtb
   ```
   ⚠️ 항상 `base.dtb`에서 새로 병합할 것 — `user-override.dtb`는 누적되지 않는다.
3. 배포 + 재부팅 (실기 세션이 아님을 먼저 확인):
   ```bash
   systemctl is-active ethercat                                  # inactive 확인
   sudo cp /boot/firmware/user-override.dtb /boot/firmware/user-override.dtb.ok   # 있으면 백업
   sudo cp user-override.dtb /boot/firmware/user-override.dtb && sync && sudo reboot
   cat /sys/class/remoteproc/remoteproc0/name                    # ff9a0000.rf5ss:r5f_0 (modprobe 불필요)
   ```
4. 실행 (**firmware 이름은 재부팅마다 리셋되므로 매번 지정**):
   ```bash
   echo none | sudo tee /sys/class/leds/heartbeat/trigger        # Linux가 LED를 놓게 함
   echo r5_freertos.elf | sudo tee /sys/class/remoteproc/remoteproc0/firmware
   echo start | sudo tee /sys/class/remoteproc/remoteproc0/state
   ```

**Gate 1**: ~~UART로 FreeRTOS 태스크 로그 출력~~ → **커널 `remote processor … is now up` + LED 점멸(사용자 육안 확인, 재검증 불가)으로 통과(2026-09-02)**. `stop`/`start` **반복** 안정성은 **미확인**(각 부팅 1회씩만 확인됨).
**디버깅 순서**: DT 병합/부팅 오류(dmesg) → 모듈 probe 실패(reserved-memory/IPI 불일치) → elf 로드 실패(링커 주소 vs 예약 영역) → 실행 후 무반응(LED trigger 해제 여부, 벡터테이블. `no resource table found.`는 **경고일 뿐 실패가 아니다**).

## Phase 2 — GEM3 이관 + raw Ethernet 송수신

**목표**: RPU가 GEM3로 raw L2 프레임 송수신.

0. **선행(2026-09-04 갱신): 사용자 재결정.** 대체 네트워크는 이미 확보되어 있다(AX88179B USB NIC 상시, Tailscale 없음) — 남은 전제는 네트워크가 아니라 **APU IgH 로봇 운용 중단 합의 + macb rebind 복구 절차 준비 + 실기 세션과 겹치지 않는 시간대**다. **사용자 결정 없이 착수 금지.**
2. Linux에서 GEM3 분리(개발 단계, 가역적):
   ```bash
   echo ff0e0000.ethernet | sudo tee /sys/bus/platform/drivers/macb/unbind
   ```
3. Vitis에서 `emacps` interrupt 예제 기반 테스트 앱: PHY = TI **DP83867** 초기화(embeddedsw emacps 예제에 포함), 1G 링크업.
4. EtherType 0x88A4 테스트 프레임 송신 → PC Wireshark로 수신 확인, 반대 방향도 확인.
5. **함정 대비**:
   - **DMA 캐시 일관성**: BD ring/버퍼 영역을 R5 MPU로 non-cacheable 설정하거나 flush/invalidate 일관 처리 (최다 삽질 포인트).
   - GEM3 인터럽트(IRQ 95)가 RPU GIC로 수신되는지 확인.
   - 클럭: `clk_ignore_unused`가 이미 있으나, unbind 후 GEM 클럭이 꺼지면 Linux CCF가 원인 — 필요시 boot-time DT에서 disable로 전환.

**Gate 2**: RPU ↔ PC 간 임의 L2 프레임 왕복 성공.

## Phase 3 — SOEM 포팅

**목표**: SOEM `simple_test`가 RPU에서 슬레이브 스캔, OP 도달.

1. SOEM v1.4.x 소스 트리를 Vitis 프로젝트에 편입.
2. `osal/freertos/` 작성: `osal_usleep`(vTaskDelay + 미세지연은 TTC 폴링), `osal_current_time`(TTC 기반 µs 시계), 뮤텍스(FreeRTOS 세마포어).
3. `oshw/freertos/nicdrv.c` 작성: Phase 2의 emacps 코드 래핑 — `hw_open/close`, send/recv, redundancy 미사용. 초기 구현은 **수신 폴링**으로 단순화, 동작 후 인터럽트+큐로 전환.
4. `simple_test` 이식 → 테스트 슬레이브 대상 PREOP→SAFEOP→OP 확인.

**Gate 3-0 (2026-09-04 신설, GEM3 불요)**: 1~3의 결과가 Vitis에서 **컴파일·링크까지 통과**. NIC 실동작 없이 여기까지는 지금 진행 가능하다 — 잔여 과제 ③의 착수 가능 범위가 정확히 이 지점까지다.
**Gate 3**: 슬레이브 OP 도달 + PDO 데이터 정상 read. (GEM3 필요 → Gate 2 이후)

## Phase 4 — 1kHz 주기 제어 + DC + Indy7 프로토콜

1. TTC 인터럽트 기반 1kHz 태스크에서 `ecx_send/receive_processdata`.
2. Distributed Clocks 설정, SYNC0 정렬.
3. R5 사이클 카운터로 주기 지터 측정/로깅 → **APU IgH(RT-PREEMPT) 대비 성능 근거 데이터** (목표: 지터 수 µs 이하).
4. Indy7 접점 확인 결과에 따라: CiA402 상태머신(직결 드라이브) 또는 별도 프로토콜 구현.
5. **RAON-RT 제어 로직 이식** — master가 IgH로 확정되었으므로 EtherCAT API 호출부만 SOEM으로 번역, CiA402 시퀀스·PDO 매핑·제어 로직(CTC, 마찰 FF 등)은 재사용.

**Gate 4a (2026-09-04 신설, GEM3 불요)**: EtherCAT 없이 R5에서 1kHz 주기 태스크가 제어 연산(궤적/IK 또는 CTC)을 예산 안에 완주 + 사이클 카운터로 지터 기록. 위 3·5 항목을 EtherCAT 전제 없이 떼어낸 것으로, **이 트랙의 존재 이유(APU 대비 정량 비교)를 GEM3 없이 조기 입증하는 지점**이다.
*착수 전 판정 필요(미확인)*: RAON-RT 제어 코드의 C++/double 연산·RBDL 의존이 R5F(단일 코어, VFP, TCM 128KB)에서 성립하는가 → 이식 범위(전체 vs 보간·CiA402만) 결정. 현 4 MiB carve-out도 재산정 대상.
**Gate 4b (기존 Gate 4)**: 1kHz 사이클에서 지터 목표 달성 + 실제 축 1개 이상 위치 제어. (GEM3 필요)

## Phase 5 — APU↔RPU bridge (RAON-RT `CROS2PickBridge` → rpmsg)

> ★ **1~3(rpmsg 링크)은 잔여 과제 ①로 앞당겨졌다 = Gate 1.5.** IPI + DDR 공유메모리만 쓰므로 GEM3와 무관하고, Phase 2를 기다릴 이유가 없다. **선행 2가지**: ⓐ DT에 IPI mailbox 노드 + `r5f_0`의 `mboxes`/`mbox-names` + vring/buffer reserved-memory 추가 → base.dtb 재병합 → 재부팅 1회 ⓑ 펌웨어를 OpenAMP echo-test 베이스로 재빌드(`.resource_table` 포함). 부수효과로 `trace0` 로그 수단도 생긴다.

1. RPU 펌웨어에 OpenAMP rpmsg endpoint 추가 (Vitis echo-test 예제 기반).
2. Linux: `rpmsg_char`로 `/dev/rpmsg0` 노출 확인.
3. 메시지 포맷 정의: APU→RPU (목표 관절값/궤적 세그먼트, watchdog), RPU→APU (관절 상태, EtherCAT 상태, 오류코드). 고정 크기 C struct, 버전 필드 포함.
   **Gate 1.5**: `/dev/rpmsg*` 등장 + 에코 왕복 N회 무손실 + watchdog 타임아웃 시 RPU 자체 정지 동작 확인.
4. `CROS2PickBridge`(RAON-RT `App/Indy7/ROS2PickBridge.cpp`) 출력 교체: 이미 `/pick_target_base`를 구독해 RT 스레드로 seqlock 전달 중 → 그 자리에 rpmsg 송신, RPU 상태 → ROS2 토픽 발행. **ros2_ws에 별도 패키지 없음**(2026-09-04 확정, 구 `apu_rpu_bridge_pkg` 재생성 안 함. 토픽 이름은 pick&place 시절 legacy — 내용은 추종 대상의 base_link 좌표)
5. 부팅 자동화: DT 확인 + 펌웨어 이름 재설정 + remoteproc start + bridge 노드를 systemd/launch에 편입 (smartcam.service와 동일 패턴).

**Gate 5**: perception → 대상 물체 좌표 → RPU → **말단부가 물체 바로 위에서 정지 + 사람이 물체를 옮기면 추종**까지 end-to-end 1회 성공. (파지·이송·놓기 없음 — 그리퍼 미보유.)

## 리스크 요약

| 리스크 | 대응 |
|---|---|
| GEM 이관 시 원격 세션 단절 | **해소** — 원격이 USB NIC라 eth0를 넘겨도 SSH 유지. 남은 전제는 **로봇 운용 중단 합의**(Phase 2 선행 0번) |
| DT/reserved-memory 오류로 부팅 문제 (**boot-time 방식이라 실재 리스크**) | 교체 전 `user-override.dtb.ok` 백업 + 항상 base.dtb에서 새로 병합. 최후 복구는 SD를 PC에 꽂아 `user-override.dtb` 삭제(순정 DTB 폴백). 재부팅은 실기 세션과 분리 |
| emacps DMA 캐시 이슈 | non-cacheable MPU region으로 시작, 성능은 이후 최적화 |
| SOEM 포팅 난항 | Phase 2에서 raw 송수신 완전 검증 후 착수, 폴링→인터럽트 단계화 |
| Indy7 인터페이스 불확실 | **해소** — master = **IgH 확정**, 주기 **1kHz 확정**. Phase 4는 IgH→SOEM 번역 이식으로 범위가 고정됐다 |
| 제어 코드가 R5F에서 성립하지 않을 가능성 (신규) | Gate 4a 착수 전 이식 범위 판정(C++/double·RBDL vs 단일코어·VFP·TCM 128KB). 축소 이식(보간+CiA402만) 선택지 포함 |
| APU IgH 작업과의 관계 | APU에서 검증한 제어 로직(CiA402 등)을 RPU로 이식 — 이 계획과 상호보완. **단 IgH는 실운용 중이므로 GEM3 이관은 운용 중단 합의가 선행** |
