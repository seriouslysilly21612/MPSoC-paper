# 실험 환경 전수 조사 — 하드웨어·OS/RT·ROS·비전·제어·툴체인 (2026-09-03)

보고서 "실험 환경" 절을 쓰기 위해 보드·문서·저장소를 전수 조사한 스냅샷.
6방향 병렬 조사(하드웨어 / OS·커널·RT / ROS·미들웨어 / 비전·AI / 제어·EtherCAT / 툴체인·RPU)
→ 완전성 비평(모순 11·공백 8·의심 7) → 보충 조사 4회 = **593개 사실(실측 435 / 문서 151 / 추론 7)**,
그 뒤 사용자가 root로 직접 확인한 2건(APU 클럭, SOM/캐리어 FRU) 반영.

**표기 규칙**: **굵은 값 = 보드에서 실측(라이브) 확인**. 굵지 않은 값은 문서·설정 파일 기준.
`근거` 열은 그 값을 다시 확인할 수 있는 명령 또는 파일:행.

**정본 관계**: 이 문서는 조사 시점의 스냅샷이다. 각 트랙의 정본(`docs/rt/rt_final.md`,
`docs/RAON-RT/merge.md`, `docs/vision/vision_final.md`, `docs/vision/throughput.md`)이 바뀌면
그쪽을 고치고 이 문서는 갱신하지 말 것. 단 §9(문서 vs 실측 불일치)는 **해당 문서들을 고칠 때
체크리스트로 쓸 것** — 보고서에 수치를 옮길 땐 반드시 이 문서의 실측값을 쓴다.

---

## 0. 한 줄 요약

> **AMD Kria KV260 Starter Kit** — SOM **SMK-K26-XCL2G** (P/N 5057-01, DT revA) + Carrier **SCK-KV-G** (P/N 5066-01, DT revB), 2021-08 제조 · Zynq UltraScale+ MPSoC (K26 = XCK26) · **Cortex-A53 ×4 @ 1.333 GHz 고정** · 4 GiB DDR4 · DPU **DPUCZDX8G B3136 @ 300 MHz** · **Ubuntu 22.04.5 + Linux 5.15.199-rt91 PREEMPT_RT (patch-5.15.197-rt91)**, 3+1 코어 격리 · **ROS 2 Humble + rmw_fastrtps_cpp (Fast DDS 2.6.11, SHM 16 MB)** · YOLOv3-tiny 6-class INT8 (Vitis AI 2.5) 21.7 Hz · **IgH EtherCAT 1.6.10 (generic) 1 kHz CST** + RAON-RT/RBDL 3.1.3 · Neuromeka Indy7 + RealSense D435i (FW 5.17.0.10)

---

## 1. 하드웨어 — 보드

| 항목 | 값 | 근거 |
|---|---|---|
| 보드 | **AMD Kria KV260 Vision AI Starter Kit**. DT model `ZynqMP KV260 revB`, U-Boot FIT config `conf-smk-k26-revA-sck-kv-g-revB` | `cat /proc/device-tree/model`, `/proc/device-tree/chosen/u-boot,bootconf` |
| SOM (FRU) | **SMK-K26-XCL2G** (Starter Kit 동봉 K26 SOM), P/N **5057-01**, revision `1` (=DT revA), S/N XFL1UANAK0SC, 제조 **2021-08-25 10:05**, 5 V 입력(4.5–5.5 V, max 4 A), FRU MAC `00:0a:35:0c:ff:64` | `sudo fru-print -b som` (2026-09-03 사용자 실행) |
| 캐리어 (FRU) | **SCK-KV-G**, P/N **5066-01**, S/N XFL1XXBCD0KB, 제조 **2021-08-25 01:03**, 12 V 입력(11.5–12.5 V, max 3 A). multirecord 체크섬 오류 있음(무해) | `sudo xmutil boardid` (첫 EEPROM 오류로 exit → SOM은 `--ignore-errors` 또는 `fru-print -b som`) |
| SoC | Zynq UltraScale+ MPSoC (K26 SOM 디바이스 = XCK26, ZU5EV급). 부품번호 문자열은 로컬 어디에도 없음 — AMD K26 자료 인용 | `/proc/device-tree/compatible` (`xlnx,zynqmp`) |
| APU | **Arm Cortex-A53 ×4 (r0p4, MIDR 0x410fd034)**, 1 cluster, aarch64 + 32-bit EL0, flags `fp asimd evtstrm aes pmull sha1 sha2 crc32 cpuid` | `lscpu`, `/proc/cpuinfo` |
| APU 클럭 | **1,333,333,320 Hz ≈ 1.333 GHz, 고정**. DT OPP 1333/666/444/333 MHz 중 최상. `CONFIG_CPU_FREQ=n`이라 DVFS·governor 없음 | `sudo cat /sys/kernel/debug/clk/clk_summary \| grep acpu` (사용자 실행); `/proc/device-tree/opp-table-cpu` |
| 캐시 | 토폴로지 실측: L1 I/D per core + L2 공유(0-3). 크기는 sysfs 미노출 → 32 KB I + 32 KB D /core, **1 MB L2** (UG1085; STATUS.md:549) | `/sys/devices/system/cpu/cpu0/cache/index*/` |
| RPU | Cortex-R5F ×2. **r5f_0로 remoteproc0 등록**(`xlnx,cluster-mode=<1>` — split/lockstep 매핑은 **미확인**, 상류 관례는 0=split), 상태 offline. FreeRTOS ELF 부팅(**펌웨어 부팅 검증 2026-09-02** 11:02·15:01 2회; 작업은 09-01~02) + **LED 태스크 실행**(점멸은 사용자 육안 확인). **제어 경로엔 미사용** → §7 | `/sys/class/remoteproc/remoteproc0/{state,name}`, `/proc/device-tree/rf5ss@ff9a0000` |
| GPU | Mali-400 (DT `arm,mali-400`) — 드라이버 blacklist, DP(`zynqmp_dpsub`)도 blacklist, 헤드리스 | `/etc/modprobe.d/blacklist-mali.conf`, `blacklist-zynqmp-dpsub.conf` |
| DPU (PL) | **DPUCZDX8G_ISA1_B3136 ×1**, IP v4.0.0 (2022-05-11, git 9bf4ccf), **DPU 300 MHz / XRT 300 MHz**, fingerprint **`0x101000016010406`**, Load/Save parallel 2. 오버레이 = **prebuilt `kv260-smartcam`** (bin 7.8 MB·xclbin·dtbo, 2024-06-25), `kv260-smartcam.service`가 부팅 시 `xmutil loadapp`. 두 번째 CU `pp_pipeline_accel`은 미사용. **Vivado/Vitis PL 설계 없음** | `xdputil query` (`~/vitis_ai_work/xdputil_query.json`), `/lib/firmware/xilinx/kv260-smartcam/`, `/sys/class/fpga_manager/fpga0/state` |
| 메모리 | **4 GiB** (DT `memory@0` 2 GiB@0 + 2 GiB@0x8_0000_0000; MemTotal 4,013,652 kB), DDRC `xlnx,zynqmp-ddrc-2.40a`. 타입/속도는 FRU에 레코드 없음 → **DDR4 64-bit non-ECC (DS987)**. **CMA 1000 MiB** (`cma=1000M`, DPU 버퍼) | `/proc/meminfo`, `hexdump /proc/device-tree/memory@0/reg` |
| 스왑 | **`/swapfile` 28 GiB** (문서의 "27G"는 반올림) | `/proc/swaps` |
| 저장장치 | **microSD SanDisk SD256 (238.3 GiB, 2024-12 제조)**: p1 1 GiB vfat `system-boot` → `/boot/firmware`, p2 237 GiB ext4 `writable` → `/` (사용 59 GB). KV260엔 eMMC 없음 | `lsblk`, `/sys/class/mmc_host/mmc1/…/cid` |
| 네트워크 | **eth0 = PS GEM3** (`ff0e0000`, Cadence macb, PHY TI DP83867 rgmii-id, MAC `00:0a:35:0c:ff:64`) → **EtherCAT 전용**, IP 없음 · **USB3 NIC ASIX AX88179B** (`0b95:1790`, OOT `ax_usb_nic` 4.1.0, 1 Gbps) → 원격/ROS 링크 **192.168.120.50/24** (`enxc8a362ec54c4`, NetworkManager `usb-net` static) | `ip -br link/addr`, `ethtool -i eth0`, `modinfo ax_usb_nic` |
| USB | xHCI (`fe200000`, IRQ 57→CPU0) + Microchip USB5734 hub: port1 D435i (5 Gbps), port4 AX88179B | `lsusb -t` |
| 콘솔 | UART1 `ttyPS1` 115200 (`serial-getty` 활성) | `/proc/cmdline` |
| 전원/냉각 | INA260(`ina260_u14`, SOM 5 V 레일): 유휴 **5.016 V / 1.2 A ≈ 6 W**; PWM 팬(`fancontrol.service`); 유휴 온도 LPD/FPD/PL **≈31 °C** | `/sys/class/hwmon/`, `xlnx_platformstats -a` |
| 부트 체인 | QSPI boot fw(FSBL/PMUFW/TF-A) → **U-Boot 2025.01-g5e0d8abc7e09** → `boot.scr.uimg` → **`image.fit`** (flash-kernel 생성, kernel+initrd+6 DTB) → `user-override.dtb` (RPU 노드, 09-02). GRUB 없음. PMU API v1.1, PSCI v1.1 | `/proc/device-tree/chosen/u-boot,version`, `ls /boot/firmware`, `mkimage -l image.fit` |

## 2. 주변장치 · 로봇 · PC

| 항목 | 값 | 근거 |
|---|---|---|
| 카메라 | **Intel RealSense D435i**, Serial **134322073014** (라벨 S/N; USB iSerial/ASIC S/N은 134523060150), **FW 5.17.0.10** (2026-07-14 5.16.0.1→업데이트, RGB UVC 웨지 해결), USB 3.2 (5 Gbps) APU 직결, IMU BMI085 미사용 | `rs-enumerate-devices`, `lsusb` |
| 스트림 | **color 848×480@30 RGB8, depth 848×480@30 Z16 (scale 0.001 m), enable_sync true, align_depth false, initial_reset true**; pointcloud/infra/IMU off. 1280×720은 노드 busy-spin 정지로 배제 | `src/system_bringup_pkg/config/realsense_pick_place.yaml`, `evidence/metrics/runs/shm_gate030_20260805/launch_console.log:57` |
| 내부 파라미터 | depth fx=fy=419.803, cx 423.589, cy 236.568; color fx 601.064, fy 599.600, cx 424.407, cy 230.548; 왜곡 0; depth→color baseline ≈15.09 mm | `docs/history.md:2246-2249` |
| 카메라 설치 | 광학 테이블 위 ~0.8 m top-down 고정. **핸드아이(eye-to-hand) 캘리브**: 16 poses, AprilTag 36h11 id0 65 mm, cv2.aruco+solvePnP IPPE_SQUARE, AX=ZB, **잔차 1.81 cm / 1.66°**; E21 베이스 보정 yaw +5.14°, t (+4.5, +10.4) cm → 배포 TF base_link→camera_link xyz (0.811666, 0.087684, 0.924974), q (0.72462829, 0.00008362, −0.68706199, 0.05347582) | `docs/RAON-RT/merge.md` §1.3, `launch/pick_place_vitis_ai.launch.py:244-276` |
| 로봇 | **Neuromeka Indy7 6-DoF 협동로봇**. Neuromeka STEP 컨트롤러 우회 → Kria eth0이 조인트 드라이브에 EtherCAT 직결 (`ROBOT_DOF=6`) | `~/RAON-RT-Revision/App/Indy7/INDY7.cfg:12`, `docs/STATUS.md:53` |
| 슬레이브 | **7개**: NRMK CORE 드라이브 ×6 (vendor `0x0000089a` / product `0x30000000`, "NRMK Drive rev1") + EOAT ×1 (`0x10000007`, "NRMK_Indy_Tool rev2"). Gate 1 스캔 = cfg 7/7 일치. CORE 모델번호는 미기록 | `INDY7.cfg` [AXIS0]–[AXIS6], `merge.md:296` |
| 드라이브 파라미터 | 엔코더 **65536 cnt/rev (16-bit)**, 기어비 **121 (J1–3) / 101 (J4–6)**, 토크상수 0.0884/0.0884/0.087/0.058×3 Nm/A, 전류비 48/48/96/96/96/96, 토크한계 117.73/117.73/47.5/21/21/21, 속도한계 150/150/150/178/178/178, `HOMING_METHOD=3` (E24 후), `ABSOLUTE_ENCODER=0`, `DRIVE_MODE=10` (CST) | `INDY7.cfg` lines 132–378 (라이브 편집본, git 대비 수정 있음) |
| URDF | `App/Indy7/indy7.urdf` (Neuromeka indy-ros2 유래). 링크 질량 1.670/11.444/5.848/2.682/2.130/2.224/0.383 kg (합 26.38 kg), **tcp = link6 +0.06 m, 질량 0**(페이로드 07-30 제거). 관절한계 ±175° (J5 ±215°), effort 431.97/431.97/197.23/79.79×3 Nm, 속도 2.618 (J0–2) / 3.1416 (J3–5) rad/s | `indy7.urdf` `<limit>`/`<mass>` |
| Indy7 데이터시트 | payload 7 kg / reach 800 mm / 반복정밀도 — **로컬 문서 어디에도 없음 → Neuromeka 데이터시트 인용** | `grep -rn payload\|가반\|reach` 결과 없음 |
| F/T 센서 | Robotous **RFT76-HA01** (Ø76×17 mm, 200 g) + 알루미늄 태그판(~85 g). EOAT 슬레이브 경유 INT16 (/50 N, /2000 Nm). URDF 285 g 모델링(07-27) → 제거(07-30, `49ac6db`); 모든 원샷 결과는 0 g | `~/RAON-RT_guide.md:1794`, `merge.md` §5/§9.5 |
| 그리퍼 | **없음**. EOAT PDO `0x7000:02` 출력만 존재, 파지는 하드웨어 대기 | `docs/STATUS.md:47`, `SlaveNRMKEndTool.h:118` |
| 데스크톱 PC | `jaehyeon@jaehyeon-Raimlab`, x86_64 **Ubuntu 22.04.5**, **NVIDIA RTX 4060 8 GB, RAM 16 GB** (+16 GB 스왑, 커널 빌드용), 디스크 여유 771 GB (07-06), nvidia-driver **580** proprietary (595-open은 CUDA 프리즈로 롤백, 정확한 580.x 빌드 미기록), IP **192.168.120.7**. 역할: RT 커널·ASIX 모듈 크로스빌드(`~/kria-rt`), YOLO 학습(`~/capstone_training`, torch cu121 venv), Vitis AI 2.5 도커, ROS 2 Humble 뷰어/오퍼레이터 GUI(`~/pp_ws/viewer_ws`). CPU 모델은 미기록 | 커널 배너 `/proc/version`, `docs/STATUS.md:120`, `docs/vision/yolov3_tiny_execution_plan.md:66`, `$SSH_CONNECTION` |
| R5 빌드 PC | Windows, Vitis 2022.1 (`d:\xilinx\vitis\2022.1`), GCC 11.2.0 arm-none-eabi, BSP `freertos10_xilinx_psu_cortexr5_0`, XSA `design_1_wrapper` | `strings ~/rpu_work/r5_freertos.elf` |

## 3. OS · 커널 · 실시간

| 항목 | 값 | 근거 |
|---|---|---|
| OS | **Ubuntu 22.04.5 LTS (jammy) arm64**. Canonical Kria 이미지("Project Limerick": `oem-limerick-kria-meta 0.17`, build serial 20240301). `multi-user.target` 헤드리스, TZ Asia/Seoul | `/etc/os-release`, `dpkg -s oem-limerick-kria-meta`, `/etc/cloud/build.info` |
| 커널 | **Linux 5.15.199-rt91-rt-kv260c #11 SMP PREEMPT_RT Mon Jul 20 12:24:18 KST 2026 aarch64**, deb `linux-image-5.15.199-rt91-rt-kv260c` **rev -11** (headers -11), `CONFIG_LOCALVERSION="-rt-kv260c"` | `uname -a`, `dpkg -s` |
| 베이스 소스 | **Ubuntu jammy `linux-xilinx-zynqmp` 5.15.0-1070.74** (upstream stable 5.15.199). vanilla도 Xilinx linux-xlnx도 아님 — zocl/usb5744/ap1302/vcu 등 벤더 드라이버가 이 트리에 in-tree | 커널 배너 `(Ubuntu 5.15.0-1070.74-xilinx-zynqmp 5.15.199)` |
| RT 패치 | **kernel.org `patch-5.15.197-rt91`** (projects/rt/5.15/older). 5.15.199 베이스에 dry-run reject 0 (198/199 stable에 fpsimd/스케줄러 변경 없음). `/sys/kernel/realtime = 1` | `docs/rt/rt_patch.md` §3, `rt_final.md` §2 |
| 핵심 config (on) | **PREEMPT_RT=y, HZ=1000, NO_HZ_FULL=y, RCU_NOCB_CPU=y, RCU_BOOST=y, HIGH_RES_TIMERS=y, CPU_ISOLATION=y, IRQ_FORCED_THREADING=y, RT_MUTEXES=y, ARM64_PSEUDO_NMI=y, NR_CPUS=4, CMA=y** | `zcat /proc/config.gz` |
| 핵심 config (off) | **CPU_FREQ=n, CPU_IDLE=n (ACPI_PROCESSOR=n), KVM=n** (arm64 `ARCH_SUPPORTS_RT if !KVM`), **PREEMPT_LAZY=n, DEBUG_PREEMPT=n, DEBUG_ATOMIC_SLEEP=n, PROVE_LOCKING=n, FTRACE=n, DEBUG_INFO=n**, XILINX_TSN=n, ARM_SMMU=n. 결과: `/sys/.../cpufreq`·`cpuidle`·`/dev/cpu_dma_latency` 없음 | `zcat /proc/config.gz`; stock 대비 diff 305줄 (`evidence/kernel_configs/diffconfig_stock_vs_rt91.txt`) |
| 커스텀 소스 패치 | ① **radix-tree local_lock 복원** — Ubuntu SAUCE `05fdd323982c`(mainline `cfa6705d89b6` revert)가 RT에서 idr_preload를 atomic으로 만들어 SLUB/fpsimd 오염 → `lib/radix-tree.c`, `radix-tree.h`, `idr.h`를 vanilla 5.15.199로 복원 (위반 253→0) ② **zocl `kds_core.c` KDS UAF 픽스** — `set_xcmd_timestamp(KDS_QUEUED)`를 submit 전으로 (3곳; upstream XRT에도 존재) ③ `PREEMPT_LAZY` Kconfig 선택가능화 후 off | `tools/kernel_patches/{radix-fix,zocl}`, `/proc/kallsyms` `radix_tree_preloads` 익스포트, `modinfo zocl` srcversion `0754F2D6…`(패치본) |
| 빌드 | x86 PC 크로스빌드 **aarch64-linux-gnu-gcc 11.4.0 (Ubuntu 11.4.0-1ubuntu1~22.04.3), GNU ld 2.38**, `make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bindeb-pkg`; DTB를 `/lib/firmware/<ver>/device-tree/xilinx/`에 먼저 복사 후 `dpkg -i` + `flash-kernel`. **#11 = #10 + `CONFIG_USB_NET_AX88179_178A=m` 1줄** (deb 두 개 config diff로 확인) | `/proc/version`, `dpkg-deb --fsys-tarfile … \| tar -xO ./boot/config-*` |
| cmdline | **`root=LABEL=writable rootwait earlycon console=ttyPS1,115200 console=tty1 clk_ignore_unused uio_pdrv_genirq.of_id=generic-uio xilinx_tsn_ep.st_pcp=4 cma=1000M skew_tick=1 isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2`** (앞부분 vendor 템플릿, 뒤 `skew_tick=1 …`가 `/etc/default/flash-kernel` `LINUX_KERNEL_CMDLINE`) | `/proc/cmdline` |
| CPU 레이아웃 | **3+1**: CPU0–2 = Linux·ROS·비전(SCHED_OTHER)·전 IRQ(`irqaffinity=0-2`, eth0 IRQ 37→CPU1, xhci IRQ 57→CPU0), **CPU3 = 격리(isolcpus+nohz_full+rcu_nocbs) → 1 kHz 제어 페어 FIFO 97/95**. 2+2(07-08)는 비전 2.8코어 시절 CPU0/1 기아로 폐기, 07-27 3+1 재도입 | `/sys/devices/system/cpu/{isolated,nohz_full}`, `merge.md` §4.1 |
| 타이머 | clocksource **`arch_sys_counter` 100 MHz** (sched_clock 57-bit @99 MHz, 10 ns), tick HZ 1000, CPU3는 tick 억제 확인(arch_timer IRQ CPU3 383 vs CPU0 2.7 M) | `/sys/devices/system/clocksource/clocksource0/current_clocksource`, `/proc/interrupts` |
| RT 권한/제한 | `limits.d/99-realtime.conf`: **ubuntu rtprio 98, memlock unlimited**; `sched_rt_runtime_us 950000 / period 1000000` (기본, 미해제); `timer_migration 1`; irqbalance 활성이나 `isolated` 존중(IRQ 52개 0-2 확인); multipathd RR 99 상주(미관리) | `ulimit -r -l`, `sysctl`, `for i in /proc/irq/*; cat $i/smp_affinity_list` |
| Xilinx 런타임 | **XRT 2.13.479 (2022.1), zocl 2.13.479 (in-tree, `/dev/dri/card0`), xmutil 1.2.1, dfx-mgr 2023.1, fpga-manager-xlnx 2022.1, xlnx-firmware-kv260-smartcam 0.12, flash-kernel 3.104ubuntu22 (db override `Kernel-Flavors: any`)** | `dpkg -l \| grep -iE 'xrt\|xlnx\|dfx'`, `/etc/flash-kernel/db` |
| 이미지·복구 | `/boot/firmware/image.fit` (2026-08-20), `.bak` (07-27), `image.fit.stock-1070`; stock 5.15.0-1070 커널 유지(`apt-mark hold`), RT 실험 커널 -rt-kv260(rev5)/-kv260b(#8) 공존; `recovery/`에 -11 deb + 보드 백업 tar | `ls /boot /boot/firmware`, `apt-mark showhold`, `~/ros2_ws/recovery/README.md` |
| 지연 ① cyclictest | `cyclictest -m -S -p90 -i200` (rt-tests 2.2, 코어당 1스레드 핀) on **kv260c #10, 격리 없음, 4코어를 stress-ng(`--cpu 4 --io 2 --vm 1`)+hackbench와 공유**: IDLE 60 s Max **112/84/105/134 µs** (Avg 11–13); LOADED 120 s Max **142/137/127/136 µs** (Avg 14–19), 커널 위반 0. DEBUG-on kv260b는 LOADED Max 189–282 µs. `Max<100 µs`는 참고 문구이지 판정 게이트가 아님 | `evidence/crash_logs/cyclic_20260715-013556.log`, `tools/rt/cyclic_rt.sh`, `rt_final.md` §7 |
| 지연 ② gate0 (격리 A/B) | RT-POSIX 주기 태스크 1 kHz 주기오차 \|dt−T\| (SCHED_FIFO 80, 20 s, mlockall, 앱 정지·비전 가동, 2026-07-28 17:02): **격리 CPU3 avg 4.3 / max 51.1 µs** vs **CPU0 avg 7.3 / max 114.6 µs**. 07-27 비격리 기준(33.9/104 µs)은 다른 세션이라 A/B 아님. cyclictest와 다른 도구·기준(PASS < 500 µs) — 같은 열에 놓지 말 것 | `merge.md:43-45,189,575-579`, `~/RAON-RT-Revision/tools/gate0_rtposix_test.c` |
| 지연 ③ 실운전 | DataLog 타임스탬프(RT 루프 내부, 로봇 토크모드, 풀스택): `DataLog_20260727_000121.csv` 23,999 주기 **mean 1000.00 / min 915.4 / max 1086.0 µs, >1.1 ms 0건**; `DataLog_20260805_222055.csv` 3,297 주기 min 943.6 / max 1073.5 µs | `awk` over `App/Indy7/rt_log_results/*.csv` (이번 조사에서 계산) |
| 소크 | `soak_rt.sh` (stress-ng cpu4 load70 vm2 fork4 …) 840 s, DEBUG_PREEMPT/ATOMIC_SLEEP 무장 → 위반 0 (2026-07-13) | `evidence/crash_logs/soak_rt_20260713-162425.log` |
| 미측정 | **격리 CPU3 운영조건(비전+DPU+EtherCAT 가동) cyclictest** — 계획만 있고(`-a 3 -i 1000 -D 5m`) 로그 없음. E34 당시 load 6에서 178 µs는 스트레스 수치 | `docs/STATUS.md:548`, `merge.md:702` |

## 4. ROS 2 · 미들웨어 · 언어 런타임

| 항목 | 값 | 근거 |
|---|---|---|
| ROS 2 | **Humble** — `ros-humble-ros-base 0.10.0`, `ros-core 0.10.0`, **rclcpp 16.0.19, rclpy 3.3.21**, rcl 5.3.13, rcutils 5.1.8 (ros-* 239 패키지; desktop 메타 없음, rviz2 11.2.26 별도) | `dpkg -l \| grep ^ii.*ros-humble` |
| RMW | **`rmw_fastrtps_cpp` 6.2.10** — launch 파일 `SetEnvironmentVariable`로 설정(07-15부터; `.bashrc`엔 없음, 주석으로 금지) | `launch/pick_place_vitis_ai.launch.py:96-100,183-184`, `ros2 doctor --report` |
| DDS | **eProsima Fast DDS 2.6.11**, fastcdr 1.0.29, foonathan-memory 1.2.0. **Cyclone DDS** (`rmw_cyclonedds_cpp 1.3.4`, cyclonedds 0.10.5, iceoryx 2.0.5) 설치되어 있으나 **미사용** (07-14까지 사용 → Fast DDS+SHM 전환으로 CPU −6.6 %p) | `dpkg -l`, `docs/STATUS.md:242` |
| 트랜스포트 | XML 프로파일 `fastdds_shm_profile.xml`: `useBuiltinTransports=false`, **SHM `segment_size` 16 MB + UDPv4 fallback**. 기본 512 KB로는 848×480 Image(1.22 MB)가 조용히 UDP loopback으로 폴백. Fast DDS 2.6.11엔 SHM 크기용 env var 없음 → XML 필수. Data Sharing(zero-copy)은 `sensor_msgs/Image`가 unbounded라 불가 | `src/system_bringup_pkg/config/fastdds_shm_profile.xml`, `docs/STATUS.md` §4.2 |
| 도메인 | ROS_DOMAIN_ID 0, ROS_LOCALHOST_ONLY 0. 데스크톱은 RMW/도메인/프로파일 **unset**(Humble 기본 fastrtps) → 보드와 UDPv4로 통신 | env, `launch_script/viewer.sh:45-60` |
| 주요 패키지 | cv_bridge 3.2.1 (핫패스는 `np.frombuffer` 직접 사용), image_transport 3.1.12, **compressed_image_transport 2.5.5** (lazy JPEG, 데스크톱 뷰어용), tf2/tf2_ros 0.25.20, sensor_msgs/geometry_msgs 4.9.1, vision_msgs 4.1.1(미사용), rosbag2 0.15.16, message_filters 4.3.16 | `dpkg -l` |
| RealSense | **`ros-humble-librealsense2 2.57.7`, `ros-humble-realsense2-camera 4.57.7`** (apt 바이너리). `src/realsense-ros`는 v4.57.7 소스 클론이나 `COLCON_IGNORE`(미빌드). `~/librealsense` v2.55.1 소스 별도. pyrealsense2 미설치 | `dpkg -l \| grep realsense`, `ros2 pkg prefix realsense2_camera` |
| 워크스페이스 | 보드: `system_bringup_pkg`(launch+config 6종), `vitis_ai_detector_pkg`, `pick_logic_pkg`, `target_3d_pkg` (모두 ament_python 0.0.0), `my_interfaces` (ament_cmake: Detection/DetectionArray/PickTarget/PickTarget3D). 데스크톱: `detection_viewer_pkg`(python), `raon_operator_gui` 0.1.0 (C++ Qt5 + rviz plugins). git `seriouslysilly21612/Capstone_Design` main `f91be3f` (2026-08-19) | `ros2 pkg list`, `package.xml`, `git log` |
| 빌드 | **colcon-core 0.20.1** (`--symlink-install` 필수), ament_cmake 1.3.14, rosdep 0.26.0, vcstool 0.3.0, **cmake 3.22.1, gcc/g++ 11.4.0** | `pip3 show`, `dpkg -l` |
| Python | **3.10.12**, numpy 1.21.5, **OpenCV 4.5.4** (`python3-opencv` + `libopencv-*` 4.5.4+dfsg-9; pip opencv 미설치), scipy 1.8.0, matplotlib 3.5.1, PyYAML 5.4.1, psutil 5.9.0, Pillow 9.0.1 | `pip3 list`, `python3 -c 'import cv2'` |
| QoS | 이미지/depth/camera_info 구독 **KEEP_LAST 1 · BEST_EFFORT · VOLATILE** (최신 프레임만); `/detections` 발행 KEEP_LAST 1 RELIABLE; extrinsics 구독 RELIABLE TRANSIENT_LOCAL. 모든 보드 노드 `rclpy.spin` 단일 스레드 | `vitis_ai_detector_node.py:237-248`, `pick_target_3d_node.py:268-278` |
| 노드 체인 | `realsense2_camera`(rs_launch 포함) → `vitis_ai_detector_node`(+VART worker 서브프로세스) → `pick_logic_node` → `pick_target_3d_node` → static TF `base_to_camera_tf` → `pick_target_base_node`. 단일 launch, 7 프로세스. 노드 병합 실험은 rclpy executor 비용으로 +5.4 pt CPU → 기각 | `launch/pick_place_vitis_ai.launch.py:182-296` |
| 토픽 | `/camera/camera/color/image_raw` (848×480 RGB8 30 Hz) → `/detections` → `/pick_target` → (+`depth/image_rect_raw` 16UC1, camera_info, `extrinsics/depth_to_color`) → `/pick_target_3d` → `/pick_target_base` (base_link, m). 옵션 `/camera/…/compressed` (lazy JPEG, 뷰어용) | 각 config yaml, README 다이어그램 |

## 5. 비전 · AI 가속

| 항목 | 값 | 근거 |
|---|---|---|
| 모델 | **YOLOv3-tiny, 6-class** (0 apple, 1 orange, 2 banana, 3 tennis_ball, 4 mustard_bottle, 5 person — person은 worker에서 억제). 입력 **416×416 letterbox**(pad 114), 2-head stride 16/32 (26×26·13×13, 33 ch = 3 anchors×(6+5)), anchors [10,14][23,27][37,58] / [81,82][135,169][344,319], yolov5식 decode. 활성화 **Hardswish** (SiLU는 DPUCZDX8G 미지원 op → `models/yolov3-tiny-hswish.yaml`) | `src/vitis_ai_detector_pkg/models/decode_meta.json`, `docs/vision/yolo_v3_process.md` §7 |
| 파라미터 | **≈8.68 M** (xmodel 상수 텐서 합 8,678,242; conv 가중치 8,674,992). 문서 "≈6.9 M"은 오류 | `xir.Graph.deserialize` 직접 합산 (이번 조사) |
| 학습 | **ultralytics yolov5 v7.0** repo (`models/hub/yolov3-tiny.yaml`), PyTorch cu121 native venv, COCO-pretrained `yolov3-tiny.pt` 파인튜닝, `--img 416 --batch 64 --epochs 150`, SGD lr0 0.01 lrf 0.01 cos, momentum 0.937, wd 5e-4, warmup 3 ep, hsv_v 0.50, degrees 180, flipud/fliplr 0.5, mosaic 1.0, mixup 0.05; 배포 런 `hswish5` 2.528 h (RTX 4060) | `yolo_v3_tiny_training/scripts/11_train.sh`, `config/hyp_pickplace.yaml` |
| 데이터셋 | 7-class 시점 **train 15,799 / val 2,093** (val: synth 640 + ycbv 384): BlenderProc 합성 YCB 006/011/013/015/017/056 (≈6,400장, 고도 55–90°, 거리 0.55–1.05 m, 회색 테이블) + COCO(apple/orange/banana/person, person 8,000·과일 1,500/종) + Open Images(tennis 1,000) + YCB-Video/BOP ycbv + **YCB real scans 600/물체(D13)** + 실카메라 apple 54장(D14, train only). 6-class 최종 수량 미기록(데스크톱 `datasets/final/{train,val}.txt`). peach는 D13에서 제외 | `docs/vision/yolov3_tiny_execution_plan.md`, `yolo_v3_tiny_training/config/classes.yaml` |
| 정확도 | float mAP@0.5: 7-class SiLU 0.758 → hswish 0.766 → 6-class D13 0.748 → **배포 D14 0.728** (apple 0.625, mustard 0.933, tennis 0.950). **INT8 mAP 미측정** — 양자화 검증은 cosine(0.9757/0.9615) + 실물 Gate 5(apple 0.88, orange 0.85, banana 0.78, mustard 0.81, tennis 0.68 @thr 0.50) | `execution_plan.md:99,109,334,355`, `docs/STATUS.md:18,367` |
| 양자화·컴파일 | **Vitis AI 2.5** docker `xilinx/vitis-ai-cpu:2.5.0.1260` (conda `vitis-ai-pytorch`, 데스크톱 CPU): `vai_q_pytorch` **INT8 PTQ** (calib 500장, test 100장 cosine, deploy batch 1; QAT 없음) → `vai_c_xir -a arch_b3136.json` → **DPU subgraph 1 / 전 연산자 DPU**. VAI 2.5 `hardswish.py` 버그 2건 sed 패치 | `docs/vision/vision_final.md` §7, `yolo_v3_tiny_training/scripts/12_run_quantize_docker.sh` |
| 배포 xmodel | `models/yolov3_tiny_7class.xmodel` **9,311,460 B, md5 `9bc6520cd05a4a914f8b95980133a8df`** (파일명 7class, 내용 6-class D14). 입력 fix_point 6, 출력 26×26 head 2 / **13×13 head 3**(문서 "둘 다 2"는 오류, 런타임은 텐서별로 읽어 무영향). 계보 e2ca87c2(7cls)→d925c711(D13)→9bc6520c(D14). `~/vitis_ai_work/models/`는 삭제됨 | `md5sum`, `xir` fix_point 조회 |
| 보드 런타임 | **vitis-ai-runtime / vitis-ai-library 2.5.0.20220720.f8f2805**, libvart/libxir/libunilog/libtarget-factory 2.5.0 (2022-07-20 빌드), python `vart`/`xir` (cpython-310), `/etc/vart.conf` → smartcam xclbin | `dpkg -l \| grep vitis`, `xdputil query` VAI Version |
| 디텍터 구조 | ROS 노드(`vitis_ai_detector_node.py` 1,032줄) + **별도 worker 서브프로세스**(`vitis_ai_worker_yolo.py` 467줄, xir/vart만; rclpy 안에서 VART `execute_async` segfault 회피). JSON-line stdin/stdout + **`/dev/shm` mmap 프레임**(`worker_shm: true`, 08-05). `process_period_sec 0.030`, thr **0.50**, NMS IoU **0.45**, top-K 100, **person thr 2.0(=OFF)**; objectness INT8 사전필터; letterbox LUT 전처리 | `src/vitis_ai_detector_pkg/…`, `config/vitis_ai_detector.yaml` |
| 3D 추정 | bbox 중심 color 픽셀 → **depth 이미지 epipolar 선분 탐색 역투영**(rs2_project_color_pixel_to_depth_pixel 등가, numpy 벡터화, 풀프레임 align 없음), 9×9 median, 0.19–1.6 m, depth frame은 `nearest_by_stamp`. `pick_target_base`: static TF (R,t) 캐시, camera z 게이트 0.20–**0.95** m (2026-09-03 재유도 — 종전 1.50은 캘리브 이전 placeholder) | `config/target_3d.yaml`, `target_base.yaml`, `docs/vision/reprojection.md` |
| pick_logic | min_conf 0.3, allowed 5클래스, edge 30 px, bbox 400 px²–50 %, score 0.5 conf+0.3 center+0.2 area, 안정 3프레임/60 px (person guard는 2026-09 제거) | `config/pick_logic.yaml` |
| 성능 (채택 런) | `shm_gate030_20260805` (300 s, 6,343 프레임, 현재 yaml과 동일 구성): **21.7 Hz** (period p50 45.5 ms), **E2E p50 118.2 / p95 142.5 / p99 160.7 ms**, **CPU 1.60 cores**, processing p50 43.0 (pre 11.6 + **DPU 17.35** + post 4.8), ipc 4.65 ms | `evidence/metrics/runs/shm_gate030_20260805/performance/{base,detector}.csv`, `docs/vision/throughput.md` §4 |
| 성능 이력 | 07-15: 15 Hz(0.045 gate)/E2E 81 ms(디텍터측 정의)/CPU 76.8→~44 %(3.07→1.8 cores) · 07-31 A_production: 17.0 Hz / 113 ms / 1.31 cores · SDK align 대안(B): 14.2 Hz / 355 ms / 1.69 cores → 기각. DPU 이론 한계 ≈56 Hz(28 % 사용) | `vision_final.md` §9, `evidence/metrics/runs/ab_compare_20260731.md` |

## 6. 실시간 제어 · EtherCAT

| 항목 | 값 | 근거 |
|---|---|---|
| 프레임워크 | **RAON-RT** ("Real-time Architecture for Open, Networked robotic systems"), C++ (`-std=gnu++17 -O2 -mtune=native`; 원본 `-O3 -flto`는 주석), GNU make, 계층 App→CRobot(HAL)→EMasterApp(IgH 래퍼)→Global. **원저 KIST AIRI (R. Delgado, 2022, 55 파일 헤더) / RAIMLAB@명지대 유지보수 (2025, 36 파일 헤더, git.raimlab.com)**. 작업 fork **`RAON-RT-Revision` 브랜치 `kv260-merge` HEAD `4b2113a`** (2026-08-06); `~/RAON-RT`(dev `596691f`)는 원본 읽기전용 | `~/RAON-RT/Makefile:3-6`, `git -C ~/RAON-RT-Revision log`, `App/Indy7/Makefile:30-45` |
| RT 라이브러리 | **RT-POSIX** `librtposix` (SeoulTech ESL, RT-AIDE, R. Delgado 2022, GPLv2, C 1,087 LOC): `SCHED_FIFO` + `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`, 기본 CPU0 핀 → cfg `CPU=` 키로 오버라이드; aarch64 최소 스택 패치(`rtposix-aarch64-stackmin.patch`), `/opt/rt_posix` | `~/RT-POSIX/src/posix_rt.c`, `include/posix_rt.h:2-6` |
| 태스크 | **TASK0 Main Control SCHED_FIFO 97 · 1 ms · CPU3**, **TASK1 EtherCAT Control FIFO 95 · 1 ms · CPU3**, TASK2 키보드 50, TASK3 프린터 49 (1 s), TASK4 로거 30, TASK5 Visual Servo 비활성. `mlockall(MCL_CURRENT\|MCL_FUTURE)`, 스택 2 MiB/태스크, `MALLOC_ARENA_MAX=2`, run.sh가 rtprio≥97·memlock unlimited 검사 | `INDY7.cfg` [TASK0]–[TASK5], `App/Indy7/run.sh` |
| 제어법칙 | **CiA402 CST (mode 0x0A) + RBDL 완전동역학 computed-torque, 1 kHz**: `CompositeRigidBodyAlgorithm`→M(q), `NonlinearEffects`→h, `InverseDynamics(q,0,0)`→g; 관절 PD **KP 800/600/400/200/150/100, KD 80/60/40/28/25/20**; **Coulomb 마찰 FF KF 0/0/4.0/6.0/3.0/0 Nm** (속도공간 lag gate, 50 ms LPF, refine 스코프 게이트); 토크 백스톱 = URDF effort 클램프 + non-finite → 전축 0 | `FullDynControllerRT.cpp:290-426,1876,1910`, `INDY7.cfg:13-51` |
| IK · 궤적 | RBDL `InverseKinematics` (max_steps 300, tol 1e-10), 결정적 ready-seed(`~/.indy7_ready_seed`) + 자세가중 사다리 + warm-start 생략 solve + **8-seed multi-start** (커버율 95.1→99.4 %), 브랜치 플립 가드(E27/E28), tilt 60°; 관절공간 5차(min-jerk) 0.6 rad/s, 자동 스테이징(Δq>2.0 rad), refine tol 12 mm×6회; 트래킹 서보 DLS λ 0.01, **vmax 0.20 m/s**, EMA 0.4, lost 500 ms, v3 하이브리드(>0.15 m는 궤적 스택) | `FullDynControllerRT.h:83-156,388-402`, `Indy7Ctrl.h:188-193`, `merge.md` D13/D15 |
| 동역학 라이브러리 | **RBDL 3.1.3** (`rbdl-orb` ORB-HD fork commit `0606f2c` 2022-04-20 + urdfreader addon, 소스 빌드 → `/usr/local/lib/librbdl.so.3.1.3`), **Eigen 3.4.0** (`libeigen3-dev 3.4.0-2ubuntu2`), yaml-cpp 0.7.0, jsoncpp 1.9.5 | `/usr/local/include/rbdl/rbdl_config.h`, `git -C ~/rbdl-orb log -1` |
| EtherCAT 마스터 | **IgH EtherCAT Master 1.6.10** (gitlab `etherlab.org/ethercat` stable-1.6, tag 1.6.10, HEAD `c4b4ac40`), **보드 네이티브 빌드** (`~/prep_kbuild_native.sh`로 fixdep/modpost 재빌드) `./configure --prefix=/opt/etherlab --enable-generic --enable-8139too=no --enable-cycles=no --enable-hrtimer=yes --enable-eoe=no --with-linux-dir=/lib/modules/5.15.199-rt91-rt-kv260c/build`; `ec_master.ko`/`ec_generic.ko` 1.6.10 (vermagic preempt_rt, unsigned), `libethercat.so.1.2.0` (ECRT 1.6). 1.5.2는 5.15에서 빌드 불가 | `/opt/etherlab/bin/ethercat version`, `modinfo ec_master`, `~/ethercat/config.log` |
| 드라이버 · NIC | **generic** (`DEVICE_MODULES="generic"`, macb 네이티브 EtherCAT 드라이버 없음 → 커널 넷스택 경유, **~1 ms 주기 천장**). `MASTER0_DEVICE="00:0a:35:0c:ff:64"` = eth0 GEM3 (FRU PS MAC과 일치). 마스터는 세션마다 수동 `ethercatctl start` (`ethercat.service` disabled). udev `/dev/EtherCAT*` 0664 ubuntu | `/opt/etherlab/etc/ethercat.conf`, `/etc/udev/rules.d/99-ethercat.rules` |
| 버스 설정 | eth0 RJ45 → Indy7 베이스 직결(스위치 없음). **`CYCLE_TIME=1000000` ns (1 kHz), `SLAVENUMBER=7`, `DC_ENABLED=1`(마스터 app time/ref clock) / 슬레이브 DC off (SM-sync, 저자 검증 구성; DC 64-bit 지원, ~600 ns/hop)**, 2 domains → **2000 frames/s, Lost 0** (300 s 서보-off 홀드; 접근 런 델타 0), PREOP→OP 시 ~2 프레임 손실 | `INDY7.cfg:127-149`, `merge.md:42,296,300,380` |
| PDO 맵 | 드라이브당 **10 entries**: Rx `0x1600` 6040:16/607A:32/60FF:32/6071:16/6060:8, Tx `0x1A00` 6041:16/6064:32/606C:32/6077:16/6061:8 → SM2/SM3 13 B, 고정 맵. (0x603F, 0x2fe4:02 등 9건은 주석 처리) **SM watchdog 100 ms** (0x0400=2498, 0x0420=1000) — 14.9 ms IK 스톨에도 Lost 0 | `EMasterApp/Device/SlaveCIA402Base.cpp:425-485`, `merge.md:403` |
| EOAT 슬레이브 | `CSlaveNrmkEndtool` (`0x089a`/`0x10000007`): F/T INT16(/50, /2000), 그리퍼 PDO 0x7000:02, RGB LED, 버튼; DC activate 0x0300 | `EMasterApp/Device/SlaveNRMKEndTool.h:14-33,118` |
| E34 대책 | IgH `EtherCAT-OP` 커널스레드가 SCHED_NORMAL nice 0 + PI 없는 `struct semaphore master_sem` → 부하 시 1 kHz ioctl이 뒤에 막힘(2–20 ms 구멍, 최대 66 % 주기 손실). **`/usr/local/sbin/ecat-op-fifo` (`chrt -f -p 40`, sudoers NOPASSWD, 설치 확인 2026-07-31) — run.sh가 앱 기동 후 자동 적용**, 40 < TASK1 95. 운영 수칙: 실기 중 보드에서 무거운 작업 금지 | `~/ethercat/master/master.c:1681`, `master.h:198`, `App/Indy7/run.sh:38-49`, `tools/install_ecat_op_fifo.sh` |
| 안전 | `ENABLE_CONTROLLER_AT_STARTUP=0`, `AUTO_SERVO_ON=0` (r→g→h 순서, servo-on은 중력보상 모드에서만), 소프트리밋 ±175°/±215° (fold-toward-zero 후 판정, E36), 워크스페이스 박스 x 0.30–0.94 / y −0.37–0.63 / z 0.10–0.50 m (r≤0.92), hover 마진 0.15 m, goal z≥0.25 m, 속도캡 0.20 m/s (코드 클램프 0.05–0.50), 키 화이트리스트, E-stop = 버스 전원 차단(→eth0 링크 다운, 0 slaves) | `INDY7.cfg:9-10,52-62`, `WorkspaceBox.h`, `merge.md` §5 Phase 4 |
| 인터페이스 | ROS2 브리지 `raon_pick_bridge` (rclcpp, 비-RT 스레드): 구독 `/pick_target_base`, `/detections`, `/operator_key|gains|jog`; 발행 `/joint_states`, `/robot_state`, `/operator_msg`. TCP ext-interface 7420. 데스크톱 `raon_operator_gui` (Qt, `operator.launch.py`) | `App/Indy7/ROS2PickBridge.cpp:19-130`, `INDY7.cfg:5` |
| 성과 | 접근 정밀도: 첫 실기 10건 3.07–11.52 mm → refine 160→10.9 mm → **원샷 #52–58 7/7 (HOME 5물체 6.9–10.7 mm, 물체→물체 11.3 mm) / #78–82 5/5 (orange 6.15 mm 최고, KF_3 6.0)**; KF=0 역검증 111 mm. 오차는 −x/−z 바이어스 지배 | `merge.md` §0/§5/§9.5, `evidence/approach_accuracy/` |

## 7. RPU 트랙 (진행 중, 제어 경로 밖)

| 항목 | 값 | 근거 |
|---|---|---|
| 코어·모드 | Cortex-R5F-0, `xlnx,cluster-mode=<1>` (dtso 원값. **1=split인지 lockstep인지는 미확인** — 보드에 커널 소스가 없어 대조 불가, 상류 바인딩 관례는 0=split; R5-1 미사용), `CONFIG_ZYNQMP_R5_REMOTEPROC=m`, `ZYNQMP_IPI_MBOX=y`, `RPMSG_CHAR=m` | DT, `/proc/config.gz` |
| 펌웨어 | **FreeRTOS 10** (Vitis 2022.1 BSP `freertos10_xilinx`, Windows 빌드, GCC 11.2.0 arm-none-eabi), `/lib/firmware/r5_freertos.elf` 505,088 B. **2026-09-02 11:02 / 15:01 두 차례 "is now up"** ("no resource table found" — rpmsg 없음), 현재 offline. **판정 수단 = LED**(`led35_task`/`led36_task` 심볼; UART 아님) — RPU 단독 FreeRTOS 부팅 + LED 태스크 실행(**작업 2026-09-01~02, 펌웨어 부팅 검증 2026-09-02**)으로 **Gate 1 실질 통과**. 점멸 자체는 **사용자 육안 확인**이고 `/sys/class/leds/`에 led35/36이 없어 보드에서 재검증 불가 | `/var/log/kern.log`, `strings`/`nm r5_freertos.elf` |
| 메모리 맵 (적용본) | reserved `rproc@36000000` **0x36000000 4 MiB no-map** + TCM `0xffe00000`/`0xffe20000` 64 KiB×2 (mmio-sram), vring/IPI 노드 없음 — `~/rpu_work/rpu_v2.dtso` → `/boot/firmware/user-override.dtb`(boot.scr가 로드). **저장소 안 `tools/rpu/rpu_rproc.dtso`(0x3ed00000 + vring + IPI)와 다름 — `rpu_work`도 `tools/rpu/`도 둘 다 git 미추적**(`git ls-files tools/rpu/` 공백, `git status` → `?? tools/rpu/`) | `/proc/device-tree/reserved-memory/`, `~/rpu_work/`, `strings /boot/firmware/boot.scr.uimg` |
| 계획 · 금지 | Phase C: R5F FreeRTOS + SOEM v1.4.x, 1 kHz EtherCAT on GEM3, OpenAMP rpmsg. **남은 과제 3**: ① APU↔RPU 통신(rpmsg — 현 DT에 vring·IPI mailbox 노드가 없고 ELF에 resource table도 없어 DT 재작업 + 펌웨어 재빌드 필요) ② RAON-RT 실시간 제어 task의 RPU 이식 ③ EtherCAT SOEM의 RPU 설치. **GEM3 핸드오버(Phase 2+)는 APU IgH가 프로덕션이라 사용자 재결정 전 금지** — ①②는 GEM3와 무관해 선행 가능. SOEM/OpenAMP 소스 보드에 없음 | `docs/rpu/rpu_guide_for_claude.md` §1–§2, `docs/STATUS.md:57,102-131` |

## 8. 개발 도구 · 저장소 · 증거

| 항목 | 값 | 근거 |
|---|---|---|
| 보드 툴체인 | **gcc/g++ 11.4.0 (Ubuntu 11.4.0-1ubuntu1~22.04.3), cmake 3.22.1, GNU make 4.3, glibc 2.35, binutils 2.38, dtc 1.6.1, u-boot-tools 2022.01**, Docker 29.1.3 (보드에선 `xilinx/smartcam:2022.1` 이미지만). clang/ninja/rust/go/java 없음 | `gcc --version` 등 |
| RT 테스트 | **rt-tests 2.2-1 (cyclictest V2.20), stress-ng 0.13.12**, `tools/rt/{cyclic_rt,soak_rt,churn,sustained,fpu_report}.sh` | `dpkg -l`, `ls tools/rt` |
| 제어 툴 | `tools/{approach_plot.py, reachmap.cpp(+plot), fk_replay.cpp, calib_grab/calib_cpo/base_shift_fit.py, gate0_rtposix_test.c, gate2b_bridge_test.cpp, test_phase3_smoke.py, install_ecat_op_fifo.sh}`; DataLog 1파일=1접근 → `rt_log_results/`, `approach_results/` (112 CSV, 07-26~08-05) | `ls ~/RAON-RT-Revision/tools`, `App/Indy7/DataRecorder.h` |
| 메트릭 툴 | `tools/metrics/{ab_one_run.sh, join_perf.py, plot_metrics(_paper).py, resource_sampler.py}` (launch `metrics:=true`로 300 s 샘플링) | `ls tools/metrics` |
| IDE · AI | VS Code Remote-SSH (server 1.118.1; cpptools 1.33.8 — **실측 전 C++ 파일 열지 말 것**, 인덱싱 ~150 % CPU), **Claude Code 2.1.235** (보드 상주), **graphify 0.9.32** (`graphify-out/` 08-19·08-29·09-01), Node 22.23.2; `CODEX.md`도 존재 | `claude --version`, `graphify --version`, `~/.vscode-server/extensions` |
| 저장소 | `~/ros2_ws` → `github.com/seriouslysilly21612/Capstone_Design` main `f91be3f` (태그 없음) · `~/RAON-RT-Revision` → `…/RAON-RT-Revision` kv260-merge `4b2113a` (태그 `pre-cleanup-20260731`) · `~/RAON-RT` → `git.raimlab.com/RAIMLAB/RAON-RT` dev `596691f` · `~/RT-POSIX` → `RAIMLAB/RT-POSIX` `2844c48` · `~/ethercat` 1.6.10 · `~/rbdl-orb` `0606f2c` · `~/librealsense` v2.55.1. ⚠️ `yolo_v3_tiny_training/`은 git 미추적 | `git remote -v`, `git log -1` |
| 백업·복구 | `tools/board_backup.sh` (sudo, git 밖 상태 ~4 MB tar + RESTORE.md; 최신 `~/board_backup_20260806_152004.tar.gz`), `recovery/` (커널 -11 deb + README "반드시 -11") | `ls ~/ros2_ws/recovery` |
| 증거 | `evidence/crash_logs/` (cyclictest·soak·SLUB oops·poison 로그), `evidence/kernel_configs/`, `evidence/metrics/runs/` (perf CSV), `evidence/approach_accuracy/`, `evidence/reachmap/`, `evidence/fpu/`, `evidence/node_graph/` | `ls evidence/*` |
| 참고 문헌 (보드) | UG1414 v2.5/v3.5 (Vitis AI), UG1354 (AI SDK), UG1085 (ZynqMP TRM, 캐시), DS987 (K26 SOM 데이터시트, DDR), 랩 가이드 `MAN-2024114-LX02H0001` (x86 RT+IgH, **1.6.2 수치의 출처**) | `ls ~` |

---

## 9. 문서 vs 실측 불일치 — 보고서엔 실측값을 쓸 것

조사 중 발견된 낡은 문서 수치. 히스토리성 문서(`rt_patch.md`, `history.md` 등)는 기록물이라 고치지 않아도 되지만, **`STATUS.md`·`onboarding.md`·`README.md`·`workflow.md`처럼 현재 상태를 주장하는 문서는 갱신 대상**.

| 항목 | 문서 | 실측 | 비고 |
|---|---|---|---|
| RT 커널 빌드 | `#10` (2026-07-15) — `rt_final.md:50,127,312,326`, `rt_patch.md:20,27,28`, `STATUS.md:113` | **`#11` (2026-07-20)**, deb `-11` | #11 = #10 config + `CONFIG_USB_NET_AX88179_178A=m` 1줄 (ASIX NIC용). RT 심볼 동일 → #10의 cyclictest 수치는 #11에도 유효. `recovery/README.md`가 "반드시 -11"로 정본. 08-06 백업 tar 내부 RESTORE.md는 -10이라 틀림 |
| IgH 버전 | `1.6.2` — `onboarding.md:134`, `STATUS.md:542` | **`1.6.10`** | 1.6.2는 x86 랩 가이드 값. 1.5.2는 5.15에서 빌드 불가였음 |
| D435i FW | `5.16.0.1` — `STATUS.md:51` | **`5.17.0.10`** | `STATUS.md:37,216`, `rt_final.md:55`는 이미 5.17. 헤더 라인만 낡음 |
| 컬러 인코딩 | `BGR8` — `docs/vision/workflow.md:23,66` | **`RGB8`** | yaml에 `color_format` 키 없음 → `rs_launch.py:38` 기본 RGB8. `perf_levers_survey_20260901.md:237`이 이미 지적 |
| 드라이브 모드 | `CiA402+CSP` — `STATUS.md:53,543,546,579,599`, `onboarding.md:135`, `docs/rpu/*.md` | **CST (`DRIVE_MODE=10`)** | 병합 전(L7N/RPU 계획) 문구. `merge.md` D1이 CST로 확정 |
| 보드 IP | `192.168.120.132` (eth0) — `onboarding.md:108,150,155`, `STATUS.md:113,122,382`, `launch_script/viewer.sh:27` | **`192.168.120.50`** (USB NIC) | 07-22 eth0가 EtherCAT으로 넘어간 뒤. `operator.sh:20`, `desktop_viewer_plan.md:174`는 .50 |
| 파라미터 수 | `≈6.9 M` — `vision_final.md:107,269` | **≈8.68 M** | xmodel 상수 텐서 직접 합산 |
| 출력 fixpos | 양 head `2` — `vision_final.md:319` | **26×26 head 2, 13×13 head 3** | 런타임은 텐서별로 읽어 동작엔 무영향 |
| 지각 성능 | `15 Hz, E2E ~81 ms` — `README.md:4`, `vision_final.md:24,395` | **21.7 Hz, E2E p50 118 ms** (`shm_gate030_20260805`) | README는 07-15 구성(gate 0.045·depth 15 fps·sync off). 81 ms는 디텍터측 E2E, 118 ms는 base까지 true_e2e — 정의 다름 |
| depth fps | `15 fps + sync off` — `vision_final.md` §9.2 P4 | **30 fps + enable_sync** (08-04) | 이동 물체 추적용 복원(skew 28.5→4.3 ms, +0.18 cores) |
| 저장매체 | `SD/eMMC 234G` — `rt_final.md:47` | **microSD** | KV260엔 eMMC 없음 |
| PDO 수 | 코드에 RegisterPDOEntry 12개 (`SlaveCIA402Base.cpp`) | **10** (2개 주석) | `merge.md:296` "10 entries"가 맞음 |
| 커널 이름 | `5.15.199-rt91-rt-kria` — `yolov8_vitisai35_feasibility.md` (07-09) | `-rt-kv260c` | 그 문서 작성 시점의 첫 RT 빌드(폐기됨) |
| CLAUDE.md | SSD stand-in, `vitis_ai_worker.py`, FW 5.16.0.1 — `cleanup_plan.md:16,159`가 지적 | YOLO, `vitis_ai_worker_yolo.py`, 5.17.0.10 | |
| 3D 깊이 범위 | `0.05–3.5 m` — `STATUS.md:182`, `reprojection.md` | **0.19–1.6 m** | `target_3d.yaml` |
| gate0 우선순위 | `prio 90` — 메모리 `raon-vs-merge-plan.md:48` | **80** (도구 기본) | `~/.bash_history:1624-1625` |
| 기관 표기 | `서울과기대 임베디드시스템연구실` — `RAON-RT_guide.md` §11 (RAON-RT 전체에 적용된 듯 서술) | RT-POSIX = SeoulTech ESL(RT-AIDE) / **RAON-RT = KIST AIRI (2022) → RAIMLAB@명지대 (2025)** | 소스 헤더·git remote 기준. 출판 전 사용자 확인 |

## 10. 보드에서 확인 불가 — 남은 항목과 확인 방법

| 항목 | 상태 | 확인 방법 |
|---|---|---|
| ~~실제 APU 클럭~~ | ✅ **1,333,333,320 Hz** (2026-09-03 root 확인) | — |
| ~~SOM 부품번호·리비전~~ | ✅ **SMK-K26-XCL2G 5057-01 rev 1** (2026-09-03) | — |
| DDR4 타입·속도·폭 | FRU multirecord에 메모리 레코드 없음, DT엔 크기만 | **AMD DS987 (K26 SOM 데이터시트)** 인용: 4 GB 64-bit DDR4 non-ECC — 데이터레이트는 인용하는 DS987 리비전에서 확인 |
| XCK26 부품번호 문자열 | 로컬 어디에도 없음 | AMD K26 SOM 제품 페이지 인용 |
| Indy7 payload / reach / 반복정밀도 / 질량 | 로컬 문서 없음 (URDF 관절한계·질량만) | Neuromeka Indy7 데이터시트 |
| NRMK CORE 드라이브 모델번호·HW/SW rev | cfg엔 vendor/product ID만, 스캔 덤프 미저장 | 실기 세션(로봇 전원 on)에서 `ethercat slaves -v`, `ethercat sii_read` |
| EtherCAT 링크 속도(100BASE-TX 예상)·프레임 카운터 | 마스터 미가동(eth0 NO-CARRIER) | 실기 세션에서 `ethtool eth0`, `ethercat master`, `ethercat domains -v` |
| **격리 CPU3 운영조건 cyclictest** | 한 번도 안 돌림 (gate0 51.1 µs만, 다른 도구) | 서보-off 세션: `sudo cyclictest -m -p90 -i1000 -a3 -D5m -q` (비전+DPU+EtherCAT 가동), 목표 Max<100 µs |
| 6-class 최종 train/val 수량, INT8 mAP | 데스크톱에만 (`~/capstone_training/`), INT8 mAP는 미측정 | 데스크톱 `wc -l datasets/final/{train,val}.txt`; INT8 mAP는 "미측정"으로 기술 |
| 데스크톱 CPU 모델, nvidia 580 정확한 빌드, ROS 패키지 버전 | 보드에서 SSH 불가 | 데스크톱에서 `lscpu`, `nvidia-smi`, `dpkg -l \| grep ros-humble-ros-base` |
| QSPI 부트펌웨어(FSBL/PMUFW/TF-A) 버전 | root 필요 | `sudo xmutil bootfw_status` |
| compressed jpeg_quality 실효값 | 오버라이드 없음 → 기본 95 추정 | 실행 중 `ros2 param get /camera/camera camera.color.image_raw.compressed.jpeg_quality` |

## 11. 조사 방법 · 재현

- **수집**: 6개 조사 에이전트(차원별) → 완전성 비평 에이전트(모순 11·공백 8·의심 7 도출, 4건은 즉석 검증) → 보충 에이전트 4개. 총 12 에이전트, 345 도구 호출, 61분. 보드 4코어라 동시 2에이전트로 제한.
- **규칙**: 실측 우선(`verified-live`) > 문서(`doc`) > 추론(`inferred`); 읽기전용 경량 명령만(빌드·벤치마크·노드 기동 금지; `xdputil query`는 기록본 `~/vitis_ai_work/xdputil_query.json`과 대조), 문서-실측 불일치는 둘 다 기록.
- **한계**: 조사 시점 보드는 유휴(앱·마스터·카메라 노드 미가동)라 EtherCAT 라이브 스캔·토픽 Hz·`ros2 param`은 못 잡음. sudo는 사용자가 2건만 직접 실행. dmesg는 `dmesg_restrict`라 `journalctl -k`로 대체(RTC 미동기라 부팅 로그 타임스탬프 뒤틀림 주의).
- **root 확인 원문** (2026-09-03, 사용자):
  ```
  $ sudo cat /sys/kernel/debug/clk/clk_summary | grep acpu
     acpu_mux  0 0 0  1333333320  0 0 50000 Y
     acpu      0 0 0  1333333320  0 0 50000 Y
  $ sudo xmutil boardid        # 1-0051(캐리어)만 출력 후 multirecord checksum 오류로 exit 1
     Product Name: SCK-KV-G  Part: 5066-01  Serial: XFL1XXBCD0KB  Mfg: 08/25/21 01:03  DC 12 V (11.5–12.5) max 3 A
  $ sudo fru-print -b som
     product: SMK-K26-XCL2G  part: 5057-01  revision: '1'  serial: XFL1UANAK0SC  date: 13490525 (=2021-08-25 10:05)
     DC_Load 5.0 V (4.5–5.5) max 4000 mA   MAC_ID_0: 000a350cff64
  ```
- **관련 메모리**: `~/.claude/projects/-home-ubuntu/memory/report-writeup-doc-vs-live.md` (§9 요약본, 세션 간 유지).
