# Kria KV260 RT-PREEMPT 커널 패치 — 작업 기록 및 인수인계 문서

작성: 2026-07-08 (Claude 세션에서 자동 작성)
목적: **다른 세션/사람이 이 파일만 읽고 즉시 작업을 이어갈 수 있게** RT 커널 패치의 전 과정, 현재 상태, 미해결 과제를 기록한다.

> 📌 **크래시 사건의 전말(증상→오진→진단→근본원인→해결→검증)을 하나로 읽으려면: `rt_kernel_postmortem.md`** (2026-07-13 작성, 종합 보고서). 이 문서(rt_patch.md)와 `rt_kernel_fix_plan.md`는 정정 이력이 겹겹이라 실시간 작업 기록용이고, postmortem은 깨끗한 서사본이다. **결론: RT 크래시 = Ubuntu SAUCE radix-tree revert가 원인, 3파일 원복으로 해결(253→0건, 2026-07-13 확정).**

---

## 0. 한눈에 보는 현재 상태 (2026-07-10 갱신)

> ⚠️ **2026-07-10 중대 업데이트**: RT 커널 크래시 2회(보드 hang, 전원 리셋 필요) 조사 결과, **이 RT 커널 빌드는 부팅 초기부터 내부 상태가 오염될 수 있는 결함**이 확인됐다(§12에 쉬운 설명 포함 전체 기록; 재현은 확률적 — 2026-07-12 정정). **RT 커널 사용 중지 — 보드는 순정 커널(5.15.0-1070)로 전환(✅ 2026-07-12 완료)**해 비전/fps 작업 진행. 진단 상세·링크: `rt_kernel_fix_plan.md`. 크래시 원본 로그: `~/ros2_ws/crash_logs/`.
>
> ★★ **2026-07-13 근본 원인 확정**: 사용자가 직접 빌드한 ⓪ 실험 커널(`5.15.199-rt91-rt-kv260`, PREEMPT_LAZY=n + DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP=y)을 부팅 → 크래시 없이 검출기가 리포트 253건을 출력, **전수가 단일 원점 `__radix_tree_preload`를 지목**. 1차 소스 전수 대조로 확정: 범인은 **Ubuntu jammy 커널 전체(제네릭+xilinx)의 `UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`**(NVIDIA 독점 모듈의 GPL export 문제 회피용, 비-RT 무해/RT 치명). vanilla·linux-xlnx는 정상, rt91 패치는 무관, LAZY 무죄.
>
> ✅✅✅ **2026-07-13 해결 확정**: ⓪.5 국소 픽스(3파일을 vanilla 5.15.199 원본으로 원복) 후 재빌드(`-rt-kv260` rev-5)해 부팅 → **어제 253건 → 오늘 0건.** 검출기(DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP) 완전 무장 상태에서 부트 저널 전수 스캔 무위반, DPU(zocl)/카메라 스택 정상. **RT 커널 크래시(radix-tree 원인) 문제 종결. ④(linux-xlnx 재조립) 폐기.** **소크 ✅통과(2026-07-13; 07-14 `-S` 4코어 재측정도 위반 0·Avg 안정)**.
>
> ⚠️ **2026-07-14 별개 발견 — zocl DPU 크래시**: radix 픽스와 무관하게, RT 커널(kv260b)에서 DPU 비전 파이프라인을 처음 가동하면 zocl(Xilinx DPU 드라이버) KDS 경로에서 SLUB freelist 손상 → 커널 Oops → 프리즈 발생 (상세: `rt_kernel_postmortem.md §12`). **✅✅ 07-14 심야 근본원인 확정 + 픽스 완성**: `slub_debug=FZPU,kmalloc-256` 계측 재현으로 corruptor 생포 → `kds_core.c`의 `xrt_cu_submit()` 뒤에 `set_xcmd_timestamp(KDS_QUEUED)`를 부르는 **submit-후-타임스탬프 UAF**(CU 스레드가 커맨드를 먼저 완료·해제하면 해제된 메모리에 타임스탬프를 씀). RT의 넓은 preemption이 레이스를 노출(upstream XRT master에도 잔존). 수정 = 순서 교체(3곳), 패치 `~/ros2_ws/zocl_patches/`. 상세 postmortem §12-8, 메모리 `zocl-dpu-rt-kernel-crash`.
>
> ✅✅✅ **2026-07-15 RT 트랙 종결**: rev-6(`-rt-kv260c` #10, DEBUG off + zocl UAF 픽스) 빌드·설치·부팅 완료(현재 구동, realtime=1). **zocl 크래시 재현검증**: slub_debug 계측(churn 5×20+sustained 180s≈330s, zocl 156클라이언트) Poison 0 + 프로덕션(계측無, 200s+) Oops 0 = 양쪽 확정. **cyclictest(DEBUG off)**: idle Max 134 / **load Max 142µs(kv260b DEBUG-on 282→절반)** / 부하 중 커널 위반 0. **radix+zocl 두 결함 모두 해결, EtherCAT 선결조건 전부 해제.** 상세: §4-4-2(빌드), postmortem §12-8(zocl 검증). 검증로그: `crash_logs/{e2_poison_report_20260714-2336,prod_verify_kv260c,cyclic_20260715-013556}.log`.

| 항목 | 상태 |
|---|---|
| RT 커널 `5.15.199-rt91-rt-kria` | ❌ **폐기 (2026-07-10, §12)** — preempt 회계 오염 → 크래시 3회. 근본 원인 = Ubuntu SAUCE radix-tree revert, 후속 커널에서 해결 |
| ⓪ 실험 커널 `-rt-kv260` rev-5 (LAZY=n+DEBUG, 07-13) | 🔬 진단 임무 완수 — 검출기 253건(전수 단일 원점)으로 radix 원인 확정. kv260b/c로 대체 |
| radix 픽스 검증커널 `-rt-kv260b` (build #8, DEBUG on, 07-13) | ✅ radix 253→0 확정 + zocl 크래시 E2 진단 완수(slub_debug). kv260c로 대체됨 |
| ✅✅ **프로덕션 `-rt-kv260c` (#10, DEBUG off + zocl 픽스) ← 현재 구동** | **RT 트랙 최종 커널(2026-07-15).** radix 픽스 + zocl UAF 픽스, DEBUG off. 크래시 해결·검증 완료(아래 상세 행). |
| ✅✅ **rev-6 프로덕션 (`-rt-kv260c` #10) = 현재 구동·검증 완료 (2026-07-15)** | kv260b config에서 DEBUG_PREEMPT/ATOMIC_SLEEP off **+ zocl `kds_core.c` UAF 순서교체 패치**. 빌드·설치·부팅 성공(realtime=1). **zocl 재현 검증: 계측(330s)·프로덕션(200s+) 양쪽 Poison/Oops 0건.** cyclictest(DEBUG off): idle Max 134 / **load Max 142µs(kv260b 282→절반)** / 부하 중 위반 0. **RT 트랙 종결 = EtherCAT 선결조건 모두 해제.** 빌드 게이트 §4-4-2, 패치 `~/ros2_ws/zocl_patches/` |
| ✅ **zocl DPU 크래시 근본원인·픽스·검증** (2026-07-14~15) | E2 계측(slub_debug=FZPU)으로 KDS submit-후-타임스탬프 UAF 확정 → 순서교체 픽스 → kv260c로 재현검증 Poison 0. postmortem §12-8 |
| 기존 cmdline (cma=1000M 등) | ✅ 유지됨 |
| cyclictest — **kv260b(헤드리스·DEBUG on·격리無) 2026-07-13** | Avg 14~20µs / **Max idle 143·load 189µs**(부하 cpu4+hackbench+io), **부하 중 커널 위반 0**. 100µs 목표 초과지만 **보수적**(DEBUG 오버헤드+격리 미적용). 실 EtherCAT값 = rev-6(DEBUG off)+3+1 격리 코어 측정. 구 GUI측정(127~698µs) 대비 698µs 스파이크 소멸. 하네스 `cyclic_rt.sh` |
| cyclictest — **07-14 재측정 (-S 4코어·1분·soak v2)** | **idle** Max 58/121/116/77µs·Avg 14~19. **soak v2 부하(75s, cyclictest와 동시 — sudo 로그로 57초 겹침 검증)** Max 146/282/105/219µs·**Avg 20~25(idle 대비 +6µs)·Min 12 유지·radix 위반 0(soak PASS)**. 07-13과 같은 결론 재확인: 커널 건강(부하에도 Avg 안정+위반0), Max<100 보장은 rev-6+3+1 격리 몫. **교훈: 비고정 단일스레드 측정(Max 45µs)은 스케줄러가 조용한 코어로 옮겨줘 낙관적 — 판정은 반드시 `-S`(코어별 고정)로** |
| 비전 파이프라인 코어 사용량 (4코어) | ✅ **~2.8코어 실측(2026-07-09)** — pick_target_3d_node가 #1(~0.73코어). frame_age~120ms 건강, SSH 정상. 상세·최적화 §6-4 |
| 격리 코어 통합부하 레이턴시 | ⬜ **EtherCAT 단계로 연기** — 3+1 재도입 후 코어3에서 측정(§6-4). 현재 격리 없음 |
| 헤드리스 전환 (multi-user.target + gdm stop) | ✅ 적용됨 |
| mali GPU 모듈 blacklist | ✅ 적용됨 (`/etc/modprobe.d/blacklist-mali.conf`, initrd 반영 완료) |
| DisplayPort IRQ 낭비 제거 | ✅ **완료·재부팅 검증**(2026-07-09) — `blacklist`+`update-initramfs`로 영구 차단, 부팅 후 미로드·IRQ 소멸 확인. CPU0 ~7.8% 회수, DPU 무영향(zocl 정상, smartcam active). 함정/절차 §6-4 |
| CPU 격리 (isolcpus) | 🔶 **2026-07-09 해제(A안)** — 2+2(isolcpus=2,3)가 비전+SSH를 죽임(코어 0,1 포화). 현재 cmdline=`skew_tick=1`만, 4코어 전부 비전용. EtherCAT 단계에서 **3+1**로 재도입 예정(§6-4) |
| 간헐적 부팅 프리즈 | ✅ **원인 규명(2026-07-10)** — §12의 preempt 회계 결함으로 통합(kern.log에서 같은 서명의 크래시/오염 확인). mali는 부차적 용의자였을 가능성. 새 커널 재빌드로 함께 해소 예정 |
| apt-mark hold (커널 3종) | ✅ **완료 확인** (2026-07-08 실측: linux-image/headers/xilinx-zynqmp 3종 hold 걸려 있음) |
| IgH EtherCAT Master | ⬜ 미착수 — **선결조건 모두 해제됨(2026-07-15): rev-6(kv260c) + zocl 픽스 완료.** 착수 가능(§7) |

**2026-07-09 실측 요약: RT 커널·apt hold·헤드리스(mali 미로드)·DPU smartcam·DP IRQ 제거 전부 ✅. CPU 격리는 해제(A안). 비전 파이프라인 ~2.8코어 실측. EtherCAT 미설치. 온도 33°C.**

**2026-07-10 갱신: 위 표의 헤드리스/apt hold 등 rootfs 설정은 순정 커널에서도 그대로 유효. RT 커널만 결함 판정으로 사용 중지(§12).**

**2026-07-12 갱신: 순정(5.15.0-1070) 전환 완료. 순정 initrd는 6/22자 구버전이라 dpsub blacklist 미반영 → DP IRQ 스톰 부활(실측 ~11K irq/s, 전부 CPU0). ★ 사용자 결정: 순정 커널에서는 dpsub를 차단하지 않음(실시간성 요구 없음) — 순정에서의 모든 fps 측정은 dpsub 포함 조건으로 일관되게 수행하고, RT 복귀 시엔 RT initrd에 blacklist가 반영돼 있어 자동 소멸. (나중에 차단하려면: `sudo update-initramfs -u -k 5.15.0-1070-xilinx-zynqmp` 한 줄 — flash-kernel은 자동 트리거되나, 순정이 최신 버전이 아닌 상태에서는 함정 ⑦에 따라 `sudo flash-kernel --force 5.15.0-1070-xilinx-zynqmp`로 명시 재생성 후 재부팅.)**

**현황 & 다음 할 일:**
- ✅ **커널 기반은 완료**: PREEMPT_RT·HZ1000·CMA 유지·헤드리스·DP IRQ 제거·apt hold·부팅 안정.
- 🔶 **CPU 격리(isolcpus)는 2026-07-09 해제(A안)**. 이유: 2+2(isolcpus=2,3)가 **비전+SSH를 죽였음** — 비전이 ~2.8코어를 써서 2코어에 안 들어감(§6-4). 현재 4코어 전부 비전용, cmdline=`skew_tick=1`만.
- **다음 작업은 두 트랙 (사용자 우선순위에 따라):**
  1. **(비전)** `pick_target_3d_node` 최적화 — 현재 #1 CPU 소비(~0.73코어, 1픽셀 작업인데 full depth 매프레임 처리 의심). ~0.5코어 절약하면 나중 3+1 격리가 여유. + 탐지주기 throttle, 카메라 FPS↓.
  2. **(제어)** IgH EtherCAT Master 설치(§7, 미설치) → EtherCAT 통합 단계에서 **3+1(isolcpus=3)** 격리 재도입 + 코어3 통합부하 레이턴시 측정.
```bash
# 현재 상태 재확인 (격리 해제됐는지 / RT / DP)
grep -oE 'isolcpus|skew_tick=1' /proc/cmdline   # → skew_tick=1 만 (isolcpus 없어야)
cat /sys/devices/system/cpu/isolated            # → 비어있어야
nproc                                           # → 4
lsmod | grep -q dpsub && echo DP살아있음 || echo "DP제거됨 ✅"
```

---

## 1. 배경과 목표

- ROS 2 프로젝트(~/ros2_ws/)용 Kria KV260 보드에서 EtherCAT 기반 로봇 제어(Neuromeka Indy7, 제어주기 1kHz 추정)를 하기 위해 리눅스 커널에 RT-PREEMPT 패치를 적용한다.
- 전체 로드맵: **APU(Cortex-A53, 리눅스)에서 IgH EtherCAT Master + 로봇 제어 먼저 구현** → 이후 RPU(Cortex-R5, FreeRTOS+SOEM)로 이전 (RPU 트랙은 별도 문서: `rpu_guide_for_claude.md`, `rpu_freertos_soem_execution_plan.md`).
- 참고 가이드: 랩(RAIMLAB) 문서 MAN-20241113-LX02H0001 (인텔 x86 NUC용 RT-PREEMPT + IgH EtherCAT 구축 가이드). 큰 흐름은 이 가이드를 따르되 Kria 특화 차이가 많음(본 문서 곳곳의 "함정" 참조).

## 2. 시스템 정보

- 보드: Kria **KV260 revB** (SOM: SMK-K26 revA), A53 4코어, RAM 4GB, swap 27G
- OS: Ubuntu 22.04.5 (Kria 공식 이미지), 루트 /dev/mmcblk1p2 (SD/eMMC 234G)
- 원래 커널: `linux-xilinx-zynqmp` **5.15.0-1070.74** (업스트림 베이스 **5.15.199**), PREEMPT_VOLUNTARY, HZ=250
- 부팅 구조: **GRUB 없음.** U-Boot → `/boot/firmware/boot.scr.uimg` → `/boot/firmware/image.fit` (커널+initrd+DTB 6종이 한 덩어리, U-Boot이 EEPROM 읽어 캐리어 보드에 맞는 DTB config 자동 선택). `image.fit`은 **flash-kernel**이 생성.
- 원래 cmdline: `root=LABEL=writable rootwait earlycon console=ttyPS1,115200 console=tty1 clk_ignore_unused uio_pdrv_genirq.of_id=generic-uio xilinx_tsn_ep.st_pcp=4 cma=1000M`
- 원격 접속: **Tailscale over eth0(GEM)** — ⚠️ eth0을 EtherCAT에 넘기면 원격 접속이 끊기므로 §7 참조.
- 빌드 PC: x86_64 Ubuntu, 사용자 `jaehyeon@jaehyeon-Raimlab`, 작업 경로 `~/kria-rt/`

## 3. 확정된 설계 결정

1. **커널 소스**: 순정 kernel.org ❌ (Xilinx 전용 드라이버 소실됨). **Ubuntu Xilinx 커널 소스(1070.74) + kernel.org RT 패치** 사용.
2. **RT 패치 버전**: `patch-5.15.197-rt91` (베이스 5.15.199에 가장 가까움. 적용 시 reject 0개).
3. **빌드**: x86 PC에서 aarch64 크로스컴파일 (`bindeb-pkg`로 .deb 생성).
4. **CMA 유지**: FPGA(PL)를 사용 중/예정이므로 랩 가이드의 "CMA 비활성화"는 **적용하지 않음** (cma=1000M 그대로).
5. **헤드리스 운용**: SSH로만 작업하므로 GUI 제거 (RT 지터 감소 + 부팅 프리즈 용의자 mali 제거 겸).
6. **CPU 격리 (2026-07-09 개정)**: 당초 2+2(CPU 0,1=하우스키핑, 2,3=제어격리)로 갔으나 **비전이 ~2.8코어를 써서 2코어에 안 맞음 → 비전+SSH 다운**(§6-4). **개정 방침: 격리는 EtherCAT 통합 때 3+1(비전=0,1,2 / 제어=격리코어 1개)로 재도입.** 현재는 격리 해제 상태. `rcu_nocbs`/`nohz_full`은 현재 커널 config에 없어 미사용(재빌드 시 추가 후보).

## 4. 수행한 작업 (순차 기록)

### 4-1. 백업 [보드]
```bash
sudo mkdir -p /root/backup
sudo cp -a /boot/firmware/image.fit /root/backup/image.fit.stock     # 순정 커널 FIT
sudo cp -a /boot/firmware/boot.scr.uimg /root/backup/
sudo tar czf /root/backup/boot-<날짜>.tar.gz /boot
```
- 백업 사본을 PC로도 복사함: 보드 `/home/ubuntu/kria-backup` → PC `./kria-backup` (scp. `/root`는 700 권한이라 직접 scp 불가 → ubuntu 홈에 복사 후 전송).

### 4-2. 커널 소스 취득 [PC]
- Launchpad **git 저장소는 504 오류로 실패** (`~canonical-kernel/...`, `~ubuntu-kernel/...` 모두). 대신 **소스 패키지 직다운로드**로 해결:
```bash
cd ~/kria-rt
BASE=https://launchpad.net/ubuntu/+archive/primary/+sourcefiles/linux-xilinx-zynqmp/5.15.0-1070.74
wget $BASE/linux-xilinx-zynqmp_5.15.0.orig.tar.gz
wget $BASE/linux-xilinx-zynqmp_5.15.0-1070.74.diff.gz
wget $BASE/linux-xilinx-zynqmp_5.15.0-1070.74.dsc
dpkg-source --no-check -x linux-xilinx-zynqmp_5.15.0-1070.74.dsc linux-5.15-rt-kria
cd linux-5.15-rt-kria && make kernelversion   # → 5.15.199
```
- **함정 ①**: dpkg-source 추출 시 스크립트 실행권한이 소실됨 (`./scripts/pahole-flags.sh: Permission denied`). 해결:
```bash
chmod -R +x scripts/ debian/rules debian/scripts/
```

### 4-3. RT 패치 적용 [PC]
```bash
wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/older/patch-5.15.197-rt91.patch.xz
xzcat patch-5.15.197-rt91.patch.xz | patch -p1
find . -name '*.rej'    # → 없음 (성공)
```
- 적용 확인 마커: 소스 루트에 `localversion-rt` 파일 생김 (내용 `-rt91`).

### 4-4. 커널 config [PC]
```bash
scp ubuntu@<KRIA_IP>:/boot/config-5.15.0-1070-xilinx-zynqmp .config
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
make olddefconfig
scripts/config --enable EXPERT
scripts/config --disable PREEMPT_NONE --disable PREEMPT_VOLUNTARY --disable PREEMPT
scripts/config --enable PREEMPT_RT
scripts/config --disable HZ_250 --enable HZ_1000
scripts/config --disable CPU_FREQ
scripts/config --disable ACPI_PROCESSOR --disable CPU_IDLE
scripts/config --disable KVM
scripts/config --disable SYSTEM_TRUSTED_KEYS --disable SYSTEM_REVOCATION_KEYS
scripts/config --disable DEBUG_INFO
scripts/config --set-str LOCALVERSION "-rt-kria"
make olddefconfig
```
- **함정 ②**: `PREEMPT_RT`가 olddefconfig 후 사라짐 → 원인은 **KVM**. 5.15-rt에서 arm64는 `select ARCH_SUPPORTS_RT if HAVE_POSIX_CPU_TIMERS_TASK_WORK`이고 이것이 `if !KVM` 조건이라 **KVM이 켜져 있으면 PREEMPT_RT 선택 불가**. KVM(가상화, 이 보드에서 불필요)을 꺼서 해결.
- **함정 ③**: `CPU_IDLE`은 `ACPI_PROCESSOR`가 select해서 단독으로 안 꺼짐 → 둘 다 꺼야 함 (ZynqMP는 DT 부팅이라 ACPI 불필요).
- 최종 확인값: `PREEMPT_RT=y`, `HZ_1000=y`, `CMA=y`(유지!), KVM/CPU_IDLE/CPU_FREQ 없음. `SYSTEM_TRUSTED_KEYS=""`(문자열 옵션이라 빈 문자열이 정상).
- 랩 가이드와 다른 점: **Memory Management 계열(CMA, compaction, page migration)은 건드리지 않음** (FPGA cma=1000M 필요). x86 전용 항목(Processor family 등) 해당 없음. NO_HZ_FULL(Full dynticks)은 이번에 넣지 않음(현재 NO_HZ_IDLE).

### 4-4-2. `rev-5` 실제 config — 러닝 커널 실측 (2026-07-13)

> ⚠️ **2026-07-13 후속**: rev-5는 이후 **`-rt-kv260b`(build #8)로 대체됨 = 현재 구동 커널.** kv260b는 이 rev-5 표에서 **① CPU_FREQ y→n(drift 교정) ② NO_HZ_FULL 신규 =y ③ RCU_NOCB_CPU 신규 =y** — 이 3개만 다르고 나머지 전부 동일. 부팅 검증(uname `…-rt-kv260b #8 SMP PREEMPT_RT`, realtime=1) 부트 위반 전수 0 재확인. cyclictest+soak 검증 통과(07-13/14). **최종 프로덕션 rev-6 = 여기서 DEBUG_PREEMPT/ATOMIC_SLEEP 2줄만 끈 것 → 2026-07-14 `-rt-kv260c`(권장 명, 또는 kv260b 덮어쓰기)로 PC 빌드 진행 중.** 상세: 메모리 `kria-rt-preempt-project`.

> ⚠️ 위 §4-4는 **폐기된 `-rt-kria`** 최초 레시피다. 이후 ⓪ LAZY 실험 + ⓪.5 radix 픽스를 거치며 실제 값이 달라졌는데, 종합 기록이 없었다(개별 CONFIG는 fix_plan/postmortem에 산문으로만 산재). 아래는 **문서·기억 재구성이 아니라, 보드가 지금 부팅 중인 커널의 `/proc/config.gz`(IKCONFIG)를 직접 읽은 실측값**이다.
>
> 확인 명령(보드): `uname -r` → `5.15.199-rt91-rt-kv260` / `zcat /proc/config.gz | grep '^CONFIG_...'` (2026-07-13 18:07 KST)

**RT 핵심**
| 옵션 | 값 | 비고 |
|---|---|---|
| `PREEMPT_RT` | y | |
| `PREEMPT_LAZY` | **n** (선택 가능한 상태에서 끔) | `HAVE_PREEMPT_LAZY=y`이나 `PREEMPT_LAZY`는 not set — Kconfig.preempt 소스 패치(def_bool→bool 프롬프트화)가 살아있고 값도 n임을 확인 |
| `PREEMPTION`/`PREEMPT_COUNT`/`PREEMPT_RCU` | y | RT 하 표준 |
| `HZ`/`HZ_1000` | 1000 / y | |
| `RCU_BOOST` (★신규 확인, 이전 미문서화) | y, delay=500 | priority-boost RCU reader — RT 커널 기본값, 이번에 처음 확인 |

**검증용 DEBUG 검출기** (★ rev-6에서 끌 유일 대상)
| 옵션 | 값 |
|---|---|
| `DEBUG_PREEMPT` | y |
| `DEBUG_ATOMIC_SLEEP` | y |
| `DEBUG_INFO`/`DEBUG_KERNEL` | y |

**메모리** (이전 세션 Q&A 대상 항목들 실측)
| 옵션 | 값 | 비고 |
|---|---|---|
| `CMA` | y | cmdline `cma=1000M`로 런타임 1GB(실측 `CmaTotal 1024000kB`). config의 `CMA_SIZE_MBYTES=32`는 컴파일 기본값일 뿐 — cmdline이 이김 |
| `MIGRATION` | y | 끄지 않음(§4-4 결정 그대로) |
| `COMPACTION` | y | 끄지 않음(§4-4 결정 그대로) |
| `SLUB_DEBUG` | y | `SLUB_DEBUG_ON`은 not set — 부팅 시 자동활성 아님, `slub_debug=` 커맨드라인 파라미터 줘야 발동 |
| `SLAB_FREELIST_RANDOM` | y | 유지 결정(fix_plan (a-2) — 끄는 건 카나리아 제거일 뿐 해결책 아님) |
| `SLAB_FREELIST_HARDENED` | y | |

**기타 Q&A 대상**
| 옵션 | 값 |
|---|---|
| `FTRACE`/`FUNCTION_TRACER` | y |
| `SYSTEM_TRUSTED_KEYS`/`SYSTEM_REVOCATION_KEYS` | `""` (빈 문자열 — 정상, "인증서 비울 필요 없음" 결정과 일치) |

**PREEMPT_RT 선택조건 재확인** (§4-4 함정②③)
| 옵션 | 값 | §4-4 레시피 대비 |
|---|---|---|
| `KVM` | not set | 일치 |
| `CPU_IDLE` | not set | 일치 |
| `ACPI_PROCESSOR` | not set | 일치 |
| `CPU_FREQ` | **y** | ⚠️ **불일치** — §4-4는 disable 지시였으나 rev-5는 y. 이후 리빌드(⓪/⓪.5) 중 `olddefconfig`가 되살렸을 가능성, 정확한 시점 미상(추정). RT 동작엔 무해(PREEMPT_RT를 막는 진짜 게이트는 KVM). 재현 시 원인 추적 필요 |

**rev-5 전용 빌드 workaround** (radix 픽스와 별개, §9 함정G)
| 옵션 | 값 | 이유 |
|---|---|---|
| `XILINX_TSN` | not set | `in_be32`/`out_be32` 빌드에러 회피 |
| `NET_VENDOR_S2I` | not set | 상동 |

**플랫폼 안전게이트** (매 빌드 필수 확인 — §9 함정B, 5개 전부 y여야 부팅 가능)
| 옵션 | 값 |
|---|---|
| `ARCH_ZYNQMP` / `ZYNQMP_FIRMWARE` / `PINCTRL_ZYNQMP` / `COMMON_CLK_ZYNQMP` / `FPGA_MGR_ZYNQMP_FPGA` | 전부 y |

**radix-tree 픽스 — config 아님, 소스 3파일**(§6). 표에 안 잡히므로 러닝 커널에서 export 심볼로 재확인:
```
grep radix_tree_preloads /proc/kallsyms
# → __ksymtab_.../__kstrtab_.../__kstrtabns_radix_tree_preloads + D radix_tree_preloads (4줄=vanilla local_lock판 지문, 확인됨)
```

**식별**: `CONFIG_LOCALVERSION="-rt-kv260"` → `uname -r`=`5.15.199-rt91-rt-kv260`.

**DPU 관련**: `CONFIG_DRM_ZOCL=m`(in-tree 모듈, 경로 `drivers/gpu/drm/zocl/` — **PC 커널 트리엔 소스 있음**(보드 설치 headers 패키지에만 미포함). 07-14 심야 E2 계측으로 `common/kds_core.c`의 KDS submit-후-타임스탬프 UAF를 zocl 크래시(trace#2) 근본원인으로 확정 → 픽스 = `~/ros2_ws/zocl_patches/apply_zocl_uaf_fix.py`(상세 postmortem §12-8). rev-6 빌드에 합류 예정. trace#1(`zocl_read_sect` vmalloc-in-RCU)은 별개 저심각 이슈로 후순위 보존.

▶ **rev-6 재빌드 시 변경 범위**: 위 표 `DEBUG_PREEMPT y→n`, `DEBUG_ATOMIC_SLEEP y→n` **+ zocl UAF 패치 적용**(아래). radix 3파일 포함 나머지 config 전부 그대로. (LOCALVERSION `-rt-kv260c` 권장; 트리는 radix 픽스 든 기존 `~/kria-rt/linux-5.15-rt-kria` 재사용 — **재추출 금지**, 픽스 소실됨.)

```bash
# zocl UAF 패치 적용 (config 변경과 별개, make 전에 1회) [PC]
python3 ~/ros2_ws/zocl_patches/apply_zocl_uaf_fix.py \
  ~/kria-rt/linux-5.15-rt-kria/drivers/gpu/drm/zocl/common/kds_core.c
# → "[ok] N건 교체 완료" 확인 (N=2~3). 0건이면 트리 버전 상이 — README 수동 확인 절차 참조.
```

**rev-6 빌드 전 안전 게이트 (2026-07-14, 과거 사고 재발 방지):**
```bash
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-     # menuconfig/빌드 전 필수
# ① DEBUG 2개가 정말 꺼졌나 (아무것도 안 나와야 함)
grep -E '^CONFIG_DEBUG_PREEMPT=|^CONFIG_DEBUG_ATOMIC_SLEEP=' .config
# ② ZynqMP 플랫폼 게이트 = 5 여야 (과거 menuconfig 사고로 ARCH_ZYNQMP 통째 off → 부팅불가·DTB 0개)
grep -cE '^CONFIG_(ARCH_ZYNQMP|ZYNQMP_FIRMWARE|PINCTRL_ZYNQMP|COMMON_CLK_ZYNQMP|FPGA_MGR_ZYNQMP_FPGA)=y' .config
# ③ 유지 항목 (전부 나와야: PREEMPT_RT, HZ_1000, NO_HZ_FULL, RCU_NOCB_CPU + LOCALVERSION 확인)
grep -E '^CONFIG_PREEMPT_RT=y|^CONFIG_HZ_1000=y|^CONFIG_NO_HZ_FULL=y|^CONFIG_RCU_NOCB_CPU=y|^CONFIG_LOCALVERSION=' .config
# ④ 빌드PC 보호 (아무것도 안 나와야: DEBUG_INFO=y는 oomd 세션킬 원인, CPU_FREQ=y는 drift)
grep -E '^CONFIG_DEBUG_INFO=y|^CONFIG_CPU_FREQ=y' .config
```
설치 시 새 버전명(`-rt-kv260c`)이면 **DTB 선복사(함정 ⑤)를 dpkg 전에 새 경로로 재수행** 필요. 설치 후 검증 = 07-14 세트 반복(무부하 `-S` 1분 + soak_rt.sh 75s 동시 cyclictest). Max 하락 예상.

### 4-5. 빌드 [PC] (~15-30분)
```bash
make -j$(nproc) bindeb-pkg
# 산출물: ~/kria-rt/linux-image-5.15.199-rt91-rt-kria_..._arm64.deb (70M)
#         ~/kria-rt/linux-headers-5.15.199-rt91-rt-kria_..._arm64.deb (8.1M)
```

### 4-6. 보드 설치 [보드]
```bash
# PC에서: scp linux-{image,headers}-*.deb ubuntu@<KRIA_IP>:~/
sudo dpkg -i ~/linux-image-5.15.199-rt91-rt-kria_*.deb ~/linux-headers-5.15.199-rt91-rt-kria_*.deb
```
- **함정 ④ (핵심)**: 설치 직후 flash-kernel 트리거가 `installing version 5.15.0-1070-xilinx-zynqmp`(순정!)로 image.fit을 만들어버림. 원인: flash-kernel DB의 Kria 항목에 **`Kernel-Flavors: xilinx-zynqmp` 필터**가 있어 이름이 다른 우리 커널을 무시함. 해결(영구 적용됨):
```bash
sudo tee -a /etc/flash-kernel/db <<'EOF'

Machine: ZynqMP *KV260*
Kernel-Flavors: any
EOF
```
- **함정 ⑤ (핵심)**: FIT 생성 템플릿(`/usr/share/flash-kernel/its/image-kria.its`)이 DTB를 **`/lib/firmware/<커널버전>/device-tree/xilinx/`에서 하드코딩 참조**하는데, bindeb-pkg는 DTB를 `/usr/lib/linux-image-<버전>/xilinx/`에 설치함. 해결(커스텀 커널 설치 때마다 필요!):
```bash
sudo mkdir -p /lib/firmware/5.15.199-rt91-rt-kria/device-tree
sudo cp -r /usr/lib/linux-image-5.15.199-rt91-rt-kria/xilinx /lib/firmware/5.15.199-rt91-rt-kria/device-tree/
```
- 그 후 명시적으로 재생성:
```bash
sudo flash-kernel 5.15.199-rt91-rt-kria
# 출력에 "installing version 5.15.199-rt91-rt-kria" 확인
```
- 검증: `dumpimage -l /boot/firmware/image.fit`의 커널 Data Size(14,410,634B)가 `/boot/vmlinuz-5.15.199-rt91-rt-kria` 크기와 바이트 단위 일치 확인함. 순정 FIT은 `.bak`으로 자동 보관됨.
- 무해한 경고: `Couldn't find DTB on the following paths:` (빈 DTB-Id 조회 메시지, 순정 커널 때도 뜸) / initramfs의 `Possible missing firmware ... tuner_xc2028` 류(TV 튜너, KV260 무관).

### 4-7. 부팅 및 검증 [보드]
- ⚠️ **첫 부팅 2회가 중간에 멈춤** (리셋 2번 후 3번째에 정상 부팅) → §6-1 미해결 이슈 참조.
- 부팅 후 확인 완료:
  - `uname -a` → `5.15.199-rt91-rt-kria #1 SMP PREEMPT_RT` ✅
  - `/sys/kernel/realtime` → `1` ✅
  - `/proc/cmdline` → 기존 인자(cma=1000M 등) 유지 ✅
- `apt-mark hold linux-image-xilinx-zynqmp linux-xilinx-zynqmp linux-headers-xilinx-zynqmp` — **완료 확인됨 (2026-07-08 실측, 3종 hold)**. 참고: flavor override(any) 덕에 5.15.199 > 5.15.0이라 apt가 순정을 업데이트해도 flash-kernel 트리거는 RT 커널을 유지함(이중 안전).

### 4-8. cyclictest 1차 (GUI 켠 상태) [보드]
```
무부하 10초:  Min 10 / Avg 12 / Max 87 µs                    → 통과
부하 10분:    Max 127 / 145 / 698 / 179 µs (CPU0~3)          → 기준(100µs) 초과
```
- `WARN: stat /dev/cpu_dma_latency failed`는 CPU_IDLE을 커널에서 제거해서 나오는 무해 경고.

### 4-9. 헤드리스 전환 + mali blacklist [보드] (적용 완료)
```bash
sudo systemctl set-default multi-user.target   # 다음 부팅부터 GUI 없음
sudo systemctl stop gdm.service                # 즉시 GUI 종료
echo 'blacklist mali' | sudo tee /etc/modprobe.d/blacklist-mali.conf
sudo update-initramfs -u -k 5.15.199-rt91-rt-kria   # flash-kernel 자동 트리거됨(RT 버전으로 정상 재생성 확인)
```
- 근거: ① 데스크톱은 RT 지터의 주요 원인 ② mali(staging GPU 드라이버)는 부팅 프리즈의 유력 용의자 ③ 사용자는 SSH로만 작업.
- GUI 끈 후 부하 1분 테스트: Max 313/60/85/240 µs — 평균(12-14µs)은 우수하나 CPU0(IRQ 몰림)·CPU3에 간헐 스파이크 잔존 → CPU 격리로 해결하기로 결정.

### 4-10. CPU 격리 적용 [보드] (⚠️ 2026-07-09 되돌림 — 아래는 2+2 격리를 적용했던 이력 기록. 비전+SSH 다운으로 §6-4에서 해제(A안)함. 재도입은 EtherCAT 때 3+1로. 명령은 참고용.)
```bash
sudo sed -i 's/^LINUX_KERNEL_CMDLINE=""/LINUX_KERNEL_CMDLINE="isolcpus=2,3 irqaffinity=0,1 skew_tick=1"/' /etc/default/flash-kernel
sudo flash-kernel 5.15.199-rt91-rt-kria
sudo reboot
```
- 원리: `/etc/default/flash-kernel`의 `LINUX_KERNEL_CMDLINE`이 boot.scr 템플릿(`/etc/flash-kernel/bootscript/bootscr.zynqmp.kria` 121행)에서 cmdline 끝에 붙음.
- 이 재부팅은 mali blacklist 후 첫 부팅 = **부팅 프리즈 재현 테스트 겸함**.

## 5. 변경된 파일 목록 (보드)

| 경로 | 내용 |
|---|---|
| `/etc/flash-kernel/db` | Kria 머신 `Kernel-Flavors: any` override 추가 |
| `/lib/firmware/5.15.199-rt91-rt-kria/device-tree/xilinx/` | 커스텀 커널 DTB 복사본 (FIT 생성용) |
| `/etc/modprobe.d/blacklist-mali.conf` | `blacklist mali` |
| `/etc/modprobe.d/blacklist-zynqmp-dpsub.conf` | `blacklist zynqmp_dpsub` + `install zynqmp_dpsub /bin/true` — DP IRQ 스톰(~10.9K/s) 제거. **initramfs 반영 필수**(§6-4 함정). 2026-07-09 |
| `/etc/systemd/system/default.target` | → multi-user.target 심링크 |
| `/etc/default/flash-kernel` | `LINUX_KERNEL_CMDLINE="skew_tick=1"` (2026-07-09 격리 해제 A안. 이전엔 `isolcpus=2,3 irqaffinity=0,1 skew_tick=1` — §6-4) |
| `/boot/firmware/image.fit` | RT 커널 FIT (`.bak`=이전 버전 자동 보관) |
| `/root/backup/`, `~/kria-backup` | 순정 image.fit 등 백업 (PC에도 사본) |

## 6. 미해결 이슈 (우선순위순)

### 6-1. 간헐적 부팅 프리즈 ⚠️ → ✅ 원인 규명됨 (2026-07-10, §12로 통합)

> **해결**: kern.log 분석 결과 이 프리즈들은 §12의 preempt 회계 결함(early-boot `BUG: scheduling while atomic` → per-CPU 상태 오염; 재현은 확률적)의 발현일 가능성이 높다. 실제로 7/8에도 SLUB oops(§12와 동일 서명) 크래시가 1회 있었음이 확인됐다. mali는 부차적이었을 수 있음(blacklist는 유지해도 무해). 아래는 당시 분석 기록(이력 보존).
- **증상**: RT 커널 최초 부팅 2회가 부팅 도중 멈춤(리셋으로 복구). journald 분석 결과 monotonic **~21초**, 서비스 병렬 기동 구간에서 **에러/패닉 로그 없이 정지** (silent freeze).
- **용의자**: freeze 4초 전 로드된 **mali**(staging 드라이버, RT와 궁합 문제 부류) 또는 병렬 probe 타이밍 레이스. 성공한 부팅에서는 mali/zocl 정상, RT 위반 경고(BUG: sleeping...) 없음.
- **조치함**: mali blacklist (4-9).
- **해야 할 것**: blacklist 후 재부팅 수 회(콜드 부팅 포함)로 재현 여부 관찰. **재현되면 시리얼 콘솔(ttyPS1, 115200)로 마지막 커널 출력 확보가 관건** — 화면(tty1)에 안 찍히는 메시지가 시리얼에는 남을 가능성 높음. 재현 시 조사 방향: zocl, dfx-mgr/smartcam(FPGA 오버레이 로드), TSN 드라이버, systemd 병렬화 축소(`systemd.default_timeout_start_sec`, probe 순서).

### 6-2. CPU 격리 전략 (2026-07-09 개정 — §6-4 참조)
- **2+2 격리는 폐기.** 비전 ~2.8코어 > 2코어라 비전+SSH 다운. 현재 격리 해제 상태(cmdline=`skew_tick=1`).
- **EtherCAT 단계에서 3+1(isolcpus=3)로 재도입** 예정: 비전=0,1,2 / 제어=격리코어 1개. 그때 코어3에서 통합부하 레이턴시 <100µs 확인이 완료 조건. 3+1도 마진 얇으면(2.8<3) `pick_target_3d_node` 최적화 선행.
- 참고: `rcu_nocbs`/`nohz_full`은 현재 config(RCU_NOCB_CPU, NO_HZ_FULL 없음)에서 동작 안 함. 향후 커널 재빌드 시 `CONFIG_RCU_NOCB_CPU=y`, `CONFIG_NO_HZ_FULL=y` 추가 검토.

### 6-3. apt hold 확인
- ✅ 완료 (2026-07-08 실측: 3종 hold 확인). 재확인은 `apt-mark showhold`.

### 6-4. RT 지터 vs 비전/DPU 부하 — 통합 부하 검증이 진짜 완료 기준 (2026-07-08 분석)

> **⚠️ 2026-07-09 중대 업데이트 — 2+2 격리는 이 시스템엔 과했음 (비전+SSH 다운).**
> 격리(isolcpus=2,3) 상태에서 비전 파이프라인을 돌리자 **코어 0,1 포화 → 카메라 USB 서비스 굶음(파이프라인 stall, frame_age 7500ms) + 네트워크 굶음(SSH/Tailscale 끊김)**. 원인: 격리로 비전+전체 시스템+모든 IRQ(USB 카메라 xhci→cpu0, eth0→cpu1)를 코어 0,1에만 몰아넣었는데, 정작 격리 코어 2,3은 (제어 루프가 없으니) 놀고 있었음. `cyclictest -a 2,3`도 EINVAL("No allowable cpus") — 세션이 0,1에 갇혀서, 같은 뿌리.
> **조치**: 격리 해제(A안). `/etc/default/flash-kernel`의 `LINUX_KERNEL_CMDLINE="skew_tick=1"`만. RT 커널·PREEMPT_RT는 유지(문제 없음, 지터원은 격리 전략이지 커널이 아님).
> **격리 재도입은 EtherCAT 통합 시작할 때 3+1(isolcpus=3)로** — 비전 3코어 + 제어 1코어. (1kHz EtherCAT+제어는 격리 코어 1개면 충분.)
>
> **비전 파이프라인 코어 사용량 실측 (2026-07-09, 4코어 정상 상태, yolov3_tiny_7class):**
> 총 **~2.8 코어**(70% of 4). frame_age ~120ms로 건강(격리 때 7500ms stall과 대조), ~75초 무정지, SSH 정상.
>
> | 프로세스 | 코어 | 비고 |
> |---|---|---|
> | **pick_target_3d_node** | **~0.73** | ★ #1 소비자. "1픽셀 reverse projection"인데 과다 → **full depth(848×480) 30fps 매 프레임 처리 의심. 최우선 최적화 대상** |
> | realsense_camera_node | ~0.55 | USB 단일스레드(기존에 알려진 병목) |
> | vitis_ai_detector_node | ~0.48 | 전/후처리·IPC |
> | vitis_ai_worker (DPU) | ~0.38 | NN은 PL DPU로 오프로드돼 적음 (dpu_ms~17) |
> | pick_target_base_node | ~0.14 | |
> | pick_logic | ~0.10 | |
> | USB kworker + softirq 등 | ~0.4 | |
>
> **함의 (재발 여부 = 몇 코어 격리하느냐):**
> - **3+1**(비전 3코어): 2.8 < 3 → **되지만 마진 얇음(~0.2코어)**, IRQ 버스트/EtherCAT softirq 겹치면 위험.
> - **2+2**(비전 2코어): 2.8 > 2 → **불가**(2026-07-09 사고).
> - **`pick_target_3d_node` 최적화(~-0.5코어 기대)** 시 3+1 여유, 2+2도 검토 가능. → **전면 재작성 불필요, targeted 최적화 1건이 핵심.** 추가 레버: 탐지주기 throttle, 카메라 FPS↓, intra-process composition, xmodel에 정규화 fuse.
> - 궁극 해답은 **RPU 이전**(제어를 R5+TCM로 옮기면 A53 4코어 전부 비전 차지, 경합 자체 소멸). 오늘 사건이 그 근거.
isolcpus는 **스케줄링 간섭**만 막는다. 실제 1kHz 제어의 결정성을 위협하는 것은 아래 **공유 자원 경합**이며, 이건 `stress-ng` 합성부하로는 안 드러나고 **실제 비전 파이프라인+DPU+EtherCAT를 돌리면서** 격리 코어(2,3)를 측정해야만 보인다.
- **[측정=완료조건] 통합 부하 레이턴시 테스트**: `ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py`로 비전+DPU 추론을 돌리는 상태에서 `sudo cyclictest -m -p 90 -i 1000 -a 2,3 -t 2 -D 5m -q`. CPU2,3 Max<100µs면 통과. (지금까지는 stress-ng 합성부하로만 쟀음 — 불충분.)
- **B2 L2/DDR 경합 (잔존 최대 리스크)**: A53 4코어가 L2(1MB)와 DDR 컨트롤러를 공유. 비전은 메모리 대역 소모가 큼(RealSense 프레임, DPU가 CMA↔DDR로 텐서 DMA). 0,1의 트래픽이 2,3의 캐시/메모리 접근을 지연 → isolcpus로도 못 막는 스파이크. (앞서 stress-ng에서 본 CPU3 240µs 스파이크의 유력 원인.)
- **B3 EtherCAT generic 드라이버의 net-stack 결합**: generic은 EtherCAT 프레임을 리눅스 net stack(macb IRQ→NET_RX softirq→마스터)으로 처리. eth0 IRQ/softirq가 0,1에 있고 0,1이 비전으로 포화되면 EtherCAT RX가 지연됨(마스터가 2,3에 있어도). **개선책**: 일반 IRQ는 0,1이되 **eth0(EtherCAT) IRQ는 격리 코어 쪽에 별도 pin**(마스터 근처) 검토. 장기적으론 native macb IgH 드라이버가 net-stack 우회.
- **[해결됨 ✅ 2026-07-09] DisplayPort IRQ 낭비**: 헤드리스인데 `fd4a0000.display`(IRQ56)가 CPU0에서 **~10.9K irq/s (≈7.8% CPU)** 상시 발생했음. 드라이버는 모듈 `zynqmp_dpsub`(DRM **card0**). DPU는 별개 카드(**card1**=zocl-drm)라 이 조치는 비전/DPU에 무영향(smartcam active, `renderD128` 유지).
  - **런타임 제거**(`sudo modprobe -r zynqmp_dpsub`)는 즉효 — IRQ56 항목 소멸, CPU0 7.8% 회수 확인됨.
  - **⚠️ 함정(2026-07-09 실측): `blacklist`만으로는 재부팅 후 다시 로드된다.** 이유: `zynqmp-dpsub.ko`가 **initramfs에 포함**돼 있고 `console=tty1`(프레임버퍼 콘솔) 때문에 **initramfs 단계에서 조기 로드**되는데, 당시 initramfs(07-08 15:45자, mali 때 마지막 갱신)에는 mali blacklist만 있고 우리 dpsub blacklist가 없었음 → 모듈이 이미 로드된 뒤라 **rootfs의 blacklist는 뒷북=무력**. (`lsinitramfs`로 dpsub.ko 포함 + 우리 conf 누락 확인.)
  - **올바른 fix (initramfs에 반영)**:
    ```bash
    printf 'blacklist zynqmp_dpsub\ninstall zynqmp_dpsub /bin/true\n' | sudo tee /etc/modprobe.d/blacklist-zynqmp-dpsub.conf
    sudo update-initramfs -u -k 5.15.199-rt91-rt-kria   # ★ 이게 빠져서 실패했었음. flash-kernel 자동 트리거됨.
    sudo reboot
    # 확인: lsmod|grep dpsub (없어야) / grep fd4a0000.display /proc/interrupts (없어야)
    ```
  - 부작용: 물리 연결된 모니터 로컬 화면(tty1) 꺼짐 — SSH/시리얼(ttyPS1)로만 접근. 되돌리기: blacklist 파일 삭제 → `sudo update-initramfs -u -k 5.15.199-rt91-rt-kria` → 재부팅.
  - 교훈: **모듈 blacklist가 재부팅 후 안 먹으면 그 모듈이 initramfs에 있는지 의심**(`lsinitramfs $IMG | grep <mod>`), 있으면 `update-initramfs` 필수. (커스텀 커널 재빌드 시 §4-9의 mali도 동일 이유로 update-initramfs 필요.)
  - **결과(2026-07-09 재부팅 검증 완료 ✅)**: 부팅 후 `lsmod|grep dpsub` 비었고 `/proc/interrupts`에 fd4a0000.display 없음, CPU0 `irq/56` 소멸 확인. smartcam active·zocl 정상. **주의: DP(card0) 제거로 zocl DRM이 card1→card0으로 리넘버됨**(`/dev/dri/card0`+`renderD128`). VART/XRT는 디바이스 열거로 찾으므로 무관하나, 파이프라인이 `/dev/dri/cardN` 인덱스를 하드코딩하면 확인 필요(통합 부하 테스트에서 자연 검증).
- **아키텍처적 함의**: L2/DDR 경합(B2)이 결국 **RPU(R5F+TCM) 이전의 근본 근거**. APU 방식은 실용적 1단계로, 튜닝하면 1kHz 도달 가능성이 높으나 이 경합 천장이 RPU가 존재하는 이유. 안 되면 제어주기↓ 또는 RPU 조기 이전.
- 보조 관찰(2026-07-08 실측): 온도 ~34°C·팬 PWM 6% (CPU_IDLE/FREQ off에도 열 여유 충분, 위험 없음). PREEMPT_RT+zocl 초기화 시 RT 위반 경고 없음(단 **추론 부하 중은 미검증**). A53 고정 주파수가 최대 OPP인지 `sudo cat /sys/kernel/debug/clk/clk_summary | grep -i acpu`로 확인 권장.

## 7. 다음 단계: IgH EtherCAT Master (미착수)

> ⚠️ **선결조건 (2026-07-10 추가, 07-12 갱신)**: §12의 RT 커널 재빌드(⓪ LAZY=n 실험 → ④ linux-xlnx+RT 머지)와 안정성 검증이 끝나기 전에는 착수 금지. 현 RT 커널은 부팅 초기 오염(확률적) 상태로 확인됨 — 이 위에 실시간 로봇 제어를 올리면 안 된다.

랩 가이드 6장 절차를 따르되 **Kria 차이점**:

1. **NIC 드라이버**: KV260의 PS 이더넷은 Cadence GEM(`macb` 드라이버, GEM3=ff0e0000, eth0). IgH에 네이티브 macb 드라이버 없음 → configure에서 인텔용 `--enable-igc` 대신 **`--enable-generic`** 사용, `ethercat.conf`의 `DEVICE_MODULES="generic"`. generic은 커널 net stack 경유라 지터가 다소 크지만 RT 커널 + 1kHz에서 검증된 조합. 성능 부족 시 커뮤니티 macb 패치 검토.
2. **⚠️ 네트워크 계획 필수**: KV260은 유선 포트가 eth0 하나뿐이고 **현재 Tailscale 원격 접속이 eth0을 사용 중**. EtherCAT이 eth0을 점유하면 원격 접속 불가 → 작업 전에 대체 접속(USB 이더넷 어댑터 / WiFi / 시리얼 콘솔) 확보할 것. 보드는 손 닿는 위치에 있음(UART 가능).
3. 빌드는 보드에서 직접 가능 (커널 headers 설치돼 있음: `linux-headers-5.15.199-rt91-rt-kria`).
4. 소스: `gitlab.com/etherlab.org/ethercat.git`, `stable-1.6` 브랜치. configure 예시(가이드 기반, Kria 수정):
   ```bash
   ./configure --prefix=/opt/etherlab --enable-generic --disable-8139too --enable-hrtimer --enable-eoe=no
   ```
   (인텔 전용 옵션 igc/igb/e1000e/cycles는 제외)
5. 이후: `ethercatctl start` → `ethercat slaves`로 슬레이브(LS Mecapion L7N 테스트 드라이브) 인식 확인 → CiA402+CSP 제어. 사용자가 PC에서 EtherCAT으로 Indy7 제어 성공한 코드 보유(마스터 종류 확인 필요) — 이식 대상.

그 다음(EtherCAT 동작 후): ROS 2 RT 튜닝 — `/etc/security/limits.conf`에 rtprio/memlock, 제어 스레드 SCHED_FIFO + 격리 코어 pinning(3+1 채택 시 코어3), mlockall, DDS 스레드 우선순위.

## 8. 운영·유지보수 규칙

### 커널 전환: 기본(순정) 커널 ↔ RT 커널

**전제 이해**: 이 보드는 GRUB 같은 부팅 메뉴가 없다. U-Boot이 `/boot/firmware/image.fit` **한 개**를 로드하므로, "어느 커널로 부팅할지"는 부팅 전에 flash-kernel로 image.fit을 원하는 커널로 다시 만들어두는 방식으로 결정한다. 두 커널 패키지(순정 `5.15.0-1070-xilinx-zynqmp`, RT `5.15.199-rt91-rt-kria`)는 `/boot`에 공존하며, **순정 커널 패키지를 절대 apt remove 하지 말 것** (복귀 수단임).

**현재 어떤 커널인지 확인:**
```bash
uname -r                    # 5.15.199-rt91-rt-kria = RT / 5.15.0-1070-xilinx-zynqmp = 순정
cat /sys/kernel/realtime    # 파일이 있고 1이면 RT 커널
```

**RT → 순정(기본) 커널로 전환 (★ `--force` 필수 — 함정 ⑦):**
```bash
sudo flash-kernel --force 5.15.0-1070-xilinx-zynqmp
# 출력 확인: "forcing install of 5.15.0-1070-xilinx-zynqmp instead of 5.15.199-rt91-rt-kria."
sudo reboot
# 재부팅 후 uname -r 로 5.15.0-1070-xilinx-zynqmp 확인
```
- **함정 ⑦ (2026-07-12 실증)**: flash-kernel은 설계상 **"설치된 것 중 최신 버전"만 설치**한다. 최신이 아닌 버전을 지정하면 `Ignoring old or unknown version ... (latest is ...)`를 찍고 **아무것도 안 하고 exit 0** (에러도 아님 — 조용히 무시되므로 출력을 꼭 읽을 것). 구버전 선택은 `--force`로만 가능하고, **`--force`는 첫 번째 인자**여야 한다(functions 923행). 순정 부팅 중엔 initrd가 구버전(6/22, blacklist 미반영)이라 DP IRQ 스톰이 일시 부활함(무해, §11 참고).

**순정 → RT 커널로 전환 (RT가 최신이라 --force 불필요):**
```bash
sudo flash-kernel 5.15.199-rt91-rt-kria
sudo reboot
# 재부팅 후 uname -r 에 PREEMPT_RT 포함(uname -a) 확인
```

**전환 시 알아둘 것:**
- 전환은 image.fit만 바꾸는 것이라 rootfs/데이터에는 아무 영향 없음. 몇 번이든 왕복 가능.
- flash-kernel 실행 시 직전 image.fit이 자동으로 `image.fit.bak`이 되므로, 전환 직후에는 .bak = 직전 커널이다.
- `Couldn't find DTB ...` 경고는 무해함 (§4-6 참조).
- ⚠️ **순정으로 전환해 둔 상태에서 주의**: flash-kernel이 (버전 지정 없이) 자동 트리거되면 — 예: apt가 어떤 커널/initramfs 패키지를 건드릴 때 — "설치된 커널 중 최고 버전"을 선택하는데, flavor override(`any`) 때문에 그게 **RT 커널(5.15.199 > 5.15.0)**이다. 즉 순정 상태가 조용히 RT로 되돌아갈 수 있다. 순정은 임시 확인용으로만 쓰고, 순정으로 오래 머물러야 하면 그 기간엔 apt 커널 관련 작업을 피하거나 재부팅 전 `uname -r`을 습관적으로 확인할 것. (RT 상태에서는 반대로 이 동작이 "RT 유지" 안전장치로 작용함.)
- isolcpus 등 `/etc/default/flash-kernel`의 `LINUX_KERNEL_CMDLINE`은 **양쪽 커널에 공통 적용**된다. 순정 커널에서 격리를 원치 않으면 그 값을 비우고 flash-kernel을 다시 실행해야 함.

### 부팅 실패 시 복구 (전환 실수 포함)
1. flash-kernel은 갱신 때마다 직전 FIT을 `image.fit.bak`으로 보관 → SD를 PC에 꽂고 첫 파티션(FAT)에서 `cp image.fit.bak image.fit` 하면 직전 커널로 복귀.
2. .bak도 손상됐으면 백업본 사용: 보드 `/root/backup/image.fit.stock`(순정) 또는 PC의 `kria-backup/`.
3. 시리얼 콘솔(ttyPS1, 115200)이 있으면 U-Boot 프롬프트에서 수동으로 .bak 부팅도 가능(필요 시 boot.scr 명령 참조: `/etc/flash-kernel/bootscript/bootscr.zynqmp.kria`).

### 커스텀 커널 재빌드/재설치 시 체크리스트
1. PC 소스: `~/kria-rt/linux-5.15-rt-kria` (RT 패치 적용된 상태). `.config` 재사용.
2. dpkg -i 후 **DTB 복사 필수** (함정 ⑤): `/usr/lib/linux-image-<새버전>/xilinx` → `/lib/firmware/<새버전>/device-tree/`
3. `sudo flash-kernel <새버전>` 으로 명시 지정, 출력의 "installing version"이 새 버전인지 확인.
4. `update-initramfs -u -k <버전>`은 flash-kernel을 자동 트리거함(별도 실행 불필요).
5. flavor override는 이미 영구 적용돼 있음(재작업 불필요).

## 9. 참고

- RT 패치 아카이브: https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/older/
- 커널 소스(소스 패키지): https://launchpad.net/ubuntu/+source/linux-xilinx-zynqmp → 버전 클릭 → 파일 다운로드
- 랩 가이드: MAN-20241113-LX02H0001 (RAIMLAB, x86용 — 절차 뼈대 참고용)
- IgH 문서: https://docs.etherlab.org/ethercat/1.6/pdf/ethercat_doc.pdf
- RPU 트랙 인수인계: `~/ros2_ws/rpu_guide_for_claude.md` (새 세션은 그 파일부터)

## 10. RT 커널 완전 제거 / 원상 복구 (RPU 이전으로 RT가 불필요해진 경우)

배경: EtherCAT을 RPU(FreeRTOS+SOEM)에 올리면 APU 리눅스는 RT 커널이 필요 없어질 수 있다. 그때 순정 커널로 되돌리고 이 작업에서 만든 흔적을 지우는 절차. **순정 커널 패키지와 rootfs는 처음부터 건드리지 않았으므로 제거는 안전하다.**

핵심 원칙(순서 중요): **① 반드시 순정 커널로 부팅한 상태에서 → ② RT 패키지를 지우고 → ③ 우리가 수동으로 만든 파일/설정을 되돌린다.** (실행 중인 커널은 제거할 수 없다.)

### 10-1. 순정 커널로 부팅
```bash
sudo flash-kernel --force 5.15.0-1070-xilinx-zynqmp   # --force 필수 (함정 ⑦, §8)
sudo reboot
# 재부팅 후 반드시 확인 (RT면 아래 단계 진행 금지):
uname -r          # → 5.15.0-1070-xilinx-zynqmp 여야 함
```

### 10-2. RT 커널 패키지 제거
순정으로 부팅됐음을 확인한 뒤:
```bash
# apt hold 먼저 해제 (걸려 있었다면)
sudo apt-mark unhold linux-image-xilinx-zynqmp linux-xilinx-zynqmp linux-headers-xilinx-zynqmp

# RT 이미지·헤더 purge. postrm이 /boot의 vmlinuz/initrd/config/System.map과
# /lib/modules/5.15.199-rt91-rt-kria 를 제거하고, flash-kernel을 트리거해
# image.fit을 "남은 최고 버전(=순정)"으로 재생성한다.
sudo apt purge linux-image-5.15.199-rt91-rt-kria linux-headers-5.15.199-rt91-rt-kria
```
확인:
```bash
dpkg -l | grep rt-kria      # 출력 없어야 함
ls /boot | grep rt-kria     # 출력 없어야 함
```

### 10-3. 수동으로 만든 파일/설정 되돌리기
패키지가 자동으로 못 지우는(우리가 직접 만든) 항목들:
```bash
# (a) FIT 생성용으로 복사했던 커스텀 DTB — dpkg 소유가 아니라 수동 삭제
sudo rm -rf /lib/firmware/5.15.199-rt91-rt-kria

# (b) flash-kernel flavor override — RT 커널이 사라지면 무해하지만 깔끔히 제거.
#     /etc/flash-kernel/db 파일 끝에 추가했던 아래 블록을 삭제:
#         (빈 줄)
#         Machine: ZynqMP *KV260*
#         Kernel-Flavors: any
sudo nano /etc/flash-kernel/db      # 파일 끝의 해당 블록 삭제 후 저장

# (c) isolcpus 등 부트 인자 — 양쪽 커널에 공통 적용되므로, 순정에서 코어 격리를
#     원치 않으면 비운다.
sudo sed -i 's/^LINUX_KERNEL_CMDLINE=.*/LINUX_KERNEL_CMDLINE=""/' /etc/default/flash-kernel

# (b),(c) 반영 위해 순정 커널 FIT 재생성
sudo flash-kernel 5.15.0-1070-xilinx-zynqmp
```
> 참고: `/boot/firmware/image.fit.bak`에는 직전(RT) FIT이 남아 있을 수 있으나 무해하다. 신경 쓰이면 순정 부팅 검증 후 `sudo cp /boot/firmware/image.fit /boot/firmware/image.fit.bak`로 덮어써도 된다.

### 10-4. 헤드리스/GPU 설정 (선택 — RT와 무관)
아래는 RT 커널 때문이 아니라 **지터·부팅 안정화** 목적이었다. 순정에서도 이 보드를 제어용으로 계속 쓸 거면 **그대로 두는 편이 낫다.** 원래의 데스크톱 환경으로 완전 복원할 때만:
```bash
sudo systemctl set-default graphical.target                 # GUI 부팅 복원
sudo rm /etc/modprobe.d/blacklist-mali.conf                 # mali GPU 블랙리스트 해제
sudo rm /etc/modprobe.d/blacklist-zynqmp-dpsub.conf         # DisplayPort 드라이버 복원(로컬 모니터 화면 살아남)
sudo update-initramfs -u -k 5.15.0-1070-xilinx-zynqmp       # 순정 initrd 갱신(+flash-kernel 자동 트리거)
```

### 10-5. 최종 검증 (재부팅 후)
```bash
uname -a                                 # PREEMPT_RT 없음, 5.15.0-1070 확인
cat /proc/cmdline                        # isolcpus 등 없음 확인
ls /sys/kernel/realtime 2>/dev/null      # 파일 없어야 함(No such file)
cat /sys/devices/system/cpu/isolated     # 비어 있어야 함
dpkg -l | grep rt-kria                   # 없음
apt-mark showhold                        # 비어 있음(의도한 경우)
```

### 10-6. 빌드 PC 정리 (선택)
나중에 RT를 다시 쓸 가능성이 있으면, 재빌드(수십 분) 대신 **.deb 2개만 백업**해두면 `dpkg -i`로 2분 만에 복원할 수 있다. 소스 트리(~2GB)만 지우는 것을 권장:
```bash
mkdir -p ~/kria-rt-debs-archive
cp ~/kria-rt/linux-image-5.15.199-rt91-rt-kria_*.deb \
   ~/kria-rt/linux-headers-5.15.199-rt91-rt-kria_*.deb ~/kria-rt-debs-archive/
rm -rf ~/kria-rt/linux-5.15-rt-kria      # 소스 트리 삭제(원하면 ~/kria-rt 전체)
```

### RT를 다시 설치하고 싶어지면
- **.deb를 보관해뒀다면**: §4-6부터 (dpkg -i → **함정 ④(flavor override) 다시 적용** → **함정 ⑤(DTB 복사)** → `flash-kernel <RT버전>`). §10-3(b)에서 override를 지웠으므로 반드시 다시 넣어야 한다.
- **.deb도 없으면**: §4-2부터 전체 재수행.

---

## 11. 순정(비-RT) 검증 빌드 — RT 패치 없이 같은 경로로 빌드해 부팅 테스트 (2026-07-12 가이드)

**목적**: RT 패치라는 변수를 뺀 **A/B 기준선**. 같은 소스·config·툴체인·설치 경로로 순정 커널(`5.15.199-stock-kria`)을 빌드해 부팅되면, "빌드 경로 자체는 건전"이 증명됨 — 부팅 프리즈 등 이슈가 RT 탓인지 가를 수 있고, cyclictest 비교 기준선도 됨.

**⚠️ 공장 순정 부팅과 목적이 다름 (2026-07-12 확인)**: 백업(`image.fit.stock`)이나 `flash-kernel 5.15.0-1070-...`으로 부팅하는 공장 순정은 **우리 빌드 경로를 안 거친 바이너리**라 §11을 대체 못 함(공장 커널이 부팅됨은 이미 아는 사실). 참고로 백업 FIT 내 커널은 설치된 `/boot/vmlinuz-5.15.0-1070-...`과 **바이트 일치 실측**(17,513,402B) — 백업 파일과 flash-kernel 재생성은 사실상 동일. 3-way 비교: 공장 1070(기지) / 우리빌드 stock-kria(§11, 빌드경로 검증) / 우리빌드 RT(운용 중). 공장 순정 부팅 시 initrd가 구버전(6/22)이라 dpsub 블랙리스트 미반영 → DP IRQ 스톰 일시 부활(무해, RT 복귀 시 소멸).

**⚠️ 신규 함정 ⑥ (2026-07-12 보드 실측)**: `linux-version sort` 기준 **`5.15.199-stock-kria`가 `5.15.199-rt91-rt-kria`보다 위로 정렬됨**(문자열 s > r). 두 가지 결과:
1. **`dpkg -i` 순간의 auto-trigger가 곧바로 stock-kria FIT을 만들려고 함** → 그 시점에 DTB(함정 ⑤)가 없으면 설치가 깨짐. → **해법: DTB를 dpkg 전에 선복사** (아래 절차에 반영).
2. stock-kria가 설치돼 있는 동안엔 **모든 auto-trigger가 RT가 아니라 stock-kria를 고름**. → 테스트 끝나면 **purge 권장** (§8의 안전장치 방향이 반대로 뒤집힌 상태이므로).

### 11-1. 빌드 [PC]
```bash
cd ~/kria-rt
ls linux-xilinx-zynqmp_5.15.0-1070.74.dsc     # §4-2에서 받은 소스 3종이 있어야 함
# 새 트리로 추출 (기존 RT 트리 linux-5.15-rt-kria는 보존)
dpkg-source --no-check -x linux-xilinx-zynqmp_5.15.0-1070.74.dsc linux-5.15-stock-kria
cd linux-5.15-stock-kria
chmod -R +x scripts/ debian/rules debian/scripts/    # 함정 ①
make kernelversion                                    # → 5.15.199 확인
# RT 패치는 적용하지 않음! (그게 이 테스트의 핵심)

scp ubuntu@<KRIA_IP>:/boot/config-5.15.0-1070-xilinx-zynqmp .config
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
make olddefconfig
# 최소 수정만 — RT빌드와 달리 KVM/CPU_IDLE/HZ 등 동작 관련은 아무것도 안 건드림
scripts/config --set-str LOCALVERSION "-stock-kria"
scripts/config --disable SYSTEM_TRUSTED_KEYS --disable SYSTEM_REVOCATION_KEYS   # 빌드용(서명키)
scripts/config --disable DEBUG_INFO                                             # 빌드 시간/용량
make olddefconfig
grep -E '^CONFIG_PREEMPT_VOLUNTARY=|^CONFIG_HZ=|^CONFIG_LOCALVERSION=' .config
# 기대: PREEMPT_VOLUNTARY=y / HZ=250 / LOCALVERSION="-stock-kria"  (RT빌드의 PREEMPT_RT/HZ=1000과 대비)
make -j$(nproc) bindeb-pkg        # 15-30분
ls -lh ../linux-image-5.15.199-stock-kria_*.deb    # headers는 부팅 테스트엔 불필요
```

### 11-2. 설치 [보드] — 순서 중요 (함정 ⑥ 때문에 DTB 선복사가 dpkg보다 먼저)
```bash
# [PC] 전송
scp ~/kria-rt/linux-image-5.15.199-stock-kria_*.deb ubuntu@<KRIA_IP>:~/

# [보드] ① DTB 선복사 — deb를 설치 전에 미리 풀어서 (함정 ⑤+⑥ 동시 해결)
dpkg-deb -x ~/linux-image-5.15.199-stock-kria_*.deb /tmp/stock-kernel
sudo mkdir -p /lib/firmware/5.15.199-stock-kria/device-tree
sudo cp -r /tmp/stock-kernel/usr/lib/linux-image-5.15.199-stock-kria/xilinx \
           /lib/firmware/5.15.199-stock-kria/device-tree/

# ② 설치 (auto-trigger가 stock-kria FIT 생성 — ①덕에 성공함)
sudo dpkg -i ~/linux-image-5.15.199-stock-kria_*.deb

# ③ 명시 재생성으로 확정 + 육안 확인
sudo flash-kernel 5.15.199-stock-kria      # "installing version 5.15.199-stock-kria" 확인
sudo reboot
```
- 만약 ①을 건너뛰어 ②에서 flash-kernel 트리거가 DTB 못 찾고 실패하면: DTB 복사 후 `sudo dpkg --configure -a`로 복구.

### 11-3. 부팅 검증 [보드]
```bash
uname -a                                   # 5.15.199-stock-kria, PREEMPT_RT 없어야 함
ls /sys/kernel/realtime 2>/dev/null        # No such file 이어야 함 (RT 아님)
cat /proc/cmdline                          # cma=1000M, skew_tick=1 유지
systemctl is-active kv260-smartcam.service # active (DPU)
lsmod | grep -E '^zocl|dpsub|mali'         # zocl만. dpsub/mali 없어야 함(블랙리스트는 rootfs 공통이라 새 initrd에도 자동 반영)
# 선택: 비전 파이프라인 스모크 테스트 + 재부팅 수 회로 프리즈 재현 관찰
```
- 참고: 이 테스트 커널도 헤드리스·mali/dpsub 블랙리스트·`skew_tick=1`이 그대로 적용됨(rootfs 공통) — "원래 1070 커널의 GUI 있던 환경"과는 조건이 다름을 유의.
- **해석**: stock-kria에서도 프리즈 발생 → 빌드 경로/HW 문제(RT 무죄). stock-kria는 멀쩡한데 RT만 프리즈 → RT 패치 연관.

### 11-4. RT로 복귀 + 정리 [보드]
```bash
# ★ stock-kria가 설치돼 있는 동안엔 그게 "최신"이라 --force 필요 (함정 ⑥+⑦)
sudo flash-kernel --force 5.15.199-rt91-rt-kria && sudo reboot
uname -r                                   # 5.15.199-rt91-rt-kria 확인

# 테스트 끝나면 stock-kria 제거 권장 (함정 ⑥: 있는 동안 auto-trigger가 stock을 고름)
sudo apt purge linux-image-5.15.199-stock-kria
sudo rm -rf /lib/firmware/5.15.199-stock-kria
sudo flash-kernel 5.15.199-rt91-rt-kria    # 확인차 재생성 ("installing version ...rt-kria")
```

---

## 12. ★ 2026-07-10 커널 크래시 사건 — 원인 진단·조치·재빌드 계획 (구 §11 — §11 순정 검증 빌드와 번호 중복이라 §12로 변경, 2026-07-12)

> 진단 상세(증거 위치·검증 과정·참고 링크 전체)는 **`rt_kernel_fix_plan.md`** 참조. 여기는 사건 기록 + 쉬운 설명 + 결정사항 요약.

### 12-1. 무슨 일이 있었나 (타임라인)

| 시각 (2026-07-10) | 사건 |
|---|---|
| 21:00~21:04 | 3코어 제약 비전 파이프라인 성능 측정(150s) — **정상 완료**, 데이터 확보 |
| 21:05:26 | bag 녹화용 재기동 ~20초 뒤 **커널 크래시 ①** (`Unable to handle kernel paging request`, SLUB freelist 오염) → 보드 hang, 원격 끊김 |
| 21:22 | 사용자가 전원 리셋 → 재부팅 |
| 21:24:10~32 | **거의 무부하 상태**에서 fpsimd WARNING 폭풍(fpsimd.c:318/173/204, CPU 2→0→1→3) → **커널 크래시 ②** → 또 hang |
| 21:45 | 두 번째 전원 리셋 → 이후 조사 시작 |

추가 발견: `/var/log/kern.log` 분석 결과 **07-08에도 동일 서명(SLUB oops)의 크래시가 1회** 있었고, §6-1의 "부팅 프리즈"들도 같은 원인의 발현일 가능성이 높다.

**후속 타임라인 (2026-07-12 ~ 07-13, 원인 규명 완결):**

| 날짜 | 사건 |
|---|---|
| 07-12 | crash_logs 전수 분석 — BUG 유무와 생사가 무상관임을 발견(무음 오염 존재), DEBUG_PREEMPT=n이라 원점 미기록 확인 (fix_plan (a-2)) |
| 07-12~13 | 사용자가 PC에서 ⓪ 실험 커널 직접 빌드 (`-rt-kv260`, LAZY=n + DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP=y). 도중 함정 3연속: ①Kconfig `def_bool` 되돌림(LAZY=y로 빌드됨) ②x86 menuconfig 저장 사고로 ARCH_ZYNQMP=n(부팅 불가 deb — 설치 전 검증에서 차단) ③PC systemd-oomd가 빌드 그룹 킬(스왑 0). 매번 deb 내부 config 사전 검증으로 잡음 |
| 07-13 00:31 | ⓪ 커널 설치·부팅 → **크래시 없이 로그인 도달, 검출기 리포트 253건 전수가 단일 원점 `__radix_tree_preload` 지목** → LAZY 무죄 확정 |
| 07-13 새벽 | 1차 소스 전수 대조(vanilla 5.15.199·linux-xlnx v2022.1·rt91 패치·Ubuntu jammy 제네릭/xilinx) → **근본 원인 확정: `UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`** (05fdd32398, NVIDIA 모듈 GPL export 회피용). ⓪.5 국소 픽스 경로 수립, vanilla 원본 3파일 준비. 보드 순정 복귀 |

### 12-2. 쉬운 설명 — 무엇이 잘못됐나

**(1) 커널에서의 버그는 프로그램 버그와 다르다.** 커널은 모든 프로그램(카메라·비전·SSH)을 관리하는 OS의 심장이라, 커널 안에서 버그가 나면 프로그램 하나가 아니라 **보드 전체가 멈춘다.** 우리가 본 "원격 끊김 + 보드 무반응"이 바로 그것.

**(2) 우리 RT 커널은 '검증된 적 없는 조합'으로 조립됐다.** 비유하면 — 현대차 개조 엔진(**우분투가 스케줄러를 자체 수정한 커널**)에, 도요타 순정용 터보킷(**순정 리눅스 전용으로 만들어진 RT 패치**, 그마저 .197용을 .199에)을 달고, 시제품 부품(**실험 단계 옵션 `PREEMPT_LAZY`**)까지 끼운 것. 볼트 구멍이 얼추 맞아 조립도 되고 시동도 걸렸지만(빌드·부팅 성공), 이 3중 조합의 설계 정합성은 아무도 검증한 적이 없다.

**(3) 실제 고장 메커니즘 — "방해금지 팻말" 이야기.** 커널 안에는 "이 작업 중엔 절대 끼어들면 안 되는 구간"이 있다(장부에 숫자를 적는 도중처럼). 그 구간에 들어갈 때 **'방해금지' 팻말을 걸고, 나올 때 내리는 게 규칙**이다. 우리 조합의 버그는: **새 프로세스가 태어날 때 팻말이 안 내려간 채로 세상에 나온다.** 로그의 `BUG: scheduling while atomic ... 0x00000002`가 바로 "팻말 2개가 걸린 채로 작업 교체가 일어났다"는 커널의 비명이다. 팻말이 걸려 있으면 커널은 "이 CPU의 전용 작업대를 독점 중"이라 믿는데, 실제로는 작업이 코어를 옮겨 다니면서 **코어마다 하나뿐인 전용 작업대를 두 작업이 동시에 만지는** 상황이 된다. 그 작업대 위의 것들이 오염된다:
- **메모리 할당 장부(SLUB)** 오염 → 장부에 엉뚱한 주소가 적힘 → 나중에 누군가 읽는 순간 즉사 (크래시 ①, 그리고 07-08 크래시)
- **소수점 계산 레지스터 관리(FPSIMD)** 오염 → 경고 폭풍 후 즉사 (크래시 ②)

**(4) 왜 '과부하 때문'처럼 보였나 — 러시안룰렛.** 커널의 비명(BUG 로그)은 부팅 4~5초에 자주 찍히지만 **확률적**이고(부팅별 0~6건, 07-12 정정), 더 무섭게는 **비명 없이 오염만 조용히 진행되기도 한다** — 치명 크래시 2건(07-08 21:08, 07-10 21:05)은 둘 다 "부팅 중 비명 0건"인 부팅에서 났고, 반대로 비명 2건이 찍힌 부팅은 42시간을 무사히 버텼다(부팅별 전수 카운트: `rt_kernel_fix_plan.md` (a-2) 표, 2026-07-12 crash_logs 정밀 분석). 즉 커널은 켜질 때부터 항상 시한폭탄 상태였고, 죽으려면 "오염된 장부를 하필 그 순간 읽는" 불운이 겹쳐야 한다. 부하가 높으면 작업 교체가 많아져 방아쇠를 더 자주 당길 뿐이다. 증거: 크래시 ②는 부팅 2분 뒤 거의 놀고 있을 때 터졌다 — 부하가 원인이면 설명이 안 된다. 그동안 측정이 잘 돌아간 건 총알이 안 걸렸던 것.

**(5) 'fpsimd는 카나리아'의 뜻.** 광산의 카나리아처럼, fpsimd 경고는 "독가스(팻말 회계 꼬임)가 퍼져 있다"를 가장 먼저 보여준 민감한 지점일 뿐이다. 새가 광부를 죽인 게 아니라 가스가 원인.

**(6) 왜 '부품 교체'가 아니라 '재조립'인가.** 이후 버전(rt92~rt96) 어디에도 이 문제를 고친 수정이 없다 = 남들이 겪고 고친 버그가 아니라 **우리 조합에서만 생기는 문제**다. 그래서 고치는 법은 하나: 검증된 짝으로 재조립 — **순정(vanilla) 리눅스 5.15.209 + 그 버전 전용 RT 패치(rt96)**, 실험 옵션 끄고, 검증 기간엔 경고등(디버그 옵션)을 켜고 시운전. *(→ (7)에서 뒤집힘: 진범이 잡혀서 재조립까지 필요 없어졌다.)*

**(7) [2026-07-13 완결] 진범이 잡혔다 — 재조립까지 필요 없었다.** 경고등(DEBUG_PREEMPT)을 단 실험 커널로 부팅하자 사고 현장 253곳이 찍혔는데, **전부 같은 부품을 가리켰다**: 커널이 새 프로세스 번호표나 sysfs 항목을 만들 때 쓰는 공용 준비 절차(radix/idr preload). 알고 보니 **Ubuntu가 NVIDIA 그래픽 드라이버 호환 문제 때문에 이 부품 하나만 옛날 방식("방해금지 팻말 걸고 작업")으로 되돌려놨던 것**이다. 일반 커널에서는 정말 무해하다(Ubuntu 커밋에도 "RT 빌드용 변경이라 무해"라고 적혀 있다). 하지만 RT 커널은 락을 "잠들 수 있는" 방식으로 바꾸기 때문에, 이 조합은 "팻말 걸어놓고 잠들기"라는 금지 동작이 돼버린다 — §12-2(3)의 팻말 이야기의 실체가 바로 이것. **수리법: 그 부품(파일 3개)만 순정 리눅스 원본으로 갈아끼우고 재빌드** — §12-3의 ⓪.5. LAZY도, Ubuntu 스케줄러 수정도, 버전 미스매치도 범인이 아니었다.

### 12-3. 결정·조치 (2026-07-10, 07-12 갱신)

1. **RT 커널 사용 중지.** 보드는 **순정 커널 `5.15.0-1070-xilinx-zynqmp`로 전환 — ✅ 2026-07-12 완료**(§8 전환 절차, ★ 구버전 선택이므로 `sudo flash-kernel --force 5.15.0-1070-xilinx-zynqmp` — 함정 ⑦). ~~전환 전 update-initramfs로 blacklist 반영~~ → **정정(2026-07-12, 사용자 결정): 순정 커널에서는 dpsub 차단하지 않음** — 실시간성 요구가 없고, DP IRQ ~11K/s(CPU0 실측)는 수용. 순정에서의 fps 측정은 전부 dpsub 포함 조건으로 일관 수행. RT 복귀 시엔 RT initrd에 blacklist 반영돼 있어 자동 소멸.
2. **비전/fps 최적화 작업은 순정 커널에서 계속.** 비전 처리량엔 RT가 필요 없다 — RT는 EtherCAT 1kHz 제어 주기 엄수용.
3. **RT 커널 재빌드 = EtherCAT(§7) 선결조건.** 레시피(`rt_kernel_fix_plan.md` 상세):
   - ~~**(선택) 0단계 값싼 실험**: 기존 소스에서 config만 `PREEMPT_LAZY=n`(+`DEBUG_PREEMPT=y`/`DEBUG_ATOMIC_SLEEP=y`) 재빌드.~~ → **★실행 완료·실패 확정 (2026-07-13)**: LAZY=n 커널(`5.15.199-rt91-rt-kv260`, Kconfig.preempt 1줄 패치로 프롬프트화하여 빌드)로 부팅 → BUG(sleeping) 250건+atomic 2건+underflow 1건. **LAZY는 무죄 — 진짜 원인 지점이 특정됨: 전 리포트 253건의 단일 원점 = `__radix_tree_preload`**. 이어진 1차 소스 전수 대조(2026-07-13)로 **근본 원인 100% 확정: Ubuntu jammy 커널의 `UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`(05fdd32398, NVIDIA 독점 모듈 빌드 호환 목적)** — 비-RT에선 무해하지만 RT 패치를 얹으면 fork/sysfs 등 idr preload 전 경로가 "선점금지 상태로 잠드는 락"이 되는 구조 결함. vanilla·linux-xlnx는 정상(local_lock), rt91 패치는 이 파일 안 건드림 — Ubuntu 트리만의 지뢰. **다음 수순 = ⓪.5 국소 픽스(3파일 원복: lib/radix-tree.c, include/linux/radix-tree.h, include/linux/idr.h — 검증된 vanilla 원본이 보드 `~/ros2_ws/kernel_configs/vanilla-5.15.199-radix-fix/`에 준비됨) → 재빌드 → 판정 253→0건. 실패 시 ④ fallback.** 상세: fix_plan ⓪ 결과 블록. 증거: `crash_logs/rt_kv260_lazyoff_debug_boot_20260713.log`. **판정(2026-07-12 정정)**: BUG 재현은 확률적(부팅별 0~5건 실측, 42h 무크래시 생존 사례도 있음)이므로 **실패 판정은 BUG 1건 재현 즉시, 성공 판정은 부팅 10회+DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP+부하 소크 전부 0건**으로. 성공하면 저비용 해결, 실패하면 아래 본 레시피로. ※ `CONFIG_MIGRATION`은 메모리 페이지용 옵션으로 무관(CMA가 강제로 켬, 순정도 y인데 정상) — 상세는 fix_plan ⓪.
   - ~~베이스: vanilla 5.15.209 + rt96~~ → **2026-07-10 밤 개정**: 보드 실측 결과 zocl/VCU/usb5744/ap1302가 전부 in-tree이고 오버레이 통로(OF_CONFIGFS)도 비-mainline 패치라 **vanilla 경로는 대규모 포팅 필요로 강등**. ~~새 우선순위(2026-07-10 밤) = ⓪ LAZY=n 실험 → ④ linux-xlnx 수동 머지 → vanilla+rt96(최후)~~ → **★최종 우선순위 (2026-07-13, 근본 원인 확정 후) = ⓪.5 국소 픽스(radix-tree 3파일 원복 — 기존 트리·config 그대로, ~30분 재빌드) → ④ linux-xlnx `xilinx-v2022.1`(5.15.19)+`patch-5.15.19-rt29` 수동 머지(fallback; Xilinx wiki 공인 절차, "공식 linux-xlnx-rt 레시피"는 존재하지 않음 확정) → vanilla+rt96(최후 — usb5744 미지원=RealSense 죽음, OF_CONFIGFS 부재 등 블로커 다수 확정)**. 비교표·근거·출처는 `rt_kernel_fix_plan.md` (b).
   - config: `PREEMPT_LAZY=n`; 검증 기간 `DEBUG_PREEMPT=y`, `DEBUG_ATOMIC_SLEEP=y`(+가능하면 `PROVE_LOCKING`), 부트 파라미터 `slub_debug=FZP`. (`SLAB_FREELIST_RANDOM/HARDENED`는 끄지 않아도 됨 — 정상 RT 커널에선 합법 경로이고, 켜두면 회계 이상의 조기 경보 역할. 07-12 정정)
   - zocl/dmaproxy/allegro 등 외부 모듈은 새 커널 headers로 재빌드
   - 검증: UART 시리얼 콘솔(다른 PC에 연결) 로그 캡처 + zocl 없이 소크 테스트 → 모듈 추가 순서로 격리 → 비전 풀 파이프라인 부하 테스트 → cyclictest
   - 빌드 절차·flash-kernel 함정(④flavor override, ⑤DTB 복사)은 §4 그대로 유효
4. **검증 통과 기준**: 부팅 로그에 `scheduling while atomic`/`BUG:`/`WARNING.*fpsimd` 0건 + 수 시간 부하 소크에서 무크래시 + cyclictest 기준 충족.

### 12-4. 증거 위치

- `/var/log/kern.log` — 부팅 BUG(4.6s) 5회 재현, oops 2건(라인 5381·12815), fpsimd 폭풍(13602~)
- journal boot ID: 크래시① `9ec1e653`, 크래시② `5be1f2e2` (`journalctl -b <ID>`)
- **추출본(보존용): `~/ros2_ws/crash_logs/` 4개 파일** — BUG 17건 전수, 크래시① oops tail, 크래시⓪/① kern.log 문맥, 크래시② fpsimd 폭풍 전체(2,383줄)
- config: `/boot/config-5.15.199-rt91-rt-kria` (⓪ 실험 커널: `config-5.15.199-rt91-rt-kv260`)
- **⓪ 실험 커널 부팅 로그(원인 확정 증거)**: `~/ros2_ws/crash_logs/rt_kv260_lazyoff_debug_boot_20260713.log` (journal 7,096줄; BUG 250+atomic 2+underflow 1, 전수 원점 `__radix_tree_preload`) + 사용자 UART 캡처(rt_config_by_kkw.txt, PC 보관)
- **근본 원인 커밋**: Ubuntu jammy `05fdd323982c` "UBUNTU: SAUCE: Revert \"radix-tree: Use local_lock for protection\"" — https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/jammy/commit/?id=05fdd323982cd09570c0eb80b22729f2bbf7adc7 (되돌려진 mainline 원 커밋: `cfa6705d89b6`, v5.8)
- **⓪.5 픽스용 vanilla 원본 3파일(검증 완료)**: `~/ros2_ws/kernel_configs/vanilla-5.15.199-radix-fix/` — radix-tree.c(1606줄/local_lock 4곳)·radix-tree.h·idr.h, gregkh stable 미러 v5.15.199에서 추출
- 진단 전 과정·웹 근거 링크: `rt_kernel_fix_plan.md` — **2026-07-12 crash_logs 전수 분석·config 해결 가능성 판정은 (a-2) 절** (경합 락 3종 분류, BUG→fpsimd 0.5ms 인과 연결, 부팅별 BUG/생사 무상관 표, DEBUG_PREEMPT=n 확인), **2026-07-13 근본 원인 확정·⓪.5 절차는 (b) ⓪ 결과 블록**
