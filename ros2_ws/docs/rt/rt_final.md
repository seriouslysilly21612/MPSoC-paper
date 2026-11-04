# Kria KV260 RT-PREEMPT 커널 구축 — 전 과정 및 두 커널 결함 규명·해결 보고서

> **문서 성격**: 학습·보고용 종합 정리. RT-PREEMPT 커널을 KV260에 적용한 전 과정, 그리고 그 과정에서 만난 두 개의 커널 결함(**radix-tree**, **zocl**)의 증상·근본원인·해결·검증을 처음부터 끝까지 다룬다.
> **작업 기간**: 2026-07-07 ~ 2026-07-15
> **대상 독자**: 커널/FPGA 경험이 많지 않아도 이해할 수 있도록 1차 원리부터 설명. 기술 용어는 English 유지.
> **정본 문서**: `rt_patch.md`(인수인계·함정), `rt_kernel_postmortem.md`(사건 서사), `rt_kernel_fix_plan.md`(진단 상세), `zocl_patches/`(zocl 픽스).

---

## 목차
1. 배경 — 왜 RT 커널이 필요한가
2. 시스템 스펙
3. RT 커널 빌드 과정 (전체 파이프라인)
4. 커널 config 상세 (스펙)
5. **문제 ① radix-tree 크래시** — RT가 만든 버그
6. **문제 ② zocl DPU 크래시** — RT가 드러낸 벤더 버그
7. 최종 결과 및 검증 (cyclictest)
8. 두 문제 비교 — 무엇을 배웠나
9. 부록 (용어집·증거 위치·참고문헌)

---

## 1. 배경 — 왜 RT 커널이 필요한가

이 프로젝트의 최종 목표는 KV260 보드로 협동로봇(Neuromeka Indy7)을 **EtherCAT 프로토콜, 1kHz 주기**로 제어하는 것이다. EtherCAT 모션 제어는 매 **1ms**마다 정해진 시각에 데이터 프레임을 보내야 하고, 이 타이밍의 흔들림(**jitter**)이 곧 제어 품질을 결정한다.

- **일반 리눅스(PREEMPT_VOLUNTARY)**: "대체로 빠르지만 가끔 수백 µs 늦는다." 평균은 좋아도 최악(worst-case)이 보장되지 않는다.
- **RT 리눅스(PREEMPT_RT)**: 커널 내부의 거의 모든 lock을 선점 가능(preemptible)하게 바꿔, **"항상 정해진 시간 안에" 반응**하도록 만든다. 최악 지연을 수십 µs 수준으로 낮춘다.

따라서 **RT 커널은 EtherCAT 착수의 선결조건**이었다. 로드맵은 (1) APU(Cortex-A53, 리눅스)에서 RT 커널 + IgH EtherCAT Master 구현 → (2) 이후 RPU(Cortex-R5, FreeRTOS)로 실시간 제어 이전이다. 본 문서는 (1)의 RT 커널 부분을 다룬다.

### PREEMPT_RT의 핵심 개념 (이해를 위한 최소 지식)
- **Preemption(선점)**: 실행 중인 작업을 더 급한 작업이 밀어내는 것. RT 커널은 선점 지점을 최대한 늘려 반응성을 높인다.
- **Atomic context(원자 상태)**: "여기서는 절대 다른 작업에 양보(sleep)하면 안 된다"는 구간. spinlock을 잡았거나 preemption을 끈 상태. **이 안에서 잠들면(sleep) 커널 규칙 위반**이다.
- **PREEMPT_RT가 하는 일**: 일반 커널의 `spinlock_t`(잠들지 않는 lock)를 대부분 `rt_mutex`(잠들 수 있는 lock)로 바꾼다. 이 변환이 반응성의 비결이지만, **동시에 이번 두 사건의 무대**가 된다 (아래 §5).

---

## 2. 시스템 스펙

| 구분 | 내용 |
|---|---|
| **보드** | Kria **KV260 revB** (SOM: SMK-K26 revA) |
| **SoC** | AMD/Xilinx **Zynq UltraScale+ MPSoC** — APU: ARM Cortex-**A53 4코어** / RPU: Cortex-R5F / PL: FPGA |
| **RAM / swap** | 4GB / 27G |
| **OS** | Ubuntu 22.04.5 (Kria 공식 이미지) + ROS2 Humble |
| **루트 파일시스템** | `/dev/mmcblk1p2` (**microSD** 234G — KV260에 eMMC는 없다) |
| **원래 커널 (stock)** | `linux-xilinx-zynqmp` **5.15.0-1070.74** (업스트림 베이스 **5.15.199**), PREEMPT_VOLUNTARY, **HZ=250** |
| **RT 패치** | kernel.org **`patch-5.15.197-rt91`** (베이스 5.15.199에 가장 근접, reject 0개) |
| **최종 RT 커널** | **`5.15.199-rt91-rt-kv260c`** (**현재 구동 = build #11**, 2026-07-20), PREEMPT_RT, **HZ=1000**. 아래 본문의 `#10`(2026-07-15)은 이 문서가 쓰인 시점의 빌드이고, **#11은 #10 config에 `CONFIG_USB_NET_AX88179_178A=m` 한 줄만 추가**한 것이라 RT 심볼이 동일하다 — 즉 **#10의 cyclictest 수치는 #11에도 그대로 유효**하다 |
| **부팅 구조** | **GRUB 없음.** U-Boot → `/boot/firmware/boot.scr.uimg` → `/boot/firmware/image.fit`(커널+initrd+DTB 한 덩어리). `image.fit`은 **flash-kernel**이 생성 |
| **가속기** | DPU `DPUCZDX8G_ISA1_B3136` (`kv260-smartcam` overlay), Vitis-AI 2.5.0 |
| **카메라** | RealSense D435i (USB, FW 5.17.0.10) |
| **빌드 환경** | x86_64 Ubuntu PC에서 **aarch64 크로스컴파일** (`bindeb-pkg`) |

### 왜 순정 kernel.org 소스가 아니라 Ubuntu-Xilinx 소스인가?
KV260의 핵심 장치들(zocl=DPU, usb5744=USB 허브(카메라 전제), ap1302=ISP, xlnx_vcu 등)의 드라이버가 **Xilinx 벤더 트리에만 in-tree**로 존재한다. 순정 kernel.org 5.15에는 이 드라이버들이 없어 카메라·DPU가 죽는다. 그래서 **Ubuntu-Xilinx 커널 소스에 RT 패치를 얹는** 방식을 택했다 — 그리고 바로 이 선택이 문제 ①의 원인이 된다(§5).

---

## 3. RT 커널 빌드 과정 (전체 파이프라인)

빌드는 x86 PC에서, 설치는 보드에서 이루어진다.

### 3-1. 커널 소스 취득 [PC]
Launchpad git 저장소는 504 오류가 잦아, **소스 패키지를 직접 다운로드**해서 추출:
```bash
BASE=https://launchpad.net/ubuntu/+archive/primary/+sourcefiles/linux-xilinx-zynqmp/5.15.0-1070.74
wget $BASE/linux-xilinx-zynqmp_5.15.0.orig.tar.gz
wget $BASE/linux-xilinx-zynqmp_5.15.0-1070.74.diff.gz
wget $BASE/linux-xilinx-zynqmp_5.15.0-1070.74.dsc
dpkg-source --no-check -x *.dsc linux-5.15-rt-kria
chmod -R +x scripts/ debian/rules debian/scripts/   # 함정①: 추출 시 실행권한 소실
```

### 3-2. RT 패치 적용 [PC]
```bash
xzcat patch-5.15.197-rt91.patch.xz | patch -p1
find . -name '*.rej'   # → 없음(reject 0 = 깨끗이 적용)
```

### 3-3. 커널 config [PC]
stock 커널의 config를 베이스로, RT용으로 수정 (핵심만):
```bash
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-   # ★ 반드시 먼저 (함정 B)
scripts/config --enable  PREEMPT_RT
scripts/config --disable PREEMPT_VOLUNTARY --disable PREEMPT
scripts/config --enable  HZ_1000  --disable HZ_250
scripts/config --disable KVM            # 함정: KVM 켜지면 PREEMPT_RT 선택 불가
scripts/config --disable CPU_IDLE --disable ACPI_PROCESSOR
scripts/config --disable CPU_FREQ       # 고정 주파수(지터 감소)
make olddefconfig
```
config 상세와 근거는 §4.

### 3-4. 빌드 [PC] (~15–30분)
```bash
make -j$(nproc) bindeb-pkg
# 산출물: linux-image-<ver>_arm64.deb (~70M), linux-headers-<ver>_arm64.deb (~8M)
```

### 3-5. 보드 설치 [보드]
KV260 부팅 구조(U-Boot + FIT)의 특수성 때문에 함정이 많다.
```bash
# ① DTB 선복사 (함정 ⑤ — 매 커스텀 커널마다 필수)
sudo mkdir -p /lib/firmware/<ver>/device-tree/xilinx
sudo cp /usr/lib/linux-image-<ver>/xilinx/*.dtb /lib/firmware/<ver>/device-tree/xilinx/
# ② 설치 (initrd 생성 + flash-kernel 자동 트리거)
sudo dpkg -i linux-image-<ver>_*.deb linux-headers-<ver>_*.deb
# ③ FIT 명시 재생성
sudo flash-kernel <ver>
# ④ 재부팅
sudo reboot
```

### 3-6. 빌드·설치에서 밟은 함정 (재현자를 위한 기록)
| # | 함정 | 대응 |
|---|---|---|
| A | `PREEMPT_LAZY`가 `def_bool`이라 menuconfig에 안 뜸 | `Kconfig.preempt` 1줄 패치로 프롬프트화 |
| B | `ARCH=arm64` 없이 menuconfig 저장 → `ARCH_ZYNQMP` 통째 삭제 → 부팅 불가 | 항상 `export ARCH=arm64` 먼저. **게이트: 플랫폼 5심볼=y 확인** |
| C | systemd-oomd가 빌드 도중 세션째 kill (RAM 부족+`DEBUG_INFO=y`) | swap 추가 + `DEBUG_INFO=n` + `-j` 낮춤 |
| ④ | 설치 후 flash-kernel이 순정 FIT을 만듦 (`Kernel-Flavors: xilinx-zynqmp` 필터가 우리 커널 무시) | `/etc/flash-kernel/db`에 `Machine: ZynqMP *KV260*` + `Kernel-Flavors: any` 추가 |
| ⑤ | FIT `.its` 템플릿이 DTB를 `/lib/firmware/<ver>/device-tree/xilinx/`에서 참조하나 bindeb-pkg는 `/usr/lib/...`에 설치 | **매 설치마다 DTB 선복사** |
| ⑥⑦ | flash-kernel은 "설치된 최신 버전"만 설치. 구버전 지정은 조용히 무시(exit 0) | 구버전 선택은 `flash-kernel --force <ver>` (--force가 첫 인자) |
| G | TSN/mv-camera staging 드라이버 `in_be32` 빌드 에러 | `XILINX_TSN=n`, `NET_VENDOR_S2I=n` (파이프라인 무관) |

> **참고 — 무해한 경고**: 설치 시 나오는 `Couldn't find DTB on the following paths`는 flash-kernel의 **generic 탐색기가 내는 노이즈**다. 실제 DTB는 Kria `.its` 템플릿이 별도로 FIT에 임베드하므로, **`dumpimage -l /boot/firmware/image.fit`로 FIT 실물을 확인**하면 kernel+ramdisk+fdt가 정상 포함돼 있다(이번에 default config `conf-zynqmp-smk-k26-revA` + 커널배너 kv260c #10 확인).

---

## 4. 커널 config 상세 (스펙)

최종 프로덕션 커널 `-rt-kv260c`의 주요 config. **stock 대비 바뀐 것과 그 이유**를 함께 적는다.

### RT 핵심
| 옵션 | 값 | 의미·이유 |
|---|---|---|
| `CONFIG_PREEMPT_RT` | **y** | RT 패치 활성화 (핵심) |
| `CONFIG_PREEMPT_LAZY` | **n** | arm64 RT 실험 옵션. 크래시 용의자로 껐고 무죄 판명(§5) |
| `CONFIG_HZ_1000` | **y** | 타이머 1000Hz (stock 250 → 1ms 분해능, EtherCAT 주기 정합) |
| `CONFIG_PREEMPT_RCU` | y | RT 하 선점 가능 RCU (표준) |
| `CONFIG_RCU_BOOST` | y (delay=500) | RCU reader 우선순위 부스트 (RT 기본) |
| `CONFIG_NO_HZ_FULL` | **y** | tickless full — EtherCAT 격리 코어에서 타이머 인터럽트 제거용 capability (cmdline 미적용 상태로 예약) |
| `CONFIG_RCU_NOCB_CPU` | **y** | 격리 코어에서 RCU 콜백 오프로드 capability (동상) |

### 플랫폼 안전 게이트 (5개 전부 y — 아니면 부팅 불가)
| `CONFIG_ARCH_ZYNQMP` / `ZYNQMP_FIRMWARE` / `PINCTRL_ZYNQMP` / `COMMON_CLK_ZYNQMP` / `FPGA_MGR_ZYNQMP_FPGA` | 전부 **y** |
|---|---|

### 메모리·안전
| 옵션 | 값 | 이유 |
|---|---|---|
| `CONFIG_CMA` | y (`cma=1000M`) | FPGA/DPU가 연속 메모리 필요 → **랩 가이드의 CMA 비활성화는 미적용** |
| `CONFIG_SLUB_DEBUG` | y | 부팅 시 자동활성 아님. `slub_debug=` 부팅인자로 발동 (문제 ② 진단에 사용) |
| `CONFIG_SLAB_FREELIST_HARDENED` | y | freelist 포인터 난독화 — **조용한 손상을 legible 크래시로 만드는 유일한 조기경보 (절대 끄지 않음)** |
| `CONFIG_SLAB_FREELIST_RANDOM` | y | 유지 |

### 전원·클럭 (지터 감소)
| `CONFIG_KVM` | **n** | arm64에서 KVM 켜지면 `ARCH_SUPPORTS_RT` 조건(`if !KVM`)에 걸려 **PREEMPT_RT 선택 자체가 불가** |
| `CONFIG_CPU_IDLE` / `CONFIG_ACPI_PROCESSOR` | **n** | ACPI_PROCESSOR가 CPU_IDLE을 select → 둘 다 꺼야 함 (ZynqMP는 DT 부팅) |
| `CONFIG_CPU_FREQ` | **n** | 주파수 고정 = 지터 감소 (동적 스케일링은 지연 스파이크 유발) |

### 검증용 DEBUG 검출기 (★ 검증 커널에서만 켜고, 프로덕션에선 끔)
| 옵션 | 검증 커널(kv260b) | 프로덕션(kv260c) | 역할 |
|---|---|---|---|
| `CONFIG_DEBUG_PREEMPT` | y | **n** | atomic-context 위반을 **범인 코드 주소까지** 찍음 |
| `CONFIG_DEBUG_ATOMIC_SLEEP` | y | **n** | "atomic 상태에서 sleep" 위반 포착 |
| `CONFIG_DEBUG_INFO` | y | **n** | 디버그 심볼 (빌드PC oomd 원인이라 프로덕션 off) |

> **핵심 방법론**: 이 DEBUG 검출기 2종이 문제 ①의 범인을 잡은 도구다. 다만 매 선점마다 런타임 체크를 넣어 **지연을 부풀리므로**, 진단이 끝난 프로덕션 커널에선 끈다. 이것이 검증 커널 `-rt-kv260b`(DEBUG on)와 프로덕션 `-rt-kv260c`(DEBUG off)를 구분하는 이유다.

### 식별
`CONFIG_LOCALVERSION="-rt-kv260c"` → `uname -r` = `5.15.199-rt91-rt-kv260c`, `/sys/kernel/realtime` = `1`.

---

## 5. 문제 ① — radix-tree 크래시 (RT가 만든 버그)

### 5-1. 증상
RT 커널로 부팅하면 겉보기엔 정상 동작했으나, **랜덤하게 hang**(전원 리셋 필요)이 발생했다. 총 3회의 커널 크래시:
| 시각(2026) | 사건 |
|---|---|
| 07-08 밤 | 부팅 중 2회 프리즈 + 작업 중 SLUB oops 1회 |
| 07-10 21:05 | 파이프라인+bag 녹화 중 `Unable to handle kernel paging request`(wild pointer) → hang |
| 07-10 21:24 | 재부팅 후 **거의 무부하** 상태에서 fpsimd WARNING 폭풍 → 크래시 → hang |

**결정적 단서**: 크래시가 "거의 놀고 있을 때"도 터졌다. 과부하가 원인이면 무부하 크래시를 설명할 수 없다 → **"부하가 아니라 내부 상태가 이미 오염됐고, 죽는 건 타이밍 문제"**로 조사 방향이 잡혔다.

### 5-2. 오진들 (정직한 기록)
사건이 오래 걸린 이유는 그럴듯한 가짜 용의자가 많아서였다. 전부 반증됨:
| 의심 | 반증 |
|---|---|
| 버전 미스매치(rt91=.197 vs .199) | 5.15.198/199에 관련 변경 전무(ChangeLog 확인), reject 0 |
| `CONFIG_PREEMPT_LAZY` | LAZY=n 재빌드에서도 재현 → 무죄 |
| fpsimd 드라이버 (크래시 현장) | fpsimd 폭풍 0.5ms 전에 이미 `scheduling while atomic` 선행 → fpsimd는 **피해자(카나리아)** |
| config 오류 | 어떤 config로도 못 고침 확인 |
| "Ubuntu에 RT 자체가 무리" | 방향은 맞았으나 충돌 지점은 스케줄러 전체가 아니라 **파일 하나** |

> **교훈 (여기서 처음 등장)**: **"현장 ≠ 원인".** 크래시가 난 코드(fpsimd, SLUB)는 오염의 피해자일 수 있다.

### 5-3. 진단 방법 — 검출기를 켜다
문제의 커널은 `DEBUG_PREEMPT=n`이라 위반이 **무음**으로 진행됐다. 그래서 **DEBUG_PREEMPT + DEBUG_ATOMIC_SLEEP를 켜고(+용의자 LAZY도 끄고) 재빌드**해 부팅했다("함정 커널", `-rt-kv260` rev-4).

결과: 크래시 없이 부팅됐지만 **위반 리포트 253건**이 쏟아졌고, **전수(全數)가 단 하나의 원점**을 가리켰다:
```
Preemption disabled at: __radix_tree_preload+0x28
```
fork든 sysfs든 cgroup이든, 겉보기 경로는 달라도 모두 `__radix_tree_preload` 한 곳에서 시작됐다. **이 순간 범인이 특정됐다.**

이어 **같은 파일을 5개 트리에서 직접 받아 대조**:
| 트리 | radix preload 보호 방식 |
|---|---|
| vanilla/stable 5.15.199 (kernel.org) | `local_lock` ✅ 정상 |
| linux-xlnx xilinx-v2022.1 (Xilinx) | `local_lock` ✅ 정상 |
| patch-5.15.197-rt91 (RT 패치) | 이 파일 **안 건드림** |
| Ubuntu jammy `linux` (제네릭) | `preempt_disable` ❌ 구식 |
| Ubuntu `linux-xilinx-zynqmp` (우리 베이스) | `preempt_disable` ❌ 구식 |

→ **Ubuntu 계열만** 옛날 방식. 범인 커밋:
**`UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`** (커밋 `05fdd323982c`, Seth Forshee, 2021-11).
커밋 사유: mainline의 local_lock 전환이 추가한 GPL 전용 export 심볼이 **NVIDIA 독점 드라이버 빌드를 깨서** 되돌렸고, "이건 RT 빌드용 변경이라 일반 커널엔 무해하다". 실제로 일반 커널엔 무해했다 — 하지만 **우리는 그 위에 RT를 얹었다.**

### 5-4. 근본 원인 — 왜 이 한 줄이 보드를 죽였나

커널은 새 PID나 새 장치 파일을 만들 때 메모리 조각을 미리 예약한다(radix/idr **preload**). 이 예약 더미를 "이 CPU 전용"으로 표시하는 방법이 둘:
- **옛날 방식 `preempt_disable()`** — "방해금지" 팻말. 걸린 동안 **절대 잠들면 안 됨**.
- **요즘 방식 `local_lock`** — 똑똑한 자물쇠. RT에선 "잠들어도 되는" 형태로 알아서 변신.

mainline은 v5.8(2020)에 요즘 방식으로 업그레이드. Ubuntu는 (NVIDIA 문제로) 이 부분만 옛날 방식으로 되돌렸다. **일반 커널에선 어차피 안 잠드니 무해.** 그러나 **PREEMPT_RT의 핵심은 "웬만한 lock을 다 잠들 수 있는 형태로 바꾸는 것"**이라:

1. 새 프로세스 생성 → radix preload → **"방해금지" 팻말(`preempt_disable`)을 건다**
2. 바로 다음 줄에서 `spinlock_t`를 잡는데
3. RT 커널이라 그 spinlock은 **`rt_mutex`(잠들 수 있는 lock)**
4. → **"방해금지 팻말을 건 채로 잠들었다"** = `BUG: sleeping function called from invalid context` / `BUG: scheduling while atomic`

이 위반이 per-CPU 상태(SLUB 메모리 장부, fpsimd 레지스터 관리)를 오염시키고, 오염된 장부를 나중에 읽는 순간 SLUB oops나 fpsimd 폭풍이 났다.

**기술 요약**: mainline `radix_tree_preloads`는 `local_lock_t`로 보호 → RT에서 `idr_preload()`가 락을 쥔 채 다른 sleeping lock을 잡아도 정상. Ubuntu revert 후엔 `preempt_disable()` → RT에서 진짜 atomic context 생성 → 뒤이은 `spinlock_t`(→sleeping `rt_mutex`)에서 **atomic-context sleep** → BUG. 영향 경로: `alloc_pid`(fork), `__kernfs_new_node`(sysfs), `cgroup_mkdir`, `crng`(난수) 등 커널의 가장 기본 동작들.

**왜 찾기 어려웠나**: (a) **확률적** — sleeping lock은 경합이 없으면 실제로 안 잠들고 통과. 부팅마다 위반 0~6건, 42시간 버틴 부팅도 존재. (b) **무음 오염** — 치명적 SLUB 크래시는 오히려 "부팅 중 BUG 0건" 부팅에서 발생. (c) **검출기 꺼짐** — DEBUG_PREEMPT=n이라 원점 미기록.

### 5-5. 해결 — "revert의 revert"
Ubuntu가 되돌린 것을 다시 되돌린다. 영향받은 **정확히 3개 파일**을 vanilla 5.15.199 원본(local_lock)으로 교체:
| 파일 | 되돌린 내용 |
|---|---|
| `lib/radix-tree.c` | `preempt_disable()` → `local_lock(&radix_tree_preloads.lock)`, `EXPORT_PER_CPU_SYMBOL_GPL` 부활 |
| `include/linux/radix-tree.h` | `struct radix_tree_preload`에 `local_lock_t lock` 부활 |
| `include/linux/idr.h` | `idr_preload_end()`가 `preempt_enable` → `local_unlock` |

NVIDIA 독점 드라이버를 안 쓰는 Kria에선 GPL export 부활이 무해. **config로는 못 고치지만(소스 로직), 전체 재조립도 불필요** — 딱 3파일 교체 + ~30분 재빌드.

### 5-6. 검증 — 253 → 0
- **바이너리 지문**: `System.map`에 `__ksymtab_radix_tree_preloads` + `D radix_tree_preloads`(local_lock 판의 지문. Ubuntu revert 판은 static이라 없음) → **소스가 진짜로 컴파일돼 들어감**을 바이너리 수준 확인.
- **런타임**: DEBUG 검출기 **완전 무장** 상태에서 부트 저널 전수 스캔:

  | 패턴 | rev-4(픽스 전) | rev-5(픽스 후) |
  |---|---|---|
  | `BUG:`/`scheduling while atomic`/`sleeping function`/`fpsimd`/`Oops` | 253+ | **0** |

- **소크 테스트**(07-13): fork/sysfs(버그 경로) + cpu/vm 부하를 DEBUG 무장 상태로 → **누적 radix 위반 0건**, PASS.

→ **가장 민감한 검출기를 다 켜고도 0건** = 결함이 실제로 사라짐.

---

## 6. 문제 ② — zocl DPU 크래시 (RT가 드러낸 벤더 버그)

radix 문제 해결 후, DPU 비전 파이프라인을 처음 가동하자 **별개의** 크래시가 나타났다.

### 6-1. 증상
2026-07-14, RT 커널에서 full 비전 파이프라인(카메라 + DPU 추론 + 3D)을 처음 가동 → **약 30초 만에 커널 Oops → 시스템 프리즈 → 하드 재부팅**.
- **trace#1 (경고, 무해)**: `BUG: sleeping function... RCU nest depth:1`, `zocl_read_sect ← zocl_xclbin_read_axlf`. DPU xclbin 로드 시 `rcu_read_lock` 안에서 잠드는 vmalloc. 별개의 저심각 이슈.
- **trace#2 (치명)**: `Unable to handle kernel paging request at 1b9b2a514fa690f9` / `Internal error: Oops PREEMPT_RT SMP`. `pc: ___slab_alloc+0x518` ← `__kmalloc` ← `kds_alloc_command[zocl]` ← `zocl_execbuf_ioctl`.

### 6-2. 레지스터 지문 분석 (어떻게 정체를 좁혔나)
Oops의 레지스터에서 크래시 유형을 확정:
- `pc = ___slab_alloc+0x518` = **`get_freepointer`** 지점 (SLUB 할당자가 "다음 빈 객체" 포인터를 읽는 곳)
- fault 주소 = `x28 + 0x80`, 그리고 `s->offset = 0x80` → **kmalloc-256 캐시의 hardened mid-object freepointer**
- **결정적 증거**: `swab64(0x1b9b2a514fa690f9) == x0(0xf990a64f512a9b1b)` **정확히 일치**. 이는 `freelist_ptr()` 하드닝 디코드(`ptr ^ s->random ^ swab(ptr_addr)`)의 중간값 → x28은 **이미 손상된** freelist 포인터.

**결론**: 손상된 **SLUB freelist next-pointer 역참조** 크래시. `kds_alloc_command`는 손상된 freelist를 밟은 **피해자**이지 corruptor가 아니다. (또 "현장 ≠ 원인".)

### 6-3. 진단 — slub_debug로 corruptor 생포
지문만으론 "누가 손상시켰나"를 모른다. 그래서 **`slub_debug=FZPU,kmalloc-256`**(F=consistency, Z=redzone, P=poison, U=user-tracking)를 켜고 부팅. 이 옵션은 kmalloc-256 캐시에 감시 패턴을 심어, 손상이 생기면 **크래시 전에** "누가 할당하고 누가 해제했는지" 스택과 함께 리포트한다.

3겹 안전장치(netconsole + `journalctl -kf` tee + `panic_on_oops=1`) 후 파이프라인 재현 → **`BUG kmalloc-256: Poison overwritten` 포획(시스템 생존)**. 리포트 해독:
- **Allocated in** `kds_alloc_command[zocl]` ← execbuf (사건 Oops의 그 객체)
- **Freed in** `kds_free_command ← xrt_cu_intr_thread[zocl]` (CU 인터럽트 스레드)
- 해제 후 **offset 128**에 8바이트 `03 d5 ce bd 25 01 00 00` = **`ktime_get()` ns 값**(부팅 후 ~1261.6s, 사건 시각과 일치). offset 128 = kmalloc-256 freelist pointer 자리.

### 6-4. 근본 원인 — submit-후-타임스탬프 use-after-free
소스(`drivers/gpu/drm/zocl/common/kds_core.c`)에서 정확한 줄을 특정:
```c
xrt_cu_submit(cu_mgmt->xcus[cu_idx], xcmd);   /* ① 커맨드를 CU 스레드에 넘김 */
set_xcmd_timestamp(xcmd, KDS_QUEUED);          /* ② 그 다음에 타임스탬프 기록 */
```
- **①을 하는 순간부터 CU 인터럽트 스레드가 이 커맨드(xcmd)를 완료 처리하고 `kfree`할 수 있다.**
- 처리가 빠르면 ②가 실행되기 전에 xcmd가 이미 **free**된 상태 → ②는 **해제된 메모리의 offset 128(=`timestamp[KDS_QUEUED]`)에 ktime을 쓰는 use-after-free**.
- 그 offset 128이 하필 프로덕션 SLUB(kmalloc-256, 하드닝)의 **freelist next-pointer 자리** → 다음 할당 때 깨진 포인터를 역참조 → `___slab_alloc` Oops.

`kds_cu_dispatch`/`kds_scu_dispatch`/ERT submit **3곳** 동일 패턴. alloc/free 모두 같은 CPU(3)에서 → **같은 코어에서 CU 스레드가 제출 스레드를 선점**하는 레이스. **PREEMPT_RT의 full preemption이 이 레이스 창을 벌린다** — 일반 커널은 이 구간에서 선점이 잘 안 일어나 창이 거의 닫혀 있어 잠복. **이것이 "RT가 드러낸 벤더 버그"인 이유.** (upstream XRT master에도 동일 버그 잔존 — 2026-07 확인.)

### 6-5. 해결 — 순서 교체
스탬프를 submit **앞**으로 이동. 의미는 완전히 동일("큐에 넣은 시각" 기록)하고 순서만 바뀐다:
```c
set_xcmd_timestamp(xcmd, KDS_QUEUED);          /* 먼저 스탬프 (아직 우리 소유) */
xrt_cu_submit(cu_mgmt->xcus[cu_idx], xcmd);   /* 그 다음 넘김 */
```
CU 스레드가 커맨드를 받는 시점엔 이미 스탬프가 찍혀 있으니, 아무리 빨리 free해도 UAF가 성립하지 않는다. 3곳 모두 교체. 패치 도구: `~/ros2_ws/zocl_patches/apply_zocl_uaf_fix.py`.

### 6-6. 검증
zocl은 in-tree 모듈이라 커널 트리에서 패치 후 재빌드(rev-6 = kv260c #10). **패치가 반영됐는지 바이너리 확인**: zocl.ko `srcversion`이 `4971DA73…`(패치 전) → `0754F2D6…`(패치 후)로 변경 = 소스 변경이 컴파일에 반영됨.

재현 검증 2건:
| 조건 | 재현량 | 결과 |
|---|---|---|
| **계측**(slub_debug 무장) | churn 5×20s + sustained 180s ≈ **330s**, zocl 156클라이언트 | **Poison 0 · Oops 0 · 생존** |
| **프로덕션**(계측 제거) | churn 4×20 + sustained 120s ≈ 200s+, zocl 130클라이언트 | **Oops 0** |

지난 버그 커널은 churn 1회차 **46초 만에** Poison 발생 → 원본 사건은 Oops→프리즈였다. 패치 후엔 그 임계(236s)를 훨씬 넘겨도 0건. **픽스 확정.**

---

## 7. 최종 결과 및 검증 (cyclictest)

최종 프로덕션 커널 `-rt-kv260c`(#10) = radix 픽스 + zocl 픽스 + DEBUG off. RT 지연을 cyclictest로 측정(방법: `-S` 코어별 고정 스레드, idle baseline + stress-ng/hackbench 부하):

| | kv260b (DEBUG on, 검증 커널) | **kv260c (DEBUG off, 프로덕션)** |
|---|---|---|
| IDLE Max | 121~143µs | 134µs (Avg 11~13) |
| **LOADED Max** | **189~282µs** | **142µs** (Avg 14~19) |
| 부하 중 커널 위반 | 0 | **0** |

- **핵심 개선 = LOADED worst-case 282 → 142µs (거의 절반).** DEBUG 검출기가 매 선점마다 넣던 오버헤드가 사라진 효과. 코어별 편차도 타이트해짐(kv260b 105~282 → kv260c 127~142).
- **부하 중 커널 위반 0** = radix 픽스 + zocl 픽스가 stress-ng+hackbench churn에도 견딤.
- 현재 142µs가 통상 EtherCAT 목표 `<100µs`보다 높은 것은 **CPU 격리 없이 4코어를 cyclictest와 stress-ng가 공유**한 조건 탓이다. EtherCAT 단계의 **3+1 격리**(전용 코어 + `nohz_full`/`rcu_nocbs`)에서 전용 코어는 일반 부하가 없어 `<100µs`에 도달할 것으로 예상된다.

**결론**: RT 커널의 두 결함(radix, zocl)이 모두 해결됐고, 프로덕션 커널이 부하 중 위반 0으로 안정 동작한다. **EtherCAT(IgH Master) 착수의 선결조건이 전부 해제**됐다. 남은 RT 항목은 EtherCAT 통합 단계의 3+1 격리 코어 실측(실제 DPU+EtherCAT 부하 동시)뿐이다.

---

## 8. 두 문제 비교 — 무엇을 배웠나

| 항목 | **① radix-tree** | **② zocl** |
|---|---|---|
| 나타난 층위 | 커널 코어 (radix/idr preload) | 벤더 드라이버 (DPU/XRT KDS) |
| 버그의 본질 | **RT가 만든 버그** (Ubuntu revert가 RT와 충돌) | **잠복 벤더 버그를 RT가 노출** |
| 증상 | 부팅 초기부터 랜덤 hang | DPU 가동 ~30초 후 크래시 |
| 손상 유형 | atomic-context sleep → per-CPU 상태 오염 | use-after-free → SLUB freelist 오염 |
| 진단 도구 | `DEBUG_PREEMPT` / `DEBUG_ATOMIC_SLEEP` | `slub_debug=FZPU` |
| 진단 원리 | 위반의 **원점 코드 주소**를 찍음 | 손상의 **alloc/free 스택**을 찍음 |
| 수정 위치 | 커널 소스 3파일 (config 불가) | 벤더 드라이버 1파일 3곳 |
| 수정 방식 | revert의 revert (local_lock 복원) | submit↔stamp 순서 교체 |

### 공통 교훈 (재사용 가능)
1. **"현장 ≠ 원인".** 두 사건 모두 크래시가 난 코드(fpsimd/SLUB, `kds_alloc_command`)는 **피해자**였다. 카나리아를 잡지 말고 가스를 찾아라.
2. **검출기 기반 진단.** 조용한 손상을 legible한 리포트로 바꾸는 도구(DEBUG_PREEMPT, slub_debug)가 결정적이었다. 둘 다 "범인의 위치"를 그 자리에서 찍어준다.
3. **작고 정확한 수정.** 두 문제 다 전체 재조립·재설계가 아니라 **몇 줄/몇 파일**의 targeted fix로 끝났다. 값싼 배제 실험(⓪ 실험, E2 계측)이 진짜 원인을 드러냈다.
4. **바이너리 수준 검증.** 소스를 바꾼 뒤 `System.map`의 export 심볼(문제①), 모듈 `srcversion`(문제②)으로 "정말 그 코드가 들어갔는지"를 확인했다.
5. **RT는 잠복 버그를 드러낸다.** PREEMPT_RT는 선점 창을 넓혀, 일반 커널에서 조용히 지나가던 레이스(zocl)와 규칙 위반(radix)을 표면화시킨다. RT 브링업은 "새 버그를 만든다"기보다 "숨은 버그를 조기 노출"하는 성격이 크다.

---

## 9. 부록

### 9-1. 용어집 (Glossary)
| 용어 | 설명 |
|---|---|
| **PREEMPT_RT** | 리눅스 실시간 패치. 커널 lock 대부분을 선점 가능하게 만들어 최악 지연을 보장 |
| **Preemption / atomic context** | 선점 = 급한 작업이 현재 작업을 밀어냄 / atomic context = 잠들면 안 되는 구간 |
| **spinlock_t → rt_mutex** | RT가 잠들지 않는 lock을 잠들 수 있는 lock으로 변환 (반응성의 비결이자 두 사건의 무대) |
| **local_lock vs preempt_disable** | per-CPU 보호 방식. local_lock은 RT 호환, preempt_disable은 RT에서 진짜 atomic 생성 |
| **radix_tree / idr preload** | PID·장치파일 생성 시 메모리를 미리 예약하는 커널 메커니즘 |
| **SLUB / freelist** | 커널 메모리 할당자 / "다음 빈 객체"를 가리키는 포인터 사슬 |
| **use-after-free (UAF)** | 해제된 메모리를 계속 사용하는 버그 |
| **freelist hardening** | freelist 포인터를 난독화해 손상을 조기 탐지 (`ptr ^ random ^ swab(addr)`) |
| **cyclictest** | RT 지연(latency) 측정 표준 도구. `-S`=코어별 고정 스레드 |
| **KDS / xcmd** | zocl의 Kernel Driver Scheduler / DPU 커맨드 구조체(`kds_command`) |
| **flash-kernel / FIT** | KV260 부팅 이미지(`image.fit`) 생성 도구 / 커널+initrd+DTB 묶음 포맷 |

### 9-2. 증거·산출물 위치
| 항목 | 경로 |
|---|---|
| radix ⓪실험 로그(253건) | `~/ros2_ws/crash_logs/rt_kv260_lazyoff_debug_boot_20260713.log` |
| radix vanilla 픽스 3파일 | `~/ros2_ws/kernel_configs/vanilla-5.15.199-radix-fix/` |
| zocl 원본 크래시 Oops | `~/ros2_ws/crash_logs/zocl_crash_boot-1_20260714.log` |
| zocl Poison 리포트(corruptor 지목) | `~/ros2_ws/crash_logs/e2_poison_report_20260714-2336.log` |
| zocl 프로덕션 검증 로그 | `~/ros2_ws/crash_logs/prod_verify_kv260c.log` |
| cyclictest 최종 로그 | `~/ros2_ws/crash_logs/cyclic_20260715-013556.log` |
| zocl 패치 도구·설명 | `~/ros2_ws/zocl_patches/` (apply_zocl_uaf_fix.py + README.md) |
| 검증 하네스 | `~/ros2_ws/rt_verify/{churn,sustained}.sh` |
| 커널 config 실측 전체표 | `rt_patch.md §4-4-2` |

### 9-3. 참고문헌
- radix 범인 커밋 (Ubuntu SAUCE): `05fdd323982c` — https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/jammy/commit/?id=05fdd323982cd09570c0eb80b22729f2bbf7adc7
- 되돌린 mainline 원본: `cfa6705d89b6` (v5.8, radix-tree local_lock 도입)
- RT 패치 아카이브: https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/older/
- 커널 소스(Launchpad): https://launchpad.net/ubuntu/+source/linux-xilinx-zynqmp
- zocl(XRT) 소스: `drivers/gpu/drm/zocl/common/kds_core.c` (Xilinx XRT 2022.1)
- 정본 상세 문서: `rt_kernel_postmortem.md`(서사), `rt_patch.md`(인수인계), `rt_kernel_fix_plan.md`(진단)

---

*작성 근거: rt_kernel_postmortem.md, rt_patch.md, rt_kernel_fix_plan.md, 메모리(kria-rt-preempt-project, zocl-dpu-rt-kernel-crash), 2026-07-14~15 실측 로그. 최종 갱신 2026-07-15.*
