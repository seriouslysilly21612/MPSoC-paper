# Kria KV260 RT 커널 크래시 — 종합 보고서 (Postmortem)

> **이 문서 하나로 사건 전체(증상 → 오진 → 진단 → 근본 원인 → 해결 → 검증)를 처음부터 끝까지 읽을 수 있게** 정리한 문서.
> 기간: 2026-07-08 ~ 2026-07-13. 작업 문서(정정 이력 포함): `rt_patch.md`(인수인계), `rt_kernel_fix_plan.md`(진단 상세·링크).
> 대상 독자: FPGA/커널 경험이 적어도 이해할 수 있게 1차 원리부터 설명. 기술 용어는 English 유지.

---

## 0. 한 줄 요약 (TL;DR)

RT(실시간) 커널이 부팅 초기부터 내부 상태가 오염되어 보드가 랜덤하게 죽었다. 오랫동안 "RT 패치를 잘못 적용했나", "config를 잘못 잡았나", "버전이 안 맞나"를 의심했지만 **전부 아니었다.** 진짜 원인은 **Ubuntu가 (NVIDIA 그래픽 드라이버 빌드 호환을 위해) 커널 소스의 `radix-tree` 파일 하나를 옛날 방식으로 되돌려 놓은 것**이었고, 그 옛날 방식이 **RT 패치와 만나면 "끼어들기 금지 상태에서 잠드는 락을 잡는" 구조 위반**을 일으켰다. 해결은 **그 파일 3개를 리눅스 본가(mainline) 버전으로 되돌리고 재빌드** — 부팅 시 위반 건수가 **어제 253건 → 오늘 0건**으로 완전히 사라졌다.

| 항목 | 내용 |
|---|---|
| 증상 | 보드 랜덤 hang(전원 리셋 필요), 원격 SSH 끊김, 커널 크래시 3회 |
| 잘못 지목했던 것 | ① 버전 미스매치(rt91=5.15.197를 5.15.199에) ② CONFIG_PREEMPT_LAZY ③ fpsimd 드라이버 ④ config 설정 ⑤ "Ubuntu에 RT 패치 자체가 무리" — **전부 오진** |
| 진짜 원인 | Ubuntu SAUCE 커밋 `05fdd323982c` — mainline의 `radix-tree: Use local_lock for protection`(v5.8) 를 되돌림. 비-RT엔 무해, RT엔 치명 |
| 해결 | 3파일(`lib/radix-tree.c`, `include/linux/radix-tree.h`, `include/linux/idr.h`)을 vanilla 5.15.199 원본으로 원복 → 재빌드 |
| 검증 | DEBUG 검출기 완전 무장 상태에서 부팅 위반 **253 → 0건**, DPU/카메라 정상 |
| 남은 것 | 소크 ✅통과(07-13, 누적 위반 0) → DEBUG 끈 프로덕션 rev-6 재빌드(PC) → EtherCAT 착수 |

---

## 1. 우리가 만든 것 (사건의 무대)

- **하드웨어**: Kria KV260 (Zynq UltraScale+, ARM Cortex-A53 4코어 + PL/FPGA), RealSense D435i 카메라, DPU(비전 가속기, `kv260-smartcam` 오버레이).
- **소프트웨어**: Ubuntu 22.04 + ROS2 Humble. 비전 파이프라인(카메라 → DPU 검출 → 3D 위치)이 이미 동작 중.
- **왜 RT 커널이 필요했나**: 나중에 로봇(Neuromeka Indy7)을 EtherCAT으로 1kHz 주기로 제어하려면, 일반 리눅스로는 부족하다. 일반 리눅스는 "대체로 빠르지만 가끔 늦는" 반면, **RT(PREEMPT_RT) 커널은 "항상 정해진 시간 안에" 반응**을 보장한다. 그래서 EtherCAT 착수의 선결조건으로 RT 커널을 만들고 있었다.
- **어떻게 만들었나**: Ubuntu가 배포하는 Kria용 커널 소스(`linux-xilinx-zynqmp` 5.15.0-1070.74, 베이스는 커널 5.15.199)에 kernel.org의 RT 패치(`patch-5.15.197-rt91`)를 얹어서 크로스컴파일.

---

## 2. 증상 — 실제로 무슨 일이 일어났나

RT 커널로 부팅하면 겉보기엔 잘 돌았다. 비전 파이프라인도 측정도 정상. 그런데:

| 시각(2026) | 사건 |
|---|---|
| 07-08 밤 | 부팅 중 2회 "프리즈"(멈춤), 그리고 작업 중 1회 커널 크래시(SLUB oops) |
| 07-10 21:05 | 비전 파이프라인 + bag 녹화 중 **커널 크래시 ①** — `Unable to handle kernel paging request`(잘못된 포인터), 보드 hang, 원격 끊김 |
| 07-10 21:24 | 전원 리셋 후 재부팅 → **거의 무부하 상태**에서 fpsimd WARNING 폭풍 → **커널 크래시 ②** → 또 hang |

**핵심 관찰**: 크래시 ②가 "거의 놀고 있을 때" 터졌다는 게 결정적 단서였다. 처음엔 "과부하 때문"이라고 생각했지만, 부하가 원인이면 무부하일 때 죽는 걸 설명할 수 없었다. → 부하가 아니라 **내부 상태가 이미 오염되어 있었고, 죽는 건 타이밍 문제**라는 방향으로 조사가 틀어졌다.

---

## 3. 오진(誤診)의 기록 — 무엇을 의심했고 왜 아니었나

이 사건이 오래 걸린 이유는 **그럴듯한 가짜 용의자가 많았기 때문**이다. 정직하게 남긴다.

| # | 의심했던 원인 | 왜 아니었나 (반증) |
|---|---|---|
| ① | **버전 미스매치** — rt91은 5.15.197용 패치인데 5.15.199에 적용 | 5.15.198/199 stable에 fpsimd/스케줄러 변경 전무(ChangeLog 전수 확인). rt91은 5.15.199에 fuzz 없이 깨끗이 적용됨. rt92~rt96에도 관련 수정 없음. → 무해 |
| ② | **CONFIG_PREEMPT_LAZY** (arm64 RT 전용 실험 옵션) | ⓪ 실험(LAZY=n 재빌드)에서도 똑같이 재현됨 → **무죄 확정** |
| ③ | **fpsimd 드라이버** (크래시 ②의 현장) | fpsimd WARN 폭풍 0.5ms 전에 이미 `scheduling while atomic` BUG가 선행. fpsimd는 원인이 아니라 **"카나리아"**(먼저 죽어 위험을 알리는 새) — 오염의 피해자일 뿐 |
| ④ | **config 설정 오류** | ⓪ 실험으로 어떤 config로도 못 고침을 확인 → 무죄 |
| ⑤ | **"Ubuntu 베이스에 RT 패치 자체가 무리"** — 아예 linux-xlnx로 갈아타야(④안, Xilinx wiki 절차) | 방향은 맞았으나(조합 문제), 실제 충돌 지점은 스케줄러 전체가 아니라 **파일 하나**로 판명 → 전체 재조립 불필요 |
| ⑥ | **fork 회계 결함** — BUG의 `preempt_count=0x2`가 `FORK_PREEMPT_COUNT`와 같아서 fork 마무리 회계가 안 풀린 것으로 추정 | 0x2의 실체는 preload(+1) + SLUB per-CPU pin(+1)의 중첩. fork에 몰린 건 fork의 `alloc_pid`가 문제의 preload를 쓰기 때문일 뿐 → 가설 기각 |

**교훈**: 크래시의 "현장"(fpsimd, zocl, SLUB)과 "원인"은 다르다. 현장만 쫓으면 계속 카나리아를 잡게 된다.

---

## 4. 진단 방법 — 어떻게 진짜 범인을 잡았나

두 단계의 결정적 수단이 있었다.

### 4-1. DEBUG 검출기를 켠 "함정 커널" (⓪ 실험)

문제의 커널은 `CONFIG_DEBUG_PREEMPT=n`, `CONFIG_DEBUG_ATOMIC_SLEEP=n`이었다. 이 두 옵션은 **"규칙 위반을 그 자리에서, 범인 코드 주소까지 찍어서" 잡아주는 검출기**인데 꺼져 있었다. 그래서:
- 위반이 **무음**으로 진행되다가(오염만 쌓임), 한참 뒤 엉뚱한 곳에서 죽었다 → 원인 추적 불가.

그래서 **이 두 검출기를 켜고 + 용의자 LAZY도 끄고** 재빌드해 부팅했다(커널 이름 `-rt-kv260`, rev-4).

결과: 크래시 없이 부팅됐지만 **위반 리포트 253건**이 쏟아졌고, **전수(全數)가 단 하나의 원점**을 가리켰다:

```
Preemption disabled at: __radix_tree_preload+0x28
```

다른 원점 0건. 즉 fork든 sysfs든 cgroup이든, 겉보기 경로는 달라도 **모두 `__radix_tree_preload` 한 곳에서 시작**됐다. 이 순간 범인이 특정됐다.

### 4-2. 1차 소스 전수 대조 (5개 트리 비교)

`radix-tree.c`가 문제라면, 이게 우리 트리만 이상한 건지 확인해야 했다. 같은 파일을 5개 트리에서 직접 받아 비교:

| 트리 | radix preload 보호 방식 |
|---|---|
| vanilla/stable 5.15.199 (kernel.org) | `local_lock` ✅ 정상 |
| linux-xlnx `xilinx-v2022.1` (Xilinx 벤더) | `local_lock` ✅ 정상 |
| `patch-5.15.197-rt91` (RT 패치, 11,384줄 전수) | 이 파일 **안 건드림** |
| Ubuntu jammy `linux` (제네릭) | `preempt_disable` ❌ 구식 |
| Ubuntu `linux-xilinx-zynqmp` (우리 베이스) | `preempt_disable` ❌ 구식 |

→ **Ubuntu 계열만** 옛날 방식. 범인 커밋을 Launchpad에서 찾아냄:

**`UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`** (커밋 `05fdd323982c`, Seth Forshee, 2021-11)

커밋 본문에 사유가 그대로 적혀 있었다(요약): mainline의 local_lock 전환이 추가한 GPL 전용 export 심볼이 **NVIDIA 독점 드라이버 빌드를 깨뜨려서** 되돌렸고, *"이건 RT 빌드용 변경이라 (일반 커널에선) 무해하다"*. 실제로 일반 커널엔 무해했다 — 하지만 우리는 그 위에 RT를 얹었다.

---

## 5. 근본 원인 — 왜 이 한 줄이 보드를 죽였나

### 5-1. 쉬운 설명 (비유)

커널은 새 프로세스 번호나 새 장치 파일을 만들 때, 미리 **자재(메모리 조각)를 조금 예약**해둔다(radix/idr **preload**). 이 예약 더미는 "지금 이 CPU 코어 전용"이라고 표시해야 하는데, 방법이 둘이다:

- **옛날 방식 — "방해금지" 팻말** (`preempt_disable`): 팻말이 걸린 동안은 **절대 잠들면(대기하면) 안 된다**. 어기면 커널 규칙 위반.
- **요즘 방식 — 똑똑한 자물쇠** (`local_lock`): 평소엔 팻말과 똑같지만, RT에선 "잠들어도 되는" 형태로 알아서 변신한다.

리눅스 본가는 2020년(v5.8) 요즘 방식으로 업그레이드했다. Ubuntu는 (NVIDIA 문제로) **이 부분만 옛날 팻말 방식으로 되돌렸다.** 일반 커널에선 팻말이든 자물쇠든 어차피 안 잠드니 무해.

그런데 **RT 패치의 핵심이 "웬만한 자물쇠를 다 잠들 수 있는 형태로 바꾸는 것"**이다. 그래서 이런 순서가 벌어진다:

1. 새 프로세스를 만들려고 창고 예약 → **"방해금지" 팻말을 건다**
2. 바로 다음 줄에서 (fork, sysfs 파일 생성 등 기본 동작 대부분이 이 순서) 자물쇠 하나를 잡는데
3. RT 커널이라 그 자물쇠는 **"잠들 수 있는" 종류**다
4. → **"방해금지 팻말을 건 채로 잠들었다"** = 규칙 위반 → `BUG: sleeping function called from invalid context`, `BUG: scheduling while atomic`

이 위반이 per-CPU 상태(SLUB 메모리 장부, fpsimd 레지스터 관리)를 오염시키고, 오염된 장부를 나중에 누군가 읽는 순간 크래시(SLUB oops)나 경고 폭풍(fpsimd)이 났다.

### 5-2. 기술적 설명

- mainline v5.8 이후: `radix_tree_preloads`는 `local_lock_t`로 보호. PREEMPT_RT에서 `local_lock`은 **RT 호환**(per-CPU sleeping lock, 선점 유지하되 sleeping context를 금지하지 않음)이라, `idr_preload()`가 락을 쥔 채 다른 sleeping lock을 잡아도 정상.
- Ubuntu 리버트 후: `preempt_disable()`로 보호. PREEMPT_RT에서 `preempt_disable()`은 **진짜 atomic context**를 만든다. 그 뒤 코드가 `spinlock_t`(RT가 sleeping `rt_mutex`로 변환)를 잡으면 → **"atomic context에서 sleep"** → BUG.
- 영향 범위: `idr_preload()` / `radix_tree_preload()`를 쓰는 **모든** 경로 — `alloc_pid`(fork), `__kernfs_new_node`(sysfs), `cgroup_mkdir`, `inotify_add_watch`, `btf_alloc_id`(BTF), `crng`(난수) 등 커널의 가장 기본적인 동작들.

### 5-3. 왜 이렇게 찾기 어려웠나 (확률적 재현·무음 오염)

- **확률적**: 3번의 sleeping lock은 **비어 있으면 실제로 안 잠들고** 통과한다. 다른 코어가 하필 같은 락을 원할 때(경합)만 진짜로 대기가 걸려 BUG. 그래서 부팅마다 위반 0~6건으로 들쭉날쭉했고, 42시간을 멀쩡히 버틴 부팅도 있었다.
- **무음 오염**: 반대편(선점 카운트 과소) 경로는 아무 로그 없이 per-CPU 장부만 오염시켰다. 치명적 SLUB 크래시 2건은 오히려 "부팅 중 BUG 0건"이었던 부팅에서 났다 — BUG 유무와 생사가 무관했다.
- **검출기 꺼짐**: `DEBUG_PREEMPT=n`이라 원점(범인 코드 주소)이 안 찍혔다. 이걸 켠 게 4-1의 전환점.

`preempt_count=0x2`의 실체: radix preload(+1) + SLUB per-CPU pin(+1)의 **중첩**. 처음엔 `FORK_PREEMPT_COUNT`(fork 회계)로 오해했으나, 소스 대조로 preload가 진짜 원인임이 드러나면서 기각.

---

## 6. 해결 — "revert의 revert"

Ubuntu가 되돌린 것을, 우리가 다시 되돌린다. 영향받은 **정확히 3개 파일**을 vanilla 5.15.199 원본(local_lock 방식)으로 교체:

| 파일 | 되돌린 내용 |
|---|---|
| `lib/radix-tree.c` | `preempt_disable()` → `local_lock(&radix_tree_preloads.lock)`, per-CPU 정의에 `.lock = INIT_LOCAL_LOCK(lock)` + `EXPORT_PER_CPU_SYMBOL_GPL` 부활 |
| `include/linux/radix-tree.h` | `struct radix_tree_preload`에 `local_lock_t lock` 부활, `radix_tree_preload_end()`가 `local_unlock` |
| `include/linux/idr.h` | `idr_preload_end()`가 `preempt_enable` → `local_unlock` |

NVIDIA 독점 드라이버를 안 쓰는 Kria에선 GPL export 부활이 **무해**. 다른 파일(예: `mm/swap.c`)은 Ubuntu도 local_lock을 원본대로 유지하고 있어서, revert가 radix-tree에만 국한됨을 확인 → 3파일만 손대면 충분.

- 검증된 vanilla 원본 3파일 위치: `~/ros2_ws/kernel_configs/vanilla-5.15.199-radix-fix/`
- 나머지 소스/config는 전부 그대로. **config로는 못 고치지만, 전체 재조립(④ linux-xlnx)도 불필요** — 딱 이 3파일 교체 + ~30분 재빌드.

---

## 7. 검증 — 253 → 0

### 7-1. 설치 전 (바이너리 지문)

빌드된 커널의 `System.map`에서:
```
__ksymtab_radix_tree_preloads   ← export 마커 (local_lock vanilla판의 지문)
D radix_tree_preloads           ← 전역(대문자 D) per-CPU 심볼
```
Ubuntu 리버트판은 이걸 `static`(export 없음)으로 만들므로, 이 심볼들의 존재 = **우리 vanilla 파일이 진짜로 컴파일되어 들어갔다**는 바이너리 수준 증거. (부팅 시 U-Boot도 커널 해시 sha1 검증 통과.)

### 7-2. 부팅 후 (런타임)

- `uname`: `#5 SMP PREEMPT_RT` — RT 커널 정상.
- **검출기 완전 무장**(`DEBUG_PREEMPT=y`, `DEBUG_ATOMIC_SLEEP=y`, `PREEMPT_RT=y`, LAZY off) 상태에서 부트 저널 전수 스캔:

  | 패턴 | 어제(rev-4) | 오늘(rev-5) |
  |---|---|---|
  | `BUG:` / `scheduling while atomic` / `sleeping function` / `Preemption disabled` / `fpsimd` / `Unable to handle` / `Internal error` | 253+ | **0** |

- 잔여 WARNING 4건은 전부 `OF: overlay: memory leak`(smartcam FPGA 오버레이 관련) — 순정 커널에도 뜨는 무해 경고, 우리 문제와 무관.
- `/proc/kallsyms`에 픽스 export 심볼 라이브 확인.
- DPU 스택 정상: `/dev/dri/card0`, `zocl`+`dmaproxy` 로드. (예전에 oops 나던 zocl 경로가 이제 정상.)

→ **가장 민감한 검출기를 다 켜고도 0건** = 운으로 안 걸린 게 아니라 결함이 실제로 사라짐.

---

## 8. 교훈 (재사용 가능한)

1. **Ubuntu 5.15 커널 베이스 + PREEMPT_RT 패치 조합은 radix-tree revert 지뢰를 밟는다.** 같은 조합을 시도하는 누구에게나 재현될 문제. (Launchpad 버그 리포트 후보.)
2. **RT 커널 진단의 핵심 도구 = `DEBUG_PREEMPT` + `DEBUG_ATOMIC_SLEEP`.** 위반의 원점을 그 자리에서 찍어준다. RT 브링업 시엔 검증 기간 동안 반드시 켤 것(프로덕션에선 지연 오버헤드 때문에 끔).
3. **"현장 ≠ 원인".** 크래시가 난 코드(fpsimd, zocl, SLUB)는 오염의 피해자일 수 있다. 카나리아를 잡지 말고 가스를 찾을 것.
4. **바이너리 수준까지 검증할 것.** 소스를 바꿨어도 `System.map`의 심볼로 "정말 그 코드가 들어갔는지"를 확인(export 심볼이 좋은 지문).
5. **가장 값싼 배제 실험부터.** 전체 재조립(수일) 전에, config/소스 1줄 실험(⓪, ~30분)으로 용의자를 지우면 진짜 원인이 드러난다.

---

## 9. 부록 — 빌드 과정에서 밟은 함정들

RT 커널을 다시 빌드하는 과정 자체에서도 여러 함정이 있었다. 재빌드할 사람을 위해 기록.

| # | 함정 | 대응 |
|---|---|---|
| A | **`PREEMPT_LAZY`는 `def_bool`** — menuconfig에 체크박스가 안 뜬다(arm64+RT면 자동 y) | `kernel/Kconfig.preempt`의 해당 줄을 `bool "..."`로 1줄 소스 패치해야 선택 가능 |
| B | **x86 모드 menuconfig 사고** — `ARCH=arm64` 없이 menuconfig를 저장하면 `ARCH_ZYNQMP` 등 arm64 심볼이 통째로 삭제됨 → 부팅 불가 deb | 항상 `export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-` 먼저. 게이트: `grep -cE "^CONFIG_(ARCH_ZYNQMP\|ZYNQMP_FIRMWARE\|PINCTRL_ZYNQMP\|COMMON_CLK_ZYNQMP\|FPGA_MGR_ZYNQMP_FPGA)=y" .config` = **5** |
| C | **systemd-oomd가 빌드를 죽임** — RAM 15G + 스왑 0 + `-j20` + `DEBUG_INFO=y`면 메모리 고갈로 tmux 세션째 킬 | 스왑 생성 + `-j10`으로 낮춤 + tmux 안에서 빌드(증분 재개 가능) |
| D | **DTB 선복사 필요** — flash-kernel의 Kria `.its` 템플릿이 `/lib/firmware/<ver>/device-tree/xilinx/`를 하드코딩 참조. bindeb-pkg는 다른 경로에 설치 | `dpkg -i` **전에** 그 경로로 DTB 복사. (`Couldn't find DTB` 경고는 generic 탐색기 노이즈 — `.its`는 별개로 임베드하므로 FIT 실물을 `dumpimage -l`로 확인) |
| E | **flash-kernel `--force`** — 설치된 "최신"이 아닌 커널을 지정하면 조용히 무시(exit 0). 구버전 선택은 `flash-kernel --force <ver>`(--force가 첫 인자) | 순정 복귀 시 `sudo flash-kernel --force 5.15.0-1070-xilinx-zynqmp` |
| F | **버전 정렬** — `-rt-kv260` > `-rt-kria` > `5.15.0-1070`(문자열 정렬). auto-trigger가 최신을 고름 | 새 RT 커널 이름을 정렬상 위로(kv260) → `--force` 없이 자동 선택. 순정 복귀 전 RT 패키지 제거하면 지뢰 소멸 |
| G | **TSN/mv-camera staging 드라이버 빌드 에러** — `in_be32`/`out_be32` implicit declaration(`-Werror`)로 빌드 중단 | `CONFIG_XILINX_TSN=n`, `CONFIG_NET_VENDOR_S2I=n` (우리 파이프라인과 무관한 장치) |
| H | **빌드 후 config 재검증** — menuconfig 저장 시점이 아니라 **빌드 끝난 deb 안의 config**가 진실(`syncconfig`가 `def_bool` 등을 조용히 재계산) | `dpkg-deb -x <deb> /tmp/chk && grep ... /tmp/chk/boot/config-*`로 설치 전 최종 확인 |

---

## 10. 남은 작업 (RT 인증 완료까지)

현재 보드는 **`-rt-kv260b`(build #8, DEBUG 검증 커널)** 구동 중 — rev-5를 대체한 개선판(CPU_FREQ drift 교정 + NO_HZ_FULL/RCU_NOCB 추가). 부트 위반 0 + cyclictest baseline(avg 14~20µs/Max load 189µs, 부하 중 위반 0) 확인. DEBUG 지연 오버헤드가 있어 프로덕션용은 아니다.

1. **[✅ 통과] 소크 테스트 (2026-07-13)** — fork/clone/sysfs(버그 경로) + cpu/vm 부하를 DEBUG 검출기 무장 상태로. **누적 결과 radix 위반 0건**:
   - v1: load 156 극한 부하 9.6분 → 0 위반 (보드는 test 부하 과다로 OOM/livelock 사망했으나, 죽은 부팅 로그에 위반 0 = 픽스는 버팀)
   - v3: load ~50 지속 14분(840s) → 0 위반, PASS. (840s 조기종료는 `--vm 2 --vm-bytes 35%`≈2.66GB 메모리 압박으로 stress-ng 워커 사망→abort, 커널 무관. 보드 회복.)
   - 부팅 이후 커널 위반 총계 0. **결론: 픽스 무결성 확정.**
   - 소크 하네스 `~/ros2_ws/soak_rt.sh` (교훈: `kill $SPID`는 --fork 자식을 고아로 남겨 좀비 폭주 → `trap`+`pkill -9 stress-ng`로 프로세스 그룹 전체 reap 필수. `--vm 35%`는 과함).
2. **[다음] DEBUG 옵션 끄고 rev-6 재빌드** = 최종 프로덕션 RT 커널 (DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP는 지연 오버헤드라 프로덕션 제외; `SLAB_FREELIST_RANDOM` 등은 유지). config만 바꿔 재빌드.
3. rev-6 검증 → **EtherCAT(IgH Master) 착수** — 그동안 막혀 있던 선결조건 해제. ⚠️ 단 **DPU 비전 + EtherCAT을 단일 RT 커널에 "통합"하는 마일스톤은 §12 zocl 크래시 해결까지 게이트**. EtherCAT 브링업 자체(DPU 미가동)는 RT 커널에서 안정(zocl 안 띄우면 무크래시)이라 병행 착수 가능.

**별개 RT 이슈 (radix crash와 무관, 2026-07-13 발견)**: DPU xclbin 로드 경로(`zocl_read_axlf_ioctl→zocl_read_sect→vmalloc`)가 `rcu_read_lock` 안에서 잠드는 할당 → DEBUG_ATOMIC_SLEEP이 `BUG: sleeping function... mm.h:229, RCU nest depth:1` 포착. **radix 크래시와 완전 별개**(Xilinx vendor 드라이버 소스 결함, config로 제어 불가). **1회성**(DPU worker 기동당 1번)·저심각(RCU reader는 RT서 preemptible, DPU 로드 정상 ret 0). rev-5 '253→0' 검증은 **부팅 저널 스캔**이라 이 로드 경로 미탐 → 이번에 파이프라인 돌려 처음 표면화. 순정 커널에도 같은 코드 있으나 DEBUG_ATOMIC_SLEEP=n이라 안 보였을 뿐. audit-clean RT 원하면 zocl 소스 패치 필요(EtherCAT/제어 경로와 무관하므로 착수 안 막음). **⚠️ 2026-07-14 추가: 같은 zocl 드라이버가 이번엔 치명적 커널 Oops(trace#2, 시스템 프리즈)도 냈다 → 아래 §12.** 위 sleeping-in-atomic은 그 크래시(trace#1이라 부름)이며, 검증 결과 §12 크래시(trace#2)의 **원인이 아니라 동일 zocl 락킹 붕괴의 독립 증상**임(공통원인, 인과사슬 아님).

**cyclictest RT 지연 baseline (2026-07-13, kv260b)**: avg 14~20µs, Max **idle 143 / load 189µs**, 부하(cpu4+hackbench+io) 중 위반 0. 100µs 목표 초과지만 **보수적**(DEBUG 오버헤드 + 격리 미적용) — 실 EtherCAT값은 rev-6(DEBUG off) + 3+1 격리 코어(nohz_full/rcu_nocbs)에서 측정. 구 GUI측정 698µs 스파이크 소멸. 하네스 `cyclic_rt.sh`.

## 11. 증거·산출물 위치

- **커널 config 전체 표**(rev-5 기준 IKCONFIG 실측, 2026-07-13): `rt_patch.md` §4-4-2. 현재 구동 커널 `-rt-kv260b`는 이 표에서 **CPU_FREQ(n)·NO_HZ_FULL(y)·RCU_NOCB_CPU(y) 3개만 차이**. 프로덕션 rev-6은 여기서 `DEBUG_PREEMPT`/`DEBUG_ATOMIC_SLEEP` 2줄만 끄면 됨.

- 크래시 원본 로그: `~/ros2_ws/crash_logs/` (BUG 17건 전수, SLUB oops, fpsimd 폭풍 전체)
- ⓪ 실험 부팅 로그(원인 확정 증거, 253건): `~/ros2_ws/crash_logs/rt_kv260_lazyoff_debug_boot_20260713.log`
- 해결 부팅 로그(0건): 사용자 `rt5_bootlog.txt`
- vanilla 픽스 3파일: `~/ros2_ws/kernel_configs/vanilla-5.15.199-radix-fix/`
- config 백업/대조: `~/ros2_ws/kernel_configs/` (stock ikconfig, rt91 백업, diff)
- 범인 커밋: https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/jammy/commit/?id=05fdd323982cd09570c0eb80b22729f2bbf7adc7 (되돌린 mainline 원본: `cfa6705d89b6`, v5.8)
- 상세 진단·전략: `rt_kernel_fix_plan.md` / 인수인계: `rt_patch.md`

---

## 12. ★ zocl DPU 커널 크래시 (2026-07-14 발견, RT 세션 인수인계 정본)

> **이 섹션은 RT 담당 세션이 바로 이어받도록 자족적으로 쓴다.** radix 크래시(§5, 해결됨)와 **완전히 별개**의 새 이슈다.

### 12-1. 무슨 일이 일어났나
2026-07-14, kv260b(#8) RT 커널에서 **full 비전 파이프라인**(`run_gate6_perf.sh 180s`: realsense + vitis_ai_detector + **vitis_ai_worker_yolo(DPU)** + pick 3노드)을 처음 가동. 약 **30초 만에 커널 Oops → 시스템 프리즈 → SSH 끊김 → 사용자 하드 재부팅**. 이것이 "측정 중 SSH 끊겨 재부팅한" 사건의 원인.

### 12-2. 타임라인 (journalctl -b -1, 영속 저널로 복원)
- **15:03:46 trace#1** (경고): `BUG: sleeping function called from invalid context ... RCU nest depth:1`, `zocl_read_sect ← zocl_xclbin_read_axlf`. = §10의 알려진 sleeping-in-atomic(xclbin 로드). xclbin 성공(ret 0), taint W만 남김.
- **15:04:15 trace#2** (치명): `Unable to handle kernel paging request at 1b9b2a514fa690f9` / `Internal error: Oops [#1] PREEMPT_RT SMP`. CPU3, PID 147289(python3=DPU worker). `pc: ___slab_alloc+0x518` ← `__kmalloc` ← `kds_alloc_command[zocl]` ← `zocl_command_ioctl` ← `zocl_execbuf_ioctl[zocl]` ← `drm_ioctl`.
- 15:04:26~32: zocl client 정리/재기동 churn, bitstream ref 2→3→4 누수(짝 unlock 없음).
- **15:07:00 systemd-oomd Watchdog timeout (3min)** → SIGABRT. **15:08:31 SIGKILL** + 저널 정지 = 시스템 완전 프리즈.

### 12-3. 근본원인 (적대적 검증 3렌즈 통과, 레지스터 지문 일치)
**확정(높은 확신)**: 손상된 **SLUB freelist next-pointer 역참조** 크래시. `pc=___slab_alloc+0x518`=`get_freepointer` 지점, fault=`x28+0x80` → `s->offset=0x80` → **kmalloc-256 캐시, hardened mid-object freepointer**. 결정적 증거: **`swab64(0x1b9b2a514fa690f9) == x0(0xf990a64f512a9b1b)` 정확히 일치** = `freelist_ptr()` 하드닝 디코드(`ptr ^ s->random ^ swab(ptr_addr)`)의 중간값 → x28은 **이미 손상된** free-pointer. ESR `0x96000004`=DABT/level-0 translation fault/READ = 쓰레기 주소 READ. **`kds_alloc_command`는 손상된 freelist를 밟은 피해자, corruptor 아님**. 손상 클래스는 write-after-free/overflow 쪽으로 기움(double-free 아님).

**배제됨**: 메모리부족(OOM/page-alloc-fail 0), 하드웨어(MCE/thermal/EDAC 0), radix 회귀(다른 경로, 트레이스에 radix/xarray/local_lock 프레임 0), kv260b config(NO_HZ_FULL/RCU_NOCB는 부팅인자 없어 dormant — `nohz_full`/`rcu_nocbs` 미지정 확인), DEBUG 아티팩트(SLUB_DEBUG_ON=n, KASAN=n; 하드닝은 조용한 손상을 legible 크래시로 만든 것뿐). systemd-oomd 워치독은 Oops 후 커널 wedge의 **2차 증상**(oom-kill 0건, oomd 자신의 하트비트 실패).

**결함 소유자 = zocl** (~88%): 크래시가 오직 zocl 경로에서만·DPU 첫 실사용 부팅에서만 발생, taint C. 단 `SLAB_MERGE_DEFAULT=y`라 kmalloc-256은 병합 캐시(DRM/dma-buf/TSN 공유) → corruptor가 반드시 zocl이라는 것은 트레이스만으론 미증명(slub_debug 필요). **수정은 RT 인프라가 아니라 zocl에** (radix와 성격 다름 — 그건 RT가 만든 버그, 이건 zocl 버그를 RT가 노출).

### 12-4. 미확정 — 메커니즘 3갈래 (판별 필요)
1. **RT가 넓힌 교차-CPU 선점 레이스** (~55-60%): stock=TREE_RCU 비선점 vs RT=PREEMPT_RCU 선점 → RT에서 zocl KDS 레이스 창이 넓어짐. 맞으면 순정은 훨씬 안전.
2. **SMMU 부재 DMA 스크리블** (~25-30%, RT 무관): **양쪽 커널 다 `CONFIG_ARM_SMMU` 미설정** → DPU(PL 마스터)가 DDR을 무보호 직접 DMA. zocl BO 수명 버그(in-flight 중 free)면 fabric이 해제된 슬랩을 덮어씀. 맞으면 **순정도 동일 빈도로 터짐**.
3. non-race double-free (~15%). / 순수 DRAM 비트플립(ECC off라 완전배제 불가, ~3-5%).

### 12-5. 판별실험 (우선순위, 대부분 리빌드 불필요)
- **E1 [최우선, 리빌드無]** 순정 5.15.0-1070(이미 설치, zocl.ko 존재)로 동일 워크로드 장시간 루프. 빨리 터지면→잠복(DMA/double-free), 버티면→RT-노출 레이스. *(이번 세션이 비전 작업을 순정에서 하므로 자동으로 E1 데이터가 쌓임.)*
- **E2 [리빌드無]** RT를 `slub_debug=FZPU`(또는 `,kmalloc-256`)로 부팅 → 손상 op 경계에서 `Object already free/Freepointer corrupt/Redzone overwritten` + alloc·free 스택 출력 → **corruptor 함수 지목**. `CONFIG_SLUB_DEBUG=y` 확인됨. 가성비 최고.
- **E6 [리빌드無]** DPU 파이프라인 단일 CPU 고정(`taskset -c 3` 또는 `maxcpus=1`). 소멸→교차-CPU 레이스(RT), 지속→double-free/DMA.
- **E3 [리빌드無]** `kfence.sample_interval=100` 부팅인자(현재 0=off)로 프로덕션 타이밍 UAF/OOB 캐치.
- **E4 [리빌드]** RT·stock-config 양쪽 `CONFIG_KASAN=y` — 결정적. KASAN 깨끗한데도 크래시=DMA 가설 강한 증거.
- **E5 [리빌드]** `CONFIG_ARM_SMMU=y` + DPU stream-id → 슬랩 스크리블 대신 SMMU translation fault 뜨면 DMA 확정.
- **trace#1 독립 수정**: `zocl_read_sect`의 `GFP_KERNEL` 할당을 `rcu_read_lock` 밖으로 hoist (zocl.ko만 리빌드) — 별개 결함이라 따로 고칠 수 있음.

### 12-6. 안전장치 (재현 전 필수)
지난 사건은 프리즈로 저널이 15:08:31에 멈춰 이후 로그 유실. 재현 시: (a) 데스크톱 **netconsole** + 별도 SSH `dmesg -w | tee`, (b) reserved-mem **ramoops/pstore**(현재 미구성), (c) 테스트 부팅에 **`panic_on_oops=1`**(wedge 대신 즉시 리부트+상태 확정). **freelist 하드닝 절대 끄지 말 것** — 유일한 조기경보.

### 12-7. 증거 위치
- 원본: `journalctl -b -1` (사건 부팅 = 2026-07-13 23:04 ~ 2026-07-14 15:08:31, 영속 저널). Oops 전문은 15:04:15 전후. 보존본: `~/ros2_ws/crash_logs/zocl_crash_boot-1_20260714.log`.
- 메모리: `~/.claude/projects/-home-ubuntu/memory/zocl-dpu-rt-kernel-crash.md`.
- 크로스: boot -1만 Oops(1건), -2~-5는 0 (zocl 미가동). boot -1이 kv260b 첫 부팅 + DPU 첫 실사용.

### 12-8. ★ 종결 — E2로 corruptor 지목 + 소스 국소화 + 픽스 (2026-07-14 심야)
**E2 실행**: kv260b를 `slub_debug=FZPU,kmalloc-256 kfence.sample_interval=100`으로 부팅, netconsole+`journalctl -kf` tee+`panic_on_oops=1` 3겹 안전장치 후 DPU 파이프라인 재현(버스트 10s→180s→churn 4회). **23:36:46 `BUG kmalloc-256: Poison overwritten` 포획 — 시스템 생존, 크래시 없이 리포트 확보** (`crash_logs/e2_poison_report_20260714-2336.log`).

**리포트 해독 (모든 조각 맞음)**:
- Allocated in `kds_alloc_command[zocl]` ← execbuf (사건 Oops의 그 객체)
- **Freed in `kds_free_command ← xrt_cu_intr_thread[zocl]`** (CU 인터럽트 스레드)
- 해제 후 **offset 0x80(128)**에 8바이트 `03 d5 ce bd 25 01 00 00` = **`ktime_get()` ns 값**(부팅 후 ~1261.6s, 사건 시각과 일치). offset 128 = 프로덕션 kmalloc-256 freelist pointer 자리 = 사건 Oops의 x2=0x80과 동일.

**소스 국소화** (`drivers/gpu/drm/zocl/common/kds_core.c`, 모듈 내장 `__FILE__`로 경로 확정):
```c
xrt_cu_submit(cu_mgmt->xcus[cu_idx], xcmd);   /* 이 순간부터 CU 스레드가 완료→해제 가능 */
set_xcmd_timestamp(xcmd, KDS_QUEUED);          /* 해제 후 실행되면 UAF: timestamp[1]=offset 128 */
```
`kds_cu_dispatch`/`kds_scu_dispatch`/ERT submit 3곳 동일 패턴. struct kds_command 레이아웃 계산으로 `timestamp[KDS_QUEUED]`=offset 128 확정. alloc/free 모두 cpu=3 → **같은 코어에서 CU 스레드가 제출 스레드를 선점**하는 레이스. RT(full preemption)가 창을 벌림 = **메커니즘 1 확정** (SMMU DMA 가설 사망 — DMA는 완벽한 ktime 값을 쓰지 않음). **upstream XRT master에도 동일 버그 잔존(2026-07 확인)** → XRT GitHub 리포트 가치.

**픽스**: 스탬프를 submit 앞으로 이동(한 줄 순서 교체 ×3, 의미 동일). 패치 도구·절차 = `~/ros2_ws/zocl_patches/`(README.md + apply_zocl_uaf_fix.py). 적용처 = PC 커널 트리(zocl은 in-tree, `linux-image` 패키지 소속) → rev-6에 합류(A) 또는 kv260b 모듈만 교체(B). 검증 = 패치 모듈로 E2 계측 churn 재실행 → Poison 0건.

**부수 확인**: trace#1(`zocl_read_sect` vmalloc-in-RCU)은 첫 xclbin 로드마다 결정적 재현("already loaded" 경로는 스킵), 원인 = `attr_rwlock`(RT에선 내부 rcu_read_lock) 아래 vmalloc. 크래시 무관, 수정은 Xilinx 자신이 쓰는 unlock/relock 패턴(zocl_xclbin.c의 zocl_create_aie 전례) — 후순위. netconsole은 RT에서 자체 sleeping 경고(macb lock)를 내지만 기능함(무해 노이즈).

### 12-9. ★★ 종결 — 픽스 빌드·설치·검증 완료 (2026-07-15)
패치를 rev-6 빌드에 합류 → **`-rt-kv260c` #10**(kds_core.c 3곳 순서교체 + DEBUG_PREEMPT/ATOMIC_SLEEP off). deb 검증: config DEBUG off, **zocl.ko srcversion `4971DA73`→`0754F2D6` 변경 = 패치 반영 확인**. DTB 81개 선복사(함정⑤; flash-kernel의 "Couldn't find DTB"는 무해한 generic 경로 소음 — FIT은 `dumpimage -l`로 kernel+ramdisk+fdt(default `conf-zynqmp-smk-k26-revA`)·커널배너 kv260c #10 확인됨). 설치·부팅 성공(realtime=1, 부트 위반 0).

**검증 재현 (2건):**
- **계측 조건**(slub_debug=FZPU,kmalloc-256 무장 — poison/redzone 속성파일 확인): churn 5×20s + sustained 180s ≈ **330s**(원래 실패임계 236s 초과), zocl **156클라이언트** → **Poison 0건·Oops 0건·시스템 생존.**
- **프로덕션 조건**(계측 제거, cmdline 클린): churn 4×20 + sustained 120s ≈ 200s+, zocl 130클라이언트 → **Oops 0건.** 순수 프로덕션 타이밍에서도 견고(픽스가 allocator-무관 소스 수정이라 당연).

**cyclictest(kv260c, DEBUG off)**: idle Max 134µs(Avg 11~13) / **load Max 142µs(Avg 14~19) / 부하 중 커널 위반 0**. kv260b(DEBUG on) load Max 282→142µs로 **절반**·코어편차 타이트(127~142). >100µs는 격리無 4코어 공유 조건 탓 → `<100µs`는 EtherCAT 3+1 격리 코어에서.

**결론: radix(§12 이전) + zocl(§12-8) 두 커널 결함 모두 해결. RT 트랙 종결, EtherCAT 선결조건 전부 해제.** 검증 로그: `crash_logs/{e2_poison_report_20260714-2336, prod_verify_kv260c, cyclic_20260715-013556}.log`. 검증 하네스(재부팅 생존): `~/ros2_ws/rt_verify/{churn,sustained}.sh`. **함정: `pkill -f "pick_place_vitis_ai"`를 그 문자열 든 명령에 인라인하면 셸 자신을 kill → churn/정리는 반드시 스크립트 파일 분리.**
