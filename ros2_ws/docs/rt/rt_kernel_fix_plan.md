# RT 커널 크래시 원인 규명 및 수정 계획 (2026-07-10 ~ 07-13)

> 📌 **깨끗한 종합 서사(사건 전체를 처음부터)는 `rt_kernel_postmortem.md`.** 이 문서는 진단 상세·근거 링크·전략 비교의 작업 기록(정정 이력 포함).

> 작성 배경: 2026-07-10 밤 보드 커널 크래시 2회(21:05 SLUB oops, 21:24 fpsimd WARN 폭풍→oops)로 hang.
> 웹 리서치 + 보드 `/var/log/kern.log` 교차 검증으로 원인 규명. 전 과정 기록은 `rt_patch.md` 참조.
> ~~결론: ①번 전략(vanilla 5.15.209 + rt96 재빌드)~~ → 07-10 밤 개정(⓪→④) → **★최종 확정 (2026-07-13): ⓪ 실험(LAZY=n+DEBUG 커널 부팅)으로 근본 원인 100% 규명 — Ubuntu jammy 커널의 `UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`(NVIDIA 모듈 호환용 revert)이 RT 패치와 만나면 구조 결함이 됨. 해법 = ⓪.5 국소 픽스(3파일 원복 후 재빌드), 실패 시에만 ④. 상세: (b)의 ⓪ 결과 블록.**
> 보드는 stock 커널(`5.15.0-1070-xilinx-zynqmp`)로 운용 중 (비전/fps 작업은 RT 불필요; ⓪ 실험 직후 2026-07-13 순정 복귀).

## 요약: fpsimd는 원인이 아니라 "카나리아"였다

fpsimd WARN 폭풍이 시작되기 0.5ms 전, 같은 태스크(PID 2653)에서 **`BUG: scheduling while atomic: 5/2653/0x00000002`가 먼저 발생**(clone→alloc_pid→rt_spin_lock 경합). 이 "원자 상태로 스케줄→CPU 간 마이그레이션"이 per-CPU fpsimd 보호(busy 플래그)를 깨뜨려 WARN 318/173/204가 CPU 2→0→1→3으로 튀다가 `task_fpsimd_load` oops로 이어짐.

더 중요하게, **이 `scheduling while atomic: …/0x00000002` BUG는 샘플링한 5회 부팅 전부에서 4~5초 시점에 재현**됨(udevd/kworker, 경로: 디바이스 probe → sysfs `__kernfs_new_node` → `idr_alloc_cyclic(GFP_ATOMIC)` → SLUB `new_slab` → `SLAB_FREELIST_RANDOM`의 `get_random_u32` → `crng_make_state`의 sleeping rt lock 경합). 즉 **RT 커널은 매 부팅 시작부터 상태가 오염되고 있었고, 그동안 안 죽은 것은 운**이었다.

하드 크래시(와일드 포인터 oops)도 사실 두 번(7/8, 7/10) 있었고 둘 다 **SLUB freelist 포인터 오염**(`get_freepointer`/`__kmalloc`, zocl `kds_alloc_command` 경로) — 원자 상태 스케줄/마이그레이션이 per-CPU slab 상태를 오염시킨 결과와 정합. 07-08의 "부팅 프리즈" 2회도 같은 원인일 가능성 높음(mali는 부차적이었을 수 있음).

> **★이 요약의 최종 해답 (2026-07-13)**: 위의 "원자 상태" 정체가 규명됨 — Ubuntu 커널이 NVIDIA 모듈 호환을 위해 되돌려놓은 radix-tree preload의 구식 `preempt_disable()`이었다. `idr_preload()`(fork·sysfs·cgroup 등 전방위 사용)가 이걸 쥔 채 RT의 잠드는 락을 잡는 구조. 재현이 확률적이었던 이유(락이 경합할 때만 실제 스케줄 발생), 무음 오염(비-DEBUG 커널에선 비경합 시 조용히 통과), crng·pidmap 등 트리거 락이 다양했던 이유까지 전부 이 하나로 설명. 상세 근거·수정 절차는 (b)의 ⓪ 결과 블록.

## (a) 원인 진단 (검증된 사실)

「알려진 rt 버그」도 「.197 vs .199 단순 버전 미스매치」도 아님:

1. **fpsimd 코드 자체는 정상 적용됨** — vanilla 5.15.199 fpsimd.c의 WARN 라인(173/195/306)에 rt91 훅(+9/+12줄)을 적용하면 정확히 173/204/318이 되어 크래시 로그와 일치. RT용 `preempt_disable()` 보호(2021 Valentin Schneider 보고 v5.13-rt1 동일 증상의 수정판, rt91 큐 `0138-arm64-sve-Make-kernel-FPU-protection-RT-friendly.patch`)는 커널에 들어 있음.
2. **stable 5.15.198/199에는 fpsimd/SVE/NEON/arm64 코어 변경 전무**(ChangeLog 전문 grep). rt91은 vanilla 5.15.199에 fuzz 없이 깨끗이 적용됨(dry-run 확인) — "197 패치를 199에 적용" 자체는 무해했음.
3. **rt92~rt96에도 관련 수정 없음** — rt91↔rt96(5.15.209) diff는 net/ipv6/route.c 하나 차이. "나중 rt에서 고쳐진 버그"가 아님.
4. ~~**진짜 문제의 서명**: … 0x2는 `FORK_PREEMPT_COUNT`와 같은 값 → fork/컨텍스트 스위치 마무리 회계가 안 풀린 신호(추정) …~~ → **기각 (2026-07-13)**: 0x2의 실체 = radix preload(+1) + SLUB per-CPU pin(+1) 중첩. fork에 몰린 이유는 fork의 `alloc_pid`가 `idr_preload()` 사용처라서일 뿐. ⓪ 결과 블록 참조.
5. **빌드 조합이 미검증 조합**: Ubuntu `5.15.0-1070.74-xilinx-zynqmp` + vanilla용 `patch-5.15.197-rt91` + `CONFIG_PREEMPT_LAZY=y`. → **확정판 (2026-07-13)**: "미검증 조합" 진단은 맞았으나 충돌 지점은 스케줄러 SAUCE도 LAZY도 아닌 **`UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`** 하나였음(LAZY는 ⓪ 실험으로 무죄 입증). ⓪ 결과 블록 참조.
6. 부수 요인: `CONFIG_SLAB_FREELIST_RANDOM=y`가 `new_slab`마다 crng 락 경로를 만들어 부팅 시 확정 발화점. `DEBUG_ATOMIC_SLEEP/DEBUG_PREEMPT=n`이라 누출이 조용히 진행. zocl/dmaproxy/allegro(staging)는 oops 현장이지만 preempt 누출 BUG는 zocl 로드 전에도 발생 → 1차 원인 아님.

**인과 사슬**: preempt 회계 결함 → 원자 상태 스케줄·마이그레이션 → per-CPU 상태(SLUB freelist, fpsimd busy) 연쇄 오염 → ① SLUB oops(부하 중) ② fpsimd WARN 폭풍+oops(저부하).

## (a-2) crash_logs 전수 정밀 분석 (2026-07-12) — "config만으로 해결 가능한가" 판정

`~/ros2_ws/crash_logs/` 4개 파일 + journal 부팅별 교차검증 결과:

1. **1차 BUG(0x00000002) 9건의 경합 락은 3종** — crng `base_crng.lock` 7건(부팅 4~5s, SLAB_FREELIST_RANDOM→`new_slab`→`get_random_u32` 경로), kernfs idr 락 1건(부팅, uvc probe), pidmap 락 1건(런타임 118s, docker 기동 fork 중 `alloc_pid`). **트리거 락이 제각각 = 결함은 특정 서브시스템이 아니라 preempt 회계 자체.** (9건 모두 평범한 process context, count=2)
2. **BUG→fpsimd 인과가 로그로 직접 연결**: 5be1f2e2 부팅 118.0375s alloc_pid BUG(CPU2/PID2653) → **0.5ms 뒤** 같은 CPU/PID에서 첫 fpsimd WARN(318) → 폭풍 25건(173/204/318, PID2653이 CPU1/2/3을 옮겨다니며) → 동일 태스크 `task_fpsimd_load` oops로 사망. "원자 상태 스케줄 → per-CPU fpsimd 보호 붕괴" 가설이 사실로 확정.
3. **★핵심 신규 증거 — 치명 SLUB oops 2건 모두 "부팅 BUG 0건"인 부팅에서 발생** (journal 부팅 ID별 실측):

   | 부팅 | 부팅 중 BUG | 결말 |
   |---|---|---|
   | 91c37f6d (07-08 20:10) | **0건** | 58분 뒤 SLUB oops 크래시 ⓪ |
   | 9ec1e653 (07-10 12:32*) | **0건** | 8.5h 뒤 SLUB oops 크래시 ① |
   | 5be1f2e2 (07-10 21:22) | 5건 | 2분 뒤 fpsimd 폭풍 크래시 ② |
   | edf16f0e (07-08 21:37) | 6건 | 106초 만에 세션 소멸(프리즈 추정) |
   | 94418d09 (07-10 23:54) | 2건 | **42h 무사 생존** |

   (*journal 표시 시각은 stale RTC로 뒤틀려 있음 — 실제 부팅 시각은 monotonic 역산 기준)
   → **오염은 BUG 출력 없이 "무음"으로도 진행**된다. BUG 라인은 결함의 탐지된 부분집합(경합 sleeping 락을 만나 `__schedule`에 걸린 경우)일 뿐이고, 반대편 무음 경로(count 과소 = 원자여야 할 구간이 선점 가능 상태 → per-CPU SLUB freelist 조작 중 선점·이동)는 아무 출력 없이 오염만 남긴다. BUG 유무와 생사가 상관없는 표가 그 증거.
4. **DEBUG_PREEMPT=n 실측 확인**: BUG 리포트 17건 전부 `Preemption disabled at:` 줄 부재(+ `/boot/config-…rt-kria`에서 =n 확인). =y였다면 매 BUG마다 preempt_disable 누수 지점(IP)이 찍히고, 무음 쪽(count 과소)도 preempt_enable underflow 경고로 원점에서 포착됐음.

**판정 — config 단독 해결 가능성:**
- **유일한 "해결" 후보 config = `CONFIG_PREEMPT_LAZY=n`** (⓪ 실험 그대로 유효). 결함이 스케줄러 회계 코어에 있다는 오늘 결론과 정합 — LAZY는 그 코어에 들어가는 유일한 실험적 config. 단 성공 보장 없음(Ubuntu SAUCE↔rt 훅 충돌이 LAZY 밖이면 config로 불가, ④행).
- **`SLAB_FREELIST_RANDOM=n`은 해결책이 아님이 입증됨**: 치명 crash 2건 모두 부팅 BUG 0건 부팅에서 발생 → 부팅 트리거(crng 경로)를 제거해도 죽음을 못 막았을 것. 끄면 오히려 가장 값싼 조기 경보(카나리아)만 사라짐 → **⓪ 실험 빌드에서 =y 유지로 권고 정정**.
- **`DEBUG_PREEMPT=y`+`DEBUG_ATOMIC_SLEEP=y`는 해결은 아니나, config만으로 "원인 지점 자동 특정"을 제공** — LAZY=n이 실패해도 로그 한 줄로 다음 수(국소 소스 수정 or ④)가 결정된다. 실험 빌드 필수.
- 나머지는 무관: MIGRATION(페이지 이동 기능, 기답변), RANDOM_TRUST_*(이미 =y이나 U-Boot가 시드를 안 넘겨 crng ready 지연은 그대로), SLUB_CPU_PARTIAL(5.15은 RT 호환 설계).

## (b) 수정 전략

> **★★ 2026-07-13 최종 개정 — 권장 순위: ⓪.5(radix-tree revert 원복 국소 픽스, 아래 ⓪ 결과 블록) → ④(linux-xlnx + 매칭 RT 패치 수동 머지, fallback) → ①(vanilla, 최후).** ⓪(LAZY=n)은 실행 완료·실패 확정(원인 특정의 결정적 수단이 됨).
> (구) 2026-07-10 밤 개정 — 권장 순위: ⓪(+버전 정합) → ④(linux-xlnx + 매칭 RT 패치 수동 머지) → ①(vanilla, 최후)
> 추가 확정(웹 5개 에이전트 + 1차 소스 재검증): **"Xilinx 공식 RT 커널(linux-xlnx-rt)"은 존재하지 않음** — PetaLinux 레시피 없음(meta-xilinx rel-v2022.1/2022.2 디렉토리 전수 확인), linux-xlnx에 5.15 RT 브랜치/태그 없음(`git ls-remote` 전수), Xilinx wiki는 RT를 "experimental, not officially supported"로 명시하고 **kernel.org RT 패치를 linux-xlnx에 직접 git merge하는 절차**를 안내. 단 **베이스와 정확히 일치하는 RT 패치 쌍은 존재**: `xilinx-v2022.1`(=5.15.19)↔`patch-5.15.19-rt29`, `xilinx-v2022.2`(=5.15.36)↔`patch-5.15.36-rt41`.
> vanilla 경로 추가 블로커 확정: **usb5744 허브는 vanilla 5.15에서 초기화 불가(= RealSense 카메라 죽음**; xlnx 전용 드라이버가 유일한 init 경로, mainline 지원은 v6.12+) · K26/KV260 DTS는 v5.16부터 · zocl은 vanilla fpga-mgr에 없는 DMA-BUF 확장 필요 · configfs 오버레이는 mainline 반려된 패치. "순수 vanilla KV260 완주" 선례 없음(유일 유사 사례 ikwzm도 xlnx 백포트 대량 적용).
> 보드 실측 근거(vanilla 비용 폭증의 발단):
> - **zocl(DPU)·al5e/al5d(VCU)·usb5744(USB 허브)·ap1302·dmaproxy가 전부 in-tree** (dkms 아님 — Ubuntu-Xilinx 트리에 포함). "외부 모듈만 재빌드"가 성립하지 않고 드라이버 뭉치를 포팅해야 함.
> - smartcam 오버레이 적용 통로인 **`CONFIG_OF_CONFIGFS`(configfs DT overlay)도 비-mainline Xilinx 패치** — vanilla엔 없음.
> - **mainline allegro 드라이버는 VCU 대체 불가 확정** (compatible 불일치: mainline `allegro,al5e` ↔ smartcam DTBO `al,al5e-1.2`). 단 VCU/ap1302는 현 파이프라인이 안 쓰므로 미바인딩 방치 가능성은 있음(미검증). OF_CONFIGFS·zocl·afi-fpga는 필수라 강등 결론엔 영향 없음.

### ⓪ (선택) 값싼 선행 실험 — 현재 소스에서 PREEMPT_LAZY만 끄고 재빌드 (2026-07-10 추가 검토)
사용자 질문("migration 옵션 끄면 해결되나?")을 계기로 검토한 옵션. **MIGRATION은 무관**(아래 참고), 진짜 용의 옵션은 `CONFIG_PREEMPT_LAZY`.
- 방법: 데스크톱의 기존 소스 트리(Ubuntu 베이스+rt91) 그대로, config만 `CONFIG_PREEMPT_LAZY=n` + 검증용 `CONFIG_DEBUG_PREEMPT=y`/`CONFIG_DEBUG_ATOMIC_SLEEP=y` 변경 후 재빌드(§4-5, ~30분). `SLAB_FREELIST_RANDOM`은 **=y 유지** (2026-07-12 정정, (a-2) 판정 참조 — 끄는 것은 해결이 아니라 카나리아 제거이고, 변인을 LAZY 하나로 유지해야 실험 판별력이 산다).
  - **⚠️ 방법 정정 (2026-07-12, menuconfig 실사용 중 발견)**: `PREEMPT_LAZY`는 `kernel/Kconfig.preempt`에 `def_bool y if HAVE_PREEMPT_LAZY && PREEMPT_RT`로 정의돼 있어 **사용자 선택 프롬프트가 없는 자동 계산 심볼**이다. `HAVE_PREEMPT_LAZY`는 arm64가 무조건 select하므로, PREEMPT_RT를 고르면 LAZY는 menuconfig UI 없이 자동 y — **config 변경만으로는 끌 수 없다.** 실행하려면 `Kconfig.preempt`의 해당 config 블록을 아래처럼 1줄 소스 패치해 선택 가능한 항목으로 바꿔야 한다(그래도 저비용, ④보다 압도적으로 쌈):
    ```diff
     config PREEMPT_LAZY
    -	def_bool y if HAVE_PREEMPT_LAZY && PREEMPT_RT
    +	bool "Enable Lazy Preemption"
    +	depends on HAVE_PREEMPT_LAZY && PREEMPT_RT
    +	default n
    ```
    저장 후 menuconfig 재시작하면 체크박스로 노출됨(파일은 매번 새로 읽음).
  - **LAZY 진짜 원인인지 정직한 평가 (2026-07-12)**: LAZY는 Ubuntu 탓으로 생긴 게 아니라 **rt91 자신의 Kconfig 설계**(순정+rt91이었어도 arm64는 자동 강제)이고, ⓪이 1순위인 이유는 "유력해서"가 아니라 **소스 1줄로 가장 싸게 배제 가능해서**다. 실제 원인 서명(`0x2=FORK_PREEMPT_COUNT`, fork/schedule_tail 회계)은 LAZY가 건드리는 코드(`resched_curr_lazy`, CFS tick 선점 결정)와 **다른 동네**라 실패(재발) 가능성이 성공만큼 현실적 — 실패해도 이상한 결과 아님, 바로 ④행.
  - **정적 검증(재부팅 없이, 2026-07-12)**: 설치된 `/boot/System.map-5.15.199-rt91-rt-kria`에서 `resched_curr_lazy` 심볼 확인(T, 176B, `resched_curr`서 13.5KB 뒤·`resched_cpu` 바로 앞 — 정상 위치). **컴파일/링크는 깨끗했다(코드가 안 죽거나 엉뚱한 데로 안 날아감)만 확인됨 — 런타임에 `resched_curr()`가 실제로 올바르게 분기하는지, SAUCE 변경 fork 경로에서 `TIF_NEED_RESCHED_LAZY`가 일관되게 관리되는지는 미확인**(확인하려면 디스어셈블 또는 RT 부팅 후 ftrace 라이브 추적 필요 — 후자는 크래시 위험 있는 커널 재부팅이라 지금 단계에서 보류). "컴파일 클린 vs 의미론적 정합성"이 정확히 원 가설의 핵심.
  - **★★⓪ 실험 결과 (2026-07-13 부팅, `5.15.199-rt91-rt-kv260` = LAZY=n+DEBUG 2종): 실패 확정 — 그리고 원인 지점 특정 성공.** 크래시 없이 로그인까지 도달했으나 부팅 ~3분 내 **`BUG: sleeping function called from invalid context` 250건 + `scheduling while atomic` 2건(0x00000002, 구 서명 재현) + `DEBUG_LOCKS_WARN_ON(val > preempt_count())` underflow 1건(무음 오염 측 실증)**. → **LAZY 무죄 확정(기각), 판정 프로토콜대로 ④행.**
  - **★범인 특정 (DEBUG_PREEMPT의 성과)**: 리포트 253건 **전수가 단일 원점 `Preemption disabled at: __radix_tree_preload+0x28`** (타 원점 0건). 의미: radix-tree/idr의 per-CPU preload 패턴(`idr_preload()` → preload 락을 쥔 채 반환 → 호출자가 spin_lock 후 `idr_preload_end()`)이 이 트리에서 **RT용 local_lock(잠들 수 있는 per-CPU 락, 선점 유지)이 아니라 구식 `preempt_disable()`로 동작**. 그래서 preload를 쥔 채 sleeping rt lock(pidmap·kernfs·crng·cgroup·inotify·BTF …)을 잡는 모든 경로가 구조적으로 발화 — fork마다, sysfs 노드마다. 구 rt-kria의 count=0x2 서명도 preload(+1)+SLUB per-CPU pin(+1) 중첩으로 정합 설명됨. **`0x2=FORK_PREEMPT_COUNT`(fork 회계) 가설은 기각.**
  - **★★★근본 원인 100% 확정 (2026-07-13, 1차 소스 전수 대조)**: 사용자 트리 grep 결과 `lib/radix-tree.c`가 구식 `preempt_disable()`(335/342/384/1473행) — 그런데 트리 오염이 아니라 **베이스 자체가 그랬다**. 대조 전수 결과:
    | 트리 | radix preload 보호 |
    |---|---|
    | vanilla/stable 5.15.199 (kernel.org) | **local_lock ✓** (v5.8 mainline 전환, cfa6705d89b6) |
    | linux-xlnx xilinx-v2022.1 (Xilinx 벤더 git) | **local_lock ✓** |
    | patch-5.15.197-rt91 (11,384줄 전수 grep) | radix-tree 훅 **0건** (패치는 이 파일 안 건드림) |
    | **Ubuntu jammy `linux` (제네릭)** | **preempt_disable ✗** |
    | **Ubuntu `linux-xilinx-zynqmp`** | **preempt_disable ✗** (동일 335/342/384행) |

    범인 커밋: **`UBUNTU: SAUCE: Revert "radix-tree: Use local_lock for protection"`** (05fdd32398, Seth Forshee, 2021-11-02, jammy 전 flavor 공통). 사유(커밋 본문): local_lock 전환이 추가한 `EXPORT_PER_CPU_SYMBOL_GPL(radix_tree_preloads)`가 **NVIDIA 독점 모듈 빌드에 새어 들어가서** — *"This is harmless, as the change is really for RT builds"*. 즉 **비-RT에선 무해(local_lock≡preempt_disable)하지만, 이 트리에 RT 패치를 얹는 순간 `idr_preload()`/`radix_tree_preload()`를 쓰는 모든 경로(fork alloc_pid·kernfs/sysfs·cgroup·inotify·BTF·crng…)가 "preempt_disable 상태로 sleeping rt lock"이 되는 구조 결함**이 된다. revert 범위 = 정확히 3파일: `lib/radix-tree.c`, `include/linux/radix-tree.h`(struct+preload_end), `include/linux/idr.h`(idr_preload_end). mm/swap.c 등 다른 local_lock 사용처는 Ubuntu도 원본 유지(15곳 확인) — **radix-tree 국한 revert**. 이것이 07-08/07-10 크래시·fpsimd 폭풍·⓪ 커널 253건 리포트의 단일 근원이며, "재현이 확률적"이었던 이유(비-DEBUG 커널은 락 경합시에만 발현)까지 전부 설명.
  - **★⓪.5 국소 픽스 = 새 최우선 경로**: 트리에서 "revert의 revert" — vanilla 5.15.199의 3개 파일로 원복 후 재빌드(NVIDIA 무관한 Kria에선 GPL export 부활 무해). 검증된 vanilla 원본 3종을 보드 `~/ros2_ws/kernel_configs/vanilla-5.15.199-radix-fix/`에 준비함(radix-tree.c=1606줄/local_lock 4, radix-tree.h=470줄, idr.h=331줄 — 전부 gregkh stable 미러 v5.15.199에서 추출·검증). 판정: DEBUG 검출기 켠 채 부팅해 **253건→0건**이면 근원 해결 확정 → 이후 부팅 10회+소크+파이프라인 부하 프로토콜. 실패 시 ④ fallback 유지.
  - **★★★ 해결 확정 (2026-07-13, `-rt-kv260` rev-5 부팅): 253건 → 0건. 근본 원인 규명·수정 완결.** 사용자가 PC에서 3파일 원복 후 재빌드(rev-5, `#5 SMP PREEMPT_RT`). 설치 전 검증에서 **System.map에 `__ksymtab_radix_tree_preloads` + `D radix_tree_preloads`**(vanilla local_lock판의 지문 — Ubuntu 리버트판은 static이라 없음) 확인, U-Boot가 커널 해시 sha1 검증(25a33cdb) 후 부팅. 러닝 커널 실측: DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP=y로 **검출기 완전 무장 상태에서** `journalctl -k -b 0` 전수 스캔 = BUG/scheduling-while-atomic/sleeping/Preemption-disabled/fpsimd/oops/Internal-error **전부 0건** (어제 동일 커널 253건 대비). `/proc/kallsyms`에 픽스 export 심볼 라이브 확인. 잔여 WARNING 4건은 `OF: overlay: memory leak`(smartcam FPGA 오버레이, 순정에도 뜨는 무해). DPU 스택 정상(`/dev/dri/card0`, zocl+dmaproxy 로드). **④ linux-xlnx 재조립 불필요 — 폐기.** 남은 것: (a) **✅ 부하 소크 통과(2026-07-13, 누적 radix 위반 0)** — v1 load156 9.6분 + v3 load50 14분, DEBUG 검출기(DEBUG_PREEMPT/ATOMIC_SLEEP) 무장 상태, 하네스 `soak_rt.sh` → (b) **[유일 잔여] DEBUG 옵션 끄고 프로덕션 재빌드(rev-6, PC config만 변경)** → EtherCAT 선결조건 해제. 부트로그 원본: 사용자 `rt5_bootlog.txt`.
  - (참고) 이는 Ubuntu 5.15 커널에 RT 패치를 얹으려는 모든 사람이 밟을 지뢰 — Launchpad 버그 리포트 후보.
  - 증거 보존: `~/ros2_ws/crash_logs/rt_kv260_lazyoff_debug_boot_20260713.log` (journal 7,096줄) + 사용자 UART 캡처. 보드는 2026-07-13 순정(1070) 복귀.
  - **⚠️ 빌드 블로커 실측 (2026-07-12, PC `bindeb-pkg` 1차 시도)**: `drivers/staging`에서 `-Werror=implicit-function-declaration`로 즉사 — `ubuntu/staging/xilinx-mv-camera-10gige-module/s2imac_m.c`와 `drivers/staging/xilinx-tsn/xilinx_tsn_ptp_clock.c` 둘 다 `in_be32`/`out_be32`(PowerPC용 MMIO 접근자, arm64엔 미선언) 사용. 컴파일러 버전은 원인 아님(보드 백업 rt91 config·순정 config·보드 크로스툴체인 셋 다 `gcc 11.4.0-1ubuntu1~22.04.3`로 동일 확인) — **`CONFIG_XILINX_TSN`(+PTP/QBV/SWITCH/QCI/CB/QBR)은 예전 성공 rt91 config에서도 이미 `=y`였음**(config 차이가 원인이 아니라, 같은 config로 예전엔 통과했었다는 뜻 — 왜 이번엔 안 되는지는 PC 소스 트리 쪽 미해결). **보드 실측으로 TSN 사용 여부 확인**: cmdline에 `xilinx_tsn_ep.st_pcp=4`가 있으나 이번 부팅 journal에 tsn 프로브/바인딩 메시지 0건, `/proc/device-tree`에 tsn 노드 자체 없음(벤더 cmdline 템플릿의 죽은 파라미터, 실제 TSN 하드웨어 없음) → **두 드라이버 모두 이 프로젝트와 무관, menuconfig에서 `XILINX_TSN`(하위 6개 자동 종속 해제)과 mv-camera-10gige 심볼(정확한 이름은 PC 트리의 `ubuntu/staging/xilinx-mv-camera-10gige-module/Kconfig` 참조) 둘 다 N으로 꺼서 우회 — 안전. `make clean` 불필요, config만 바뀐 재빌드.**
- **⚠️ 재현성 정정 (2026-07-12, UART 부트로그+journal 실측)**: BUG는 "매 부팅 100%"가 아니라 **확률적 race** — 부팅별 카운트 실측: 크래시부팅 5건(+oops 25건), 2h 부팅 2건, **42h 부팅 2건(무크래시 생존)**, 07-12 부팅 **0건**. early-boot udevd coldplug 창에서 crng 락 경합이 겹칠 때만 발화.
- **판정 프로토콜 (정정판)**: **실패 판정은 즉시**(BUG 1건이라도 재현 = LAZY 단독 원인 아님 확정 → ④행). **성공 판정은 다회 검증 필요** — 부팅 10회 반복 + `DEBUG_PREEMPT/DEBUG_ATOMIC_SLEEP`(경합 우연에 의존하지 않고 preempt 회계 이상을 발생 지점에서 직접 포착 — 이게 핵심 검출기) + 수 시간 부하 소크에서 전부 0건.
- 성공 시: vanilla 재조립 없이 값싸게 해결 — 단, ①과 동일한 검증 프로토콜은 통과해야 함.
- 실패 시(BUG 잔존): Ubuntu SAUCE 스케줄러와 rt 훅의 의미론적 충돌이 LAZY 밖에 있다는 뜻 → **④(linux-xlnx+RT 머지)로 진행** — 재발 자체가 ④행 확정 증거. (fork 회계 0x2 서명은 LAZY 단독으로 설명 안 될 수 있어 실패 가능성 상존.)
- (보강, 2026-07-10 밤) LAZY=n과 함께 **버전 정합화 병행 권장**: rt91은 5.15.197 전용 패치이므로 재빌드 시 가능하면 쌍을 맞출 것(예: 트리 스테이블 레벨 확인 후 그에 맞는 rt 패치 재적용). 부정합을 줄일수록 실험의 판별력이 올라감.
- **참고 — MIGRATION이 무관한 이유**: `CONFIG_MIGRATION`은 태스크가 아니라 **메모리 페이지** 이동 기능이고, CMA(DPU용 cma=1000M)가 Kconfig에서 강제로 켬(`select MIGRATION`) — 끌 수도 없고, 순정 커널도 `MIGRATION=y`인데 멀쩡하므로 원인이 아님. 태스크의 코어 간 이동은 빌드 옵션이 아니라 SMP 스케줄러의 본질 기능이며, 이동을 막아도 같은 코어 내 선점만으로 per-CPU 상태 오염이 가능(크래시① SLUB 유형)하고, 부팅 BUG는 시스템 태스크(udevd/kworker)에서 발생해 pinning으로도 못 막음.

### ① vanilla 5.15.209 + patch-5.15.209-rt96 재빌드 (⚠️ 2026-07-10 밤 강등 — 위 개정 블록 참조. OF_CONFIGFS+Xilinx 드라이버 포팅 비용 때문에 최후 수단)
- **Ubuntu/Xilinx 트리를 베이스로 쓰지 말 것.** vanilla `linux-5.15.209` + `patch-5.15.209-rt96`(2026-06-05, 5.15-rt 최신·정확 매칭).
- zocl/dmaproxy/allegro/al5e/al5d는 모듈이므로 vanilla 커널 + 기존 config(참조) + 외부 모듈 재빌드로 동일 기능 구성 가능. config 참조: Ubuntu linux-xilinx-zynqmp 1075.79(2026-07-03, 베이스 ~5.15.209)도 적합.
- **config 조정 (첫 빌드)**:
  - `CONFIG_PREEMPT_LAZY=n` (실험 기능 배제)
  - `CONFIG_SLAB_FREELIST_RANDOM=n`, `CONFIG_SLAB_FREELIST_HARDENED=n` (new_slab→crng 지뢰 제거)
  - 검증 기간: `CONFIG_DEBUG_PREEMPT=y`, `CONFIG_DEBUG_ATOMIC_SLEEP=y` (+가능하면 `PROVE_LOCKING`) — 잔여 누출을 발생 지점에서 즉시 포착. 안정 확인 후 프로덕션에서 debug만 끔.
- **검증 프로토콜**: 부트 파라미터 `slub_debug=FZP`(freelist 오염 조기 검출) + UART 시리얼 콘솔 로그 캡처(다른 PC 연결 예정) + zocl/dmaproxy 없이 소크 테스트 → 모듈 추가 순서로 격리 → 비전 풀 파이프라인 부하 테스트.
- 기존 크로스컴파일 절차·flash-kernel 함정은 `rt_patch.md` 그대로 유효 (dpkg-source 대신 tarball이라 §의 실행권한 함정은 무관).

### ② 특정 커밋 체리픽 — 부적합
rt91→rt96에 픽할 수정이 없고 stable에도 관련 fpsimd 수정 없음. 픽 대상 자체가 없음.

### ④ linux-xlnx 5.15 + 매칭 RT 패치 수동 머지 — ⓪.5 국소 픽스 실패 시의 fallback (2026-07-10 밤 검증 완료, 07-13 지위 변경: 확정 경로 → fallback)
"공식 RT 레시피"는 없지만, **Xilinx wiki가 공인하는 절차**(linux-xlnx에 kernel.org RT 패치 git merge, 충돌 예시 axienet → `git checkout --ours`)가 이것. 사실상 공식에 가장 가까운 조합:
- **1안(권장): `linux-xlnx xilinx-v2022.1`(=5.15.19) + `patch-5.15.19-rt29`** — 보드 rootfs 스택(xrt 2.13 / VCU 2022.1 / smartcam 0.12)과 **세대 일치**.
- 2안: `xilinx-v2022.2`(=5.15.36) + `patch-5.15.36-rt41`.
- 장점: usb5744·ap1302·OF_CONFIGFS·afi-fpga·패치된 xlnx_vcu·K26 DTS **전부 트리에 포함**. OOT 빌드는 zocl(XRT)과 vcu-modules 2건뿐(둘 다 이 커널 세대가 원래 타깃 — 게다가 현 크래시 커널에서 5.15 PREEMPT_RT 위 구동이 이미 실증됨).
- 작업: 클론+RT 머지(충돌 소수)+defconfig 이식+bindeb-pkg+flash-kernel 오버라이드 (§4 절차 재활용, 수일).
- 리스크: 베이스가 5.15.19/36으로 오래됨(LTS update 태그로 일부 완화, 정확한 5.15.y 레벨 미확인), RT는 벤더 "실험적" 등급. config는 `PREEMPT_LAZY=n` + 검증 옵션 동일 적용.

### ③ 더 새로운 베이스(6.1-rt 등) — 차선/장기
6.1은 `patch-6.1.176-rt64`(2026-06-24)까지. Xilinx 공식 스택은 2024.x부터 6.6 계열. 마이그레이션 비용 커서 지금은 ① 우선. (6.12+는 RT 메인라인 편입.)

## (c) 증거 위치 · 참고 링크

- **로컬 증거**: `/var/log/kern.log` — 부팅 BUG 4.6s 시점 5회 재현, oops 2건 라인 5381·12815, fpsimd 폭풍 13602~. journal boot ID: 세션 크래시 `9ec1e653`, 2분 부팅 크래시 `5be1f2e2`. config: `/boot/config-5.15.199-rt91-rt-kria`.
- **추출본(재부팅·로테이션과 무관하게 보존)**: `~/ros2_ws/crash_logs/` 4개 파일 — BUG 17건 전수, 크래시① oops tail(call trace는 journald 사망으로 잘림), 크래시⓪/① kern.log 문맥, 크래시② fpsimd 폭풍 전체 2,383줄. 부팅별 BUG/oops 카운트는 (a-2) 표. **+ ⓪ 실험 부팅 로그 `rt_kv260_lazyoff_debug_boot_20260713.log`(7,096줄 — 원인 확정 증거)**
- **근본 원인 커밋 (2026-07-13)**: Ubuntu jammy `05fdd323982c` — https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/jammy/commit/?id=05fdd323982cd09570c0eb80b22729f2bbf7adc7 · 되돌려진 mainline 원 커밋 `cfa6705d89b6`(v5.8). 대조에 쓴 1차 소스: gregkh stable 미러 v5.15.199(radix-tree.c/h, idr.h), github.com/Xilinx/linux-xlnx xilinx-v2022.1, kernel.org patch-5.15.197-rt91 전문, Launchpad linux-xilinx-zynqmp `applied/ubuntu/jammy-updates`
- **⓪.5 픽스 재료**: `~/ros2_ws/kernel_configs/vanilla-5.15.199-radix-fix/` (vanilla 원본 3파일, local_lock 체계 검증 완료)
- RT 패치 저장소(5.15 최신 rt96): https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/ · 이력(rt91=5.15.197, rt92/93=5.15.201, rt94=5.15.202, rt95=5.15.206, rt96=5.15.209): https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/older/
- 6.1 RT: https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/6.1/
- stable changelog(fpsimd 변경 없음 확인): https://cdn.kernel.org/pub/linux/kernel/v5.x/ChangeLog-5.15.198 · ChangeLog-5.15.199
- 2021 동일 증상 스레드(수정의 기원): https://lore.kernel.org/lkml/20210722175157.1367122-4-valentin.schneider@arm.com/ · 후속(2022): https://lore.kernel.org/linux-arm-kernel/20220505163207.85751-4-bigeasy@linutronix.de/t/
- Ubuntu 패키지: https://launchpad.net/ubuntu/+source/linux-xilinx-zynqmp/5.15.0-1070.74 (베이스 Ubuntu 5.15.0-177.187)

## 미확정/추정으로 남는 것 (2026-07-13 대부분 해소)
- ~~`0x2=FORK_PREEMPT_COUNT` 해석과 "의미론적 충돌" 부분은 정황이 강한 추정~~ → **기각·해소**: 0x2 = radix preload(+1) + SLUB per-CPU pin(+1) 중첩, 근원은 Ubuntu SAUCE radix-tree revert (⓪ 결과 블록 참조).
- ~~zocl 자체 버그 가능성~~ → **혐의 사실상 해소**: zocl oops 현장은 `kds_alloc_command→__kmalloc`(고빈도 할당 경로)이 오염된 SLUB freelist를 읽은 것 — 근원 동일. 단 ⓪.5 검증 소크에서 zocl 포함 부하 테스트로 최종 확인.
- 남은 유일 미확정: Ubuntu 트리에 radix-tree 외 "RT에서만 유해한" SAUCE가 더 있는지 — ⓪.5 부팅에서 DEBUG 검출기 0건이면 사실상 없음이 입증됨(검출기가 모든 preempt/atomic 위반을 원점 표시하므로).
