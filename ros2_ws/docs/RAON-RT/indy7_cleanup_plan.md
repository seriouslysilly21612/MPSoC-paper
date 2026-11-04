# RAON-RT-Revision / App/Indy7 — FINAL CLEANUP PLAN

기준: branch `kv260-merge` @ `6c090ae`. 모든 경로는 절대경로. **본 조사에서 파일 수정/삭제 없음 (read-only).**

전제 3가지를 먼저 확정해 둡니다.
- `make clean`은 `obj/`와 `bin/`을 함께 지웁니다. `obj/FullDynControllerRT.o`, `obj/Controller.o`는 `reachmap`/`fk_replay` 타깃의 **선언된 prerequisite** (`/home/ubuntu/RAON-RT-Revision/App/Indy7/Makefile:174` REACHMAP_OBJS). 이번 정리 중 `make clean` 금지.
- `tools/reachmap.cpp` / `tools/fk_replay.cpp`가 controller에서 실제로 호출하는 것은 확인 결과 정확히 다음뿐: `Init`, `Enable`, `Update`, `ComputeReadySeed`, `GetReadyQ`, `SolveReadyIK`, `GetLastIkDiag`, `ToolPosAt`. 아래 어느 항목도 이 8개를 건드리지 않습니다.
- `Indy7Ctrl.o`는 두 tool 어디에도 링크되지 않습니다 (`REACHMAP_OBJS`에 없음). 따라서 Indy7Ctrl 쪽 삭제는 tool을 깨뜨릴 수 없습니다.

---

## TIER 1 — SAFE (라이브 참조 0, tracking/gripper 무관, 빌드 영향 0)

| # | 대상 | 정확한 삭제 액션 | 근거 (재검증) | 규모 |
|---|---|---|---|---|
| T1-1 | `rt_log_results/plotted_csv/` | `git rm -r /home/ubuntu/RAON-RT-Revision/App/Indy7/rt_log_results/plotted_csv` | tracked 36개. 유일 소비자였던 `rt_log_results/plot_tcp_trajectory.py`가 죽음. `tools/approach_plot.py:371`은 rt_log_results **top level**만 glob (`plotted_csv` 문자열 없음) | 161 MB |
| T1-2 | `rt_log_results/plots/` | `git rm -r /home/ubuntu/RAON-RT-Revision/App/Indy7/rt_log_results/plots` | tracked 12 PNG. 생산자 = 같은 죽은 스크립트. 라이브 출력은 `approach_results/plots/` (`run.sh:31-32`) | 1.9 MB |
| T1-3 | `rt_log_results/plot_tcp_trajectory.py` | `git rm /home/ubuntu/RAON-RT-Revision/App/Indy7/rt_log_results/plot_tcp_trajectory.py` | 외부 참조 0 **+ 실행하면 유해**: `:201-203`이 CWD의 `DataLog_*.csv`를 전부 `plotted_csv/`로 shutil.move → `fk_replay`가 만든 `*_fk.csv`까지 옮겨서 approach_plot 페어링이 조용히 깨짐. **삭제 전 salvage 권장**: `:100-190`의 settle-window 지표(TAIL=500, goal-vs-actual mm + `acos((tr(R_g R_a^T)-1)/2)`)를 `tools/approach_plot.py`로 이식. 이건 moving target의 steady-state lag 지표로 그대로 쓰임 | 206 LOC |
| T1-4 | 수동 LED 키 `y/u/o` | `Indy7Ctrl.cpp:527-538` 삭제 (case 'y'~'O' + `break;`) | `proc_main_control:1596-1614`의 자동 LED 블록이 매 1 ms 무조건 덮어씀 (확인함). `'h'`로 fall-through 하던 버그도 같이 소멸 | 12 LOC |
| T1-5 | 주석 처리된 대체 `'i'` 핸들러 | `Indy7Ctrl.cpp:1751-1756` 삭제 | 전부 `//` 주석. 라이브 경로는 `DoInput:612-618` | 6 LOC |
| T1-6 | `WriteDataLog` 일체 | `Indy7Ctrl.cpp:905-939` 삭제 · `Indy7Ctrl.h:60` 선언 삭제 · `Indy7Ctrl.h:29-31,34-49`(VECDOUBLE/LISTINT/LISTULONG/ST_DATALOG) 삭제 · `Indy7Ctrl.h:75-76`(`m_strDataLog`, `m_stDataLog[32]`) 삭제 · `main.cpp:91` 주석 줄 삭제 | 유일 호출부가 `main.cpp:91`에서 주석 처리됨(확인). `m_stDataLog`에 push하는 코드가 repo 전체에 없음 → 호출돼도 warn만. **덤: `:910-911`에 실제 buffer overflow** (`TSTRING strAxisNo; sprintf(&strAxisNo[0], ...)` → 길이 0 std::string에 6바이트 기록). 라이브 logging 경로(`m_logBuffer`/`proc_logger`/`make_csv`)와 **공유 지점 0** | 58 LOC |
| T1-7 | `InitConfig()` stub | `Indy7Ctrl.cpp:509-513` + `Indy7Ctrl.h:80` 삭제 | repo 전체 grep: 정의+선언뿐. `CRobot/Robot.h`에 base virtual **없음** (즉 override도 아님) | 6 LOC |
| T1-8 | `DoAgingTest()` stub | `Indy7Ctrl.cpp:515-519` + `Indy7Ctrl.h:81` 삭제 | `CRobot/Robot.h:81`에 `virtual void DoAgingTest() { return; }` 기본 구현 존재 → override 제거해도 컴파일/링크 무해. 호출부 0 | 6 LOC |
| T1-9 | `DoHoming()` stub | `Indy7Ctrl.h:89` (`void DoHoming() {};`) 삭제 | 로봇 레벨 빈 stub. 호출부 0. `CAxis::DoHoming`(Axis.h:161)과 **무관한 별개 심볼** — 혼동 주의 | 1 LOC |
| T1-10 | `m_bEnableTriangleControl` | `Indy7Ctrl.h:102` 삭제 | repo grep 결과 선언 1줄이 전부 (읽기·쓰기 모두 0) | 1 LOC |
| T1-11 | `m_nEcatCycle` (write-only) | `Indy7Ctrl.h:101` + `Indy7Ctrl.cpp:38` + `Indy7Ctrl.cpp:499` 삭제 | 두 대입만 존재, reader 0 | 3 LOC |
| T1-12 | `m_eRectState` + `eRECT_IDLE`/`eRECT_TO_CORNER` | `Indy7Ctrl.h:63`(enum) + `m_eRectState` 멤버 줄 삭제 · `Indy7Ctrl.cpp:635`, `:669`의 대입 2줄 삭제 | **RECT 클러스터 전체는 TIER 3이지만 이 부분만은 확실히 죽음**: enum 값 2개 모두 비교되는 곳이 없고, SM은 `m_bRectTrigger`로 분기. 즉 write-only. RECT를 남겨도 이건 지워도 됨 | 4 LOC |

TIER 1 합계: **C++ 97 LOC · Python 206 LOC · 디스크 163 MB** (그중 162 MB가 git-tracked → `git rm` 필요, `git clean`으로는 안 지워짐)

---

## TIER 2 — CARE (제거 가능하되 공유 파일/동반 수정 필요)

### T2-1. ISO 클러스터 (반드시 **원자적으로** 한 커밋에)

두 감사자가 갈렸던 항목입니다. files 담당은 "`FullDynControllerRT.cpp:2255`에 라이브 `system()` 호출이 iso_csv를 가리키므로 DEFER"라 했고, Indy7Ctrl 담당은 "ISO 코드 자체가 죽었다"고 했습니다. **둘 다 맞고, 함께 지우면 충돌이 사라집니다.** `system()` 호출은 `RunISOCubeIKValidation()` 안에 있으므로 그 함수와 같이 사라집니다.

| 삭제 지점 | 액션 |
|---|---|
| `Indy7Ctrl.cpp:680-709` | `case 'q'/'Q'` (ISO HW 테스트 런처) 삭제 |
| `Indy7Ctrl.cpp:1249-1300` | `proc_main_control` 내 ISO HW state machine 블록 삭제 (주석 헤더 포함) |
| `Indy7Ctrl.cpp:1741-1747` | `proc_keyboard_control`의 `'z'/'Z'` 후크 삭제 |
| `Indy7Ctrl.cpp:1966-2024` | `CRobotIndy7::SaveISOHWResults()` 삭제 |
| `Indy7Ctrl.h:62`, `:131-139`, `:150` | `eISOHWState` enum, ISO 멤버 블록(`m_isoHWRecords[5][10][3]` 등 ≈1.2 KB/instance), `SaveISOHWResults` 선언 삭제 |
| `FullDynControllerRT.cpp:2137-2260` + `FullDynControllerRT.h:313` | `RunISOCubeIKValidation()` 삭제 (`:2255`의 `system("python3 ...")` 동반 소멸 — RT 앱에서 `system()` 제거는 그 자체로 이득) |
| `App/Indy7/iso_csv/` | `git rm -r` (plot_iso_virtual.py 278 LOC + virtual/*.csv) |
| **동반 수정 (필수)** | `Indy7Ctrl.cpp:1436`: `else if (!pRobot->m_bIsoHWTrigger.load() && !pRobot->m_bRectTrigger.load() && ...)` → `!m_bIsoHWTrigger.load() &&` 항만 제거. **RECT 항은 남길 것** (T3-2 참조). 이 한 줄이 ISO·RECT가 라이브 vision goal 경로와 닿는 유일한 접점 |
| **동반 수정 (권장)** | `Indy7Ctrl.h:153` 주석 "'z' arms the hand-eye capture"는 **거짓** — 실제 arming 키는 `'w'`(`:715-745`). ISO 삭제로 `'z'`가 해방되므로 이 줄 정정 |

추가로 확인된 사실 2개(문서 정정용): ① `proc_keyboard_control:1735`는 소문자 `'q'`만 가로챕니다 → **Shift+Q는 지금도 `DoInput case 'Q'`에 도달**합니다. 즉 ISO HW 테스트는 "도달 불가"가 아니라 "도달하지만 `m_bVerifyDone`에서 hang" 상태입니다(`eInverseKinematics_6dof` 모드가 `UpdateTrajectory()+ComputeComputedTorque()`로 dispatch하여 `CheckIKConvergence`를 절대 호출하지 않음, `FullDynControllerRT.cpp:244-250`). 삭제는 **버그 제거**이기도 합니다. ② `'z'/'Z'`는 대소문자 모두 가로채져 정말로 `DoInput`에 도달하지 않습니다.

규모: **약 285 LOC (C++) + 278 LOC (Python) + 28 KB**

### T2-2. `ComputeInverseKinematics_6dof()` (legacy IK)

- 액션: `FullDynControllerRT.cpp:1929`부터 닫는 중괄호까지(≈`:2006`, 바로 다음 함수 `ComputeJacobianBasedInverseKinematics`가 `:2008`) 삭제 + `FullDynControllerRT.h:517` 선언 삭제.
- 근거: repo 전체 호출부 0. 소스에 저자 본인이 `// Lagacy function`(`:1927`)로 표기. 모드 enum `eInverseKinematics_6dof`는 이 함수를 부르지 않음(위 dispatch 확인).
- 동반 확인: `CheckIKConvergence`는 **남겨야** 합니다 — `:2126`(Jacobian IK)에서 여전히 호출되고, 이게 `rt_ik_error_log/ik_accuracy_log.csv` writer 경로입니다. 삭제 후 `m_bIkTrigger`의 6-dof 소비자가 사라지고 `'i'`/Jacobian 경로만 남습니다(의도된 상태).
- **tool 영향**: `FullDynControllerRT.o`가 재생성되므로 `make reachmap && make fk_replay` 재링크 필수. 삭제 함수는 tool이 호출하지 않으므로 링크는 통과합니다.
- 규모: **약 79 LOC**

### T2-3. `proc_terminal_output` (본체 전체가 주석인 RT task) — **가장 위험, 반드시 단독 커밋**

`CRobot::InitRTTasks`(`CRobot/Robot.cpp:117-125`)가 `m_vTaskFunctions.size() != nTotalRtTasks`면 진단 없이 `FALSE`만 반환하고, 이후 인덱스를 cfg 섹션과 **위치로 매칭**합니다. 아래 6개를 한 번에 다 해야 합니다.

1. `Indy7Ctrl.cpp:1762-1813` 함수 본체 삭제 (확인: `wait_next_period(NULL)` 외 전부 주석)
2. `Indy7Ctrl.cpp:26` forward 선언 삭제
3. `Indy7Ctrl.cpp:65` `AddTaskFunction(&proc_terminal_output);` 삭제
4. `Indy7Ctrl.h:95` `friend void proc_terminal_output(void*);` 삭제
5. `INDY7.cfg:54` `NO_OF_TASKS=5` → `4`
6. `INDY7.cfg:82-88` `[TASK3] Terminal Printer` 삭제 **+ `[TASK4] Logger`(`:89-93`)를 `[TASK3]`으로 번호 변경**. 같은 김에 `[TASK5]`(`:96-100`, Visual Servo, `ENABLED=0`이며 `NO_OF_TASKS=5`가 TASK0..4만 커버하므로 이미 **읽히지 않는 텍스트**)도 삭제.

주의: `INDY7.cfg`는 지금 **유일한 로컬 수정 tracked 파일**(`M App/Indy7/INDY7.cfg`)이며 `[TASK0]/[TASK1]`의 `CPU=3`(D10 격리 배치)이 들어 있습니다. 편집 전 백업 필수.

규모: **약 67 LOC + RT 스레드 1개 회수**

### T2-4. `calib_data/`

- 액션: `git rm -r /home/ubuntu/RAON-RT-Revision/App/Indy7/calib_data`
- 동반: `CalibCapture.h:13`의 default ctor 인자 `"calib_data"` (호출부 없음, `CalibCapture.cpp:9`가 `mkdir`하므로 자기치유) — 문자열만 `App/CalibUtils/kv260`으로 맞추거나 default 인자 자체를 제거. `summary.md:161` 문서 참조도 정정.
- 근거: 유일 생성자는 `Indy7Ctrl.cpp:16`에서 `"/home/ubuntu/RAON-RT-Revision/App/CalibUtils/kv260"` 절대경로로 구축, `SaveRobotPose`(`:1932-1933`)도 동일 경로.
- 규모: **8 KB** (169 B CSV) — 이득이 작으니 gripper 재캘리브 스윕 때 묶어도 무방 (아래 이견 정리 #9)

**TIER 1+2 합계: C++ 약 528 LOC, Python 약 484 LOC, 디스크 약 163 MB (162 MB는 git-tracked).**
소스 자체는 Indy7Ctrl 파일쌍 기준 약 15~18% 감소, `mlockall` 대상 per-object 데이터 약 1.3 KB 회수, RT 스레드 1개 감소.

---

## TIER 3 — DEFER (오늘은 미사용으로 보이나 tracking / gripper / baseline 용도)

| 대상 | 왜 남기나 |
|---|---|
| **T3-1 `VisualServo.cpp` (234) / `VisualServo.h` (50)** | 빌드에서 빠져 있고 ViSP 헤더도 이 보드에 없어 컴파일 자체가 불가 — 전형적 스윕 대상이지만, `Indy7Ctrl.h:22-25`에 "remain in-tree (unbuilt) as **closed-loop reference**"라고 보존 의도가 명시. 그리고 다음 기능이 정확히 그 closed loop입니다. 헤더가 특히 값집니다: `State{IDLE,TRACKING,TAG_LOST,SINGULARITY}`, `m_buf[2]/m_latest/m_valid` double-buffered SPSC slot, `lostFrameThreshold`, `NotifySingularity()`. cpp 쪽엔 TAG_LOST hold(`:195-206`), EMA α=0.15 goal filter(`:213-222`), singularity→grav-comp degradation(`:130-137`). 9 KB에 tracking 상태모델 원본이 들어있음 |
| **T3-2 RECT 클러스터** (`Indy7Ctrl.cpp:631-677`, `:1301-1320`, `Indy7Ctrl.h:141-147`) | **두 감사자 정면 충돌 항목.** 남깁니다: 이 앱에서 **IK6dof 모드를 유지한 채 타이머로 re-target 하는 유일한 선례**(비-RT에서 N waypoint 사전 해결 → RT 루프가 2 s마다 `StartJointTrajectory`)이고, 그게 저속판 tracking 루프 그 자체입니다. 또 `m_bRectTrigger`는 `:1436` vision goal guard의 인터록입니다. 단, T1-12의 write-only 잔재는 지금 제거. tracking 루프를 실제로 작성한 뒤 "패턴 복사 완료" 시점에 폐기 |
| **T3-3 perf-print-block** (`Indy7Ctrl.cpp:1160-1162` 지역변수 + `:1616-1638` 주석 블록) + `RTPerformance`/`GetPerformance`/`ResetPerformance`/`CheckRTViolation` | 15 Hz closed loop는 이 앱에서 **처음으로 1 kHz 예산을 실제로 위협**하는 작업입니다. 재활성화 비용 = 주석 8줄 해제. 26 LOC 아끼자고 계측기를 버리는 건 손해 |
| **T3-4 ecat-timing-stats** (`Indy7Ctrl.cpp:1649-1652`, `:1674-1694`) | 소비자가 전부 주석인 건 사실(확인함: `tmMaxPeriod`/`tmMaxResp` non-comment reader 0). 그러나 E34(IgH master priority inversion)가 막 끝난 직후이고, tracking 부하가 들어가면 EtherCAT period/response jitter가 첫 번째 의심 대상입니다. 비용은 cycle당 `read_timer()` 1회(수십 ns). 남기고 주석만 풀 것 |
| **T3-5 키 `'d'` (`:710-714`)** | `LogDistanceError(m_Pose)`의 **유일한 진입점**. goal_tcpPose 대비 task-space pos+rpy 오차를 그대로 찍어주는 기성 tracking-error readout |
| **T3-6 키 `'m'` (`:619-630`) / `'a'` (`:771-802`)** | 이 둘이 `SetTargetPose()`(full-pose IK re-target entry)의 **유일한 실행 수단**이고, `'a'`는 `StartRefine()`까지 태웁니다. 지우면 re-target 경로를 손으로 때려볼 방법이 사라짐. `'a'`의 하드코딩 좌표는 tracking 리그 세팅 때 갱신 대상 |
| **T3-7 키 `'i'` (`:612-618`)** | 명시적 제외. `eInverseKinematics` 모드 + `m_bIkTrigger` + `ComputeJacobianBasedInverseKinematics`(`FullDynControllerRT.cpp:2008`)로 가는 앱 내 유일 통로. **다만 결함 수정 필요(삭제 아님)**: `'i'`는 마지막 approach 또는 `Init`이 남긴 `goal_tcpPose` 위치(Init 하드코딩 0.24/-0.19/1.33)를 그대로 목표로 삼아 최대 0.5 rad/s로 **실제로 움직입니다** |
| **T3-8 `SetTargetPose_Jacobian()`** (`FullDynControllerRT.cpp:1472`) | 호출부 0 = 순진한 스윕의 1순위 희생자. 그러나 이게 이번 기능의 **최대 자산**: manipulability 적응 DLS damping, 8 cm 내 z축 attitude alignment, angular-rate 0.30 rad/s clamp, joint accel 2.0 rad/s² clamp, **비누적 q_ref**(`m_Q + qd*dt`, 드리프트 없음) + `MAX_REF_LEAD` 0.10 rad leash. 이 비누적 형태가 정확히 **E17 revert 이후의 안전판**입니다. 처음부터 다시 짜는 게 곧 E17 재현 |
| **T3-9 servo working set** (`FullDynControllerRT.h:342-351`: `m_J`, `m_JJt`, `m_J_pinv`, `m_e_task`, `m_Qd_ref_prev`, `m_Kp_task_pos/rot`, `m_lambda`, `m_dt`) | T3-8/`ComputeJacobianBasedInverseKinematics` 전용 사전할당 버퍼. 멤버 단위 스윕이 servo state 전체를 날림. allocation-free가 1 kHz RT-safety의 근거 |
| **T3-10 `plot_tcp_trajectory.py`** (top-level, 57 LOC) | `tools/approach_plot.py`는 `approach_log.csv` 행과 **페어링된** DataLog만 그림(자체 docstring `:366-368`: "Manual 'l'/'a' logs have no matching row and stay unpaired"). 즉 수동 트리거 로그의 유일한 플로터. tracking bring-up 중 가장 많이 찍을 로그 유형이 바로 그것. rt_log_results 쪽 쌍둥이와 달리 CSV를 move 하지 않아 무해 |
| **T3-11 `rt_ik_error_log/plots/` + `plotted_csv/`** (397 KB) | **부모 디렉터리와 `plot_ik_accuracy.py`는 절대 삭제 금지**(라이브 writer: `FullDynControllerRT.cpp:492-494`, 상대경로 + `run.sh:9`의 `cd`). 아카이브 2개는 per-solve IK 정확도의 유일한 pre-tracking baseline이고, 그 로그를 쓰는 두 진입점 중 하나가 15 Hz에서 두들겨 맞을 Jacobian IK입니다. tracking이 자기 baseline을 만든 뒤 폐기 |
| **T3-12 `indy7_v2.urdf`** (371 LOC / 15 KB) | 단순 사본 아님 — 링크별 mass가 전부 다름(5.84766553 vs 6.11383823 …), v2는 tcp가 빈 링크라 mass 엔트리 8 vs 7. **in-repo 유일 대체 inertial set**이고, 3-finger gripper payload 재튜닝이 곧 그 실험입니다. 참조처 `App/Indy7_ROS2/INDY7.cfg:8`은 tracked sibling app (91 tracked files) |
| **T3-13 EOAT/gripper 경로** | `CRobot/SensorNRMKEndTool.{h,cpp}`, `CRobot/SensorFT.cpp`, `INDY7.cfg [AXIS6] EOAT`, `Indy7Ctrl.cpp:478-490` 열거. 명시적 future work |
| **T3-14 `CRobot/ExtInterface.cpp`** | `INDY7.cfg:4 EXTERNAL_INTERFACE_ENABLED=1` → 부팅 시 실제로 뜸. `SUBCMD_SET_POS` 파싱까지 되어 있고 `CRobot::OnRecvAxisCommand`에서 큐 push만 주석 처리된 반쯤 지어진 streaming 경로. 삭제하면 부팅 동작이 바뀜 |
| **T3-15 `obj/` , `bin/*.out`** | `obj/`는 3개 Makefile 타깃의 prerequisite. 게다가 `bin/reachmap.out`(Jul 29)·`bin/fk_replay.out`(Jul 30)이 `obj/FullDynControllerRT.o`(Jul 31)보다 **오래됐습니다** — 즉 obj/를 지우면 검증된 99.4% coverage run과 동일 리비전으로 재링크할 수단이 사라집니다 |

---

## TIER 4 — KEEP (반박됨: 실제로 라이브)

| 대상 | 무엇이 참조하는가 (1줄) |
|---|---|
| `DataRecorder.h` | `Indy7Ctrl.h:118` `LogRingBuffer m_logBuffer`, `ST_LOG_ENTRY` producer `Indy7Ctrl.cpp:1221` / consumer `:1821` — per-approach DataLog 그 자체 |
| `WorkspaceBox.h` | `ROS2PickBridge.h:27` include **및** `tools/reachmap.cpp:38` (rclcpp를 피하려고 일부러 이 헤더를 직접 include) |
| `Controller.cpp/.h` | `CControllerFullDynamicsRT`의 base(`FullDynControllerRT.h:12`) + `REACHMAP_OBJS`에 명시 → 지우면 앱·reachmap·fk_replay 전부 사망 |
| `CalibCapture.cpp/.h` | `Indy7Ctrl.cpp:16` 인스턴스화, `'w'` gate 후 `'s'` 핸들러(`:747-755`)에서 호출. 이미 de-ViSP 완료 |
| `rt_ik_error_log/` (부모) + `plot_ik_accuracy.py` | `FullDynControllerRT.cpp:492-494` 상대경로 writer |
| `approach_results/` | `run.sh:29` mkdir + watcher, `ROS2PickBridge.cpp:288-291` append |
| `rt_log_results/` (top level, 138 CSV) | `tools/approach_plot.py`의 라이브 입력 |
| `bin/gate2b_bridge_test.out` | **반박**: `tools/test_gate2b_bridge.py:42-43`이 이 경로를 하드코딩하고 `:144-147`에서 없으면 `FATAL`. 삭제가 아니라 **재빌드**가 정답(현 바이너리 Jul 26 21:36 < `obj/ROS2PickBridge.o` Jul 29 16:17 → 지금 stale 코드를 테스트 중) |
| `CRobot/SensorFT.cpp` | `CSensorNRMKEndTool : public CSensorFT`(SensorNRMKEndTool.h:15). F/T 하드웨어는 떼어냈지만 **클래스 계층이 살아 있어** 매 RT cycle `m_pEcatSensor[0]->LED_*()`가 이 계층을 통과. Makefile SOURCES(`:114`)에도 존재 → 지우면 링크 실패 |
| `CROS2PickBridge::HasLockedTarget()` (`ROS2PickBridge.h:94`) | 앱 호출부 0이지만 `tools/gate2b_bridge_test.cpp:74`가 호출. App/Indy7로 스코프를 한정한 스윕은 이걸 죽었다고 오판함 |
| `ToolPosAt()` (`FullDynControllerRT.cpp:659`) | 앱 호출부 0, `tools/fk_replay.cpp:154-155`가 호출 (확인함) → 지우면 `make fk_replay` 실패 → per-approach 궤적 그래프 사망 |
| `StartCollect`/`TickCollect` 통계 게이트 | 라이브. **단 tracking의 안티테제**: 15 프레임 수집 후 축별 std > 8 mm면 "object/camera moving?"으로 거부. tracking은 이걸 **삭제가 아니라 `SetGate()`로 파라미터화/우회**해야 함 |
| `TickLockWatch` + `LOCK_MAX_AGE_S`/`LOCK_LOST_DEBOUNCE_S` | 라이브. detector flicker를 모션 명령으로 만들지 않는 유일한 방어 |
| `'i'`,`'l'`,`'w'`,`'s'`,`'p'`,`'v'`,`'k'`,`'b'`,`0-9`,`'h'/'j'`,`'e'/'x'`,`'r'`,`'g'` | 전부 현행 워크플로 또는 안전 인터록 |

---

## 부가 정리 (삭제 아님, 같이 고치면 좋은 것)

1. `App/Indy7/Makefile:195` `.PHONY: all clean gate2b_test reachmap` — **`fk_replay` 누락**. App/Indy7에 `fk_replay`란 이름의 파일/디렉터리가 생기면 타깃이 조용히 죽음.
2. `/home/ubuntu/RAON-RT-Revision/.gitignore:1-4` — `App/Indy7/raim_csv/` 규칙 3줄이 죽은 규칙(디렉터리 없음).
3. `Indy7Ctrl.h:153` — `'z'` 주석 거짓(T2-1 참조). `Indy7Ctrl.h:233` — "jieun — ViSP / Calibration" 배너가 오해 유발(CalibCapture는 `b3b8209` 이후 ViSP-free). `Indy7Ctrl.cpp:1213-1215` ViSP tombstone 주석은 60줄 아래 라이브 bridge 블록으로 대체됨.
4. `main.cpp:19/25` `CExtInterface *g_cExtInterface;`는 NULL 대입 후 재사용 없음 — 다만 T3-14와 얽히므로 ExtInterface 결론이 날 때 같이 처리.

---

## 권장 실행 순서

`make clean` 절대 금지. 각 단계 후 `make`(앱 재링크)를 최소 게이트로 둡니다.

| 단계 | 내용 | 사이에 돌릴 것 |
|---|---|---|
| **0. 기준선** | `make` 성공 확인, `md5sum bin/*.out` 기록, `cp INDY7.cfg INDY7.cfg.bak`, `git status` 클린 확인(현재 `M INDY7.cfg` 하나) | — |
| **1. 데이터 purge** (빌드 무관) | T1-1, T1-2, T1-3 (`git rm -r`). T1-3은 settle-window 지표를 `tools/approach_plot.py`로 이식한 **뒤** | 없음. 커밋 후 `git status` 만 확인. 163 MB 중 163 MB가 여기서 회수됨 |
| **2. C++ 미세 삭제** | T1-4 ~ T1-12 (키 y/u/o, 주석 i, WriteDataLog 일체, 3개 stub, triangle/ecatCycle 멤버, RECT write-only 잔재) | `make` → **servo-off smoke**: `./run.sh` → `r`(controller) → `g`(grav-comp) → `s`(pose print) → `l`(24 s 로그) → `q`. `rt_log_results/DataLog_*.csv` 생성 확인. **`h` 누르지 말 것** |
| **3. ISO 클러스터 (원자적)** | T2-1 전부 + `:1436` guard 항 제거 + `iso_csv/` `git rm -r` + `Indy7Ctrl.h:153` 주석 정정 | `FullDynControllerRT.cpp`가 바뀌므로 **`make && make reachmap && make fk_replay`**. 그 뒤 reachmap을 이전과 같은 파라미터로 1회 돌려 **coverage 99.4% / BRANCH 14가 유지되는지** 확인(변하면 무언가 잘못 지운 것). smoke 재실행 |
| **4. legacy IK 제거** | T2-2 (`ComputeInverseKinematics_6dof`) | 3단계와 동일한 3-타깃 빌드 + reachmap 회귀 + smoke. 추가로 `i` 키 경로가 여전히 컴파일되는지(모드 dispatch) 확인 |
| **5. 잡정리** | T2-4 (`calib_data/` + `CalibCapture.h:13`), `.PHONY`에 `fk_replay` 추가, `.gitignore` raim_csv 규칙 삭제, `Indy7Ctrl.h:233` 배너 문구 수정 | `make` |
| **6. `proc_terminal_output` — 단독 커밋, 마지막** | T2-3의 6개 수정 전부 | `make` 후 **반드시 실기 부팅**: `Init()`이 진단 없이 `FALSE`만 반환하므로 "그냥 안 뜨는" 실패 모드임. 확인 항목 = 앱 기동 성공 / RT task 4개 기동 로그 / `l` 로그가 여전히 파일로 떨어짐(= Logger가 `[TASK3]`으로 정상 매핑됨). 실패 시 `INDY7.cfg.bak` 복구 |
| **7. 후속** | `make gate2b_test`로 stale 하네스 재빌드 후 `python3 tools/test_gate2b_bridge.py` 통과 확인 | — |

각 단계는 개별 커밋으로. 특히 3·4·6단계는 되돌릴 때 단독으로 revert 가능해야 합니다.

---

## 두 감사자가 갈린 항목 + 내 tie-break

| # | 항목 | files/dirs 담당 | Indy7Ctrl 담당 / preservation lens | 내 판정 |
|---|---|---|---|---|
| 1 | `iso_csv/` | **DEFER** — `FullDynControllerRT.cpp:2255`에 라이브 `system()` 호출이 이 디렉터리를 가리킴 | **DELETE** — ISO 코드 전체가 죽음(+hang 버그) | **TIER 2, 코드와 원자적 동시 삭제.** 둘 다 옳음. `system()`은 `RunISOCubeIKValidation()` 내부에 있어 함수와 같이 사라짐. 디렉터리만 먼저 지우면 files 담당 말대로 dangling `system()`이 남으므로 **순서가 아니라 원자성이 핵심** |
| 2 | RECT 클러스터 | (도메인 밖, "load-bearing 여부 재확인 요망") | 담당: DELETE / lens: dormant, "패턴 먼저 복사 후 폐기" | **TIER 3 DEFER.** IK6dof 유지 상태에서 timer 기반 re-target 하는 유일 선례 = tracking 루프 골격. 단 **write-only 잔재(`m_eRectState`, `eRECT_*`)는 지금 TIER 1로 제거**. 참고로 감사자 주장대로 RECT는 로깅 경로를 **소유하지 않고 arm만 함**(`:1307` 한 줄) — 지시사항 #5의 우려는 확인 결과 "공유 블록은 `:1217-1249`/`proc_logger`/`make_csv`이고 RECT 삭제와 무관" 으로 해소됨 |
| 3 | perf-print-block (`:1616-1638`) | — | 담당: DELETE(1순위 저위험) / lens: **KEEP**(8줄 uncomment로 RT 예산 계측) | **TIER 3 KEEP.** lens 승. 26 LOC 대 15 Hz 루프의 1 kHz 예산 계측기 |
| 4 | ecat-timing-stats | — | 담당: DELETE(cycle당 죽은 산술) / lens 언급 없음 | **TIER 3 KEEP.** E34 직후이고 비용이 `read_timer()` 1회. 다만 담당 주장(비-주석 reader 0)은 사실 확인됨 — 남기되 주석을 풀어 실제로 쓰는 쪽을 권장 |
| 5 | `VisualServo.{cpp,h}` | **KEEP** (보존 의도 명시, closed-loop 레퍼런스) | lens: status **"dead"** (미빌드) | **TIER 3 DEFER.** 9 KB에 TAG_LOST hold / EMA goal filter / SPSC double-buffer / singularity degradation 원본. lens도 본문에서 `Indy7Ctrl.h:22-25` 보존 의도를 인용하고 있어 실질적으로는 keep 쪽 |
| 6 | `indy7_v2.urdf` | **KEEP** (tracked sibling app 참조 + 대체 inertial set) | 후보 제출자: DELETE | **TIER 3 DEFER.** 링크별 mass가 전부 다른 별개 dynamic model이고 gripper payload 재튜닝의 A/B 대상. 다만 이 목록에서 **가장 약한 keep** — sibling app을 나중에 정리하면 같이 사라져도 됨 |
| 7 | `bin/gate2b_bridge_test.out` | **KEEP** (반박: `tools/test_gate2b_bridge.py:42-43`이 하드코딩, 없으면 FATAL) | 후보 제출자: DELETE(stale) | **TIER 4 KEEP + 재빌드.** stale이라는 관찰은 맞지만 처방이 다름: `make gate2b_test` |
| 8 | `plot_tcp_trajectory.py` (top-level) | **DEFER** (수동 `l`/`a` 로그의 유일 플로터) | 후보 제출자: DELETE(approach_plot로 대체됨) | **TIER 3 DEFER.** `approach_plot.py:366-368`이 스스로 "unpaired manual logs"를 안 그린다고 명시 → 대체 주장이 부분적으로 틀림 |
| 9 | `calib_data/` | **DEFER** (gripper 재캘리브 때 묶어서) | 후보 제출자: DELETE | **TIER 2 (지금 해도 됨).** 유일 결합이 자기치유되는 default 인자뿐이라 위험 0. 다만 이득이 8 KB라 "gripper 스윕 때 묶기"도 합리적 — 5단계에 넣되 생략해도 무방 |
| 10 | `rt_ik_error_log` 아카이브 2개 | **DEFER** (pre-tracking IK 정확도 baseline) | 후보 제출자: DELETE | **TIER 3 DEFER.** 397 KB이고, 그 로그의 writer 중 하나가 곧 15 Hz로 돌 Jacobian IK. tracking이 자기 baseline을 만든 직후 폐기 |
| 11 | `obj/` | **KEEP** (3개 타깃의 prerequisite, tool 바이너리가 .o보다 오래됨) | 후보 제출자: 3.3 MB 회수 가능 | **TIER 4 KEEP.** 재빌드 시 검증된 99.4% reach map과 다른 controller 리비전으로 tool이 링크될 위험이 실질적 |

---

## 마지막 경고 3가지

1. **`Indy7Ctrl.cpp:1436` 한 줄이 ISO·RECT가 라이브 vision goal 경로와 닿는 유일 지점**입니다. ISO만 지우고 RECT를 남기므로, 이 줄은 **`m_bIsoHWTrigger` 항만** 제거해야 합니다. 통째로 지우면 RECT 실행 중 vision goal이 동시에 소비됩니다.
2. **HOME 재기록 금지** — 이번 정리 중 `b`로 HOME을 다시 잡으면 검증된 reach map(99.4%)이 무효화되어 3·4단계의 회귀 판정 기준이 사라집니다.
3. `s` 키는 `w`로 arm된 상태에서 **캘리브 세트를 덮어씁니다**. smoke run에서 `w`를 누르지 마세요.