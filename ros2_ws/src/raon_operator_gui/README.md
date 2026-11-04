# raon_operator_gui — 데스크톱 오퍼레이터 스테이션 (C++ / RViz 임베드, 단일 창)

KV260 pick&place 파이프라인을 **한 창에서** 보고 조작하는 **C++ Qt** 콘솔.
(2026-08-04 Python/PyQt5 → C++ 전환 — RViz를 창 안에 임베드하기 위함.
RViz 임베드는 C++ 전용이라 이 전환이 유일한 경로였다.)

**탭 4개** (2026-08-05): 창 전체가 QTabWidget이고, 운전 탭이 아닌 탭에는
전용 **정지 줄**(비상 정지 'e' + 동작 정지 'x', 같은 게이팅)이 상단에 있다 —
탭 전환이 정지 버튼을 가리면 안 되기 때문.

| 탭 | 내용 |
|---|---|
| **운전** | 기존 단일 창 구성: 좌상 bbox 영상(~30 fps) · 좌하 **임베드 RViz**(RobotModel, Orbit, 마우스 orbit/줌) · 좌하단 로봇 메시지 로그 · 우측 해상도/FPS · 상태 배지 · **게이팅되는 키 버튼 19개** + **제어 모드 dropdown**(현재 모드 표시 + g/f/c 전환 — UiCtrlNG.py 스타일) + 객체 선택 |
| **Workspace** | **탑다운 뷰**(TopDownOrtho — 같은 scene의 두 번째 RenderPanel, §4) + workspace box/rmax 마커 + **TCP 궤적 trail**(주황, 최근 ~60 s) + 하단 **TCP 숫자** x/y/z/R/P/Y (TF `world→tcp`, 5 Hz) |
| **Joint** | UiCtrlNG.py식 축별 행 `[Jn][−][슬라이더][+][Target][Actual]` — **슬라이더를 놓으면 그 축을 jog**(§jog 채널). Actual은 limit 근접 색 경고(−45° 주황/−15° 빨강) |
| **게인 (관리자)** | `--admin`(launch `admin:=true`)일 때만 존재. Kp/Kd/KF 6×3 편집 + 로봇 readback('현재' 열) + 적용 — §게인 채널 참고 |

실행 (PC):
```bash
ros2 launch raon_operator_gui operator.launch.py             # 3탭
ros2 launch raon_operator_gui operator.launch.py admin:=true # +게인 탭
```
launch가 콘솔 + `robot_state_publisher` + static TF 3프로세스를 띄우지만 **창은
하나다.** robot_state_publisher 없이는 RViz RobotModel이 TF가 없어 아무것도
안 그리므로 launch로 띄우는 것이 표준 경로다. (`--no-rviz`로 콘솔만 단독 실행
가능 — GL 없는 환경/디버깅용.)

> **⚠️ 보드가 아니라 PC에서 돌린다.** 디코드+그리기 비용을 보드에 올리면
> 15 Hz 예산이 깨진다. 보드는 압축만 한다.

> **⚠️ 실기 검증 이력 없음.** 아래 검증은 하네스/오프스크린 수준. 첫 실기는
> 서보 OFF에서 `s`(TCP 출력) 같은 무해 버튼 → 로봇 터미널 로그 일치부터.

---

## 1. 파일 지도

| 파일 | 역할 |
|---|---|
| `src/state.hpp` | **순수 로직**: State 구조체 + `keyEnabled()`(게이팅) + JSON 파서(kp/kd/kf readback 포함). Qt/ROS 무관 → `test_logic`이 그대로 검증 |
| `src/ros_link.{hpp,cpp}` | ROS 노드: 구독(영상/검출/state) + 키/게인 발행 + TF(world→tcp) lookup + workspace 마커 1회 발행. JPEG 디코드/bbox는 executor 스레드에서 끝내고 완성된 QImage만 GUI로 |
| `src/main_window.{hpp,cpp}` | 탭 4개 레이아웃 + 버튼 + 게이팅 적용(탭별 정지 줄 포함) + 상태 배지 + 링크 워치독 + TCP 숫자 폴링 |
| `src/video_pane.{hpp,cpp}` | 영상 위젯 (BGR888, 채널 스왑 금지 — 현장 검증) |
| `src/joint_pane.{hpp,cpp}` | Joint 탭 — 축별 슬라이더 jog(릴리즈 시 `/operator_jog`) + Actual 표시. 게이트는 브리지 OnJog의 거울. limit 상수는 URDF와 동기 주석 |
| `src/gains_pane.{hpp,cpp}` | 게인 탭: 6×3 스핀박스 + readback + 적용(readback 존재 && 유휴일 때만) |
| `src/rviz_pane.{hpp,cpp}` | **RViz 임베드** — `RvizPane`(운전 탭, **유일한** VisualizationManager 소유 + Grid/RobotModel/마커/trail display) + `RvizTopDownPanel`(Workspace 탭 — 같은 scene의 두 번째 RenderPanel, §4). rviz를 만지는 유일한 파일 |
| `src/main.cpp` | 진입점: Qt 메인 스레드 + executor 스레드(우리 노드 + rviz 노드) |
| `operator.sh` | **PC 실행 스크립트** — 실행/`--update`(scp+클린빌드)/인자 전달 |
| `launch/operator.launch.py` | 콘솔 + robot_state_publisher + static TF. `urdf:=` 인자 |
| `urdf/indy7_mesh.urdf` | **기본.** 생성물 — 실제 STL 셸 (kinematics는 로봇 앱과 동일) |
| `meshes/indy7/visual/` | vendored Indy7 STL 7개 (≈9.4 MB) |
| `urdf/make_mesh_urdf.py` | 위 생성기 — 로봇 kinematics 바뀌면 재실행 |
| `urdf/indy7_viz.urdf` + `make_viz_urdf.py` | mesh 없는 폴백 URDF + 생성기 |
| `test/test_logic.cpp` | 게이팅+JSON assert 스위트 |
| `test/test_gui_smoke.cpp` | 실창 offscreen: 버튼 32개 클릭→발행 확인 + 게이팅 5단계 |

### 지켜야 할 규칙 하나

**ROS 콜백(executor 스레드)에서 Qt 위젯을 절대 직접 건드리지 않는다.** 유일한
통로는 `RosSignals`의 Qt signal(queued connection). 반대 방향 `sendKey()`는
예외 — rclcpp publish는 thread-safe이고 Qt를 안 만진다.

---

## 2. 토픽 (Python 시절과 동일 — 로봇/보드 쪽 계약 불변)

| 방향 | 토픽 | 비고 |
|---|---|---|
| 구독 | `/camera/.../compressed` | BEST_EFFORT depth1. lazy 인코딩(구독자 없으면 보드 비용 0) |
| 구독 | `/detections` | RELIABLE depth1 |
| 구독 | `/robot_state` (String JSON) | 로봇 앱 발행 20 Hz. 관절값+게이팅 플래그 |
| 구독 | `/operator_msg` (String) | 로봇 앱의 오퍼레이터용 터미널 라인 미러 — `p` 메뉴/게이트 판정/LOCK/GOAL. RELIABLE depth20(이벤트 유실 금지) |
| 발행 | `/operator_key` (String 1글자) | RELIABLE depth1 |
| 발행 | `/operator_gains` (String JSON, **관리자 탭**) | RELIABLE depth1 — `{"kp":[6],"kd":[6],"kf":[6]}`, 공백 없는 콘솔 snprintf 포맷이 계약(브리지 파서와 양단 소유) |
| 발행 | `/operator_jog` (String JSON, **Joint 탭**) | RELIABLE depth1 — `{"axis":N,"q":rad}` 절대 목표. 최신 명령 승리(드래그는 스트림) |
| 발행 | `/operator_ws_markers` (MarkerArray) | transient_local, 시작 시 1회 — workspace box + rmax 링. 상수는 로봇의 `WorkspaceBox.h` 복사본(한쪽 수정 시 다른 쪽도) |
| 발행 | `/operator_tcp_path` (nav_msgs/Path) | TCP 궤적 trail — 5 Hz TCP 폴링이 성공할 때마다 최근 ≤300점(≈60 s) 전체 재발행. RViz Path display가 소비 |
| (RViz) | `/joint_states`, `/tf`, `/robot_description` | rsp 경유, 임베드 RViz가 소비. `/tf`는 콘솔의 tf2 listener도 소비(TCP 숫자 + trail) |

---

## 3. 버튼 → 로봇, 게이팅

버튼은 문자 1개 발행뿐. 로봇 앱 `CROS2PickBridge::OnKey` 화이트리스트 → 원자
슬롯 → `DoInput`이 로컬 키와 동일 소비(로컬 우선). **게이팅(`keyEnabled`)은
DoInput 거부 가드의 거울** — DoInput을 고치면 `state.hpp`도 같이. 콘솔이
로봇보다 앞서 판단하면 안 되고, 최종 방어는 항상 로봇 쪽.

### 정지 버튼의 실제 의미 (2026-08-04 재정의 — 실기 리포트로 수정)

| 버튼 | 동작 |
|---|---|
| `e` 비상 정지 | **모든 동작 소스 취소**(추종/접근/ISO/RECT/refine/궤적) + 전 축 **서보 OFF(브레이크)**. 재개는 'h' 재무장 |
| `x` 동작 정지 | **모든 동작 소스 취소**, 서보 유지 — **중력보상**으로 그 자리 정지 |

이전 구현은 드라이브에만 명령해서 안 멈췄다: `e`(SetEmgStop)는 QUICK_STOP을
한 번 쓰지만 `SlaveCIA402Base::WriteToSlave`의 cyclic 상태머신이
QUICK_STOP_ACTIVE → … → ENABLE_OPERATION으로 **자동 재무장**시키고, 살아 있는
컨트롤러 goal이 즉시 재개했다(짧게 멈췄다 다시 움직임). `x`(SetHalt)는
profile 모드 전용 halt 비트라 **CST(토크) 모드에서 무의미**했다.
수정: 공용 `CRobotIndy7::KillMotionSources()`가 근원을 먼저 죽인다.
`e`는 이어서 servo-off 래치(`m_bIsSetServoOnOff=FALSE` ⇒ 매 사이클 SHUTDOWN —
상태머신이 되돌릴 수 없는 상태)로 브레이크 체결. **실기 검증 필요**:
추종('o') 중 `e` → 정지 유지, `x` → 그 자리 유지 확인.

| 버튼 | 활성 조건 |
|---|---|
| `e` `x` `j` `g` `p` | state 수신 중이면 항상 |
| `r` `t` `f` `c` `i` `k` `s` `w` `l` `d` `m` `a` `n` | 추종 아닐 때 |
| `h` | ctrl + 중력보상 |
| `b` | ctrl + 서보 + HOME/ready |
| `o` | 추종 중=정지(항상) / 시작=ctrl+서보+유휴+LOCK |
| `v` | LOCK + 비추종 |
| 객체 버튼 5종 | 메뉴 열림 **AND** 해당 클래스가 `menu_items`에 존재 |
| AUTO / 취소 | 메뉴 열림 |
| HOME 기록 | **항상 비활성** (아래 전용 절) |
| **전부** | `/robot_state` 1.5 s 침묵 → 전체 비활성 + NO LINK |

**`q`·`z` 버튼 없음**: 터미널의 `q`는 proc_keyboard_control이 종료로 먹지만
`/operator_key`는 DoInput에 직접 닿아 **ISO Cube 10분 시험**을 시작한다.
브리지 화이트리스트도 차단(이중 방어).

**`p` 메뉴는 UI 로그 pane에 표시된다** (2026-08-04): 브리지의 `Say()`가
오퍼레이터용 printf를 그대로 유지하면서 `/operator_msg`로도 발행 — 메뉴,
선택 결과, TARGET LOCKED/lost, GATE FAIL/GOAL READY, 원격 키 거부까지.
단 **DoInput의 거부 메시지(RT 스레드 printf)는 예외** — RT 스레드에서 publish
금지라 터미널에만 남는다(대부분 버튼 게이팅이 선제 차단하므로 실사용 영향 적음).

### 객체 선택 (숫자 버튼의 후신, 2026-08-04)

숫자 0–9 버튼 대신 **객체 버튼**: 사과/오렌지/바나나/테니스공/머스타드병 +
AUTO + 취소 + HOME 기록. 동작 원리:

- `p`로 메뉴가 열리면 브리지가 `/robot_state`에 **`menu_items`**(메뉴 번호
  순서의 클래스 배열, `m_vMenu` 그대로)를 실어 보낸다. 콘솔은 배열에 있는
  클래스의 버튼만 활성화하고, 클릭 시 **배열 내 위치+1**을 digit으로 발행 —
  번호 매김의 출처가 ShowMenu와 동일(`m_vMenu`)이라 어긋날 수 없다.
  (콘솔이 `/detections`를 보고 스스로 번호를 추측하는 방식은 금지 — 어긋나면
  **팔이 다른 물체로 간다.**)
- AUTO = `menu_items.size()+1`, 취소 = `0` — 메뉴 열림 동안만 활성.
- 버튼은 위 5클래스뿐이다(모델의 person 클래스는 detector 단에서 발행되지 않음).

### HOME 기록 버튼 — 영구 비활성 (해제 절차 포함)

메뉴의 마지막 항목("record HOME here")에 해당하는 버튼은 **어떤 조건에서도
비활성**이다. 이유: 현재 신뢰 기준 자세는 init이 계산하는 **q_ready**(+
`~/.indy7_ready_seed`)이고, GUI에서 실수로 HOME을 재기록하면 — 팔이 어정쩡한
자세일 때 눌리면 — `'b'` 홈 복귀가 그 나쁜 자세로 가게 된다.

**체인은 UI 버튼만 빼고 전부 살아 있다**: 터미널에서 `p` → 마지막 번호
숫자키는 지금도 동작하고, 브리지 `HandleDigit`의 HOME 분기, RT의
`PopHomeRecord()` 스냅샷, `'b'`의 E20 counter-frame 안전(폴딩)까지 그대로다.

나중에 해제하려면 (`src/main_window.cpp` `buildSelectBox()`):
1. `btn_home_rec_->setEnabled(false);` 줄 삭제
2. `applyGating()` 끝에 `btn_home_rec_->setEnabled(menu_open);` 추가
3. digit 매핑은 이미 연결돼 있음(`menu_items.size()+2`) — 다른 수정 불필요.
   재빌드 후 `test_gui_smoke`의 "HOME-record dead" assert를 반대로 뒤집을 것.

**콘솔 종료 버튼**: 창을 닫으며, launch의 `on_exit=Shutdown()`이 rsp/static TF
까지 정리한다(터미널 Ctrl-C 불필요). 게이팅에서 의도적으로 제외 — 죽은 콘솔을
닫는 것은 항상 정당하다.

### 게인 채널 (관리자 탭, 2026-08-05)

키 문자 경로와 별개인 두 번째 원격 입력. 흐름과 방어선:

```
게인 탭 [적용] ──/operator_gains(JSON)──▶ 브리지 OnGains
    ▲                                      │ ① 엄격 파싱(각 배열 정확히 6개+']')
    │                                      │ ② 유휴 게이트(track/busy면 거부)
    │                                      │ ③ clamp Kp[0,2000] Kd[0,200] Kf[0,12]
    │                                      ▼ SPSC mailbox
    │                             RT 루프 PopGains → SetControlGain /
    │                                      SetFrictionFFAxis (per-axis 평범한
    │                                      store — RT에서 할당 없음)
    └── /robot_state의 "kp"/"kd"/"kf" ◀── PushState가 매 사이클 컨트롤러에서
        (20 Hz readback, '현재' 열)        읽음 (캐시 아님 — cfg Kf 포함 진실)
```

- 콘솔 쪽 게이트는 브리지의 거울: **readback이 있어야**(구 바이너리 차단) +
  링크 fresh + 유휴일 때만 [적용] 활성. 첫 readback이 스핀박스를 자동 시드 —
  손대지 않은 0을 실수로 쏘는 사고 방지.
- 적용/거부/[CLAMPED]는 브리지 `Say()` → 로그 pane에 표시된다.
- 파서는 콘솔 snprintf의 **공백 없는** JSON만 받는다(`"kp":[` — /robot_state
  파서와 같은 양단 소유 스키마 원칙). CLI로 수동 발행 시 공백을 넣으면
  "bad JSON" 거부가 로그에 뜬다.
- 검증(2026-08-05, `gate2b_bridge_test.out` 하네스 + rclpy): 정상 세트
  적용+readback 갱신, 범위 밖 9999/−5/99 → 2000/0/12 clamp, 7-원소 배열 거부,
  비JSON 거부 — 전부 PASS. **RT 루프 실적용은 실기 미검증** (아래 §7).

### jog 채널 (Joint 탭, 2026-08-05)

세 번째 원격 입력 — Joint 탭 슬라이더가 축별 **절대 목표**를 보낸다.

```
슬라이더 릴리즈 ──/operator_jog {"axis":N,"q":rad}──▶ 브리지 OnJog
                                   │ ① 파싱 + axis 0–5 검사
                                   │ ② 게이트: 모드 FullDyn(1)/CT(2) + ctrl
                                   │    + servo + 유휴(비추종·비busy)만 수락
                                   │ ③ URDF limit clamp (±175°, J6 ±215°)
                                   ▼ seqlock 슬롯 (최신 승리)
                    RT 루프 PeekJog → SetJogTarget → Update()가 q_ref를
                    목표로 0.5 rad/s(IK follow와 동일 leash)로 «걸어감»
                    — 절대 reference step이 아님. qd_ref도 같이 실어
                    damping/마찰 FF가 실제 이동으로 인식.
```

- **모드 제한의 이유**: FullDyn/CT는 q_ref를 다른 누구도 쓰지 않는 두 모드다
  (궤적은 IK6dof 모드 소유, 비전 서보는 TrackingServo 소유, 중력보상은 무시).
- **정지 체인**: 모드 변경 시 `SetControlMode`가 jog 취소 +
  `KillMotionSources`('e'/'x')가 명시적으로 `CancelJog()` — jog는 정식 동작
  소스로 등록돼 있다.
- 콘솔 게이트는 브리지의 거울(슬라이더 자체가 비활성) + 로봇이 최종 방어.
  수락된 jog는 로그에 안 찍는다(드래그=스트림) — 거부만 Say로 표시.
- 검증(2026-08-05, 하네스+rclpy): `"mode":2` readback, 수락→q 반영,
  9.9 rad→3.0543 clamp, axis 9 거부 — PASS. **실기 미검증**: 첫 실기는
  'c' 모드 + 낮은 Kp에서 ±1° 버튼부터.

---

## 4. 임베드 RViz

`rviz_pane.cpp`가 bare `RenderPanel` + `VisualizationManager`를 우리 위젯에
직접 생성한다(메뉴/툴바 있는 VisualizationFrame이 아님). 로봇은 stock
`rviz_default_plugins/RobotModel`이 그린다 — 즉 일반 rviz2와 **같은 코드
경로**: 브리지 `/joint_states` → robot_state_publisher → TF → RobotModel.

- **주의**: 이 임베드는 커뮤니티 정착 패턴이지 안정성 보장 API가 아니다.
  Humble 기준으로 컴파일·동작하며, ROS 업그레이드 시 `rviz_pane.cpp`(만)
  재점검 대상.
- **마우스 조작에는 tool이 필요하다** (2026-08-05 실보고 — 드래그 무반응):
  bare VisualizationManager는 tool이 0개다(stock rviz는 display config에서
  받는다). tool이 없으면 모든 마우스 이벤트가 handleMouseEvent에서 죽는다.
  `ToolManager::addTool("rviz_default_plugins/MoveCamera")` + setCurrentTool
  로 해결 — 드래그 orbit/휠 줌이 두 pane 모두에서 동작(각 pane의
  ViewController로 라우팅).
- **RViz 초기화는 반드시 show 이후** (2026-08-04 실크래시, backtrace 확보):
  Ogre는 initialize 시점의 네이티브 X 윈도우에 바인딩되는데, Qt는 위젯 트리를
  보여주는 과정에서 네이티브 윈도우를 재생성한다. 생성자에서 초기화하면 Ogre가
  죽은 핸들을 쥐고, 첫 resize에서 `XFree`가 쓰레기 포인터로 사망. 그래서
  `RvizPane`은 `showEvent` + `QTimer::singleShot(0,…)`으로 한 이벤트 루프 뒤에
  초기화한다.
- **rviz 노드를 우리 executor에 절대 넣지 말 것** (2026-08-04 실크래시):
  `VisualizationManager`가 자체 executor로 rviz 노드를 add_node하고 30 Hz
  타이머에서 spin_some한다(`visualization_manager.cpp:159,252,406`). 두 번째
  executor에 또 넣으면 wait set이 오염돼 기동 직후 SIGSEGV. stock rviz2의
  main이 노드를 안 도는 이유가 이것.
- **VisualizationManager는 프로세스당 정확히 1개** (2026-08-05 실크래시):
  생성자가 전역 Ogre material(`RVIZ/Red` 등)을 등록해서 두 번째 생성은
  `ItemIdentityException: Material RVIZ/Red already exists`로 즉사한다 —
  Workspace 탭을 처음 열 때 콘솔이 통째로 죽었던 원인. Workspace 탭의
  탑다운 뷰는 stock rviz의 CameraDisplay 패턴을 그대로 쓴다: **두 번째
  RenderPanel을 `initialize(manager, use_main_scene=true)`로 같은 scene에
  붙이고**, `ViewManager::create("TopDownOrtho")`로 만든 전용 카메라를
  `setViewController`로 단다(`RvizTopDownPanel`). 소유권은
  `ViewManager::add`로 manager에 넘긴다(직접 delete 금지). scene이 하나라
  Grid/RobotModel/마커/trail은 **양쪽 pane에 모두** 보인다 — 카메라만 다르다.
  각 pane은 자기 탭이 처음 보일 때 초기화된다(showEvent 지연 규칙 그대로;
  탑다운 쪽은 primary manager가 생길 때까지 100 ms 재시도).
- **실제 Indy7 외형이 기본이다 (2026-08-04)** — `Indy7_0..6.stl`(≈9.4 MB,
  neuromeka-robotics/indy-ros2 `jazzy-indyDCP3`의 `indy_description`)을
  `meshes/indy7/visual/`에 vendored, `urdf/indy7_mesh.urdf`가
  `package://raon_operator_gui/...`로 참조한다(설치만 되면 어느 머신에서든
  해석됨). 단위/프레임은 bounding box로 검증(base z 0–0.08 m = joint0 높이
  0.0775와 정합). 생성기: `urdf/make_mesh_urdf.py` — 로봇 앱 URDF에서 visual
  경로만 바꾸고 collision(-미보유)과 ros2_control을 뗀다. kinematics 변경 시
  재실행. mesh 없는 `indy7_viz.urdf`는 `urdf:=` 폴백으로 유지.
- `world`(URDF 루트) ↔ `base_link`(퍼셉션): 같은 물리 프레임, launch의 static
  TF가 잇는다.
- **stamp 없는 JointState는 rsp가 조용히 무시** — 브리지 `TickStatePub`이
  `now()`를 찍는 이유. 모델이 원점 자세로 굳으면 여기부터 의심.

## 5. 로봇 앱 쪽 (`RAON-RT-Revision/App/Indy7`)

`/operator_key` 구독+화이트리스트, `/operator_gains` 구독(OnGains — 파싱/
유휴게이트/clamp → SPSC mailbox, RT가 `PopGains`로 소비), `/operator_jog`
구독(OnJog — 게이트/clamp → seqlock 슬롯, RT가 `PeekJog`→`SetJogTarget`),
`/joint_states`+`/robot_state` 발행(20 Hz, kp/kd/kf readback +
`"mode"`=eControlMode int 포함), `/operator_msg` 발행(`Say()` —
printf+publish, **worker/콜백 스레드 전용**), RT 루프의 wait-free
`PushState`. 컨트롤러 추가분: per-axis `SetFrictionFFAxis`/`GetFrictionFFAxis`
(RT-safe, [0,12] clamp), jog(`SetJogTarget`/`CancelJog` + Update()의
0.5 rad/s 추종 — FullDyn/CT 전용, 모드 변경·KillMotionSources가 취소).
빌드 `make`, 하네스 `make gate2b_test`(게인/jog 소비+readback 시뮬 포함).
**콘솔 버튼 전부 비활성이면 1순위: 옛 바이너리(`/robot_state` 없음).**
게인 탭/Joint 슬라이더만 죽어 있으면: readback(kp·mode) 없음 = 신규 채널
이전 바이너리.

**`Global/Comm` 랩 ROS2 래퍼와의 관계**: 래퍼(CROS2Node/CROS2IndyIface)의 설계
의도 — "RT 루프는 그대로, 조작 인터페이스만 ROS2로 노출" — 는 이 브리지가
이미 같은 형태로 구현한다(`/robot_state`+`/operator_key` ↔ 래퍼의 상태 발행+
서비스 6종). 래퍼를 직접 쓰지 않는 이유: ① `indy_iface` 메시지 의존(우리는
`my_interfaces`) ② 서비스 방식은 키 문자 경로와 달리 DoInput과 GUI가 따로
놀 수 있음(현 설계의 핵심이 "GUI=터미널과 동일 경로") ③ 기능 이득 없이 재작성.
게인 조회/변경(set_gains) 같은 **새 원격 기능**이 필요해지는 시점에는 래퍼의
서비스 패턴(ROS2IndyIface)을 참고할 것.

---

## 6. PC에서 빌드 & 실행

### 버전 (검증된 조합 — Ubuntu 22.04 / ROS2 Humble)

| 항목 | 버전 | 비고 |
|---|---|---|
| **Qt** | **5.15.3** (`qtbase5-dev 5.15.3+dfsg-2ubuntu0.2`), **Widgets 모듈만** | **Qt6로 올릴 수 없다** — Humble `rviz_common`이 Qt5 빌드라 같은 프로세스 임베드는 같은 Qt ABI를 요구한다. `QImage::Format_BGR888`도 Qt≥5.14 전용(§1의 채널 스왑 금지가 성립하는 전제) |
| OpenCV | 4.5.4 (`core` `imgcodecs` `imgproc`) | JPEG 디코드 + bbox. Qt는 그림을 안 그린다 |
| rviz | `rviz_common`/`rviz_rendering`/`rviz_default_plugins` 11.2.26 | 렌더러는 Ogre3D → **OpenGL 필요**(보드에 없어서 `--no-rviz` 경로 존재) |
| 빌드 | C++17, CMake≥3.8, **ament_cmake**, `AUTOMOC ON` | Q_OBJECT 헤더를 `add_library` 소스에 같이 넣어야 moc이 돈다 |

UI는 **전부 코드**다 — `.ui`/Qt Designer/QML 없음, 스타일은 인라인
`setStyleSheet` 4개뿐. `console_core` STATIC 분리 이유는 하나:
`operator_console`과 `test_gui_smoke`가 같은 위젯 코드를 링크하기 위함.

일상 사용은 `operator.sh` 하나로 끝난다 (최초 1회 PC로 복사):

```bash
# 최초 1회
sudo apt install qtbase5-dev libopencv-dev        # 빌드 의존성
scp ubuntu@192.168.120.50:~/ros2_ws/src/raon_operator_gui/operator.sh ~/pp_ws/viewer_ws/

# 이후
cd ~/pp_ws/viewer_ws
./operator.sh              # 실행 (빌드돼 있으면 바로 launch)
./operator.sh --update     # 보드 최신 소스 scp → 클린 재빌드 → launch
./operator.sh urdf:=...    # launch 인자 그대로 전달
```

`--update`는 stale-copy 사고(§이력: viz URDF가 남아 원통이 나오던 문제)를
막으려고 src/build/install을 지우고 받으며, meshes 누락 시 빌드를 거부한다.
스크립트 자체가 바뀌었을 때만 위 scp 한 줄을 다시 실행할 것 (루트의 복사본은
--update가 갱신하지 않는다).

수동 절차(스크립트가 하는 일과 동일):

```bash
cd ~/pp_ws/viewer_ws
rm -rf build/raon_operator_gui install/raon_operator_gui src/raon_operator_gui
scp -r ubuntu@192.168.120.50:~/ros2_ws/src/raon_operator_gui src/
source /opt/ros/humble/setup.bash
colcon build --packages-select raon_operator_gui
source install/setup.bash
ros2 launch raon_operator_gui operator.launch.py   # 보드: 퍼셉션 launch + Indy7Ctrl.out
```

`my_interfaces`는 PC에 이미 빌드돼 있으면 재작업 불필요(C++ 라이브러리는 항상
같이 생성돼 있다).

### 안 붙을 때

1. **버튼 전부 비활성 + NO LINK** → `ros2 topic hz /robot_state`. 0이면 로봇
   앱이 옛 바이너리거나 미실행. 로봇 앱은 `~/ros2_ws/install` source한 셸에서.
2. **RViz 구역에 로봇 없음** → `ros2 topic hz /joint_states` 확인 후
   `ros2 run tf2_ros tf2_echo world tcp` — 안 나오면 rsp가 안 떴다(launch로
   띄웠는지). 원점 자세로 굳어 있으면 stamp(§4).
3. 영상 없음 → 보드 퍼셉션 launch.
4. DDS 환경변수 기본값/domain 0 확인.
5. 버튼 무반응 → 로봇 터미널 `[PickBridge] /operator_key:` 로그.

---

## 7. 검증 상태 (2026-08-04)

```bash
./build/raon_operator_gui/test_logic
QT_QPA_PLATFORM=offscreen ./build/raon_operator_gui/test_gui_smoke
```

| 항목 | 방법 | 결과 |
|---|---|---|
| 게이팅=DoInput 거울 + JSON (kp/kd/kf/mode 포함) | `test_logic` | PASS |
| 실창 5단계 게이팅 + 19 키버튼 발행 | `test_gui_smoke` (offscreen) | PASS |
| 탭 4/3개 · 탭별 정지 줄 · 게인 탭 게이트 · 모드 dropdown 키 게이팅 · jog 슬라이더 모드/유휴 게이트+발행 | `test_gui_smoke` 7–10단계 | PASS (2026-08-05) |
| 실바이너리 vs 실브리지 | 하네스 + `--no-rviz` offscreen 12 s | PASS (무크래시) |
| `p` 메뉴 → `/operator_msg` | 하네스에 'p' 입력 → topic echo | PASS — 메뉴 전문 수신 |
| 게인 채널 E2E (적용/clamp/거부/readback) | 하네스 + rclpy 스크립트 | PASS (2026-08-05, §게인 채널) |
| jog 채널 E2E (mode readback/수락/clamp/거부) | 하네스 + rclpy 스크립트 | PASS (2026-08-05, §jog 채널) |
| 로그 pane + 종료 버튼 비게이팅 | `test_gui_smoke` 6단계 | PASS |
| 컴파일/링크 (rviz 임베드 포함) | 보드 aarch64 colcon + 로봇 앱 `make` | PASS |
| **RViz 렌더 화면 (운전 + Workspace 탑다운/마커/trail)** | — | **PC에서 육안 확인 필요** (보드는 GL 없음). 1차 시도의 2-manager 크래시는 §4대로 1-manager+2-RenderPanel로 수정 — 재검증 필요 |
| **실기(로봇 연결, RT 게인 실적용 포함)** | — | **미검증** — 첫 실기에서 게인 탭은 현재값 그대로 [적용] → readback 불변 확인부터 |

## 8. `detection_viewer_pkg`와의 관계

별개 패키지, bbox 로직 의도적 중복(`kClassColors` ↔ `CLASS_COLORS` —
**한쪽 고치면 다른 쪽도**, 키는 반드시 `class_name`). 정렬 검증은
`detection_viewer_node --sync`. 동시 실행 금지(보드 egress 2배).

## 9. 안 한 것

- `p` 메뉴 목록 GUI 표시(stdout 파싱 필요), `/pick_target_base` 위젯(요구로 제거).
- ctest 등록 — 테스트 2개는 수동 실행 바이너리.
- 영상 렌더: 프레임 즉시(~30 fps)+최신 bbox(≤0.5 s, 초과 시 안 그림) 유지.
