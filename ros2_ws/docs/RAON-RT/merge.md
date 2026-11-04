# RAON-RT 병합 트랙 — 통합 기록 (merge.md)

> **이 문서의 목적**: `~/RAON-RT-Revision` (KIST/RAIMLAB RAON-RT 프레임워크)을 이 저장소의
> perception 파이프라인과 병합해 **"파이프라인이 인식한 테이블 위 물체 위로 Indy7 TCP가 접근하고,
> 사람이 그 물체를 옮기면 TCP가 따라간다(추종)"**를 만드는 트랙의 정본. **파지·이송·놓기는 범위 밖**
> (그리퍼 하드웨어 미보유 — §8.1). 결정·진행·게이트·버그·재개 방법을 이 한 파일로 파악한다.
>
> **작성**: 2026-07-26 (Phase 0~2a + pick_logic v2까지 완료된 시점) · 세션 기록은
> `~/.claude/projects/-home-ubuntu/memory/raon-vs-merge-plan.md`가 병행 보존.
>
> **참조 정본**:
> - RAON-RT 코드 구조/버그 전수: `~/RAON-RT_guide_for_CLAUDE.md` (348KB, §1~§14 — §13 통합지침·§14.2 차단요소)
> - 로봇제어 코드: GitHub `seriouslysilly21612/RAON-RT-Revision` **브랜치 `kv260-merge`** (로컬 `~/RAON-RT-Revision`)
> - 이 저장소(ros2_ws)는 perception 쪽 절반만 담당 — 두 저장소는 ROS2 토픽으로만 만난다.

---

## 0. 30초 요약 (2026-07-31 현재)

- **목표**: perception 파이프라인은 그대로 살린 채, `/pick_target_base`(base_link 좌표)를
  RAON-RT 제어 앱이 구독 → 오퍼레이터가 터미널 메뉴로 물체 선택 → `z+margin` 호버점으로
  **look-then-move 접근**(`'v'`) → 이어서 **사람이 물체를 옮기면 TCP가 따라가는 실시간 추종**
  (`'o'`, 2026-08-03 D15). CST 토크 + RBDL, 1 kHz. **파지·이송·놓기는 범위 밖**(그리퍼 하드웨어
  미보유) — 두 모드의 계약은 §8.1.
- **완료**: Phase 0(기반: RT-POSIX·EMasterApp aarch64) → Phase 1(버스 검증: 7슬레이브 PDO
  비트단위 일치) → Phase 2a(ViSP 제거 앱 빌드 + §14.2 픽스) → **pick_logic v2**(선택·안정성·
  person 가드[2026-09 제거], 합성테스트 6/6) → **Gate 2b**(ROS2 브릿지 + 대화형 메뉴 'p'/숫자/'v',
  합성테스트 5/5 ×2연속) → **Phase 3**(서보-오프 통합 런: 300 s 무단절 OP·정상상태 유실 0·
  파이프라인 동시·in-app 메뉴/param/'v'·PDO=SDO 조인트 대조·person_guard 실환경 실증) →
  **Phase 4**(서보온 게이트1 grav-comp + 게이트2 'a' 실모션) → **refine 160→10.9 mm** →
  **Phase 5 전체**(손-눈 캘리브 rMc residual 1.8 cm → TF/박스 정본화 → **'v' 접근 데모
  전 클래스 성공**: 접근→'b' homing→다음 물체 사이클. 1차 시도 테이블 충돌(E13)은
  soft-R IK+Δq 게이트+T 스케일링으로 해소) → **2026-07-27 주간**: F/T 페이로드 URDF
  반영(공구축=−X 함정 E16), sticky-float(속도게이트 완화판 오퍼레이터 OK), 서보-오프
  goal 가드(E15), radial 게이트, refine oriWeight=0 회귀픽스(E18) → **ready-seed IK**
  (`19b15ba`: q_ready 고정 시드로 접근 해가 시작 자세와 무관하게 결정적 + 2π 폴딩 +
  자동 스테이징 — 당시 실기 검증 대기, 같은 날 20~22시 **v3로 검증 완료**(다음 항목)).
- **20~22시 실기 — E21/E22/E23 연쇄 복구 완결**: v3 정상(괴랄 궤적 소멸) → 도달점이 물체를
  ~11 cm 벗어남 = **E21 로봇 베이스 물리 이동**(프로브 fit yaw +5.14°/t(+4.5,+10.4) cm →
  launch TF 재베이스) → 게이트 상수도 옛 프레임 유물(E22, 박스·r 재베이스) → 원거리 soft-R
  정체(E23, w 사다리+60° tilt 캡) → **전 5클래스 접근 성공**(refine 4.9~10.6 mm; 원거리
  3종은 tool 41~48° tilt로 도달 — 수직-hover 한계 표시).
- **22~23시 D10 3+1 격리 배치**: cmdline `isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2`
  적용·재부팅, cfg `[TASKn] CPU=` 키로 1 kHz 페어(RT97/RT95)를 CPU3 단독 점유, 나머지 15개
  비RT 스레드 0-2 — /proc 토폴로지 확인 완료, EtherCAT OP·2000 f/s·Lost 2(재베이스).
  **순수 지터 A/B 완결(07-28 17:02, 앱 정지 상태에서 측정)**: 격리 CPU3 avg 4.3 / max
  **51.1 µs** vs 비격리 CPU0 avg 7.3 / max **114.6 µs** — 최악 지터 절반 이하, 둘 다 PASS.
  (엄밀히는 "격리 유무"가 아니라 "격리 코어 vs IRQ·비전이 도는 코어"의 비교. 순수 분리를
  재려면 재부팅이 필요해 그만한 가치는 없다고 판단.)
- **2026-07-28 새벽 — E24 손목 0점 사고 해결**: `h` 순간 팔이 뒤로 넘어가고 전 물체가
  119~139° tilt로 거부 → 원인은 AXIS4/5의 `HOMING_METHOD=1`이 **cfg의 POS_BEFORE_EXIT를
  시작 각도로 디코드**(팔 위치와 무관)하는 것. J4 81.7° 오차 → 중력보상이 손목을 능동적으로
  넘김 → 무너진 자세로 HOME 기록되며 앵커까지 오염. **AXIS4/5 → HOMING_METHOD=3**으로
  파일 유래 0점 제거, 엔코더 카운트 고고학으로 잔차 ≤2.8° 검증, 실기 전 5클래스 재현(§7 E24).
  잔여: `w=0` 폴백 브랜치 사고(E25), refine 진동(E26).
- **2026-07-28 오후 — E26 해결 / E27 발견 / RT 예산 숙제 확정**: ① **E26 refine 해결**:
  자세 기준점이 접근 전 대기 자세를 가리키던 버그 + refine w 사다리 → tennis **6 pass 실패
  14.8 mm → 1 pass 9.4 mm**, 3물체 합계 **14 pass → 5 pass**, 전 pass `R held within 0.0 deg`
  (이전 9~29° 표류). 남은 5.8~9.4 mm는 **stiction 바닥** → 파지 <5 mm는 게인/적분 과제
  ② **E25 사다리는 유지하되 판단은 정정**: 접근 거부를 고친 것은 **앵커**였고 사다리는 첫 단을
  거의 벗어난 적이 없음(§7 E25) ③ **E27 브랜치 플립**: 머스타드가 어깨를 ~180° 돌린 해로
  빠지는데 **툴 자세가 거의 그대로라 60° 게이트가 못 걸러냄**. 목표 **2 mm 차이**가 좋은 해와
  플립 해를 오가게 함 — 하루 종일의 오락가락이 이것 ④ **RBDL `num_steps`는 출력 필드**(입력
  한계는 `max_steps`) — 원래 코드부터 무효였음. 예산을 실제로 조이니 해가 망가져 되돌림 →
  **IK가 1 kHz 루프에서 최대 56 ms** 사용, **off-RT 이전이 남은 최대 숙제**
- **2026-07-28 저녁 — reach map 완성, 그리고 그게 찾아낸 것**(`a0a2c26`, §9/§7 E28·E29):
  `n/8` 코너 프로브를 **격자 스윕 지도**로 대체. 핵심 설계는 **솔버를 재구현하지 않고
  `FullDynControllerRT.o`를 링크한 것** — 로직 두 벌은 드리프트하고, 로봇과 다른 말을 하는
  지도는 없느니만 못하다. ① **비순환 검증**: 필드 좌표 재생 모드가 실기의 `J5 2.90 rad` 거부를
  **축·수치까지 재현** ② **E28 즉발 발견**: E27 브랜치 가드 뒤의 **polish 단이 재검사를 안 받아**
  플립된 해를 되돌려줄 수 있었다(자세만 보고 고르는데 플립 자세가 더 좋아 보임) → 실측
  갭 2.96 rad짜리 해가 통과하고 있었음, 수정 후 1.00 ③ **E29 전경**: 박스 안 거부는
  **사실상 BRANCH 하나뿐**(TILT·NO_SOLUTION 0건) ④ **E27 표현 정정**: "2 mm가 가른다"는
  날카로운 경계를 연상시키지만 1 mm 스윕 결과는 **32 mm 폭의 얼룩덜룩한 혼합 지대**이고
  머스타드는 그 한가운데 — **2 mm 옮기기는 대책이 아니고 1.5~2.5 cm가 필요하다**
- **2026-07-29 — 다중 시드 IK 실기 검증·병합**(`2e911a3`, §7 **E30**): 브랜치 플립을 솔버 쪽에서
  해소. 커버율 **95.1 → 99.4%**, BRANCH 1,345 → 14. **실행 전에 기록한 지도의 예측이 실기와 전 필드
  일치**(gap 0.39 rad J2 / tilt 29° / 8 solve). 워치독 100 ms에 정체 14.92 ms, **Lost 델타 0**.
  ⚠️ **효과를 낸 것은 시드 팬아웃이 아니라 warm-start를 건너뛴 solve 한 번**(회복 셀의 89%).
- **⚠️ 앵커 민감도 재평가(같은 날)**: 오퍼레이터가 HOME을 재기록하자 **J3가 12° 달라졌을 뿐인데**
  z=0.30의 반경 내 BRANCH 셀이 **176 → 386개로 2배 이상 늘고 로봇 쪽으로 내려왔다**. 07-28에
  "앵커 비교 스윕은 값어치 낮다"고 접었던 판단은 **이 관측으로 뒤집힌다** — 앵커는 생각보다 큰
  지렛대다. 다만 지금 막는 일은 아니므로 우선순위는 그대로. **운용 수칙 추가: HOME을 재기록하면
  기존 reach map은 무효다**(좌표를 뽑아 쓸 때 반드시 현재 앵커로 재계산할 것)
- **2026-07-29 — E16 공구축 모순 해소(잠정)**: URDF 190~202행 주석이 실측 근거 2개를 들어 공구축을 **−X**라 단정하고 있었고(캘리브 자세에서 `tcp x`가 아래에서 7° 이내, hand-eye `eMo`가 태그를 x=−29.4 mm에), 나는 어제 그 주석을 끝까지 읽지 않고 `tcp` 관절의 `xyz 0 0 0.06`만 보고 "주석이 틀렸다"고 단정했다 — **그 단정은 부적절했다**. 오늘 두 가설이 예측하는 HOME 자세의 플랜지 방향을 계산해 **오퍼레이터가 실물을 눈으로 확인**: 센서 면이 앞·오른쪽을 향하고 수평보다 18° 아래 → **+Z 가설과 일치, 공구축 = +Z 확정**. **옛 실패 기록은 축이 아니라 부호 문제였다** — 주석의 `z=-0.060` 시도는 **음수**라 질량을 로봇 안쪽으로 6 cm 보낸 것이고, 그 자세에서 공구축이 거의 수평이므로 "6 cm 옆으로 매달렸다"는 관측도 +Z와 모순되지 않는다. **⚠️ 미해결**: 캘리브 근거(x=−29.4 mm)는 여전히 설명 안 됨 — 태그 판 장착 기하를 공구축과 혼동했을 수 있으나 미확인. **조치는 보류**: 방향은 정해졌지만 CoM이 tcp 원점에서 +Z로 몇 mm인지는 실물 스택 측정이 필요하고(=TCP 재정의와 같은 측정), 중력보상을 바꾸는 변경이라 실기 확인이 필요하다. 현 오차는 **0.059 N·m**(E24 폭주값의 1/38)로 작다 → **그리퍼 단계에서 일괄 처리**
- **⚠️ 접근 자세는 top-down이 아니다(같은 날 계산)**: 자세 기준이 `m_EReady`(기록된 HOME의 툴 방향)이므로 로그의 `tilt 0°`는 "수직"이 아니라 "HOME과 같다"는 뜻이다. 저장된 앵커 4개 전부 공구축이 **수직에서 71.7~104.1°** 벗어나 있다. 관절 감도 실측: **J5는 공구축 roll이라 기울기에 0.000°/deg**(마지막 관절로는 수평을 못 만든다), J1·J2·J4가 각 **1.0°/deg**, 그중 **J4가 TCP를 가장 적게 움직인다(4 mm/deg)**. 현 앵커에서 **J4만 −33.2°** 돌리면 수직 1.2°까지 간다. top-down 강제는 가능하지만 **도달 작업공간 ⊃ 손재주 작업공간**이라 원거리 물체(41~57° tilt로 겨우 닿던 것들)가 거부로 바뀐다 → 비용은 **자세 기준을 수직으로 놓은 reach map**으로 계산 가능(로봇 불필요)
- **2026-07-29 저녁 — 수직 앵커 검증 / 접근 정확도 로거·그래프**(RAON `6bab43b`→`526f9b4`,
  §7 **E31**, §9): ① **"J4를 돌려 수직 자세로 HOME을 기록하면 top-down이 되나" = 아니오**.
  합성 수직 앵커로 17,109셀 스윕: PASS **96.5 → 83.2%**, BRANCH **31 → 1,018**, TILT
  565 → 1,859, solve 평균 1.76 → 13.30 ms. 앵커는 **자세 기준인 동시에 IK 출발점**이라
  브랜치 골짜기 구조까지 같이 바뀐다. 건질 것 = **박스의 74.6%는 수직 5° 이내로 도달
  가능**하고, TILT 1,859셀은 **팔의 물리**(멀리 뻗으면 손목이 아래를 못 봄)라 **어떤 앵커로도
  안 고쳐진다 — 답은 배치**(물체를 안쪽으로/로봇을 테이블 가까이). ⚠️ 한계: J4 하나만 돌린
  **합성 자세 1개** 검증 ② **접근 정확도 자동 기록·작도**: 완주마다 CSV 1행(앱, RT 밖 워커)
  + 워처가 3D 그래프. **첫 실기 10건 3.07~11.52 mm, 전 건 완주** — 그리고 **오차가 랜덤이
  아니라 물체마다 방향까지 재현된다**(apple 두 회차 차 0.5 mm, orange 0.7 mm) → 파지용
  <5 mm에 직접적인 단서. mustard만 12.9 mm로 튀는데 **이 물체만 8 solve·tilt 29°** = 07-28
  지도가 경계 지대로 지목한 그 물체. 증거 `evidence/approach_accuracy/`
- **2026-07-29 — CPU3 격리와 EtherCAT IRQ 관계 확정**(§4): eth0 IRQ 37은 **CPU1 전담**
  (CPU3 0회)이고 이는 cmdline `irqaffinity=0-2`로 **의도한 설계**다. 다만 IRQ 스레드
  `irq/37-eth%d`가 **SCHED_FIFO 50**이라 SCHED_OTHER인 비전 파이프라인을 **항상 선점**하고,
  CPU1엔 eth0·타이머·IPI 말고 없다(USB NIC는 CPU0). Lost 델타 0으로 관측 비용도 없음 →
  **현 상태 유지**. ⚠️ **eth0 IRQ를 CPU3으로 옮기는 것은 금지** — 앱이 prio 95/97인데 IRQ
  스레드는 50이라 굶고, **IK 최대 58 ms 동안 프레임 처리가 멈춘다**. off-RT가 선행조건
- **2026-07-30 — 접근 궤적 자동 기록·3계열 그래프**(RAON `592998e`, §6/§9.5 부록): 접근마다
  DataLog 1파일 자동 생성(로거 창 24 s→완료 에지), 워처가 3D+변위 그래프 작도. FK는
  `fk_replay`(컨트롤러 `.o` 링크, RT-FK 교차검증 0.002 mm). 이후 07-30~31 사용자 스펙으로
  **2패널 고정**(3D=one-shot plan+actual, 변위 1plot=IK입력/FK(q_ref)/FK(q_act) 3계열,
  `bd01c6d`→`b884069`). 이 도구의 첫 판독이 settle leash 발견 → 정밀화 트랙의 도화선.
- **2026-07-30 저녁 ~ 07-31 새벽 — 접근 정밀화 트랙 완결(§9.5): first miss ~40 mm → 전 물체
  원샷 7~11 mm 🎯**: 진범 = 중력·게인이 아니라 **손목 stiction + inertia-scaled PD 저권한**
  (페이로드 A/B #24~26·Kd #30으로 소거, j3 breakaway 7.2~10.6 Nm 직증). 해법 = **Coulomb
  마찰 FF**(M-바깥, KF_3=5.0/KF_4=3.0) + 함정 2건을 게이트로 봉인(E33 캐터펄트→refine scope
  gate `9ba929d`, E35 소이동 과주→lag gate `90e5409`). 결과 **#52~58 7/7 refine 0회**
  (HOME 5/5 + mustard→orange 전이 11.3 mm, 과주 0). 남은 오차는 stiction이 아니라 **일관
  바이어스+j2 lag** — 파지 <5 mm의 지렛대는 KF_2+바이어스 보정으로 좁혀짐. 같은 기간
  타이밍 사고 2건 규명: **E32/E34 = IgH master 스레드 priority inversion**(PI 없는
  semaphore+SCHED_NORMAL) → EtherCAT-OP FIFO 40 **자동화**(`6c090ae`, 1회 설치
  `sudo bash tools/install_ecat_op_fifo.sh`) + 실기 중 보드 무거운 작업 금지 수칙.
- **★ 2026-08-03 — 앱 전환: 접근 + 추종(D15 `'o'`)**: 목표가 "정지한 물체 위 접근"에서
  **"접근 + 사람이 옮기는 물체 추종"**으로 바뀌었다. `'o'` 하나로 어디서든 추종 토글, 목표까지
  0.15 m 초과면 **검증된 접근 스택이 leg를 자동 발화**하고 0.15 m 이내에서만 1 kHz DLS 서보가
  따라가는 **v3 하이브리드**(`8d64868`·`25bc044`·`38a787a`). 실질 안전장치는 **TCP 선속도 캡
  0.20 m/s** 하나로 단순화됐고(v2 `2993804` — 게이트 연쇄가 추종 자체를 죽였다는 실기 판정),
  그 위에 워크스페이스 박스·goal z 하한 0.25 m·상실 시 TCP 동결·키 화이트리스트·토크 백스톱·
  관절 드리프트 예산 1.5 rad가 얹혀 있다. 실기 판정으로 E36(카운터 프레임 소프트리밋)·
  E37(lag gate 우회 → 테이블 다이브, 오퍼레이터 E-stop)을 잡았으나 **v3 하이브리드 자체는
  실기 미검증**(검증 사다리 8단계 대기). perception 쪽 대응 = depth-color 시간 정합(`b1d05c4`)과
  갱신 주기 15.3 → **21.7 Hz**(`7e4c639`) — `docs/vision/throughput.md`. 상세 §2 D15 / §7 E36·E37 / §8.1
- **★ 2026-08-06 — 오퍼레이터 콘솔**(RAON `4b2113a` + ros2_ws `cfdaa03`): 인바운드
  `/operator_key`·`/operator_jog`·`/operator_gains` + 아웃바운드 `/robot_state`·`/joint_states`·
  `/operator_msg`, 데스크톱 Qt GUI(`raon_operator_gui`). 정지키 실동작 수정 = `'e'`(전 동작 소스
  취소 + 전축 servo-off) / `'x'`(취소 후 grav-comp 유지), 공통 근원 `KillMotionSources()`.
  **커밋 본문 자체가 "Built and linked. Not yet exercised on the robot." — 실기 미검증**.
- **다음**(2026-09-04 기준): ① **추종 v3 실기 검증 사다리 8단계**(정적 호버→상실→저속→
  **속도 캡 증명**→경계→폴트 드릴→핸드헬드) — 속도 캡 증명 전에는 물체를 손으로 들지 말 것.
  오퍼레이터 콘솔 첫 실기도 같은 세션 ② IK off-RT ③ 충돌 검사(RViz+MuJoCo — 오퍼레이터
  지시로 우선순위 하향, 원샷 트랙 완결로 재논의 가능). **파지 준비**(TCP 재정의(끝면 기준,
  공구 스택 ~5 cm 실측 반영) + top-down 고정 R + <5 mm 정밀도(지렛대: KF_2·바이어스 보정,
  §9.5 ⑦), 물체 6D pose)는 **그리퍼 하드웨어 도착 시 재개하는 범위 밖 항목**.
  task-space servo는 사고 부검(E17) 전제조건 하 백로그.
- **사건 1건 해소(2026-07-26)**: 첫 Phase 3 시도에서 앱 `bad_alloc`(memlock 한도 유한 세션 +
  `mlockall(MCL_FUTURE)`+DDS arena) → teardown 중 커널 하드 락업 → 보드 재부팅. §7 E6~E8.
- **로봇**: Indy7이 eth0에 직결, 제어전원 인가 시 7슬레이브 PREOP 상시 응답. **서보온 모션·
  비전 접근 데모 상시 가동 중**(2026-07-26 첫 서보온 이래).

## 1. 목표와 최종 아키텍처

### 1.1 목표의 변천 (오해 방지용 히스토리)

| 시점 | 목표 후보 | 결말 |
|---|---|---|
| 계획 초기 | RAON-RT의 ViSP AprilTag visual servoing 재현 | **폐기** — `VisualServo`가 librealsense로 카메라를 직접 열어 perception 파이프라인과 **배타 점유 충돌**. 병합 목적(파이프라인 검출→로봇)과 모순 |
| 최종 확정 | **인식 물체 위 TCP 접근 (look-then-move)** | 파이프라인 유지 + `/pick_target_base` 구독. AprilTag/ViSP는 **손-눈 캘리브 1회용**으로만 (그마저 이미지 처리는 데스크톱 오프라인) |
| 2026-08-03 앱 전환 | **접근(`'v'`) + 실시간 추종(`'o'`)** | **현재의 앱.** 접근한 뒤, 사람이 물체를 옮기면 TCP가 따라간다. **파지·이송·놓기는 범위 밖**(그리퍼 하드웨어 미보유 — 코드에 파지 경로 자체가 없음). 계약 §8.1 |

이동 중 연속 추적(진짜 closed-loop)은 **2026-08-03에 구현됐다**(D15) — 컨트롤러에 사장돼 있던
`SetTargetPose_Jacobian` 서보를 재배선한 것이고, 비전 갱신은 당시 15 Hz(2026-08-05 이후
**21.7 Hz**). v3 하이브리드까지 진행됐으나 **실기 검증은 대기 중** — §2 D15 / §7 E37 / §8.1.

### 1.2 아키텍처 (병합 접점 = ROS2 구독 하나)

```
[ros2_ws perception — 무수정 유지]
  camera → detector(DPU) → pick_logic v3 → t3d(camera frame) → base node(TF) 
                                │                                   │
                                │ desired_class (LIVE param)        ▼
                                │ ← ros2 param set ────  /pick_target_base (PickTarget3D, base_link, m)
                                                                    │ 구독
[RAON-RT App/Indy7 (kv260-merge)] — CROS2PickBridge                 ▼
  'p' 키: /detections 집계 메뉴 → 숫자로 물체 선택 → param set ──┘
  'v' 키: N프레임 통계 게이트(std·워크스페이스 박스·radial) → goal=(x, y, z+margin)
          → SolveReadyIK(q_ready 고정 시드, 2026-07-27~) → 직행 or q_ready 경유 스테이징
          → quintic 궤적 → CTC → CST 토크 → EtherCAT → Indy7
          (자세 = q_ready 가족의 soft-R; top-down 고정 R·물체 6D는 백로그)
```

### 1.3 좌표 변환의 소재 (자주 헷갈리는 지점)

**변환은 전부 ros2_ws 파이프라인에서 한다** (`pick_target_base_node` + 정적 TF).
RAON-RT는 이미 base_link로 변환된 좌표를 소비만 한다 — RAON-RT의 rMc/VS 변환 기계는 안 쓴다.
실제 남은 일은 코드 이동이 아니라:

1. placeholder TF 정본화 — **완료(2026-07-27)**: rMc(base→color 광학) t=(0.7612, -0.0997,
   0.9262), 광축이 base -Z에서 5.6° 기울어진 top-down. launch의 정적 TF는
   `base_link→camera_link` = rMc·inv(camera_link→color_optical, realsense 라이브 TF 실측)
   — RPY가 pitch 90° gimbal 특이점 근처라 **의도적으로 quaternion 표기**
   (재합성 오차 0.0004 mm 확인). 원데이터·solver: RAON `App/CalibUtils/kv260/`.
   **E21 보정(2026-07-27 저녁)**: 사고로 로봇 베이스가 밀림(yaw +5.14°, t (+4.5, +10.4) cm
   — 손-프로브 2점 fit, §7 E21) → 현행 정본 = **t (0.811666, 0.087684, 0.924974),
   q (0.72462829, 0.00008362, -0.68706199, 0.05347582)** (`tools/base_shift_fit.py`로 합성).
2. rMc의 "r"은 RAON-RT FK의 로봇 베이스 → 파이프라인 base_link = RBDL 모델 베이스로 자동 정합
   (검증: 캘리브 태그 z-거리와 FK 플랜지 높이 일치, apple 실좌표 기하 일치).
3. reachability 게이트는 별도 노드 파라미터가 아니라 **브릿지 워크스페이스 박스가 전담**
   (실측치 §5 Phase 5 행). **z 의미 주의**: `/pick_target_base`의 z는 물체 **윗표면**(depth가
   닿는 면)이지 부피 중심이 아니다 — 접근 z+0.15는 표면 기준이라 그대로 안전.
   **테이블면은 base z≈+0.10** (브레드보드가 로봇 base 0면보다 높음, depth 실측).

## 2. 확정 결정 로그

| # | 결정 | 근거/비고 |
|---|---|---|
| D1 | 제어 = **CST 토크 + RBDL** `CControllerFullDynamicsRT` 재사용 | 구 "CSP 직결" 계획 대체. Indy7=NRMK CORE라 토크변환 정상 경로(비-NRMK 항등변환 버그 비해당) |
| D2 | 구조 = **독립 make + ROS2 브릿지** | RAON-RT는 colcon 밖. ros2_ws엔 아무것도 안 들어감(브릿지가 my_interfaces를 링크) |
| D3 | 코드 정본 = **Revision** | 원본(lab git)은 Revision의 순수 부분집합(원본에만 있는 건 .git뿐). 가이드 162항목 검증도 Revision 기준 |
| D4 | rtposix = **RT-POSIX 원본 빌드** (shim 불필요) | `git.raimlab.com/RAIMLAB/RT-POSIX` = RT-AIDE 정품 소스(같은 저자). aarch64 이식 OK |
| D5 | sim-first **철회** → 실기 중심 | Indy7이 eth0 직결로 판명 → "서보-오프 통합 런"이 sim이 하려던 검증을 모션 0으로 대체+와이어까지 검증 |
| D6 | 물체 선택 = **대화형 터미널 메뉴** | 오퍼레이터가 로봇측 터미널에서 보기 중 선택 → `desired_class` LIVE 파라미터로 전달 |
| D7 | person 안전 = **근접(대형 bbox)만 차단+래치** | 배경 통행인엔 무반응, 워크스페이스 접근 시 타깃 무효화. **2026-08-03 앱 전환(접근+추종)의 후속으로 2026-09 코드 제거** |
| D8 | 슬레이브 DC = **저자 구성 그대로 OFF(SM-sync)로 브링업** | INDY7.cfg에 `DC_SUPPORT` 키 자체가 없음(기본 0) = 저자의 검증된 구동 상태. DC 실험은 cfg 주석 해제로(코드 준비됨) |
| D9 | 사이클 = **1 ms 유지** | `m_dt=0.001` 하드코딩과 결합돼 있고, 동급 GEM 실측 "1 ms 안정" 근거. 2 ms 완충안은 폐기 |
| D10 | 3+1 코어 격리 = 보류였다가 → **착수(2026-07-27 밤, 오퍼레이터 지시 — 그리퍼 대기 중 인프라 작업)** | RT-POSIX가 전 태스크를 CPU0에 기본 pin(`posix_rt.c:257`) → isolcpus만으론 무효, 앱 pin 코드와 한 세트. **실증 확인(2026-07-27)**: 접근 데모 전 구간 비격리(cmdline `skew_tick=1`만), Lost 0·지터 이상 무. **구현(`a6e2097`)**: cfg `[TASKn] CPU=` 키(파서+`InitRTTasks` pin, 미지정=-1=기존 CPU0) — TASK0/1(1 kHz 페어)만 CPU3, 키보드/터미널/로거는 CPU0 잔류. gate0에 cpu 인자. **비격리 베이스라인(파이프라인 부하, 20 s)**: CPU3 avg 1.0/max 33.9 µs, CPU0 avg 1.2/**max 104 µs**. cmdline 전환은 `/etc/default/flash-kernel` → `"skew_tick=1 isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2"`(+`flash-kernel`+재부팅; 커널 NO_HZ_FULL=y·RCU_NOCB=y 확인, 비전 ~1.8코어라 0-2 3코어 충분 — 7/9의 2+2 실패와 다름). GEM IRQ의 CPU3 이전 여부는 격리 후 실측으로(→ §4.1: eth0 IRQ CPU1 전담=의도 설계). **격리 A/B 완결(07-28 17:02, §0)**: 격리 CPU3 max 51.1 µs vs 비격리 CPU0 114.6 µs |
| D11 | 캘리브 이미지 = **파이프라인 토픽에서 캡처** | librealsense 직접 열기 금지(충돌). intrinsics도 camera_info에서 → `save_camera_params` 불필요 |
| D12 | 캘리브 체인 = **ViSP 완전 배제, 태그 pose는 OpenCV로** (2026-07-26) | `visp-compute-apriltag-poses`는 x86-64 바이너리+소스 미포함+원저자 홈 rpath → 어디서도 실행 불가. `cv2.aruco`(APRILTAG_36h11)+`solvePnP`로 동일 YAML 산출. intrinsic은 공장값(camera_info) 1순위 — 파이프라인 3D와 같은 카메라 모델이어야 rMc 정합(불일치가 rMc에 흡수되는 계통오차 방지). 원저자 camera.xml은 640×480이라 어차피 재사용 불가(우린 848×480). 잔차 불량 시에만 `cv2.calibrateCamera` 재캘리브로 escalation |
| D13 | 접근 IK 시딩 = **init 계산 q_ready 고정 시드 + 자동 분기** (2026-07-27, 오퍼레이터 선택) | 근본 원인: RBDL IK는 local 반복법이라 **시드가 유일한 브랜치 선택 장치**인데 시드=현재 자세였음 → 성공이 "오퍼레이터가 팔을 어디 놔뒀나"에 종속(시도마다 자세도 상이). 시드는 계산상 출발점일 뿐 물리 자세와 분리 가능 → init에서 워크스페이스 박스 중심을 풀어 q_ready 산출·코너 8점 검증, 매 접근을 q_ready(+그 자세의 soft-R)에서 해석 = **같은 목표, 같은 해**. 이동 정책(사용자 선택지 4개 중): **자동 분기** — Δq≤2.0 직행, 초과 시 q_ready 경유 자동 스테이징. 대안 검토: 해석적 IK(브랜치 전수 열거)가 근본 치료지만 기계변환 URDF의 프레임 뒤틀림(E16)이라 수식 유도 고위험 → 백로그 |
| D14 | 정지 잔차 해법 = **Coulomb 마찰 FF(M-바깥) + 게이트 2종**, 연속 Ki 도입 안 함 (2026-07-31) | 실패 지점이 속도≈0이라 Kd 무효(#30 실증), 연속 Ki는 stiction·leash와 limit-cycle 위험 + refine이 이미 이산 적분기. 확정값 **KD 28/25/20 · KF_3 5.0 · KF_4 3.0**(cfg=오퍼레이터 상태, 커밋 안 함) — 도출·검증 전 과정 **§9.5** |
| D15 | **실시간 추적('o') = 기존 `SetTargetPose_Jacobian` 서보 재배선, IK 재해석 아님** (2026-08-03, 구현 완료 · **실기 검증 대기**) | 이동 목표에 min-jerk 재계획은 재계획마다 속도 불연속(quintic이 정지에서 재시작) → 15 Hz 갱신엔 1 kHz 국소 서보(DLS, 비누적 q_ref=m_Q+qd·dt)가 정답이고 그 코드가 이미 사장 상태로 있었음. 비전 무수정(스트림은 원래 연속) — 브리지에 seqlock 샘플 파이프 신설: **finite→in-box→jump 0.10 m 거부**(동일 클래스 인스턴스 전환=순간이동은 rate-limit이 아니라 **거부**해야 사람 사이 스윕이 안 됨)→EMA α=0.4. 진입은 'v' 접근 hold에서만(TCP 0.15 m 이내), 추적 중 키 화이트리스트 {o,j,e,x,g}, **상실 시 goal=현재 TCP 동결**(마지막 비전 goal은 캡 속도에서 TCP를 ~10 cm 선행 — 그걸 유지하면 상실 후 맹진), 2 s 지속 상실=OFF 래치. 선속도 캡 0.20 m/s(cfg `TRACK_VMAX_MPS`, 코드 클램프 0.05–0.50), 추적 호버 마진 0.20 m(접근 0.15 유지). 선행 커밋으로 안전 백스톱: NaN 게이트+**토크 새추레이션**(SaturateOutput 미배선·MoveTorque 무제한이었음)+CTC 진입 q_ref 스냅샷. 커밋 `9204372`(백스톱)+`4efb05d`(추적). 검증 사다리 8단계(정적 호버→상실→저속→**속도 캡 증명**→경계→폴트 드릴→핸드헬드)는 다음 실기 세션. **v2(`2993804`, 같은 날 실기 판정 반영)**: v1은 "따라오는 경우보다 안 따라오는 경우가 더 많음" — 글리치 1프레임에 jump 기준이 낡고, 이동 중이면 연쇄 거부→LOST→근접 재획득 실패→2 s 래치로 사망하는 **게이트 연쇄 자멸 구조**였음. **진짜 안전장치는 TCP 속도 캡**(goal이 박스 안 어디로 튀어도 팔은 0.20 m/s로 미끄러질 뿐)이라는 판단으로 단순화: jump 게이트·'v' 선행·진입/재획득 근접·2 s 래치 **전부 삭제**, 'o' 하나로 선택 물체를 어디서든 추종, 추적 중 'p'+숫자 재타게팅 허용, 가림=그 자리 정지 후 재출현 시 무조건 재추종. 유지=속도 캡·박스(admission+클램프)·LOST TCP 동결·진입 4게이트(servo/controller/정착/타깃 lock)·키 화이트리스트·토크 백스톱. 모션 시작만 'o' 1회 명시 조작으로 유지(E17). **FF 재활성(`3ceeb9f`, 같은 날)**: 실기에서 마지막 몇 cm를 J3/J4가 안 움직여 TCP가 중심 못 미침 — v1이 추적 진입 시 KfScale=0으로 FF를 껐고, 켜도 **E35 lag gate가 서보 레짐에서 FF를 5%로 무력화**(비누적 참조라 lag=qd·dt≈1e-4 rad ≪ 0.002 램프). 수정=진입 시 KfScale=1 + eTrackingServo에서만 lag gate 우회 — 서보 qd_ref는 잔여 오차에 폐루프라 sat(qd_ref/0.01) 램프 자체가 과주 방지 장치(E35 과주는 leash 위를 행진하는 open-loop 참조가 전제, 여기 없음). 접근/refine 경로는 lag gate 불변. **v3 하이브리드(`8d64868`·`25bc044`·`38a787a`)와 후속 픽스는 §7 E37, 두 모드 계약 요약은 §8.1** |

**시뮬레이션 도입 판단 (2026-07-28)** — "백그라운드에서 검증하고 검증된 것만 실기 사용"에
대한 결론:

- **지금 우리를 막는 문제는 전부 기구학**(IK 브랜치·도달성·사다리 수렴)이고, 여기엔
  **물리 엔진이 필요 없다**. 앱과 같은 RBDL + 같은 URDF를 링크한 **헤드리스 스윕**이 더 싸고
  더 정확하다(같은 솔버를 그대로 돌리므로). → **1순위 = reach map 툴**(§9)
- **MuJoCo는 2순위.** 값을 하는 지점은 정확도가 아니라 **충돌 검사와 궤적 사전 검증**이다
  (E13 테이블 강타, 카메라 마운트 충돌은 실기에서 겪으면 비싼 사고). 트리에 이미
  `AxisMuJoCo.cpp`/`MuJoCoAxisExample.cpp` 훅이 있다. 단 stiction·마찰 모델이 실물과 달라
  **안전 필터용이지 정확도 검증용이 아니다**
- **Isaac Sim은 보류.** 강점이 photoreal 렌더링·도메인 랜덤화, 즉 perception 쪽인데 현재
  병목은 manipulation이다. KV260에서 못 돌아 데스크톱 RTX + Omniverse 스택을 새로 세워야
  하므로 비용 대비 효익이 안 맞는다
- ⚠️ **함정**: 시뮬레이션은 *모델*을 검증하지 *플랜트*를 검증하지 않는다. 이 트랙에서 가장
  크게 다친 세 건(**E24 호밍 0점, E21 베이스 이동, E26 stiction 바닥**)은 어떤 시뮬레이터도
  잡지 못했을 것들이다. sim 통과를 "안전하다"의 근거로 쓰면 오히려 위험하다 —
  **기구학·기하학적 유효성의 사전 필터**로 쓰고 플랜트는 실측으로 지킨다

## 3. 저장소·브랜치 체계

| 트리 | 정체 | 규칙 |
|---|---|---|
| `~/RAON-RT-Revision` | **작업 저장소** — GitHub `seriouslysilly21612/RAON-RT-Revision` clone (Revision 개발 실히스토리) | 작업은 **`kv260-merge` 브랜치만**, `main` 무접촉, push는 이 GitHub로만 |
| `~/RAON-RT` | lab 원본 clone (`git.raimlab.com`, dev) | **읽기 전용** — push 절대 금지. 히스토리 참조용 |
| `~/RAON-RT-Revision-main` | zip 스냅샷 | 무수정 참조본 |
| `~/RT-POSIX` | RT-AIDE rtposix 소스 clone | 로컬 패치 1건 적용(빌드용) — 패치 파일은 kv260-merge에 보존 |
| `~/ros2_ws` | 이 저장소 | perception 쪽 변경(pick_logic v2 등)만. RAON-RT와 토픽으로만 결합 |

`kv260-merge` 커밋 트레일 (main 이후):
`59fc801` 브랜치 셋업 → `2cdba3a` x86 빌드잔재 untrack → `9c3a28a` Gate0 테스트+rtposix 패치 →
`973a57e` gitignore → `f59f49b` gate0 warm-up 스킵 → `b3b8209` **Phase 2a** (ViSP 제거+픽스+빌드) →
`f7c27fc` **Gate 2b** (ROS2 pick bridge + 대화형 흐름 + 합성테스트) → `1ed4881` **Phase 3**
(스모크 스크립트 + E8 키보드 가드 + 사건 방어책) → `2c8fae5` **Phase 4 준비** (런타임 enable
픽스 + 'h'/'j' 서보 arm/disarm 인터록) → `1334658` run.sh 런처 → `d94c1c1` B6+게이트2 → `0668ed2` **refine 10.9 mm** →
`8093e8a` **Phase 5 캘리브** (grabber/cpo 툴 + 16자세 데이터셋 + rPc + 박스 실측치) →
`61a4a86` **E13 안전 IK + HOME + 브릿지 세션화** (접근 데모 완주 커밋) →
`19b15ba`/`f9a411b`/`0662b98` **ready-seed IK v1→v3**(D13·E19·E20) → `5140a83` sticky-float →
`e69555c`/`1694414` E22/E23 게이트 재베이스 → `9aca402` E25/E26 → `3862570` E27 브랜치 가드 →
`a0a2c26` **reach map**(+E28 픽스) → `929aa90`→`2e911a3` **다중 시드 IK**(E30) → `416da9a`
캘리브 'w' 게이트 → `6bab43b`→`526f9b4` 접근 정확도 그래프(E31) → `592998e` **궤적 자동
기록** → `bd01c6d`→`b884069` 궤적 그래프 2패널 스펙 → `49ac6db` F/T 페이로드 제거 A/B →
`00c6d2c` **마찰 FF** → `9ba929d` refine scope gate(E33) → `90e5409` **lag gate**(E35) →
`6c090ae` E34 chrt 자동화 → `pre-cleanup-20260731`(태그)→`159886c`/`c26b239` **정리 티어1**(163 MB+97줄) →
`9204372` **안전 백스톱**(NaN 게이트+토크 새추레이션+CTC 스냅샷) → `4efb05d` **실시간 추적 'o'**(D15) →
`2993804` **추적 v2 단순화**(실기 판정: 게이트 연쇄가 추종을 죽임 → 속도 캡 중심으로 재편).
정밀화 트랙(#24~58) 상세는 **§9.5**.
ros2_ws(main, 로컬): `a9644dc` **TF 정본화 + person_guard off** → `1470d98` docs → 이후
docs 커밋 다수(`git log -- docs/RAON-RT/merge.md`가 타임라인).

## 4. 시스템 배치 현황 (보드, 전부 검증됨)

| 항목 | 위치 | 상태 |
|---|---|---|
| IgH EtherLab (stable-1.6) | `/opt/etherlab` + `ethercat` CLI | 마스터 기동 검증. `ethercat.conf`=eth0 MAC+generic |
| librtposix (aarch64, 패치) | `/opt/rt_posix/{include,lib}` | `--coverage` 제거+`-fPIC`+stack-min 클램프 반영 |
| libEMasterApp (PDO 픽스 포함) | `/opt/emaster_app/{include,lib}` | `sudo make install`로 갱신됨(20:03) — **런타임은 이걸 로드** |
| RBDL 3.1.3 (rbdl-orb+urdfreader) | `/usr/local/lib` | ldconfig 등록 |
| ld 검색 경로 | `/etc/ld.so.conf.d/raon-rt.conf` | 3경로 등록 |
| RT 권한 | `/etc/security/limits.d/99-realtime.conf` | ubuntu rtprio 98 + memlock unlimited (재로그인 후 발효) |
| EtherCAT 디바이스 udev | `/etc/udev/rules.d/99-ethercat.rules` | `/dev/EtherCAT0` 0664/ubuntu — 일반유저 CLI 가능 |

### 4.1 CPU 배치 — 확인된 사실 (2026-07-29 실측)

cmdline: `isolcpus=3 nohz_full=3 rcu_nocbs=3 **irqaffinity=0-2**`

| 주체 | 스케줄링 | CPU | 근거 |
|---|---|---|---|
| 앱 Main (`[TASK0]`) | FIFO **97** | **3** | cfg `CPU=` 키 |
| 앱 EtherCAT (`[TASK1]`) | FIFO **95** | **3** | cfg `CPU=` 키 |
| **eth0 IRQ 37** | hardirq | **1 전담** | `/proc/interrupts` CPU1 21.9M / **CPU3 0** |
| `irq/37-eth%d` ×2 | **FIFO 50** | **1** | `ps -eo class,rtprio,psr` |
| USB NIC (원격 접속) | — | **0** | `xhci-hcd:usb1` CPU0만 |
| 비전 파이프라인 | SCHED_OTHER | 0–2 | 기본 |

**"격리했는데 EtherCAT은 CPU1에서 돈다"는 사실이다.** `irqaffinity=0-2`가 CPU3을 명시적으로
제외하고, IgH는 `ec_generic`(native macb 드라이버 없음)이라 프레임이 커널 네트워크 스택을
경유한다. **다만 이건 결함이 아니라 설계다** — 격리 코어를 인터럽트로부터 지키는 것이
목적이고, IRQ 스레드가 **FIFO 50**이라 SCHED_OTHER인 비전 파이프라인에 밀리지 않는다.
CPU1에는 eth0·arch_timer·IPI 외에 아무것도 없다.

⚠️ **eth0 IRQ를 CPU3으로 옮기지 말 것.** 앱 태스크가 95/97인데 IRQ 스레드는 50이라
앱이 도는 동안 굶고, **IK가 최대 58 ms 걸리는 미해결 문제**(§9)가 그대로 프레임 처리
정지 시간이 된다. **IK off-RT가 선행조건.** 손댈 여지가 있다면 `chrt -f -p 85`로 IRQ
스레드 우선순위를 올리는 정도이고, 현재 CPU1에 RT 경쟁자가 없어 실익은 낮다.

**앱 빌드 명령** (App/Indy7에서):
```bash
make clean   # 필수 1회 — 저자의 x86 obj/*.d 잔재가 남아 있으면 빌드 즉사
make         # 정식 (RBDL=/usr/local, EMasterApp=/opt)
# 개발 중 /opt 미갱신 상태로 돌릴 땐:
make RBDL_DIR=~/rbdl-stage/usr/local ECAT_INCLUDE=../../include/EMasterApp ECAT_LIB=../../lib/EMasterApp
```
⚠️ 링크는 어디서 하든 **런타임 로더는 `/opt/emaster_app` 것을 집는다** — EMasterApp 소스를
고치면 실행 전 `cd ~/RAON-RT-Revision && sudo make install` 필수.

## 5. Phase/Gate 진행 현황

| Phase | Gate | 결과 |
|---|---|---|
| **0 기반** | Gate 0: RT(SCHED_FIFO) 1 ms×5000 지터 | ✅ **avg 1.3 µs / max +35 µs** (984 µs 아웃라이어는 stale-first-deadline 시작 아티팩트로 규명 — 테스트에 warm-up 10사이클 스킵 반영) |
| **1 버스 검증** | Gate 1: 스캔↔cfg 대조 (모션 0) | ✅ 7/7 슬레이브(6×Drive `0x089a/0x30000000` + EOAT `0x10000007` 실존→cfg 무수정), **PDO 맵 10엔트리 비트단위 일치**, DC 64bit, Lost frames 0 |
| **2a 앱 포팅** | Gate 2a: aarch64 빌드 | ✅ `bin/Indy7Ctrl.out` — ViSP-free, 의존성 전해소 |
| **2a' 파이프라인** | pick_logic v2 합성테스트 | ✅ **6/6 PASS ×2연속** + 프로세스 누수 0 (`tools/vision/test_pick_logic_v2.py`) — 2026-09 S4 person 시나리오 제거 후 4/4 |
| **2b 브릿지** | Gate 2b: 브릿지 합성테스트 (카메라·로봇 불필요) | ✅ **5/5 PASS ×2연속** + 누수 0 (`tools/test_gate2b_bridge.py`) — 메뉴/param 왕복(`ros2 param get`=apple 실증)/lock/std게이트(≈1.5 mm<8 mm)/goal z=0.27 m(0.12+마진 0.15) |
| **3 서보-오프 런** | 수 분 무단절 + 조인트 판독 + 파이프라인 동시 + in-app 'p'/'v' | ✅ (2026-07-26, `tools/test_phase3_smoke.py`) app-only **4/4** → 풀 런 **6/6** → **300 s 홀드**: 정상상태 Lost frames 0(활성화 천이 2개는 베이스라인 제외), 에러 0, 7슬레이브 OP 유지. **라이브 메뉴에 5클래스 전부** 표시→apple 선택→실 pick_logic param 반영→'v'는 person_guard(area 0.08≥0.06)로 정당 거부 = **D7 첫 실환경 실증**. 조인트 판독 = DeInit **PDO값과 SDO 대조 전축 수 카운트 일치**(servo-off 브레이크로 손-이동 검사는 불가·불필요). SIGINT DeInit 경로(위치저장·마스터 해제·브릿지 다운·PREOP 복귀) 반복 검증. 코어 격리 A/B는 **비격리로 이미 유실 0이라 보류** — Phase 4 실토크에서 지터 보이면 재개(D10 갱신) |
| **4 서보온 — 게이트1 grav-comp** | 오퍼레이터 게이트 서보온('r'→'g'→'h') | ✅ (2026-07-26) **6축 동시 0x0237(OPERATION ENABLED), grav-comp 34 s 유지, 손밀기 컴플라이언스 확인, 'j' disarm 클린**. 사전에 E-stop 실효성 확인(버스 전원 차단→해제 후 fault 0 복구, E10). 실행은 `App/Indy7/run.sh` |
| **정확도 개선 (refine)** | CTC droop 반복 보정 | ✅ (2026-07-27, `0668ed2`) **160 mm → 10.9 mm** (4 pass 단조수렴 60→25→19→17→10.9). 감쇠 0.65 편향 + 속도게이트(|qd|<0.02, 0.3 s) 측정 + tol 12 mm(=stiction 바닥). 'a'와 비전 접근 모두 자동 적용. <5 mm는 게인/closed-loop 백로그 |
| **4 서보온 — 게이트2 'a' 위치추종** | 하드코딩 목표 실모션 | ✅ (2026-07-26) 사전에 **손-유도 프로브로 카메라 위치 실측**: base (+0.70, -0.13, 0.98) → **테이블/카메라 = +X 확정**(워크스페이스 박스 입력). 'a' 목표는 반대편(-X)이라 GO → 3 s quintic 정상 실행·유지·'g' 소프트중단 검증. **정밀도 발견: 정상상태 오차 ~16 cm**(도달 (-0.330,-0.194,0.866) vs 목표 (-0.181,-0.181,0.931)) — 무적분 CTC + 하모닉 마찰의 전형. **Phase 5 전 개선 필수**(게인/적분/도달 후 재타겟 반복 — 궤적 CSV는 B6 픽스로 분석 가능해짐) |
| **5-1 캘리브 캡처** | 12~15자세, 회전 다양성 | ✅ (2026-07-27) **16쌍 lockstep**(grabber Enter→앱 's', 태그 검출시에만 저장). 재검출 16/16, 쌍별 회전각 중앙값 40.8°·92%≥15°, FOV x225~736/y45~426, 이동 스프레드 33×56×31 cm. 태그 = 36h11 id0, 검은 사각형 **65 mm**(흰테두리 포함 80 — 8:10 규격비 일치로 측정 교차확인) |
| **5-2 AX=ZB solve** | rMc + residual | ✅ (2026-07-27, `8093e8a`) `calib_cpo.py`(IPPE_SQUARE, reproj 0.12~0.55 px, 모호성비 ≥2.8) → 원저자 `eye_to_hand_calib.py` 무수정 실행. **rMc t=(0.7612,-0.0997,0.9262), residual 1.81 cm/1.66°**. 손-프로브(0.698,-0.132,0.981)와 9 cm 정합(프로브는 ±10 cm짜리 참고치). eMo(플랜지→태그)도 동시 해결 — 태그 장착 실측 불필요 |
| **5-3 TF+박스 정본화** | 파이프라인 반영 + 실좌표 검증 | ✅ (2026-07-27, ros2_ws `a9644dc`) launch TF quaternion 교체(§1.3), 브릿지 박스 실측치 x[0.30,0.85] y[-0.50,0.45] z[0.10,0.50](카메라 아래 >0.4 m 클리어). **실검증: apple → (0.841, -0.102, 0.179)** — x/y 이미지 기하 일치, z = 테이블(+0.10) + 사과 높이 표면값과 4 mm 일치. person_guard는 우측 케이블 FP(conf≤0.41, 36%)로 상시 래치 → **오퍼레이터 결정으로 off**(E12) |
| **5-4 접근 데모** | 'v' 라이브 lock→approach | ✅ (2026-07-27 심야, `61a4a86`) **1차 시도에서 테이블 충돌 사고(E13)** → 원인(점-구속 IK의 nullspace 방랑 + 무게이트 quintic) 제거 후 재시도: 수직 시작=게이트 거부(정상), 기울인 시작=orange/tennis_ball/banana 접근 성공, **물체 중앙 배치 후 접근→homing('b')→다음 물체 사이클로 전 클래스 성공**. apple(0.85 m)=reach 한계 거부, mustard(x 0.906)=박스 게이트 거부(정상). z마진 15 cm 상수 — 관측 편차는 표면기준+refine tol 12 mm+미모델 공구길이(내일 반영) 합성 |
| **HOME 기능** | 메뉴 기록 + 'b' 복귀 | ✅ (2026-07-27, `61a4a86`) 'p' 메뉴 마지막 항목이 **현재 관절값 스냅샷**(IK 없음 = 브랜치 점프 원천 불가), 'b'가 속도상한 quintic으로 복귀. refine/접근 자동 취소. **07-27 주간**: HOME 미기록 시 'b'는 계산된 q_ready로 복귀(`19b15ba`) |
| **grav-comp 보완 (F/T+sticky)** | 말단 페이로드 모델 + 침하 정지 | ✅ (2026-07-27 주간) URDF tcp에 **285 g**(RFT76-HA01 200 g + 어댑터판 등) 반영 — 1차에 +Z로 넣어 악화, **공구축=−X 함정**(E16) 교정 후 CoM (−0.021,0,0). 전방 뻗은 자세 잔류 sag는 모델 바닥(태그판 비자성=알루미늄 확인, 질량 추정 유지) → **sticky-float**가 완충: dead-band 앵커 스프링(0.15·Kp·0.03 rad 상한) + **속도게이트 완화판**(|qd|>0.08 rad/s면 앵커 추종=순수 grav-comp 감각, Kd 0.08→0.03) — **오퍼레이터 "이정도면 OK"**(`5140a83`). 'k' 토글 |
| **ready-seed IK** | 접근 해의 시작자세 독립화 | ✅ (2026-07-27, `19b15ba`→**v2 `f9a411b`** — 같은 날 저녁 **v3 `0662b98` 실기 검증 완료**, §0/§7 E19·E20) init에서 박스 중심 부트스트랩 → r-클램프 코너 8점을 런타임과 동일 방식(soft-R=q_ready의 R)으로 전수 검증 → `[SEED]` 부팅 로그. 매 'v'를 q_ready 시드로 해석(결정적), **2π 폴딩+관절한계 검사**(J3 10.42 rad류 감김 해 → −2.15 rad 등가해로 구제, legacy 경로에도 적용), Δq≤2.0 직행 / 초과 시 **자동 스테이징**(q_ready 경유, settle 0.2 s→goal 레그. 모드/서보 변동 시 즉시 취소=이연 모션 금지). 부트스트랩 실패 시 live-시드 폴백. **v1 필드 결함(E19)**: 합성 사다리가 손목 접힌 기형 브랜치(J4 2.61/J5 1.65)를 q_ready로 뽑아 "괴랄한 회전" → **v2 = 오퍼레이터 자세 앵커**: HOME 기록 시 그 자세로 즉시 re-base(`SetReadyAnchor`)+`$HOME/.indy7_ready_seed` 영속화(RT 메일박스→브릿지 워커 파일 IO), 다음 부팅은 파일 자세를 1순위 시드로. 사다리엔 \|J4\|,\|J5\|>2.0 거부 가드. 부팅 로그에 `[anchored to operator posture]`/`[synthetic — record HOME to re-base]` 표기 |
| **접근 정밀화 (원샷)** | first-pass 도달, refine 0회 | ✅ (2026-07-31, **§9.5**) Coulomb 마찰 FF(KF_3 5.0/KF_4 3.0, M-바깥) + 게이트 2종(refine scope + lag) → **#52~58 7/7 refine 0회**: HOME 출발 5물체 6.9~10.7 mm + **mustard→orange 전이 11.3 mm**, 전 로그 과주 0·갭 0%. KD 28/25/20(정확도 무효 판정 후 유지), 페이로드 0 g. 남은 오차 = 바이어스+j2 lag(파지 <5 mm 지렛대 = KF_2·바이어스 보정) |

## 6. 코드 변경 요약

### RAON-RT (kv260-merge)

| 파일 | 변경 |
|---|---|
| `App/Indy7/Indy7Ctrl.{h,cpp}` | VisualServo 완전 제거(멤버·TASK5·구 'v'키·RT루프 주입블록·proc), `SetAsDCRef(slave0)` 가드콜 추가, calib 출력 경로 `App/CalibUtils/kv260/` · **Gate 2b**: 브릿지 수명주기(Init 비치명 실패 허용/DeInit), 'p'/숫자/'v' 키 핸들러(원자플래그만), RT 루프 goal 소비 SM('n'키 검증 시퀀스 재사용, ISO/RECT 상호배제·이동중 재트리거 금지) · **refine SM**(감쇠 편향+속도게이트, oriWeight=0) · **HOME**('p' 기록+'b' 복귀) · **서보-오프 goal 폐기 가드**(E15) · **ready-seed 접근**(`TryReadyApproach` 직행/스테이징 + `eAPPROACH_STAGING` 레그2: 모드·서보 불변시에만 발화) · init에서 박스 코너 프로브로 `ComputeReadySeed` 호출 · **접근 정확도 리포트**(2026-07-29, `EmitApproachReport`) — `eAPPROACH_MOVING→IDLE` 에지에서 1회, 목표(`m_vRefineDesired`) vs FK TCP + **refine 직전 TCP**(`m_vFirstTcp`, 첫 정착에서 캡처)를 브릿지 메일박스로. RT 경로라 `strncpy`(printf 계열 회피) · **마찰 FF 배선**(2026-07-31, `00c6d2c`: cfg vKf → `SetFrictionFF`, refine 사다리 호출엔 `adKfScale=0.0` 전달 `9ba929d`) |
| `App/Indy7/Indy7Ctrl.{h,cpp}` (키) | **캘리브 캡처 게이트(2026-07-29, `416da9a`) — 실기 검증 완료(같은 날)** — `s`가 진단 출력처럼 보이면서 hand-eye 데이터셋을 무조건 덮어쓰던 것(원저자 `d38e9ec` 유래)을 `m_bCalibMode` 뒤로 가둠. `w`/`W`로 토글, **매 실행 OFF**. 쓰기 호출부는 `case 's'`의 2줄뿐이라 가드 하나로 완결(전 저장소 grep 확인). 배너는 `DoInput`이 **1 kHz RT 스레드**라 printf 1회로 합침 — 파일 IO가 RT 스레드에 있는 건 원래부터이고, 이 변경으로 기본 경로에서는 사라짐. ⚠️ `w`가 유일한 자유 알파벳이었다(`q`·`z`는 `proc_keyboard_control`이 선점). **오퍼레이터가 로봇에서 직접 확인**: 무장 전 `s` → `App/CalibUtils/kv260/` 무변경, `w` 무장 → `s` → 변경 발생, `w` 해제까지 전체 시퀀스 정상 |
| `App/Indy7/CalibCapture.{h,cpp}` | **ViSP-free 재작성** — Eigen `AngleAxisd`로 theta-u 변환, `vpPoseVector::saveYAML` 포맷 호환(→ `eye_to_hand_calib.py` 무수정 소비) |
| `App/Indy7/FullDynControllerRT.{h,cpp}` | x86 SSE 헤더 `__SSE__` 가드 (aarch64 빌드 차단 해소) · **E13 3중 방어**(soft-R IK·Δq 게이트·T 스케일링, FK 잔차 합격판정) · `IsTrajectoryRefDone()`(B7) · **sticky-float 홀드**(dead-band 앵커+속도게이트, 'k') · **ready-seed IK**(`ComputeReadySeed`/`SolveReadyIK`/2π 폴딩+관절한계/`ScaledTrajTime`, D13) · **Coulomb 마찰 FF**(2026-07-31, §9.5): `SetFrictionFF`/`m_Kf`/`m_dKfScale`, `τ += KF_i·sat(q̇_ref/0.01)`을 **M(q) 곱 바깥에**(`00c6d2c`) + **refine scope gate**(per-trajectory `adKfScale`, `9ba929d`) + **lag gate**(`clamp((q_ref−q)·sign(q̇_ref)/0.002,0,1)` 곱, `90e5409`) |
| `App/Indy7/indy7.urdf` | tcp 링크에 F/T 페이로드 **285 g @ CoM (−0.021,0,0)** — 공구축이 tcp 프레임 **−X**라는 경고 주석 포함(E16) → **2026-07-30 A/B로 페이로드 제거**(`49ac6db`, 0 g 복귀): 어댑터판 질량이 추정치였고 CoM 축(−X)도 07-29 실물 확인(+Z)과 모순 — 1차 착지가 **위(+z)** 로 뜨는 원인 후보. kinematics 불변(fk_replay byte-identical)이라 캘리브·reach map 유효. 복원=`git checkout 9dc5ae3 -- App/Indy7/indy7.urdf` |
| `App/Indy7/Makefile` | ViSP/librealsense/OpenCV/PCL 제거, `RBDL_DIR` override 추가 · **Gate 2b**: humble+my_interfaces 배선(E5의 include **화이트리스트** 방식), rpath 내장(ROS env source 불필요), `make gate2b_test` 타깃 |
| `App/Indy7/ROS2PickBridge.{h,cpp}` | **신규(Gate 2b)** — `/pick_target_base`·`/detections` 구독 + `AsyncParametersClient`로 `desired_class` LIVE 설정. 스레딩 계약: ROS2 I/O·메뉴·통계게이트는 브릿지 자체 non-RT 스레드(spin+worker), RT는 wait-free 원자 API+SPSC goal 슬롯만. `SignalHandlerOptions::None`(앱 SIGINT 보존). 게이트: N=15, 축별 std<8 mm, 워크스페이스 박스 실측치+**radial 게이트 r≤0.80**, z마진 0.15 m · E14 세션 위생(lock 방송 세션화·lost 1 s 디바운스·desired_class 자동 초기화) · 메뉴 HOME 기록 항목 · 박스 상수 public(ready-seed 프로브 파생용) · **접근 리포트 메일박스**(2026-07-29, `ApproachReport`/`LogApproach`/`TickApproachLog`) — seed-persist와 동일 SPSC 계약, RT는 POD 채우고 반환·워커가 시각 스탬프+CSV append. 슬롯 점유 중 재요청은 **드롭**(RT 블로킹 금지) |
| `tools/approach_plot.py` | **신규(2026-07-29)** — 접근 정확도 그래프 자동 생성. `approach_log.csv`를 폴링해 접근 1회당 PNG 1장과 누적 `approach_history.png`. 구성: **① 3D 목표-vs-도달**(원점=목표, 5/12 mm 허용 구, ±15 mm로 확대) **② 실척 측면도**(물체·호버 150 mm·도달을 축척 그대로 — 3D 패널엔 물체를 못 넣는다, 10배 축소하면 mm가 사라지므로) **③ 축별 막대 ④ 숫자 패널**. **앱이 직접 그리지 않는 이유**: `mlockall` + 1 kHz RT 프로세스에서 `fork()`는 전 쓰기페이지를 COW로 표시해 **RT 스레드의 다음 쓰기가 폴트**를 먹는다 → 별도 프로세스 폴링. `run.sh`가 `--watch --exit-with-parent`로 자동 기동(`exec`이 셸을 앱으로 치환하므로 워처의 부모 = 앱 → 앱 종료시 자동 회수). `INDY7_NO_PLOT=1`로 끔. matplotlib+numpy만 사용(pandas 보드에 없음), 라벨 전부 영문(한글 폰트 부재) · **궤적 그래프 `plot_traj` 추가**(2026-07-30~31, `bd01c6d`→`0550e47`→`9606588`→`91d7fb4`→`a09f129`→`3bc5336`→`b884069`): DataLog+fk_replay 짝을 사용자 스펙 **2패널**로 — ① 3D = one-shot plan(이론)+actual(실제)+○(원래 목표=호버점) ② 변위 1 plot = IK입력/FK(q_ref)/FK(q_act) 3계열 + refine 세로 점선, RT-FK 교차검증 배지 |
| `tools/fk_replay.cpp` | **신규(2026-07-30)** — DataLog(관절 로그) → task-space 변환기. `q_ref`/`q`를 **로봇과 같은 `FullDynControllerRT.o`를 링크한 `ToolPosAt()`**(신설, `ToolRotAt` 미러)으로 FK → `ref_*`/`act_fk_*` 6열 append. `act_fk_*`는 RT가 기록한 `tcp_*`와의 **상시 교차검증용**(실기 24k행에서 max **0.002 mm** = CSV 반올림 잡음). `make fk_replay` |
| `tools/traj_plot_selfcheck.py` | **신규(2026-07-30)** — 합성 접근(droop 37 mm·0.65 재조준·IK 잔차 2 mm)을 `plot_traj`에 직접 주입하는 자가검증 — 은퇴한 RECT 빌린 스모크 입력의 대체(`3bc5336`) |
| `App/Indy7/run.sh` + `tools/install_ecat_op_fifo.sh` | **E34 자동화(2026-07-31, `6c090ae`)** — run.sh가 백그라운드 워처로 `EtherCAT-OP` 커널 스레드 출현을 대기(0.5 s×180) 후 고정 명령 wrapper `/usr/local/sbin/ecat-op-fifo`(sudoers NOPASSWD)로 **SCHED_FIFO 40** 부여. **1회 설치**: `sudo bash tools/install_ecat_op_fifo.sh`(미설치 시 앱 콘솔 WARN + 수동 명령 안내) |
| `CRobot/ConfigRobot.{h,cpp}` | `[SYSTEM] KF_n` 파싱(`vKf`, 키 없음=0.0=기능 꺼짐) — 마찰 FF 배선의 cfg 쪽 절반(`00c6d2c`) |
| `tools/gate2b_bridge_test.cpp` | 브릿지 단독 하네스(EtherCAT/로봇 불필요) — stdin 명령 p/숫자/v/q, RT 소비자 대역 poller |
| `tools/test_gate2b_bridge.py` | 합성 검증 드라이버 — 실브릿지+실 pick_logic 노드 vs 합성 `/detections`·`/pick_target_base` 피더. C1 메뉴 / C2 param ack / C3 `ros2 param get` 실증 / C4 lock / C5 게이트 통과 goal(z=0.27). E1~E3 방어 패턴 이식 |
| `tools/test_phase3_smoke.py` | **Phase 3 정본 스크립트** — P0 preflight(rtprio/memlock/슬레이브/중복실행) → P1 파이프라인 기능적 대기 → P2 앱+OP → P3 홀드(`--hold N`, E9 규칙) → P4 in-app 'p'/선택/'v' → P5 축별 SDO → P6 SIGINT DeInit 검증. `--app-only` 격리 스테이지. 앱 실행에 `MALLOC_ARENA_MAX=2`+`stdbuf -oL` |
| `App/Indy7/Indy7Ctrl.cpp` (keyboard) | E8 개행 가드 — '\n'/'\r'는 키로 취급 안 함 |
| `App/Indy7/INDY7.cfg` | **전 축 `AUTO_SERVO_ON=0`**, `ENABLE_CONTROLLER_AT_STARTUP=0`, 5태스크(VS 태스크 제거), 경로 로컬화, DC 실험용 주석 템플릿 — ⚠️ **이후 오퍼레이터 상태로 운영(커밋 안 함)**: 현행 `[TASKn] CPU=3`(D10) · **KD_3/4/5 28/25/20 · KF_3 5.0/KF_4 3.0**(2026-07-31) — 값의 정본과 근거는 §9.5 타임라인 표 |
| `EMasterApp/Device/EcatSlaveBase.{h,cpp}` | `RegisterPDOEntry` **UINT32→INT64** (내부 `<0` 체크도 unsigned라 이중 사망 상태였음) |
| `EMasterApp/Device/Slave{CIA402Base,NRMKEndTool}.cpp` | **죽어있던 PDO 실패 가드 27개 소생** — signed 임시변수 캡처(대입식이 unsigned면 반환형만 고쳐도 무효) |
| `CRobot/AxisCIA402.h` | `GetEcatSlave()` 접근자 (DC ref 지정용) |
| `tools/gate0_rtposix_test.c` | 1 ms 주기 지터 스모크(RT/NRT 폴백, warm-up 스킵) |
| `tools/patches/rtposix-aarch64-stackmin.patch` | RT-POSIX 스택 버그 패치 보존본 |
| `tools/calib_grab.py` | Phase 5 캡처 grabber — 파이프라인 압축토픽 구독, **태그 검출시에만 저장**(이미지 N ↔ 앱 pose_rPe_N lockstep 강제), camera_info 1회 저장(D12: 파이프라인과 동일 카메라 모델) |
| `tools/calib_cpo.py` | 이미지→pose_cPo (36h11 + solvePnP IPPE_SQUARE + 모호성비 리포트). 코너 정밀화 **per-image 폴백**(B8) |
| `App/CalibUtils/kv260/` | 16자세 데이터셋 + solve 결과(rPc.yaml/txt, 시각화) — 원저자 예제 데이터(상위 디렉토리)와 격리 |

### ros2_ws (main)

| 파일 | 변경 |
|---|---|
| `src/pick_logic_pkg/.../pick_logic.py` | **v2**(2026-07-26): L2 스코어 선택(conf/중앙/크기 가중) + `desired_class` LIVE 파라미터('':자동) + L3 안정성 3프레임·히스테리시스·5프레임 생존 + L4 근접 person 래치 → **v3(2026-09-04)**: 점수식·hysteresis·AUTO 삭제. `desired_class` **필수**(''=`no_class_selected` invalid), 그 클래스의 인스턴스 1개를 confidence 최대로 채택 → 최근접 이어받기(match_px 게이트는 2개 이상일 때만) → 상실 전까지 고정(sticky). 게이트·metrics·yield CSV 계약 불변 |
| `config/pick_logic.yaml` | v2 파라미터 블록(주석 포함). **2026-07-27 `person_guard_enable: false`**(오퍼레이터 결정, E12 — 재활성 대안 주석 포함) → **2026-09 person 블록 전체 삭제** → **v3: `score_w_*`·`hysteresis_*` 삭제** |
| `launch/pick_place_vitis_ai.launch.py` | `base_to_camera_tf` placeholder → **캘리브 실측치**(quaternion, §1.3). v3: launch 인자 `desired_class`(기본 '') — 로봇 앱 없이 metrics 돌릴 때 `desired_class:=apple` 필요 |
| `tools/vision/test_pick_logic_v2.py` | 카메라-불필요 합성 검증(실노드+합성 DetectionArray). v3: 6건 — S1 선택 없음→invalid / S2 desired_class / S3 desired 부재 / S4 confidence 최대 채택 / S5 같은 클래스 2개 sticky / S6 가림 후 단독 재획득. **6/6 PASS(2026-09-04)** |

## 7. 발견한 버그·함정 대장

**RAON-RT/RT-POSIX 소스 결함** (우리가 픽스/회피한 것):

| # | 결함 | 처치 |
|---|---|---|
| B1 | RT-POSIX `DEFAULT_STKSIZE` 64K < aarch64 `PTHREAD_STACK_MIN` 128K → 기본스택 태스크 생성 전멸(EINVAL) | 소스 클램프 패치(+보존). RAON 본체는 항상 2 MB라 잠복해 있었음 |
| B2 | RT-POSIX `wait_next_period` overrun 판정 부등호 반전(미래 deadline을 overrun으로 셈) | 카운터 불신 — 판정은 dt 실측으로 (미패치, 기록만) |
| B3 | `RegisterPDOEntry` unsigned 반환 → 실패 가드 27개 + 내부 에러로그 전부 죽은 코드, 실패 시 offset 0xFFFFFFFF로 다음 사이클 OOB | INT64 + signed 캡처로 소생 (§14.2 #1) |
| B4 | 저장소에 저자 x86 빌드잔재(obj/*.d, bin, lib) 커밋됨 → aarch64에서 make 즉사 | 브랜치에서 untrack + `make clean` 선행 규칙 |
| B5 | `INDY7.cfg`에 DC 키 부재 → 슬레이브 DC 전부 OFF가 저자의 실구동 상태 | 그대로 브링업(D8), DC 켤 준비만 완료 |
| B6 | `make_csv`가 저자 홈 절대경로(`/home/raimlab/...`) 하드코딩 → 궤적 CSV 저장 전부 실패 | 로컬 경로로 픽스(`d94c1c1`) — 추종오차 분석 도구 복구 |
| B7 | `IsTrajectoryDone()`은 궤적 종료에 **settle 조건(전 조인트 0.15 rad 이내)까지 요구** → 정상상태 droop(마찰) 하에선 영원히 미완료. 저자 rect SM이 done 대신 사이클 카운터를 쓴 이유 | `IsTrajectoryRefDone()`(참조 생성 완료 기준) 신설 — refine·접근 완료 게이트는 이걸 사용(`0668ed2`) |

**개발환경 함정** (재발 방지 규칙):

| # | 함정 | 규칙 |
|---|---|---|
| E1 | `ros2 run` 래퍼에 SIGTERM → 노드 고아화. **유휴 rclpy spin은 SIGTERM을 삼킴**(EINTR 재대기, 콜백 없으면 python 핸들러 실행 기회 없음) | 하네스는 `start_new_session` + 그룹 SIGTERM 후 **무조건 그룹 SIGKILL** 마무리. (launch 환경은 트래픽이 executor를 깨워 무관) |
| E2 | SIGKILL된 노드는 미등록해제 → **ros2 daemon 그래프 캐시 오염** — half-dead 노드가 graph-count 대기를 속이고 `param set`이 죽은 노드로 감 | 준비판정은 **기능적 핸드셰이크**(빈 프레임→응답 수신)로. preflight에서 `ros2 daemon stop` + param set 재시도 |
| E3 | `pgrep/pkill -f`는 패턴이 **자기 셸 명령줄에 있으면 자살** | 킬 전 `/proc/PID/comm` 검사(python3/노드명만), 확인 명령은 별도 호출로 분리 |
| E4 | pick_logic 실행파일명은 `pick_logic` (노드명 `pick_logic_node`와 다름) | — |
| E5 | **Humble include의 CycloneDDS 지뢰**: `include/dds/features.h`·`include/idl/{string.h,endian.h}`가 glibc 표준 헤더를 섀도잉 — 저자 Indy7_ROS2 Makefile의 `find -maxdepth 1` -I 샷건이 이 디렉토리들을 포함해 libstdc++ 컴파일이 `__throw_length_error` 류로 붕괴(저자의 `-include cstdio/cstring/cstdlib`는 이것의 밴드에이드) | -I는 실제 include하는 패키지만 **화이트리스트** (`ROS_PKGS` 변수) — 섀도 디렉토리 원천 배제, 누락 시 명시적 에러로 드러남 |
| E6 | **에이전트/VSCode 셸은 PAM limits 미적용**(limits.d 이후에 뜬 세션이라도 데몬 계열은 안 탐): rtprio=0 → RT-POSIX 태스크 생성 EPERM 전멸, memlock 유한 → E7 | `sudo prlimit --rtprio=98:98 --memlock=unlimited:unlimited --pid <claude PID>`로 조상 프로세스에 주입(자식 상속) 또는 재부팅 후 새 세션. 스크립트 preflight가 둘 다 검사 |
| E7 | **`mlockall(MCL_FUTURE)` + ROS2/DDS = 잠금 폭발**: 스레드별 glibc malloc arena(가상 64 MB)·8 MB 스택이 전부 물리 잠금 → memlock 490 MB 한도 초과 시 **할당 자체가 실패**(`std::bad_alloc`, OP 중 앱 사망). Gate 2b 하네스는 mlockall이 없어 잠복 | memlock **unlimited 필수** + 앱 실행 환경 `MALLOC_ARENA_MAX=2`. **사건**: 이 크래시가 유발한 동시 teardown(마스터 해제+DPU+realsense) 중 **커널 하드 락업 1회**(저널 무flush 단절, 21:56:11) → 보드 재부팅. 트리거(앱 비정상사망) 제거로 재발 방지; 재발 시 시리얼 콘솔 필요. 슬레이브는 SM watchdog으로 SAFEOP 낙하 = 버스 측 안전거동 정상 |
| E8 | **파이프 stdin 키 레이스**: `"p\n"` 두 글자가 한 번에 도착하면 keyboard 태스크가 `m_cKeyPress`를 'p'→'\n'으로 즉시 덮어써 1 kHz 소비자가 'p'를 못 봄(실터미널은 사람 속도라 저자 미조우) | 앱 keyboard 태스크에 '\n'/'\r' 무시 가드 + 스크립트는 개행 없이 단일 문자 전송 |
| E9 | IgH `ethercat master`의 **Lost frames는 감소 가능한 파생 통계**(지연 프레임이 뒤늦게 계상되면 ↓). PREOP→OP 활성화 천이 자체가 프레임 2개 정도 유실(정상) | 유실 판정은 **OP 도달 후 재베이스라인** + **증가 시에만 FAIL** |
| E10 | **E-stop = 로봇측 버스 전원 차단**: eth0 Link DOWN + 슬레이브 0 → 앱은 "There are 0 Responding Slaves!"로 초기화 실패. 버튼이 물리 래치라 해제 전까지 지속 | 버튼 비틀어 해제 → 슬레이브 7 복귀·전축 0x0220 fault-free 확인(2026-07-26 실측) 후 재실행. E-stop 실효성 테스트로는 오히려 정석 경로 |
| E11 | **rosidl typesupport는 런타임 dlopen** — 실행파일의 RUNPATH(-Wl,-rpath)가 dlopen 검색에 적용되지 않아 env 미소스 실행 시 `libmy_interfaces__..._fastrtps_cpp.so` 로드 실패(브릿지만 비활성, 앱은 진행) | 앱 실행은 항상 `App/Indy7/run.sh`(ROS env source + MALLOC_ARENA_MAX=2 + rtprio/memlock preflight) |
| E12 | **person_guard 상시 래치 FP(2026-07-27)**: 테이블 우측 가장자리의 케이블/그림자 세로 영역이 conf 0.32~0.41 person으로 **36% 프레임**에서 오검출(area 0.08~0.10 ≥ 0.06) → 사람이 없어도 2 s 래치가 계속 갱신되어 타깃 전면 무효. 증거 프레임 확보 | **오퍼레이터 결정으로 guard off**(yaml). E-stop이 주 안전장치. 재활성 시 `person_min_confidence: 0.5`면 이 FP는 걸러짐(실측 FP max 0.41 — 근접 실사람은 통상 그 이상). **→ 2026-09 guard 코드 자체 삭제** |
| B8 | `CORNER_REFINE_APRILTAG`이 기본 검출기가 잡는 마커를 **떨어뜨릴 수 있음**(image0012). cPo 하나가 비면 기존 solver의 `sorted(glob)` zip 페어링이 **그 뒤 쌍 전부를 조용히 어긋나게 함**(사전순 정렬이라 1,10,…,16,2,… 순서 — 인덱스 결번 시 재앙) | `calib_cpo.py`가 정밀화 방법을 per-image 폴백(APRILTAG→SUBPIX→기본)으로 시도해 결번 자체를 방지 + 16/16 강제 |
| **E13** | **접근 1차 시도 테이블 충돌(2026-07-27 01:32)**: `SetTargetPosePositionOnly`가 **점 구속 하나뿐인 3-DOF 구속 IK**라 RBDL이 1000 iter 동안 3차원 nullspace를 방랑 — "현재 자세 유지"는 주석뿐, 해가 현재 관절에서 임의로 먼 구성으로 수렴 가능. 관절공간 quintic이 그 델타를 3 s에 쓸며 **TCP가 테이블 관통 호를 그림**. quintic 최고속 구간(t≈2 s)에서 Axis4 드라이브 FAULT(0x0000), 공구 스크래치 경미. goal 자체는 완벽했음(banana 0.652,-0.058,0.269 = 테이블+17 cm) — 인식이 아니라 모션 생성 결함. 'a'가 멀쩡했던 건 full 6-DOF 구속(SetTargetPose)이었기 때문 | 3중 방어(`61a4a86`): ① orientation **soft** constraint(w=0.3; hard로 걸면 손으로 잡은 시작 자세에 과민 — 합격판정은 RBDL 수렴플래그가 아니라 **FK 잔차<2 mm**로, soft 잔차는 플래그를 영원히 false로 두므로) ② **Δq 게이트**: 한 관절이라도 2.0 rad 점프 시 REFUSE ③ **T 스케일링**: quintic 피크 관절속도 ≤0.6 rad/s로 T 자동 연장(≤10 s). 부수: 접근/refine SM의 중복 StartJointTrajectory 제거(스케일 T 덮어쓰기 방지), StartRefine이 Pose 전체 수령(구버전은 rotation identity 잠복) |
| E14 | **desired_class는 pick_logic의 LIVE 파라미터라 앱보다 오래 산다**: 앱 재시작 후에도 이전 세션의 orange가 남아 부팅 즉시 stale 타깃 lock 방송. lock-watch가 세션 개념 없이 상시 방송 + 검출 깜빡임 무디바운스 → 초당 LOCKED/lost 스팸 | 브릿지(`61a4a86`): 방송을 **메뉴 선택~GOAL READY 사이로 한정** + lost 1 s 디바운스 + **worker 기동/DeInit 시 desired_class 자동 초기화**(DeInit은 500 ms 1회 best-effort — teardown 신속 유지 E7) |
| E15 | **j→v 브레이크 트랩(2026-07-27)**: 브레이크 잡힌('j') 상태의 'v'는 접근+refine 전체가 얼어있는 팔을 상대로 돌아 거대한 "droop"으로 읽힘 — "전 물체 접근 실패"의 실체가 이것(오퍼레이터가 진단). 이연(deferred) 처리하면 'h' 순간 예상 못 한 모션이 튀는 게 더 위험 | 접근/'b'에 **IsServoOn 전축 가드** — goal은 **폐기**(이연 금지) + "h 먼저" 안내. 같은 원칙이 스테이징 레그2 취소 규칙에도 적용(모드/서보 변동 시 stash 폐기) |
| E16 | **기계변환 URDF의 프레임 축 함정**: tcp 프레임의 공구축이 +Z가 아니라 **−X** (관절 프레임 순열). F/T 페이로드를 관례대로 z=−0.060에 달았더니 질량이 옆으로 매달려 **수직 유지가 오히려 악화**(오퍼레이터가 즉시 감지). 캘리브 부산물 eMo x=−29.4 mm(플랜지면 −10.9 + 스택 18.5)로 −X 확정 — **TCP 원점은 태그면 안쪽 29 mm**(호버 실클리어런스 ≈ 150−29=121 mm) | CoM (−0.021,0,0)으로 교정 + URDF에 축 경고 주석. 교훈: 이 URDF에서 "tool 방향" 가정 금지 — 프레임 관련 삽입은 FK/캘리브 부산물로 축부터 실증 |
| E17 | **터미널 Jacobian servo 사고(2026-07-27)**: "한 번에 접근" 시도로 접근 말단을 누적형 DLS servo(적분 복원)로 교체 → 손목 특이점 근처에서 DLS가 **J5를 9.39 rad 감아** 사과는 이상한 곳으로, 나머지는 전부 REFUSED. 감김 워치독·태스크 진척 워치독·손목 nullspace 처리 없이 누적 적분만 넣은 게 원인 | **전면 리버트**(git checkout, URDF 픽스는 보존 확인). 재시도 전제조건 3종을 부검으로 명문화: ① 관절 감김 워치독 ② 태스크 진척 워치독(오차 미감소 시 중단) ③ 손목 특이점 nullspace 처리. 당분간 refine(12 mm, 2~6 pass)이 프로덕션 — "여러 번 시도"는 속도 IK 탓이 아니라 **stiction 바닥의 반복 보정**이 정체 |
| E18 | **soft-R×refine 회귀(2026-07-27)**: E13 방어로 넣은 soft-R(w=0.3)이 refine 재타겟에도 걸려 **~10 cm급 편향에서 위치/자세 gradient 평형으로 IK 정지**(apple 155 mm 잔차 → pass2 97.8 mm 정체). refine 편향은 소규모 국소 이동이라 R 유지가 불필요 | refine 재타겟은 `oriWeight=0.0`으로 호출(접근 1차 명령만 soft-R). 같은 평형 메커니즘이 접근 실패(residual 133.7 mm)의 원인이기도 → D13 ready-seed가 시드 차원에서 제거 |
| E19 | **합성 부트스트랩의 기형 자세(2026-07-27 저녁, ready-seed v1)**: 시드 사다리+순수 위치 IK가 뽑은 q_ready가 **도달·한계·Δq 전부 합격인데 물리적으로 기형**(J4 2.61/J5 1.65 rad — 손목이 몸통 쪽으로 접힘, 5/8 코너도 통과). 스테이징이 그 자세로 이동하니 "로봇 근처에서 괴랄하게 회전"으로 관측, 자가충돌 위험도 있었음. 교훈: **기하 조건만으로는 자세의 sane함을 판정할 수 없다** — 사람이 시연한 브랜치가 유일하게 신뢰 가능한 앵커 | v2(`f9a411b`+`f3c4569`): ① HOME 기록=즉시 앵커 re-base+파일 영속화(다음 부팅 1순위 시드) ② 사다리 후보 \|J4\|,\|J5\|>2.0 즉시 거부 ③ 부팅 로그에 앵커/합성 출처 표기 ④ **기록 즉시 커버리지 검증**: 8프로브를 사이클당 1해석(amortized — 일괄 9해석은 1 kHz 루프 정지)으로 재검증 → `anchor coverage: n/8` + 미달 좌표 로그. 사람의 역할은 "정상 브랜치 1비트"뿐, 도달성 판정은 계산이 담당. 운영 수칙: **세션 첫 행동 = 자연스러운 자세에서 HOME 기록**(전 생애 1회 — 파일 영속, 접근 시 그 자세에 있을 필요 없음). **후속: 손목 가드로도 부족 판명(2차 합성도 J2 −2.80 팔꿈치 접힘) → v3(E20)에서 사다리 완전 제거 — q_ready는 오직 시연 자세**. **v3 실기(20시)**: 파일 앵커 부팅([operator anchor] 6/8, 감김 turn 로그 정상), 괴랄 궤적 소멸, apple/mustard/tennis 전부 직행(dq 0.17~0.79) refine 수렴 9.6~11.9 mm — **모션 스택 정상**(도달점이 물체를 벗어난 건 E21 베이스 이동) |
| **E20** | **멀티턴 카운터 감김 = 랩 불일치 폭탄(2026-07-27 저녁)**: 기록된 HOME이 **J1 −6.21 / J4 −18.93 rad**(물리 자세는 0 근처 — E17 서보 감김 + E-stop 버스전원 사이클이 멀티턴 인코더를 재참조하며 통째 회전수가 카운터에 잔류, **랩은 기록/재생 시점 간 달라질 수 있음**). FK·중력보상·IK는 2π 주기라 제어는 멀쩡히 돌지만: ① 한계검사가 감긴 값을 거부 → 앵커 실패·refine 전체 오거부 ② canonical 목표와 감긴 카운터를 섞으면 **관절 목표가 실제 다중 회전 명령이 됨** — 'b' 괴랄 궤적의 정체, staged 'v'였다면 J4 ~20 rad 회전 시도 | v3(`0662b98`) 원칙: **판정은 물리 자세(0-접힘)로, 실행은 라이브 카운터 좌표계로** — 모든 관절 목표를 명령 직전 현재 카운터 쪽으로 접어 **관절당 이동 ≤π 보장**. `CheckLimitsPhysical`/`FoldTowardRef` 분리, 적용처=SetTargetPosePositionOnly(refine 오거부 해소)·SolveReadyIK 출력·스테이징 레그·'b' 재생·TickAnchorVerify. SetReadyAnchor는 감긴 입력을 접어서 수용(+관절별 감김 turn 수 로그), 저장은 canonical만. 합성 사다리 제거(앵커 없으면 live-시드 폴백+안내) |
| **E21** | **로봇 베이스 물리 이동(2026-07-27 20시)**: v3 실기에서 접근이 FK상 ±1 cm 도달인데 실물은 물체를 ~11 cm 벗어남 + banana/orange가 갑자기 r-게이트 탈락. **손-프로브 판별**(사과 꼭대기 접촉 's'): 로봇 FK (0.744, −0.138) vs 카메라 보고 (0.681, −0.235) → **Δ(+6.3, +9.7) cm, z는 8 mm 일치** = 테이블 평면 강체 이동(기울지 않음). 원인 = E17/E19 사고 시 팔 요동으로 베이스가 밀림(오퍼레이터도 육안 확인). 캘리브 rMc는 "옛 베이스" 기준이라 전 좌표가 계통 오프셋 | ① **베이스 볼트/클램프 고정**(재발 방지 — 안전) ② 멀리 떨어진 3점(apple+tennis+banana류) 손-프로브 ↔ LOCKED 좌표쌍으로 **2D 강체(yaw+t) 최소자승 fit** → launch TF에 합성(빌드 불필요, 파이프라인 재시작만). 잔차 1~2 cm 초과 시(카메라도 움직인 비강체) 16-pose 정식 재캘리브(도구 준비돼 있어 20~30분). 교훈: **접근이 "자신 있게 빈 공간"으로 가면 모션이 아니라 좌표(외부 캘리브) 붕괴부터 의심** — 판별은 손-프로브 5분. **TF 보정 후 apple/tennis 접근 성공으로 복구 검증(21시대).** **복구 완료(같은 날 저녁)**: 데이터 함정 2회를 강체성 검사가 잡아냄(① 락과 프로브 사이 물체 이동 — 거리 불변량 위반 최대 13 cm ② 접촉 해제 후 's' — TCP z가 꼭대기+30 cm, 세 프로브 모두 공구좌표 동일점(~37 cm) 접촉으로 역산됨) → sticky-float로 접촉 유지 프로브 재취득 → **2점(apple+tennis) fit 잔차 0.2 mm·상호거리 0.4 mm 일치, banana 검증 2.1 cm(길쭉한 물체의 bbox중심 vs 접촉점 모호성), 3점 fit과 ≤1 cm 일치** → yaw +5.14°/t(+4.5,+10.4) cm 확정, launch TF 교체(§1.3). 베이스 볼트 고정 완료. 주의: 보정 후 banana는 신좌표 r=0.84로 r-게이트 정당 거부 — 5 cm 안쪽으로 옮겨야 접근 가능(→ E22에서 게이트 자체를 재보정) |
| **E22** | **게이트 상수도 옛 프레임 유물(E21 후폭풍, 21시대)**: TF 보정 후 orange/mustard/banana가 박스(x≤0.85)·r-게이트(0.80)에 걸렸는데 손-유도로는 전부 도달 가능. 두 상수 모두 **밀리기 전 좌표계에서 어림잡은 선**이라 5.14°/11 cm 프레임 변화 후엔 오보정(r 0.80의 근거였던 "orange 0.87 스톨" 라벨도 옛 프레임 값). 부수 규명: **z 마진 실측 10 cm = 명령 15 cm(TCP 기준) − 공구 스택 ~5 cm**(프로브 z델타 +4.5~6 cm로 실측) — 설계대로이며 파지 단계 TCP 재정의 때 물리 기준화 | (`e69555c`) 박스를 fit 변환으로 재베이스 x[0.30,0.94] y[−0.37,0.63] z 불변, r-게이트는 0.92 조잡 안전망으로 강등 — **도달 판정의 심판은 ready-seed IK**(목표별 해석, 불가 시 무동작+명확 거부). 유의: hover(꼭대기+15 cm, 수직계열 R)는 터치보다 reach 요구가 큼 — IK가 soft-R을 ~40°까지 굽혀 시도하고도 거부하면 그게 진짜 hover 한계 |
| **E23✅** | **원거리 soft-R 정체 재발 + 'd' 허위 오차(21시대)**: 게이트 완화 후 banana(r 0.83)가 잔차 235 mm, mustard(r 0.86)가 62 mm로 거부 — **가까운 쪽이 더 크게 "모자란" 것은 reach가 아니라 E18 gradient 정체**(잔차 = 부족량이 아니라 솔버가 멈춘 위치). 계산상 banana는 공구 ~40° 기울이면 hover 가능. 부수: 'd'(LogDistanceError)가 Target 0,0,0 대비 0.79 m/214° 출력 — ready-seed 경로가 `goal_tcpPose` 레지스터를 안 채워 초기값과 비교된 허위 수치(제어 오류 아님; 실제 오차는 `[REFINE] done` 줄) | (`1694414`) **w 사다리**: soft-R(0.3) 실패 시 같은 q_ready 시드로 순수 위치(w=0) 재시도, 해의 공구 기울기가 ready 자세 대비 **60° 이내일 때만 수용**(초과=extreme tilt 거부). 실패 시 잔차가 진짜 부족량에 가깝게 보고됨. 방어선(폴딩·물리한계·Δq 게이트·스테이징)은 뒤에서 유지 — E13 방랑 위험 격리. + TryReadyApproach가 goal_tcpPose를 채워 'd' 정상화. **실기 검증(22시대): 전 5클래스 접근 성공** — banana 41°/orange 44°/mustard 48° tilt(= 수직-hover 한계 밖임을 자세가 표시; 손목이 tool길이·sinθ만큼 안으로 들어와 도달), apple은 soft-R 경로로 평행 유지. refine 최종 4.9~10.6 mm. 기울기 개선은 파지 단계 top-down 고정 R에서 정식 해결 |
| **E24✅** | **파일에서 나온 손목 0점 = 중력보상 폭주(2026-07-27 22:47)**: AXIS4/5만 `HOMING_METHOD=1`(eAxisHomeStartPos) → 첫 eServoIdle에서 `home = raw_start − POS_BEFORE_EXIT` 래치(`CRobot/AxisNRMKCore.cpp:75`) → **시작 각도 = cfg의 PBE를 각도로 디코드한 값**이며 팔의 실제 위치와 무관. 성립 조건은 "**시작 자세 == 직전 종료 자세**"인데 아무도 검사하지 않음. 22:16~22:29 성공 런이 접힌 자세로 종료하며 DeInit이 PBE 갱신(`CRobot/Robot.cpp:87-94`) → 재부팅 사이 팔을 수직으로 세움 → 22:47 실행 시 모델은 J4를 **−93.29°**로 믿음(실제 −10.3°, **오차 81.7°**) → `h` 순간 중력토크가 0이어야 할 손목에 **2.22 N·m** 허위 토크 → 손목이 **능동적으로 −93.6° 넘어감**(처짐이 아니라 구동됨) → 원위 질량이 어긋나 어깨·팔꿈치도 주저앉음(sticky-float는 0.15·Kp·0.03 rad 상한이라 못 잡음). 무너진 자세에서 HOME 기록 → `SetReadyAnchor`의 유일한 게이트가 관절한계(J4 3.0207 < 3.0543 통과) → **m_EReady 재베이스 = 잣대가 뒤집힘** → 같은 물체가 119/136/139° tilt로 전량 거부. 한 달간 무사했던 이유: 6/18~7/26 PBE가 계속 "손목 곧은" 값(J4 −1.33°, J5 −0.006°)이었고 오퍼레이터가 늘 팔을 세워두고 시작 — **"접힌 채 종료 → 세워서 시작" 조합이 처음** | ① **AXIS4/5 → `HOMING_METHOD=3`**(eAxisHomeManual, AXIS0~3와 동일한 고정 `HOME_POSITION_OFFSET` 기준 — `App/Indy7/Indy7Ctrl.cpp:426`): 파일 유래 0점 자체를 제거, **종료 자세 규칙 폐기** ② 검증은 **엔코더 카운트 고고학** — pose 출력은 `FK(m_Q)`라 자기 0점을 검증할 수 없음(순환논법, 잔차 3e-14는 어떤 HPO에서도 나옴): HPO_4는 5/28 park raw와 **정확히 1회전 modulo 437카운트(0.024°)**, HPO_5는 2614카운트(0.142°), 5/17~7/26 커밋된 park 7개가 J4 3.53°/J5 2.62° 밴드에 군집 → **잔차 0점 오차 ≤2.8° = 허위 토크 ≤0.16 N·m**(폭주값의 1/14) ③ 교차확인: cfg 카운트로 예측한 J5 **−32.0858°** ↔ pose 역산 **−32.0859°** = 디코드·조인트매핑·FK 체인 정상 + 롤축 무이동 ④ 실기: 플랜지면 수평계 **23°**(예측 21°, 실패가설 10~12°) → `g`/`h` 정상 유지 → **전 5클래스 재현**. **미구현 가드**: 래치 시 두 기준 잔차 교차검증(당시 91.96° → 즉시 정지 가능했음), `h` 무장 전 전축 `IsHomeSet()` + 모델이 믿는 q/FK 출력, `SetReadyAnchor` 자세 급변 거부 + temp→rename+`.prev`(좋은 앵커가 복구불능으로 소실됨), DeInit의 PBE 샘플링을 브레이크 낙하 **이전**으로. 부수 발견: `POSITION_LIMIT_U/L=±35.0`인데 q가 라디안 → **소프트웨어 관절 리밋 부재**(35 rad=2005°), `ONE_TURN_REF=360.0`은 사문(`Axis.cpp:506-512`가 revolute면 미할당), `ABSOLUTE_ENCODER`는 파싱만 되고 호출처 없음 |
| **E25✅** | **`w=0` 폴백의 브랜치 사고(2026-07-28 01:46, 미해결)**: E23 "사다리"가 실제로는 2단(0.3 → 0)뿐이라 soft-R 실패 시 **자세를 전혀 구속하지 않는 순수 위치 해**로 점프 → 수용 여부가 하강 경로의 우연에 좌우됨. 새 앵커(툴이 수직에서 9.8°)로 mustard(베이스원점 3D 0.912 m) **36° 통과** / **banana(0.877 m — 더 가까움) 87° 거부** / orange(0.946 m) 102° 거부 — **더 쉬운 목표가 더 크게 튄 것 자체가 물리가 아니라 솔버 아티팩트라는 증거**. mustard는 어제 48°→오늘 36°로 오히려 개선(앵커가 나빠서가 아님) | (`9aca402`) **w 사다리 구현**: 0.3→0.1→0.03→0.01→0 중 위치 잔차 <2 mm를 만족하는 첫 단 채택 + w=0까지 내려갔으면 **그 해를 시드로 soft-R 재정제**(위치는 이미 만족 → nullspace만 써서 자세를 당김). 60° 캡을 **모든 완화 단**에 적용(이전엔 w=0에만). 1 kHz 예산 실측 로그 + 0.8 ms 초과 시 WARN. **⚠️ 실기 결론이 예상과 반대**: 재시험에서 5클래스 전부 **첫 단 w=0.30, 1 solve, 0.20~0.46 ms**로 풀려 **사다리가 한 번도 발동하지 않았다**. orange가 어제 (0.882, 0.050, 0.338)에서 102°로 거부됐는데 오늘 거의 같은 (0.878, 0.051, 0.335)에서 첫 단 통과했고, banana는 오히려 더 먼 위치(0.870 vs 0.814)인데도 통과 → **거부를 고친 것은 앵커이지 사다리가 아니다**("앵커 재기록은 부차적, 사다리가 구조적 수정"이라던 판단이 틀렸음). 사다리는 all-or-nothing 절벽을 없앤 **미검증 보험**으로 유지. **RT 예산 후속(같은 날 오후, `81c9205`→`cd4d259`)**: 사다리가 처음 내려간 mustard에서 **4 solve = 31.99 ms** WARN 발생 — 비용은 전부 *실패하는 단*에 있음(수렴 solve는 ~10 반복 0.25 ms, 정체 solve는 한계까지). **RBDL `CS.num_steps`는 출력 필드**("iterations performed")이고 입력 한계는 **`CS.max_steps`(기본 300)** → 원래 코드의 `CS.num_steps = 1000; // 수렴 기회 늘리기`부터 **줄곧 무효**였고 첫 수정도 같은 필드라 무효. 같은 착각이 있는 다른 3곳은 **값을 고치지 않고 주석만 달았음**(거기를 `max_steps=1000`으로 "고치면" 300 스텝 경로가 **35 ms 정지**가 됨). 진짜 필드에 40을 걸어보니 시간은 32 → 1.88/7.82 ms로 잡혔지만 **해가 망가짐** — 굶은 solver는 양방향으로 오답을 냄(될 단이 실패하고, 덜 수렴한 점이 위치 2 mm만 통과해 채택; mustard가 dq 1.11 rad 직행 → dq 2.98 rad 스테이징 실패, refine 16.7 mm 정체로 퇴행) → **되돌림**. warm-start는 예산과 무관하고 q_ready 브랜치를 보존하므로 유지. **결론: 밴드에이드로 못 고친다 — IK를 RT 스레드 밖으로 빼고 RT엔 완성된 관절 벡터만 넘기는 것이 정답**(브릿지의 non-RT 워커 + 별도 RBDL 모델 인스턴스; 모델이 스레드 안전하지 않음). 현 위험도는 낮음: 오퍼레이터 명령 시 1회성, 그 순간 팔은 grav-comp로 정지, 버스 실측 무사(Operation, Tx == Rx, Lost 0 — 단 *사이클 건너뜀은 Lost frames에 안 잡힌다*는 점 유의) |
| **E26✅** | **refine 진동·미수렴(2026-07-28 01:56, 미해결)**: tennis_ball refine이 39.5→36.3→23.1→18.6→**26.2**→13.1 후 최대반복 도달, **잔차 14.5 mm**로 종료(목표 (0.705,0.159,0.314) vs 실제 (0.700,0.148,0.323) = 11 mm 이탈). 같은 세션 apple 4.4 mm / mustard 4.3 mm는 정상 수렴 → 목표 의존. 진행 중 `R held within 11~17°`로 자세가 계속 흔들림(refine은 oriWeight=0인데 해가 자세를 흔듦) | (`9aca402`) **원인이 2개였다**: ⓐ **자세 기준점이 엉뚱한 곳을 가리킴** — `TryReadyApproach`가 `GetCurrentPose`로 목표 pose를 만들어 `m_rotation`에 **'v'를 누른 순간의 대기 자세**가 담기고, 팔이 물체로 이동해도 갱신되지 않음 → `R held within 24.4°`는 refine 표류가 아니라 **대기 자세까지의 거리**를 재고 있었음. 여기에 그냥 가중치를 걸었으면 **툴을 접근 전 대기 자세 쪽으로 24° 끌어당길 뻔했다**(개선이 아니라 개악). 신규 `ToolRotAt()`으로 기준을 **해가 실제 도달하는 자세**로 재설정 ⓑ **refine 사다리** 0.1→0.03→0.01→0 — **마지막 단이 곧 기존 동작이라 원리적으로 나빠질 수 없다**(자세 유지가 공짜인 곳에서만 유지, 위치를 방해하면 즉시 내려놓음). E18은 *고정* 0.3이 10 cm급 편향에서 정체한 문제였고 현재 refine 편향은 10~26 mm라 되돌리는 게 아니라 그 사이를 메우는 것. 중간 단 실패는 `abQuiet`로 로그 억제, 성공 단의 w를 로그에 표기. **실기 결과(2026-07-28 16:35)**: tennis **6 pass 실패 14.8 mm → 1 pass 9.4 mm**, apple **6 pass 11.8 mm → 2 pass 8.4 mm**, orange 2 pass 4.9 → 2 pass 5.8 mm(회귀 아님 — 목표 z가 2 mm 다르고 둘 다 stiction 바닥). **전 pass가 첫 단 w=0.10에서 `R held within 0.0 deg`**(이전엔 9~29° 표류). 총 14 pass → **5 pass**. 남은 5.8~9.4 mm는 **stiction 바닥**이므로 파지 <5 mm는 게인/적분 쪽 과제로 이월 |
| **E27✅** | **IK 브랜치 플립 — 위치·자세 어느 게이트로도 안 잡힘(2026-07-28 17:34~17:46)**: mustard가 시드 자세에서 `solution 3.13 rad away (J0) — staging via q_ready first` → `still 3.13 rad away after staging`로 반복 실패. 정체는 **어깨 플립**: 베이스를 ~180° 돌리고 나머지 관절을 되접으면 **같은 점에 거의 같은 툴 자세**로 도달하므로, 위치 2 mm 테스트도 **60° 자세 게이트도 브랜치 변화를 볼 수 없다**(실측 J0 3.13 rad인데 tilt 1°). 게다가 **스테이징은 이 경우를 원리적으로 못 구한다** — 스테이징은 팔을 q_ready로 옮기는데 갭이 측정되는 기준이 바로 q_ready라서, "팔이 q_ready에서 멀 때"만 유효하고 "해가 q_ready에서 멀 때"는 무효. 옛 코드는 매번 3초를 들여 그 사실을 재발견했다. **★ 더 중요한 발견**: 두 실행의 차이는 **목표 좌표 2 mm뿐**(x 0.828 vs 0.830)인데 하나는 플립, 하나는 정상 해. `SolveReadyIK`는 항상 q_ready에서 시드하고 브랜치 검사도 자세 무관이므로, **이 물체는 브랜치 경계선 위에 얹혀 있고 비전 지터가 어느 쪽에 떨어질지를 결정**한다 — 하루 종일 결과가 오락가락한 진짜 이유 | (`3862570`) `ReadyBranchGap()` 신설 — q_ready 쪽으로 접은 뒤(2π 랩은 브랜치 변화가 아님) 관절별 최대 거리를 잼. **사다리 단의 합격 조건에 "q_ready와 같은 브랜치인가"를 추가**했고, 순수 위치 단은 이유를 밝히며 즉시 거부(스테이저에 넘기지 않음). 실기: 시드 자세에서 3초 헛이동 대신 **즉시 명확한 거부**(J5 2.90 rad), 사과에서는 직행 + refine 1 pass 8.0 mm. **대가**: 기각되는 단이 늘어 사다리가 더 내려가므로 사과→mustard가 **8 solve / 56 ms**(이전 4 / 32 ms) — off-RT 이전의 시급성이 올라감. **미해결**: 경계선 위에 있다는 사실 자체는 그대로. 앵커를 바꾸면 경계가 어디로 움직이는지 = reach map 과제(§9) |

| **E28✅** | **polish 단이 브랜치 가드를 우회(2026-07-28 18:20, reach map이 첫 재현 실행에서 발견)**: E27의 브랜치 검사는 **바닥 단(w=0) 직후**에 있고, 그 뒤의 **polish 재정제는 브랜치 재검사를 받지 않는다**. polish의 채택 기준은 **자세 개선 하나뿐**인데 어깨가 뒤집힌 자세는 오히려 자세가 더 좋은 경우가 많으므로 — **polish는 플립된 해를 적극적으로 선호한다**. 실측: 목표 (0.828, 0.234, 0.300)이 바닥 단 검사를 통과한 뒤 polish가 **J2 갭 2.96 rad**(한계 2.0)짜리 해로 갈아치웠고 하류의 어떤 게이트도 못 잡았다. E27 커밋 자체에 뚫린 구멍이었고, **실기 로그만으로는 영원히 안 보였을 것**(로그에 갭이 안 찍힘) | (`a0a2c26`) polish 결과에 `ReadyBranchGap` 재검사 추가 — 플립이면 **polish를 버리고 정제 전 해를 유지**(목표 전체를 거부하지 않음: 정제 전 해는 정상이었다). 해당 목표 갭 **2.96 → 1.00 rad**, 대가는 tilt 0.0° → 2.9°뿐. 부수: `IkDiag` 신설로 solve 결과(승리 w·solve 수·ms·tilt·갭·축)를 **데이터로 노출** — 지금까지 전부 printf 후 버려졌고, **1 kHz 초과 WARN은 승리 w를 찍지도 않았다**(사다리가 가장 깊이 내려간 바로 그 순간에 가림). 그 WARN도 이제 w를 찍는다 |
| **E29 (미해결·설계상 한계)** | **reach map이 밝힌 진짜 그림(2026-07-28 18:25)**: 박스 안에서 **거부 사유는 사실상 BRANCH 하나뿐**이다 — 28,515셀 스윕(z 0.15~0.35, 1 cm)에서 PASS 27,132 / **BRANCH 1,345** / LIMITS 38 / **TILT 0 / NO_SOLUTION 0**. 즉 **"멀어서 못 간다"는 경우는 박스 안에 없고**, 60° 자세 캡도 한 번도 발동하지 않는다. 위험 지대는 두 곳: ⑴ **먼 +y 바깥 모서리**, 높이에 따라 급격히 커짐(z 0.15 거의 없음 → z 0.35 큰 덩어리) — mustard가 정확히 그 가장자리, ⑵ 베이스 근처 x≈0.30~0.40 블롭(LIMITS 몇 셀 포함). **★ E27의 "2 mm가 성패를 가른다"는 표현은 경계가 날카롭다는 뜻으로 읽히는데, 1 mm 정밀 스윕 결과는 그보다 나쁘다**: y=0.234·z=0.30 라인에서 x 0.814~0.846 구간이 **32 mm 폭의 얼룩덜룩한 혼합 지대**(PASS/BRANCH가 1 mm 단위로 교대, 창 전체 통과율 50.7%)이고 **mustard(x 0.828)는 그 한가운데**. 그래서 **2 mm 옮기는 건 대책이 못 된다** — 동전던지기의 다른 면일 뿐. 정량: ±5 mm 지터에 견디려면 **14 mm 이동**((0.818, 0.224)), ±10 mm면 19 mm, ±15 mm면 25 mm | 단기 운용: **물체를 1.5~2.5 cm 안쪽으로**. 근본: 경계 위치는 **앵커(q_ready)의 함수**이므로 `--seed`를 바꿔 스윕하면 "어느 HOME이 박스를 가장 넓게 덮나"를 계산으로 고를 수 있다(미실행). 그 뒤에도 남는 것은 **RBDL local IK의 구조적 한계** — analytic IK나 브랜치 열거로 가야 근본 해결 |
| **E30✅** | **다중 시드 IK — 실기 검증 완료(2026-07-29 14:41, RAON `2e911a3`)**: E27/E29의 브랜치 플립을 솔버 쪽에서 푼 것. **핵심은 시드가 아니었다.** 설계는 "q_ready에서 8개 브랜치 시드로 팬아웃"이었는데, 실제로 효과를 낸 것은 **1b단계 — warm-start를 거치지 않고 q_ready에서 곧장 w=0을 푸는 solve 한 번**이었다. 사다리의 warm-start 체인은 자세 품질을 만들어내지만(tilt 3° vs 무웜스타트 55°) **바로 그 체인이 E27 목표에서 뒤집힌 골짜기로 끌고 간다**. 스윕 기준 1,220개 회복 셀 중 **시드 팬아웃 기여는 132개(11%)**, 나머지 ~1,088개가 1b단계. **⚠️ 첫 설계는 틀렸다** — 사다리를 "골짜기를 못 바꾸니 잉여"로 보고 들어냈더니 판정은 살았지만 살린 목표가 전부 tilt 53~57°가 됐다. 구조를 **"1단계=기준선 경로 통째, 실패했을 때만 다중 시드"**로 바꿔 회귀를 원리적으로 차단 | (`929aa90`→`2e911a3`) **사전 선언한 합격조건 4개 중 3개 통과**: ①잃은 셀 **0** ②필드 23점 PASS 17→22·BRANCH 6→0 ③최악 시간 55.24→**58.49 ms 실패**(기준선에서 물려받은 사다리 구간이 원인, 시드 구간 최악은 43.62 ms — 2단계를 빼도 안 고쳐짐) ④머스타드 1 mm 창 통과율 50.7→**87.0%**. 박스 커버율 **95.1→99.4%**, BRANCH 1,345→14, LIMITS 38→0. **★ 실기 검증(예측을 먼저 기록)**: 기준선이 `(0.699,0.357,0.392)`에서 `J2 2.70 rad` 거부 → B안이 **같은 목표를 접근**(refine 70.0→6.9 mm, 2 pass). 지도의 사전 예측 `PASS/w=0.00/seed 0/8 solve/gap 0.39 J2/tilt 29°/16.3 ms` vs 실기 `…/14.92 ms` — **전 필드 일치**. 회귀 시험: 5클래스 전부 정상. **버스**: SM 워치독 **100 ms**(7슬레이브 `0x0400`=2498, `0x0420`=1000), 14.92 ms 정체에 **Lost frames 델타 0**. 오퍼레이터 관찰: 동작 매끄러움·충돌 위험 없음. **미해결**: ⑴ 최악 시간은 여전히 예산 초과(빚으로 남김) ⑵ 살린 목표들이 **tilt 47~57°** — hover는 되나 top-down 파지엔 부적합할 수 있음, 그리퍼 도착 시 재검토 ⑶ 2단계(시드 팬아웃)가 이긴 132셀은 **실기 미검증** |
| **E31✅** | **그래프 원점 오해 — 로봇이 아니라 그림이 틀렸다(2026-07-29 저녁)**: 접근 정확도 그래프 첫 실기분을 본 오퍼레이터가 "눈으로 보면 말단부가 모든 물체 **위**에 있는데 그래프의 ΔZ는 몇 mm뿐 — 이상하다"고 지적. **로봇도 그래프도 맞았고 라벨이 틀렸다.** `ROS2PickBridge.cpp:643`이 `dGoal[2] = 물체 z + 0.15`로 **호버 마진을 목표에 이미 굽는다** → 그래프의 원점은 **물체가 아니라 호버 지점**이고, 실측 TCP는 물체 **141~161 mm 위**(관찰과 정확히 일치). **자초한 회귀**: 첫 판본에는 `hover margin: 150 mm above the object` 줄이 있었는데 "단순화" 요청에 응하면서 그 줄을 빼버렸고, 그 순간 그림만으로는 원점이 물체인지 호버인지 판별 불가능해졌다. **교훈: 시각화에서 "무엇을 뺄까"를 고를 때, 좌표계 기준을 알려주는 항목은 장식이 아니라 축 라벨이다** | (`762a98c`→`526f9b4`) 3D 패널 제목이 원점을 명시하고, 숫자 패널에 `object [m]`·`target = object z +150 mm`·`→ N mm above the object`(파란색) 추가. 물체 자체는 **실척 측면도 패널**로 별도 표시 — 3D 패널은 ±15 mm 확대라 150 mm 아래 물체를 넣으려면 10배 축소해야 하고 그러면 정작 봐야 할 mm가 사라진다. **오탐이었지만 값어치가 있었다** — 이 지표가 재지 **않는** 것 두 가지(캘리브 편향·FK 자체)를 문서에 명문화하는 계기 |
| **E32~E35✅** | 정밀화 트랙(2026-07-31)의 사고·함정 4건: **E32** RT 스톨로 #27~29 오염 · **E33** FF leash-release 캐터펄트(#34/38/39 발산) · **E34** 스톨 진범 = IgH master priority inversion(#40 확정) · **E35** FF 소이동 과주(mustard 미스 부호 반전) | 상세·수정·검증 전부 **§9.5**(타임라인 재구성 시 그쪽으로 통합 — 대장에는 이 색인만). 수정: E33→scope gate `9ba929d` / E34→chrt 자동화 `6c090ae`+조용한 보드 수칙 / E35→lag gate `90e5409` |
| **E36✅** | 추적(D15) 소프트리밋을 **인코더 카운터 프레임으로 판정** → TCP가 +y·z는 따라가는데 **−y로만 안 옴** (2026-08-03 실기) | 라이브 카운터 J1 −5.8/J4 −12.6/J5 −11.9/J6 +5.7 rad = 물리적으론 0 근방인데 2π·k 감김(E20). raw 비교가 J1을 "하한 고정"으로 오판 → 음방향 qd_ref 상시 0 = −y에 필요한 베이스 요 차단. **E20 원칙(판정=물리/실행=카운터) 위반 사례** — 새 코드가 m_Q를 한계와 비교하면 반드시 fold 먼저(`CheckLimitsPhysical` 관용구). 수정 `8656327`(fold-toward-zero; J6은 ±3.75가 fold 창 밖이라 게이트 무력=무해) |
| **E37✅** | 추적 FF의 lag gate **전면 우회**(`3ceeb9f`) → stick-slip 꿈틀거림 + **테이블 다이브**(오퍼레이터 E-stop, 2026-08-03 실기) | `sat(qd_ref/0.01)`은 mm 오차에도 포화라 FF가 0 오차 주변 bang-bang ±KF(꿈틀). 하강 중엔 breakaway 후 **운동마찰 ≪ 정적**(E33)인 관절에 풀 FF가 계속 실림 — **속도 캡은 명령 속도만 제한**, Kd 단독으론 6 Nm 못 잡음 → 하강 폭주. 교훈: **lag gate의 원리(뒤처질 때만 보조, 앞서면 제동)가 하중을 받치는 부분** — 서보 레짐에서 틀린 건 측정 공간뿐. 수정 `e5edfa1` 3종: ① 추적 FF 게이트를 속도 공간으로 `clamp((qd_ref−qd)·sign(qd_ref)/0.01,0,1)` (정지=풀 보조/명령 속도 도달=운동마찰만큼 자기평형/초과=0) ② 10 mm rest 데드밴드(연속 shaping) ③ **goal z 하한 0.25 m**(`TRACK_GOAL_ZMIN_M` — 박스 z_min 0.10=테이블면이라 depth 글리치가 테이블행 goal 생성 가능했음; admission 거부+클램프 백스톱). **후속(`40aaa43`)**: 접근 중 J3 상시 공회전+도달 후 미세 크립+저속의 범인 = **z-정렬 능동 복원 항** — v2는 아무 데서나 진입하므로 동결 자세가 어색한 경우가 많고, 회전 명령이 위치를 밀고 위치 루프가 되돌리는 시소 + 데드밴드 안에서도 회전 명령이 FF를 깨워 크립. 각속도 0 명령만으로 DLS 최소노름이 자세를 소프트 유지하므로 능동 복원 삭제(정지 시 전 명령 0→FF 0→진짜 정지) + `Kp_task_pos` 1.0→2.0(τ=0.5 s; 정확도 아닌 시정수 노브, 캡·가속클램프·데드밴드 불변). **v3 하이브리드(`8d64868`)**: 서보 단독 대이동은 국소 법칙이라 브랜치 미선택·breakaway 되는 관절에 일 몰림(손목 방랑·J4 채터·다이브 재시도) — 사용자 판정대로 **대이동은 검증된 접근 스택**(SolveReadyIK=최소 관절이동 랭킹+quintic, `StartTrackLeg` 신설=direct leg의 린 트윈, 스테이징·refine·로그 없음)이 자동 발화, 서보는 0.15 m 이내 추종만. 물체가 0.25 m 이상 도망 → TCP서 제동(eTRACK_STAGE, quintic은 정지 출발 전제) → 정착 → 다음 leg. leg 중 모드=IK6dof라 **FF는 원샷 검증된 위치 lag gate 그대로**. J4 채터=속도 게이트 램프 0.01→0.04 rad/s(협대역=bang-bang이었음 → 비례 보조). cfg `KF_2=4.0` 활성(접근 시대 예고 항목 "j2 lag 상시" — 엘보 무보상이 손목 몰림의 공범). 잔여 J4 미세 떨림(`25bc044`)=sat 램프도 0.01→0.04(추적만): ±0.005 rad/s 명령 지터(15 Hz 비전 노이즈)가 FF를 ±절반 스케일로 흔들던 것 — dS·dG 비례 대역 통일, 접근/refine 램프 불변. **최종(`38a787a`)**: ① axis3 잔떨림 근치=FF 명령속도에 **50 ms 저역통과**(램프 확폭은 감쇠일 뿐, 근본은 스펙트럼 — 15 Hz 지터 분리, breakaway는 DC라 무손실, 추적 전용·진입 시 0 초기화) ② **간헐 axis2/3 한바퀴 감김→테이블 붕괴 = 특이점 널 방향**(오퍼레이터 진단 적중): TCP는 안 움직이고 관절만 도는 운동이라 Cartesian far-check가 못 보고 λ 댐핑은 감속만 시킴(leg 경로는 면역 — SolveReadyIK가 라이브 카운터로 폴딩+dq 2.0 게이트) → **관절 드리프트 예산**: 서보 진입마다 스냅샷, 어느 관절이든 1.5 rad 초과 이동 시 제동→재계획 leg(궤적 스택이 최소이동 브랜치 재선택). 장거리 정상 추종도 주기적 클린 재계획으로 수렴 |

> **철회 기록(2026-07-28 04:00)**: 03:50에 "J2가 잠겨 있어 실질 5-DOF"라는 제보가 있어
> 그 전제로 전면 재해석을 기록했으나(커밋 `29b40cd`/`9d7a7c5`), **오퍼레이터가 착각으로 정정** —
> **J2는 정상 가동하며 이 로봇은 6-DOF가 맞다.** 따라서 그 재해석(IK에서 J2 제외, top-down R
> 원리적 불가, E26 원인 지목, n/8 낙관치 등)은 **전부 무효**이며, E25/E26은 원래의
> 솔버·수렴 문제로 되돌아간다. 세 번의 HOME 기록에서 J2가 −0.92→−0.87→−0.72 rad로 변한 것도
> 이제 정상 거동으로 설명된다. 옛 커밋을 다시 읽고 혼동하지 말 것.

**안전 제약 (실기 확인, 2026-07-26)**: **팔 전방-하강 경로에 카메라 마운트가 있어 충돌 가능**
(오퍼레이터 실측 확인). → 'a' 위치추종의 하드코딩 목표 검증 선행, Phase 5 워크스페이스 박스에
카메라 배제 존 반영 필수.

## 8. 대화형 선택 계약 (파이프라인 ↔ RAON-RT) — **구현 완료(2026-07-26, `f7c27fc`)**

구현된 확정 흐름 (`CROS2PickBridge` + Indy7Ctrl):

1. 오퍼레이터 `'p'` → 브릿지가 `/detections` 최근 ~1 s 집계(클래스당 ≥3프레임)
   → 번호 메뉴 + 마지막 항목 **AUTO**(desired_class 해제). ⚠️ **pick_logic v3(2026-09-04)부터 AUTO는
   무효** — ''는 "선택 없음"이라 LOCK이 안 걸린다. 브릿지 메뉴·GUI 버튼·`test_gate2b_bridge.py` C1에서 제거 예정
2. 숫자 입력 → `AsyncParametersClient`로 `/pick_logic_node`의 `desired_class` 설정
   (rcl_interfaces `set_parameters` 서비스, 2 s×3회 재시도). `0`=취소
3. pick_logic v3가 그 클래스의 인스턴스 1개를 채택·고정 → `/pick_target_base` 유효 스트림 →
   브릿지 "TARGET LOCKED: <class> @ base(x,y,z)" 표시 (신선도 <1 s 감시, lost 시 표시)
4. `'v'` → **N=15프레임(≈1 s, 타임아웃 3 s) 수집 — `target_valid && depth_valid` && 수집
   시작 시점 클래스 일치 샘플만** → 축별 std<8 mm && goal=(x̄, ȳ, z̄+0.15) 워크스페이스 박스
   통과 시 SPSC 슬롯으로 RT에 전달 → RT가 grav-comp→위치전용 IK→quintic(3 s)→IK6dof+CTC
   ('n'키 검증 시퀀스). **v1은 현재 TCP 자세 유지(위치만)** — top-down 고정 R은 URDF TCP
   관례 검증과 함께 Phase 5에서
5. 안전: 게이트 전 조건 불충족 시 사유 출력 후 거부, 이동 중(`eAPPROACH_MOVING`) 및
   ISO/RECT 활성 중 재트리거 금지, goal 미소비 상태 덮어쓰기 금지,
   도달 후 홀드('g'로 grav-comp 복귀)

> **2026-09 person 로직 제거**: pick_logic의 person guard·파라미터 5종이 삭제됐다. 브릿지의
> person 메뉴 제외/경고줄(`ROS2PickBridge.cpp` L28/L913/L928-929/L962-963)은 detector가 person을
> 발행하지 않아 죽은 분기 — 다음 브릿지 커밋에서 삭제 예정(L963 "pick_logic will veto"는 이제 거짓).

> 키 변경 이력: 계약 초안의 접근 트리거 `'g'`는 기존 **grav-comp 키와 충돌**해 `'v'`(vision,
> 구 VisualServo 키 재사용)로 확정. `'p'`/숫자는 기존 키맵과 무충돌.

### 8.1 앱 정체성 — 접근 모드 / 추종 모드 / 파지 범위 밖 (2026-08-03 전환)

계약은 이제 **두 모드**다. 이름(`pick_place_vitis_ai.launch.py`, `pick_logic_pkg`,
`/pick_target_base`, `PopGoal`)은 전부 **pick&place 시절 legacy** — 코드가 하는 일은 아래 둘뿐이고
**파지·이송·놓기 경로는 코드에 존재하지 않는다**(그리퍼 하드웨어 미보유).

| 모드 | 키 | 목표점 | 제어 | 실기 상태 |
|---|---|---|---|---|
| **접근** | `'p'` → 숫자 → `'v'` | 물체 꼭대기 **+0.15 m** 호버(`DEF_ZMARGIN`) | 15프레임 수집 + 축별 std ≤ 8 mm 게이트 → ready-seed IK + quintic(3 s) → refine(최대 6회, 허용 12 mm) | ✅ **검증 완료** — #52~58 원샷 7/7, 최근 실기 4건(2026-08-05) 4.21~5.93 mm·refine 0회 |
| **추종** | `'o'` 토글 | 실시간 물체 **+0.20 m** 호버(`TRACK_ZMARGIN_M`), goal z 하한 0.25 m | 목표 > 0.15 m면 접근 leg 자동 발화(`StartTrackLeg`), 0.15 m 이내만 1 kHz DLS 서보. 0.25 m 도망 시 제동(STAGE) → 재leg | ⚠️ **v3 하이브리드 실기 미검증** — 검증 사다리 8단계 대기 |
| **파지·이송·놓기** | — | — | — | **범위 밖** — 그리퍼 하드웨어 미보유, 코드 없음 |

- **추종 진입 게이트 4종**(각각 개별 거부 메시지): 브릿지 존재 · 컨트롤러 enable · 전축 서보 ON ·
  접근/ISO/RECT 미실행 + 팔 정지(Σ|q̇| ≤ 0.05) · 타깃 lock + 클래스 일치 최신 샘플. 다시 `'o'` =
  현 위치 hold(정지).
- **실질 안전장치는 TCP 선속도 캡 0.20 m/s 하나**(cfg `TRACK_VMAX_MPS`, 코드 클램프 0.05–0.50 —
  어떤 값으로도 비활성화 불가). 그 위에 워크스페이스 박스(admission +2 cm eps·클램프),
  goal z 하한 0.25 m, 상실 시 **TCP에 goal 동결**(마지막 비전 goal이 아님 — `TRACK_LOST_MS` 500 ms),
  키 화이트리스트 `{o, j, e, x, g, p, 0-9}`, URDF effort 토크 백스톱, 관절 드리프트 예산 1.5 rad.
- **추종 중 재타게팅 허용**: `'p'`+숫자(화이트리스트에 포함). 추종 중 뒤늦게 팝된 `'v'` goal은 폐기.
- **추종은 정량 로그를 남기지 않는다** — `EmitApproachReport`(CSV 1행 + PNG 워처)는 **접근 완료
  에지에서만** 호출된다. `approach_results/`·`rt_log_results/`에 추종 기록이 없는 이유.
- **코드 주석의 "15 Hz vision goal"은 스테일** — 2026-08-05 이후 실측 **21.7 Hz**
  (`docs/vision/throughput.md`). 접근 게이트 N=15의 "≈1 s" 주석도 실제로는 ≈0.7 s.
- 오퍼레이터 콘솔(`4b2113a`, 2026-08-06)이 원격 키/jog/gains 채널과 정지키 재정의(`'e'`/`'x'`,
  공통 근원 `KillMotionSources()`)를 넣었다. 원격 키도 로컬 키와 같은 `DoInput` 경로라 추종 중
  화이트리스트가 그대로 적용된다. **실기 미검증**(커밋 본문 "Not yet exercised on the robot").

## 9. 남은 것

- 위치 정확도 개선 — **완료(2026-07-27)**: refine으로 **10.9 mm** — §5 표, `0668ed2`
- Phase 5-1~5-4 — **완료(2026-07-27)**: 캘리브 → rMc → TF/박스 정본화 → apple 실좌표 검증
  → **접근 데모 전 클래스 성공**(접근→'b' homing→다음 사이클) — §5 표,
  `8093e8a`/`a9644dc`/`61a4a86`. E13 사고·픽스 포함. 물리 검증 완료: 테이블-베이스 높이차
  ~10 cm(depth +0.096 일치), apple 수평거리 85 cm(계산 0.847 일치)
- grav-comp 보완 — **완료(2026-07-27)**: F/T 285 g URDF(−X 축, E16) + sticky-float
  속도게이트판 오퍼레이터 OK — §5 표. 잔류: 전방 뻗은 자세 미세 sag(모델 바닥, sticky가
  완충 — 필요 시 HOLD_KP_FRAC/DB 상향 나사만)
- 접근 IK 모드 검토 — **결론(2026-07-27, D13)**: task-space servo 1차 시도는 손목 감김
  사고로 리버트(E17, 전제조건 3종 명문화 후에만 재시도) → **ready-seed IK 채택**(`19b15ba`,
  자동 스테이징이 경유점 스테이징 아이디어를 흡수). REFUSED→home→재시도 수동 레시피의
  자동화도 이 안에 포함됨
- ready-seed 실기 검증 — **v3 확인(2026-07-27 20시)**: [operator anchor] 부팅·괴랄 궤적
  소멸·전 접근 직행 FK 9.6~11.9 mm — 스테이징 레그·취소 키는 아직 실기 미조우(전부 직행)
- E21 복구 — **완료(2026-07-27 21시대)**: 베이스 고정 → 프로브 강체 fit → launch TF 보정
  → 전 5클래스 접근 재검증(§7 E21/E22/E23)
- E25 w 사다리 / E26 refine 진동 — **완료(2026-07-28, `9aca402`)** — §7 E25/E26.
  refine 14 pass → 5 pass, tennis 실패 → 1 pass 9.4 mm. 잔차 5.8~9.4 mm는 stiction 바닥
- park 자세와 IK 시드 분리 — **불필요로 결론(2026-07-28)**: "좋은 시드는 작업공간 근처,
  좋은 park는 시야 밖이라 구조적으로 상충한다"는 가정이 실측으로 반증됐다. **시드는 계산의
  출발점일 뿐 물리적으로 목표 근처일 필요가 없고**(올바른 IK 브랜치만 고르면 됨), 16:34에
  기록한 앵커 `q=[0.47,−0.41,−2.14,0.17,0.73,−0.56]`는 **팔꿈치를 접어 카메라 시야 밖에
  있으면서** 5클래스를 전부 첫 단에 풀었다. 상태와 키를 하나 더 만들 이유가 없음. 다만
  **결합 자체는 잠재 함정**으로 남는다 — 나중에 시야를 가리는 자세로 HOME을 기록하면 같은
  불편을 겪는다(해법: 시야 밖 + 좋은 브랜치를 동시에 만족하는 자세가 존재한다는 것이 위 증거)
- reach map 툴 — **완료(2026-07-28 18시대, `a0a2c26`)**: `tools/reachmap.cpp` +
  `tools/reachmap_plot.py`. **설계상 가장 중요한 결정 — 솔버를 재구현하지 않고
  `FullDynControllerRT.o`를 그대로 링크한다.** 사다리·warm-start·폴딩·브랜치·tilt 로직을
  두 벌로 가지면 드리프트가 생기고, **로봇과 다른 말을 하는 지도는 없느니만 못하다**.
  ROS·EtherCAT 불필요(박스 상수만 `WorkspaceBox.h`로 분리). 사용법:
  `make reachmap` → `./bin/reachmap.out --step 0.01 --z 0.15,0.20,0.25,0.30,0.35 --out m.csv`
  → `python3 tools/reachmap_plot.py m.csv -o m.png`. `--targets`는 좌표 목록을 재생하는
  모드로, **필드 로그와 대조**하는 데 쓴다(첫 실행에서 실기의 `J5 2.90 rad` 거부를 축·수치까지
  재현 → 지도-로봇 일치를 비순환으로 확인, 그리고 **E28 버그를 즉시 발견**).
  결과 해석은 §7 **E29**. 증거: `evidence/reachmap/`
  - **`n/8`의 대체물**: "코너 8점 중 몇 개"가 아니라 **박스 안 PASS 비율 + 위험 지대 지도**.
    참고로 현행 앵커는 n/8이 만점이어도 z=0.30에서 6.4%가 BRANCH다 — 두 지표는 별개다
  - ⚠️ **충돌 모델이 없다**. 테이블·카메라·물체·자기 링크 전부 미반영이므로 PASS는
    "IK가 이 점을 받는다"이지 "팔이 안전하게 간다"가 아니다. 그 검증은 MuJoCo 몫(§2)
- **앵커 비교 스윕(백로그로 강등)** — 경계 위치는 q_ready의 함수다. `--seed`를 바꿔가며
  같은 스윕을 돌리면 **"어느 HOME 자세가 박스를 가장 넓게 덮나"를 계산으로 고를 수 있다**
  (지금까지는 사람이 팔을 놓아보고 n/8을 읽는 방식). **부분 실행(07-29 저녁, §0 E31 옆)**:
  합성 수직 앵커 1개 스윕 → 역효과(PASS 96.5→83.2%) 확인, 체계적 후보 스윕은 미실행.
  E30 다중 시드가 브랜치 문제를 대부분 해소해 시급성 하락. 후보 시드는
  `~/.indy7_ready_seed.*` 백업들
- 다중 시드 IK — **완료·실기 검증·병합(2026-07-29, RAON `2e911a3`)** — §7 **E30**. 브랜치 플립
  거부가 박스에서 사실상 소멸(1,345 → 14셀), 커버율 95.1 → 99.4%. **효과의 89%는 시드 팬아웃이
  아니라 warm-start를 건너뛴 solve 한 번**이었다는 점이 이 작업의 실질 교훈
- **다음(즉시): IK를 RT 스레드 밖으로** — 사다리가 최대 **56 ms**를 1 kHz 루프 안에서 쓴다
  (§7 E25/E27). 브릿지의 non-RT 워커에 **별도 RBDL 모델 인스턴스**(모델이 스레드 안전하지
  않음)를 두고 solve를 시킨 뒤 RT에는 완성된 관절 벡터만 넘긴다. 예산 조이기는 이미 시도했고
  해가 망가져 되돌렸으므로, 이게 유일하게 남은 정공법
- **E24 후속 가드(미구현, §7 E24)**: 호밍 래치 두 기준 교차검증 / `h` 무장 전 전축
  `IsHomeSet()` + 모델이 믿는 q·FK 출력 / `SetReadyAnchor` 자세 급변 거부 +
  temp→rename+`.prev` / DeInit의 PBE 샘플링을 브레이크 낙하 이전으로 /
  **소프트웨어 관절 리밋 2차 방어선 부재** — ⚠️ **정정(2026-07-28, 코드 확인)**: 앞서 이
  자리에 "`POSITION_LIMIT_U/L`이 라디안이라 ±35 rad = 2005°로 리밋 부재"라고 적었으나
  **결론은 맞고 이유가 틀렸다**. ⑴ `Indy7Ctrl.cpp:407`이 `SetPositionLimits(L,U)`를
  세 번째 인자 없이 부르고 기본값이 `abIsRad=FALSE`이므로 **±35.0은 도(degree)로 해석되어
  ±0.611 rad로 변환**된다 — 무한이 아니라 오히려 실제 한계(±175°)보다 훨씬 좁다. ⑵ 그런데도
  아무 일이 없는 이유는 **실기 축이 이 값을 아예 검사하지 않기 때문**이다. 위치를 클램프하는
  코드는 `AxisRT.cpp:362`뿐이고 그건 **시뮬레이션 축**이며, `AxisCIA402.cpp:181`은 값을
  로그로 출력만 한다. 증명: q_ready의 J2가 −122°인데 ±35°가 강제됐다면 현행 동작이 전부
  불가능하다. **⇒ cfg 값을 고쳐도 아무 일도 일어나지 않는다. 필요한 것은 RT 루프 안의 새
  가드**(매 사이클 실제/명령 관절값을 URDF 한계와 대조해 정지). **긴급도는 낮다** — 1차
  방어선 `CheckLimitsPhysical`(±175°/±215°)이 모든 IK 해를 검사하고, 한계 안 두 자세를 잇는
  관절공간 궤적은 중간에도 한계를 못 벗어난다(박스 제약=볼록). 뚫린 곳은 그 검사를 거치지 않고
  위치를 직접 명령하는 경로뿐인데 = E17 task-space servo(J5 9.39 rad 감김)이고 이미 리버트됨
- 접근 정확도 그래프 — **완료(2026-07-29)**: 접근이 끝날 때마다 **목표 좌표 vs 실제 TCP**를
  자동 기록·작도. 앱은 `App/Indy7/approach_results/approach_log.csv`에 **행 하나만 append**하고
  (RT 밖 워커 스레드), `tools/approach_plot.py` 워처가 `plots/approach_NNN_<class>.png` +
  `approach_latest.png` + 누적 `approach_history.png`를 만든다. `run.sh`가 워처를 자동
  기동/회수(§6 표). **그래프가 답하는 것**: 불스아이(목표=원점, 링 5/12/25/50 mm)로 "얼마나
  빗나갔나", XY/XZ 두 장으로 "옆으로인가 높이인가", 회색 점·막대로 "**refine이 실제로 얼마나
  벌었나**"(CTC 드룹 수 cm → 잔차 수 mm). 메타 패널에 IK 시드·solve 수·ms·tilt·branch gap이
  같이 박혀서, 나쁜 결과가 **인지 오차인지 제어 오차인지 IK 선택 문제인지** 한 장에서 갈린다
  - **첫 실기 10건(5클래스 × 2회차) 완료 — `evidence/approach_accuracy/`**: 전 건 완주,
    **3.07~11.52 mm**, refine 1~2 pass, 비전 std 0.0~2.3 mm(게이트 8 mm).
  - **★ 오차가 랜덤이 아니라 물체마다 방향까지 재현된다**: 같은 목표 두 회차 차이가
    apple **0.5 mm**, orange **0.7 mm**(apple은 두 번 다 `+8,+5` 방향). **재현되는 편향은
    보정 가능한 종류**이므로 파지용 <5 mm에 직접 단서다 — "stiction 바닥이라 게인/적분밖에
    없다"던 기존 진단보다 좋은 소식. ⚠️ 다만 표본 2회차이고, refine이 **12 mm에서 성공
    선언하고 멈추므로** 관측 분포 자체가 그 임계값의 산물이라는 점을 감안할 것
  - **mustard만 회차 간 12.9 mm로 튄다** — 그리고 **이 물체만 IK가 다르다**(`8 solve,
    12.4 ms, tilt 29°` vs 나머지 넷 `1 solve, 0.2 ms, tilt 0°`). 07-28 지도가 **경계 혼합
    지대 한가운데**로 지목한 그 물체이고, 지도의 예측이 이번엔 **IK 계산량과 재현성**으로
    다시 확인됐다
  - ⚠️ **이 오차는 "명령한 좌표 대비 TCP"다.** 재지 **않는** 것 둘: ⑴ **카메라→베이스 캘리브
    편향**(목표 좌표 자체가 틀렸으면 여전히 0 mm로 보고 — E21이 이 사각지대였다) ⑵ **FK 자체**
    ("도달"은 외부 측정이 아니라 엔코더+URDF 계산값 — 관절 영점이 틀어지면 못 본다, E24)
  - **목표는 물체가 아니라 호버 지점**(물체 z + 0.15 m) — E31. 그림/문서에서 이 구분이
    빠지면 즉시 오해가 생긴다
  - 접근이 모드 변경/서보 오프로 중단되면 행이 남지 않는다(완주한 접근만 기록)
- **`tilt` 지표에 축 정보가 없다(파지 단계 선결)**: `AttitudeDevDeg`는 `m_EReady`와의
  **3D 회전 각도 하나**만 남기고 회전축을 버린다(`acos((trace(R)−1)/2)`). 그래서 **"공구를
  30° 숙임"과 "제자리에서 30° 비틂"이 같은 값**으로 찍힌다 — hover만 하는 지금은 무해하지만
  파지에서는 정반대 의미다. 축 3성분을 `IkDiag`에 추가하는 건 몇 줄(같은 `R_err`에서
  `w = (R32−R23, R13−R31, R21−R12)` 정규화). ⚠️ 또한 **`tilt`는 이름과 달리 "기울기"가 아니고
  수직 기준도 아니다** — HOME 자세 기준의 자세 거리이며 roll을 포함한다
- **IK 게이트 상수의 근거 강도가 균일하지 않다(2026-07-29 git 추적)**: §3의 판정 상수
  **11개 전부가 병합 브랜치 산물**이고 분기점 원본에는 `m_dt`·`m_lambda` 둘뿐이었다(원본은
  RBDL 수렴 플래그만 믿는 구조, 접근 거부 로직 자체가 없었음). 그중 **기계 사양에서 온 것은
  관절 한계 ±175°/±215° 하나뿐**이고, `IK_DQ_MAX_RAD 2.0`·`APPROACH_RDEV_MAX_DEG 60°`·
  `IK_POS_TOL_M 2 mm`는 **사고 한 건을 막으려 고른 값이지 최적화된 값이 아니다**(60°는
  "원거리 41~48°가 실제로 되더라"는 데이터 몇 점 위에 여유를 얹은 것). 재조정할 일이 생기면
  이 차이를 기억할 것
- 접근 궤적 자동 기록·3계열 그래프 — **구현+실기 검증 완료(2026-07-30, RAON `592998e`)**:
  접근마다 **DataLog 1파일 = 접근 1회**가 자동 생성되고, 워처가 `approach_NNN_<class>_traj.png`
  (3D 경로 + **시작점 기준 변위 |P(t)−P0| 1패널** + 목표거리 패널; X/Y/Z 3패널은 07-30
  사용자 요청으로 변위 1패널에 병합, RAON `bd01c6d` · **one-shot plan 곡선** 추가 `0550e47`+`9606588` —
  1차 pass의 reference를 끝점 고정 연장한 "원래 원했던 한 방 궤적", 새 계산 없이 로그에서 추출,
  3D 패널에도 파란 곡선+계획 끝점 X 마커로 표시;
  pass-1 경계는 goal 스텝 크기 <80 mm 휴리스틱으로 refine 편향/staging leg-2를 구분)를 만든다.
  **원점 재정의(`a09f129`)**: 3D의 `+`는 bias 포함 **최종 조준점**이라 X와 안 겹치는 게 정상 —
  ○(**원래 목표**=호버점, 첫 refine 편향 직전의 goal) 마커를 추가해 X가 ○ 위에 포개지는 것(≤2 mm,
  IK Accept 잔차)이 정상 판독이 되게 함.
  **최종 레이아웃 = 사용자 스펙 2패널 고정(`3bc5336`, 마커 정리 `b884069`)**: ① 3D = **one-shot
  plan(이론)과 actual(실제) 둘만**(reference 선 제거; 재조준은 actual 경로와 세로 점선으로 보임) +
  마커는 ○(원래 목표)·start만 — 계획 끝점 X는 ○와 ≤2 mm 중복이라 삭제, `+`(final aim)도 사용자
  요청으로 삭제(bias 수치는 필요시 CSV/재조준 스텝에서),
  ② displacement 1개 plot = **IK 입력(호버점, 점이므로 수평선)/FK(q_ref)/FK(q_act) 3계열**.
  목표거리 패널은 스펙에 따라 제거 — miss 수치는 제목 final miss와 정확도 PNG가 담당.
  **RECT 빌린 스모크 입력 은퇴**: goal 열이 무의미한 임시 입력이었고 측정 로직 문제가 아니었음 —
  `tools/traj_plot_selfcheck.py`(합성 접근: droop 37 mm·0.65 재조준·IK 잔차 2 mm를 plot_traj에 직접
  주입)로 대체, 기하 검증: X-○ 1.6 mm 포개짐·`+` 24 mm 분리·final miss 8.1 mm=제목 일치.
  **실기 검증 완료(2026-07-30 저녁)**: 접근 23건+ 그래프 자동 생성, 실데이터 교차검증 배지 0.002 mm.
- 접근 정밀화(원샷 도달) — **완료(2026-07-31) → §9.5 참조**: Kd vs Ki 분석,
  마찰 FF(KF_3/KF_4), E32~E35, #24~58 실기 전 과정을 타임라인·표로 재구성해 이동
- **파지 단계 준비 — 그리퍼 하드웨어 미보유라 현 앱(접근+추종) 범위 밖, 도착 시 재개**:
  TCP 재정의(현 원점은 태그면 안쪽 29 mm — 그리퍼 TCP로 이동, E16),
  <5 mm 정밀도(게인/적분 검토, refine 바닥 12 mm), top-down 고정 R, 물체 6D pose
  estimation(나중 — 들어오면 soft-R 타깃만 물체 기준으로 교체)
- **D10 3+1 격리 배치 완료(2026-07-27 22~23시)** — cmdline + cfg `CPU=` 키 + /proc 토폴로지
  확인 + EtherCAT OP/Lost 재베이스. 순수 지터 A/B는 당시 앱의 RT97/95가 같은 CPU3를 점유해
  오염(미완)이었으나 → **완결(07-28 17:02, §0)**: 앱 정지 상태에서 격리 CPU3 avg 4.3/max
  **51.1 µs** vs 비격리 CPU0 avg 7.3/max **114.6 µs**. 참고 비격리 베이스라인(부하 중):
  CPU3 avg 1.0/max 33.9 µs, CPU0 avg 1.2/max 104 µs
- **EtherCAT I/O는 격리 코어 밖(CPU1)에서 돈다 — 확인됐고, 현재는 유지가 정답**(§4.1).
  `irqaffinity=0-2`가 의도적으로 CPU3을 뺐고, IRQ 스레드가 FIFO 50이라 비전 파이프라인에
  밀리지 않으며, CPU1엔 경쟁자가 없다. Lost 델타 0으로 관측 비용도 없음. **⚠️ CPU3으로
  옮기는 것은 IK off-RT 이후에만 검토** — 지금 옮기면 IK 58 ms 동안 프레임 처리가 멈춘다.
  근본책(macb native IgH 드라이버)은 IgH에 없어 직접 작성해야 하고 크리티컬 패스가 아님
- **백로그**: 해석적 IK(브랜치 전수 열거 — E16 프레임 뒤틀림 리스크로 후순위), task-space
  servo 재시도(E17 전제조건), MuJoCo sim 합류, indy_iface GUI, DC sync0 실험(cfg 주석 해제).
  연속 추적(closed-loop)은 **백로그에서 나와 2026-08-03에 구현됨**(D15, §8.1)
- **RPU+SOEM 트랙**(제어 경로 밖 — 정본 `docs/rpu/`): **R5F 단독 FreeRTOS 부팅 + LED 태스크 실행**
  (작업 2026-09-01~02, **펌웨어 부팅 검증은 2026-09-02 2회** — 커널 로그 `is now up`;
  09-01은 DT enumeration까지. 판정 수단은 UART가 아니라 **LED**였고, 점멸 자체는
  **사용자 육안 확인**이라 보드에서 재검증 불가). 적용은 configfs가
  아니라 **부팅 DTB 병합**(`/boot/firmware/user-override.dtb`)이고, carve-out은 문서의
  0x3ed00000이 아니라 **`rproc@36000000` 4 MiB**(0x3ed00000은 이 보드 CMA 한복판). 남은 3과제 =
  ① **APU↔RPU 통신**(rpmsg — 현 DT에 vring·IPI mailbox가 없고 펌웨어에 resource table도 없어
  DT 재작업 + 재부팅 1회 + 펌웨어 재빌드가 필요) ② **RAON-RT 실시간 제어 task의 RPU 이식**
  (EtherCAT 없는 1 kHz dry-run + R5 지터 측정부터 — 이식 범위 판정이 선행) ③ **EtherCAT SOEM의
  RPU 설치**. ⚠️ ③의 GEM3(eth0) 인계는 **APU IgH가 Indy7을 실제로 제어 중**이라 사용자 재결정
  전 금지 — ①②는 GEM3와 무관해 병행 가능. RPU DT 변경은 재부팅을 수반하므로 실기 세션과
  반드시 분리할 것

## 9.5 접근 정밀화 트랙 — first miss ~40 mm → 전 물체 원샷 7~11 mm (2026-07-30 저녁 ~ 07-31 새벽)

**결론 먼저**: 1차 미스(~40 mm)의 주범은 중력 모델도 게인도 아니고 **손목 stiction +
inertia-scaled PD의 손목 저권한**이었다. 해법은 **Coulomb 마찰 feedforward**(cfg `KF_n`)
하나인데, 현장에서 함정이 두 번 드러나(E33 캐터펄트, E35 소이동 과주) **게이트 두 개**
(refine scope gate, lag gate)로 막았고, 마지막으로 KF_4 용량을 반중력 방향에 맞추자
(1.5→3.0) **7/7 원샷**이 나왔다. 같은 기간의 타이밍 사고 2건(E32/E34)은 제어와 무관한
별도 축(IgH priority inversion)으로 판명·처방됐다.

### 타임라인 한눈 요약

| 단계 | approach | 적용/사건 | 결과 | 판정 |
|---|---|---|---|---|
| 베이스라인 | ~#23 | F/T 페이로드 모델 포함 상태 | first 33~46 mm, refine 1~3 | #23 판독에서 settle leash 발견 |
| ① 페이로드 A/B | **#24~26** | 실물 센서+어댑터판 탈거 + URDF 질량 0 (`49ac6db`) | first 26~40 mm **거의 불변**, 방향만 회전 | 질량·중력 무죄 → stiction 확정 |
| ② Kd 인상 | **#27~29**(무효) → **#30** | KD_3/4/5 = 20/15/10 → **28/25/20** (ζ=1, cfg만) | #30: first 39.7 mm **불변**, 챠터 없음 | Kd는 정지 잔차에 무효 (값은 유지) |
| 사고 E32 | #27~29 | RT 사이클 27~73% 상실 (원인은 E34에서 확정) | 슬로모션 4배 + "덜덜", 데이터 격리 | 앱 재시작으로 일시 해소 |
| ③ 마찰 FF | **#34~39** | **KF_3=5.0** Coulomb FF (`00c6d2c`) | 큰 이동 대성공(tennis first 9.97 mm) / refine **발산** = E33 | scope gate로 수정 (`9ba929d`) |
| 사고 E34 | **#40** | 갭 66% 재발 → IgH priority inversion 확정 | 데이터 격리 | `chrt -f 40` + 조용한 보드 처방 |
| ④ 게이트 검증 | **#41~46** | E33 scope gate + E34 처방 첫 실기 | 갭 6/6 0.0%, final 3.45~9.57 mm 전원 합격 | mustard first 26→58 악화 발견 → E35 |
| ⑤ lag gate | **#47~51** | **E35 lag gate** (`90e5409`) + **KF_4=1.5** | **원샷 3/5** (banana 47→7.9, mustard 58→6.5 mm) | 잔여 orange/tennis = j4 반중력 방향 |
| ⑥ KF_4 용량 | **#52~58** | **KF_4=3.0** (잉여 토크 계산 기반) | **원샷 7/7** (HOME 5/5 + 물체→물체 전이 11.3 mm) | 🎯 목표 달성 |

최종 상태: KP 원값 유지 / KD 80·60·40·**28·25·20** / **KF 0·0·0·5.0·3.0·0** + 코드 게이트 2종
(refine scope gate + lag gate). cfg는 커밋하지 않는 오퍼레이터 상태임에 유의.

### ⓪ 근본 원인 분석 — 원작자 "Kd만 올려보라" 제안의 검증 (#24~26 + DataLog 관절 판독)

- **페이로드 A/B 판정** (정정 2026-07-31: #24부터는 실물 센서+어댑터판도 물리적으로 탈거 —
  모델과 하드웨어가 양쪽 다 일관된 진짜 A/B였음): 페이로드(실물+모델)를 통째로 제거해도
  **1차 미스 크기는 거의 불변**(banana 46→40 mm, mustard 33→26 mm), 방향만 회전(banana y
  +28→−23 mm 플립 = WITH 상태의 편향은 페이로드 몫) → **질량·중력 모델은 droop 크기의
  주범이 아님이 확정**. j4 shortfall −0.18→−0.25 악화는 물체 배치가 달라 IK가 손목 이동을
  2배 요구한 탓(모델 문제 아님). **현 URDF(질량 0)는 현 하드웨어와 일치 = 올바른 상태**;
  페이로드 모델 복원은 **센서를 재장착할 때만**(그때 어댑터판 실측 + +Z CoM).
- **1차 미스의 지배 관절은 항상 손목**: j0~j2는 전 로그 82~99% 추종, j4는 0.1~33%
  (row23/26은 아예 0.1%), j5는 **전 로그 0%**(0.39 rad 명령에도 τ≤0.05 Nm — M55·Kp·e 산수
  그대로, 자세 오차로 숨음).
- **stiction 직접 증거**: j3 breakaway — 움직일 땐 |τ| 7.6~10.6 Nm, 정지 순간 7.21 Nm
  (**유지엔 충분했던 토크가 재기동엔 불충분** = 정적마찰); j4는 목표 방향이 −인데 τ가 늘
  +0.8~+3.8 Nm(중력 유지 우세, inertia-scaled PD ~0.65 Nm로는 부호 못 뒤집음).
- **클리어된 용의자**: 토크 포화 없음(리밋 자체가 미배선 — Indy7Ctrl.cpp:408이
  TORQUE_LIMIT_U 대신 L/JERK를 넘기고 Axis.cpp CheckLimits 주석처리, 실측 τ도 한계 미접근);
  qd 스케일 정상(PDO vs dq/dt 중앙값 비 0.990~1.000); **게인은 fork 이후 무변경**
  (cfg KP 800/600/400/200/150/100, KD 80/60/40/20/15/10 = 원작자 값 그대로).
- **leash(MAX_REF_LEAD 0.15)는 원작자 코드**(d34d1fd 2026-05-17, fork 이전) — 전 4로그에서
  발동, 정적 푸시 상한을 M_jj·Kp_j·0.15(j4 ≈0.65 Nm)로 못박음.
- **Kd 권장(원작자 제안 평가)**: ζ=Kd/2√Kp → j0~j2는 1.0~1.41(그대로), 손목만 저감쇠
  j3 0.71/j4 0.61/j5 0.50 → **KD_3 20→28, KD_4 15→25, KD_5 10→20**(ζ=1, cfg만 수정)이
  1단계. 단 **기록된 실패는 속도≈0 지점이라 Kd 항이 구조적으로 0** — 이동 중 추종만 개선
  기대, 정지 잔차엔 무효.
- **Ki 판정**: 증상(정지 잔차)은 적분류가 맞지만 진범은 **마찰 미모델**(URDF friction
  태그는 rbdl-orb urdfreader가 아예 파싱 안 함 = dead data) + **PD-inside-M 구조의 손목
  저권한**. 연속 Ki는 stiction·leash와 얽혀 limit-cycle 위험, **refine이 이미 이산
  적분기로 수렴 실증**(finals 5.2~11.5 mm). 다음 후보: 마찰 feedforward(sign(qd_ref)·
  Coulomb, j3≈7 Nm급) 또는 손목 Kp 인상/PD-outside-M(eFullDynamics 모드가 코드에 이미 존재).

### ① Kd 인상 실기 (#27~30) — E32 오염과 최종 판정

**이 시점 누적 구성**: 실물 F/T 센서+어댑터판 탈거 + URDF tcp 질량 0(#24~) · **[신규] KD_3/4/5 = 28/25/20** · KF 없음(원작 CTC 그대로)

**E32 — Kd 실험 세션(#27~29, 07-31 00:25)이 RT 스톨로 오염**: "이동 4배 느림+덜덜"의
원인은 Kd가 아니라 **1 kHz 루프가 주기의 ~73%를 상실**(Δt median 1.000 ms인데 27~29%가
1.5 ms 초과, 최대 120~164 ms; 직전 세션까지는 전 로그 0.0%). 궤적 시간은 실행 사이클로
흐르므로(`t_elapsed += m_dt`) 3.4 s 궤적이 벽시계 12.7 s로 늘어짐 — 12.70 s 시점까지 실행
사이클 3,350개 = 3.35 "궤적 초"로 정확히 일치. "덜덜" = 1 ms 스텝 사이 수십 ms 토크 동결이
반복되는 스터터. refine 지연 9 s도 같은 원인(REFINE_VEL_EPS 게이트가 crawl 지속으로 리셋).
**#27~29의 정확도 수치(first 39.7/54.1/27.6)는 Kd 판정에 사용 금지.** 커널 로그
무특이(EtherCAT 에러는 종료 시퀀스뿐, RT throttling은 07-26 다른 부팅, UVC 에러는 만성).
cfg 핀 CPU=3 무결, isolated=3 유지. 참고: 07-30 20:32 세션 첫 로그도 7.3%/8 ms 경미 갭 —
전조였음(진범은 E34에서 확정).

**해소·최종 판정(#30, 00:44)**: 앱 재시작만으로 갭 0.0% 복귀. #30 유효 데이터 판정 —
챠터 없음(qd 고주파 rms가 구 게인보다 오히려 감소 0.0036→0.0021), 타이밍 정상, 그러나
**first miss 불변 39.7 mm**(j4 여전히 leash −0.15, z +39.6 순수 잔존) — 예측대로 **Kd는
정지 잔차에 무효, 지렛대 아님**. row24→30의 y 오차 소멸(−22.7→+1.9)은 Kd가 아니라 물체
배치/IK 해가 j3 요구를 0.247→0.041 rad로 줄인 효과(오염됐던 #27과 #30의 miss 벡터가
동일한 것이 증거). 게인은 28/25/20 유지(무해 + 마찰 FF 도입 시 감쇠 여유).

### ② Coulomb 마찰 feedforward 설계 (`00c6d2c`)

`τ += KF_i·sat(q̇_ref,i/0.01)` 을 **M(q) 곱 바깥**에 추가(마찰은 관절 토크지 가속도가
아님 — inertia-scaled PD가 손목에서 지는 바로 그 이유). **q̇_ref 게이트**라 정지 시 정확히
0(버즈·적분식 헌팅 원천 불가, 실측 속도 부호 노이즈 면역), 0.01 rad/s 포화 램프로 시작/끝
토크 계단 제거. cfg `[SYSTEM] KF_n`(키 없음=0.0=기능 꺼짐, 동작 불변), 코드 클램프
[0,12] Nm. 배선: ConfigRobot(vKf) → Indy7Ctrl(dof==6) → SetFrictionFF. RT 비용 = 사이클당
곱셈 몇 개. **1차 현장값: KF_3=5.0**(j3 breakaway 7.2~10.6의 ~70%, 일부러 미달 — FF는
돕기만, 스스로 끌지 않게)만 켜고 접근 A/B. 궤적 꼬리(q̇_ref→0)에선 FF도 소멸하므로 마지막
잔차는 여전히 refine 몫. j4는 중력과 얽혀 j3 결과 후 판단(→ ⑤에서 투입).

### ③ E33 — leash-release 캐터펄트 (#34~39) → refine scope gate (`9ba929d`)

**이 시점 누적 구성**: 페이로드 탈거+질량 0 · KD 28/25/20 · **[신규] KF_3=5.0 마찰 FF 첫 투입** (KF_4/5=0, 게이트 없음 — q̇_ref 게이트만)

큰 이동은 극적 개선(tennis first 9.97 mm·refine 0회 역대 최고, apple 20 vs 50 mm) —
그러나 #34/38/39는 refine 6회 상한 소진 후 **final이 first보다 악화**(73/59/73 mm),
육안으론 "마지막에 좌우 왕복". 메커니즘: 첫 이동이 leash(0.15 rad)에 물린 채 끝나면 다음
미세 이동에서 FF가 정적마찰을 무너뜨리는 순간 갇혀 있던 관절이 풀리며 **느리고 짧은
reference를 추월해 반대편 leash까지 관통**(j3가 매 pass "명령+0.30 rad"=2×0.15 이동,
goal_dy 부호 완벽 교대 −28/+39/−38/+46/−42/+44 mm). 성공/실패의 분기 = 첫 이동이 leash
없이 끝났는가(#36 lead −0.018 → ×0.7 정상 수렴). **수정**: `SetTargetPosePositionOnly`에
per-trajectory `adKfScale`(기본 1.0=불변) 추가, refine 사다리만 0.0 전달 — "FF는
총이동(gross transport) 전용, 정밀 위치잡기는 검증된 PD+refine 체제로". 부수 관찰: 풀린
관절이 저속 ref를 추월했다는 것 자체가 **운동마찰 ≪ 정적마찰**의 실증 — KF를 breakaway까지
올리면 큰 이동 꼬리에서도 위험, KF_3는 5.0 유지.

### ④ E34 — 간헐 RT 스톨의 진범 = IgH master priority inversion (#40에서 확정, #27~29 소급 설명)

**#40 시점 누적 구성**: 페이로드 탈거+질량 0 · KD 28/25/20 · KF_3=5.0 · **[신규] refine scope gate(`9ba929d`)** — 단 #40 데이터는 타이밍 오염으로 격리

세션 지도 = 00:25 나쁨(27~29%) / 00:44 좋음 / 01:14 좋음(#34~39 전부 0%) / 01:35
나쁨(66%) — 빌드·cfg·Kd·KF와 무관한 "재시작 복불복"이 아니라 **세션 중 배경 부하 복불복**.
증거 사슬: ① 153 ms 갭에도 SM watchdog 무발동 = EtherCAT 태스크는 계속 돎(=TASK0이 spin이
아니라 **blocking**) ② cyclictest CPU3 1 kHz 20 s **max 178 µs**(load 6 상태에서!) =
커널/격리/CPU3 무결 ③ eth0 에러 0, UVC 만성(무관), 클럭 MONOTONIC, mlockall 정상(run.sh
가드), 앱 내 fork 없음 ④ **IgH: master 락 = `struct semaphore`(master.h:198, PREEMPT_RT에서
PI 없음) + master 스레드 EtherCAT-OP/IDLE = SCHED_NORMAL nice 0(master.c:1681, 실측
TS/CPU0)**. 메커니즘: 부하 높으면(비전 재시작 워밍업 00:24/00:43/01:33 + 빌드 +
VSCode/에이전트) TS인 master 스레드가 semaphore를 쥔 채 CFS 대기 → 앱 TASK1의 매 사이클
ioctl이 그 semaphore에서 대기(상속 없음=무한 역전) → TASK0 연쇄 → 2~20 ms 구멍이 세션
내내(66%까지). 궤적은 실행 사이클로 흘러 4배 슬로모션+스터터. **왜 이제서야**: 이전
세션들은 보드가 조용했음 — 07-30 20:32 첫 로그 7.3% 갭이 미니어처 전조.
**처방**: ① 앱 시작 후 `sudo chrt -f -p 40 $(pgrep -x EtherCAT-OP)` (OP 스레드는 앱마다
재생성되므로 세션마다; IDLE도 무해) — **자동화 완료(07-31, RAON `6c090ae`)**: run.sh가
백그라운드 워처로 OP 스레드 출현을 기다려 고정 명령 wrapper(`/usr/local/sbin/ecat-op-fifo`,
sudoers NOPASSWD)를 호출. **1회 설치 필요**: `sudo bash tools/install_ecat_op_fifo.sh`
(미설치면 앱 콘솔에 수동 명령 WARN) ② 실기 세션 중 보드에서 빌드/무거운 분석 금지(운영
수칙, 에이전트 포함) ③ 장기적으론 IgH master 스레드 FIFO화 패치/재빌드 옵션.
**#40 데이터는 FF-gate 판정에 사용 금지.**

### ⑤ 검증 #41~46 → E35 소이동 과주 발견 → lag gate (`90e5409`) + KF_4=1.5

**이 시점 누적 구성**: 페이로드 탈거+질량 0 · KD 28/25/20 · KF_3=5.0 · refine scope gate · **[신규 운영] `chrt -f 40` + 조용한 보드(E34 처방)**

**#41~46 (07-31 02:00)**: 갭 6/6 0.0%(E34 처방·조용한 보드) + E33 scope gate 실증(발산
소멸, refine 1~2회, final 3.45~9.57 mm 전원 합격, refine 패스의 j3 act/cmd ≈ 0 복귀), FF
큰-이동 이득 유지(apple 17.2 vs pre-FF ~50 / tennis 17.4 vs ~43 / orange 31 vs 37~94 mm),
banana 47 불변(z=j4 몫, KF_4=0=예상대로). 유일한 ⚠️ = **mustard first 26→58 mm 일관 악화**.

**E35 해명(DataLog 비교)**: pre-FF(#26/#32)는 pass-1 종료 시 j3가 +0.06~0.09 rad
**뒤처져** 정지(stiction 미달)인데 FF(#46)는 j3가 cmd 0.28 rad 이동을 **0.41 rad 주행**
(144%), 최종 ref를 +0.13 rad **지나쳐** 정지 — 부호 반전이 그대로 +36 mm y-미스(레버
~0.28 m, CSV의 y +36.5와 일치). 메커니즘: **이동량이 작으면**(mustard만 j3 0.1~0.3 rad)
감속 구간까지 유지되는 풀 FF(+5 Nm)를 관성이 소화 못 해 과주하고, settle은 q̇_ref=0→FF=0
이라 stiction이 과주 지점에 고정(**편도 티켓** — FF는 밀어주긴 해도 되돌려주진 못함). 다른
물체는 j3 이동이 커서 순이득이었던 것.

**수정 = lag gate**(`90e5409`): FF 항에 `clamp((q_ref−q)·sign(q̇_ref)/0.002, 0, 1)` 곱 —
관절이 명령 방향으로 **뒤처진 동안만** FF(존재 이유인 stiction 상황; 실측 중이동 lag
0.01~0.13 rad라 견고히 1), 따라잡거나 앞서면 0(그때 마찰 제동은 아군). 연속형이라 토크
챠터 없음, 정지 시 0 불변, E33 scope gate는 이중 방어로 유지. **KF_4=1.5 동시 투입**
(cfg만, 커밋 안 함) — j4 이동도 전형적 소이동이라 같은 함정인데 lag gate가 선방어. 참고:
전 mustard 로그에서 pass-1 j4 오차 = 정확히 **+0.150 rad(leash 핀)** — j4는 비행 중에도
전혀 안 움직였음(act d = −0.000). 판독 스크립트: scratchpad `mustard_ff_analysis.py`
(pass 경계 = goal 점프, pass-1 종료 관절오차 + overshoot 추적).

### ⑥ #47~51 — 원샷 3/5, j4 방향 비대칭 정량 → KF_4 1.5→3.0

**이 시점 누적 구성**: 페이로드 탈거+질량 0 · KD 28/25/20 · KF_3=5.0 · refine scope gate · E34 운영수칙 · **[신규] lag gate(`90e5409`) + KF_4=1.5**

**원샷 3/5 달성** — apple 9.9 / banana **47→7.9** / mustard **58→6.5 mm**, 전부 refine
0회(E35+KF_4 적중; j3는 1.31 rad 이동 97% 추종, lag +0.040으로 과주 없음 = lag gate 실증).
잔여 = orange 33.8(refine 2)·tennis 31.2(refine 1), **원인 = j4 방향 비대칭**: 두 물체
모두 j4 **+방향**(반중력) 0.13~0.14 rad 요구 → act +0.003~0.007(전혀 breakaway 못 함,
j5 leash 핀은 roll이라 위치 기여 0), banana는 **−방향** 0.152 요구 → 0.092(60%) 이동.
τ4 정량: 중력 유지 **+2.8 Nm**, +이동 시 PD+FF 최대 5.1 Nm = 잉여 2.3 Nm < stiction(미동),
−이동은 중력이 아군이라 1.5로 충분했던 것. **KF_4 1.5→3.0**(cfg) = 잉여 ~3.8 Nm — lag
gate가 과주 상한이라 인상 안전. 참고: 전 접근은 HOME(ready) 출발 궤적(시작 TCP 5/5 동일
0.63/0.10/0.35). **물체→물체 직행도 동작함**(07-31 사용자 수동 확인; 궤적은 항상 m_Q에서
시작 + D13 시드는 q_ready 고정이라 출발 자세 무관).

### ⑦ #52~58 — 🎯 원샷 목표 달성 (07-31 02:41, KF_4=3.0)

**이 시점 누적 구성**: 페이로드 탈거+질량 0 · KD 28/25/20 · KF_3=5.0 · refine scope gate · lag gate · E34 운영수칙 · **[신규] KF_4 1.5→3.0**

**7/7 refine 0회** — HOME 출발 5/5(orange **33.8→7.7** / apple 10.7 / tennis **31.2→9.2** /
banana 7.8 / mustard 7.0 mm) + **mustard→orange 전이도 원샷 11.3 mm**(#58; j0 −0.99 rad
대이동 99% 추종 = 출발 자세 무관 실증). **KF_4 3.0이 반중력 +방향 해방**: #52 j4 +0.28 rad
**100% 추종**(직전 0.005), #58 +0.31 rad 94%. **전 로그 과주 0.000 rad**(KF_4 2배 인상에도
lag gate가 상한 역할 실증), 갭 7/7 0.0%.

**남은 오차의 구조가 바뀜**: stiction 지배 → **바이어스 지배**(miss가 전 건 −x 2~7 /
−z 3~9 mm로 일관 + j2(elbow) lag −0.014~−0.028 rad 상시 잔존 — apple 10.7 mm는 거의 전부
j2×레버 ~0.4 m 몫). **파지 <5 mm가 필요해지면 다음 지렛대 = KF_2**(elbow, τ2 여유 큼) +
일관 바이어스 보정(방향 재현성은 07-29 그래프 자동화 때 이미 확인). j5 leash 핀(0.15 rad,
roll=위치 무관)은 파지 자세 단계 항목으로 유지.

**역검증(#59, 07-31 15:07) — 마찰 인과 양방향 확증**: cfg만 원작 제어로 되돌려(KD 20/15/10 ·
KF 0/0) 같은 배치에서 재실행 → **orange first 111.2 mm · refine 6회 상한 소진 · final
13.8 mm(tol 12 초과)** vs 같은 날 KF-on #52의 7.7 mm 원샷. "KF=0 = 원작 토크식"의 근거는
주장이 아니라 **함수 단위 전수 diff**(fork `d390436` 대비 52함수): 사이클 토크 경로의 수정은
④개뿐 — FF 블록(`if(m_Kf>0)` 게이트=KF 0이면 사문)/sticky-float(grav-comp 모드 게이트)/
앵커 플래그 리셋 5줄/공백 1줄 — 이고 **`UpdateTrajectory`(min-jerk+settle leash)는 무수정**
(leash=원작 재확인). 나머지 수정·신규 함수는 전부 목표 선택(IK/폴딩/T스케일)과 진단 —
FK(q_ref)가 매번 목표 도달한다는 그래프가 그쪽의 무죄 실측. 켜면 사라지고 끄면 돌아온다 =
**stiction 결론 확정, 교수님 보고 가능**.
**콜드 마찰 발견 + KF_3 최종 6.0(#60~82, 07-31 오후)**: 12 h 휴지 후 재개하자 같은
goal·IK·갭 0%인데 **j3 lag만 2~4배**(orange first 7.7→20.1 mm) — 마찰 크기가 세션
상태(온도/휴지)에 의존. 워밍업 ~10회로 20→12.7 mm까지 회복 후 **정체**(12 mm 문턱 위)
→ **KF_3 5.0→6.0**(+1 Nm, breakaway 하한 7.2 미만, lag gate가 과주 상한) → **#78~82
5/5 원샷 복귀**(orange **6.15 mm 역대 최고**/apple 10.9/tennis 8.3/banana 8.2/mustard
7.0). 육안 "말단 미세 떨림"은 정량으로 5.0 동급 이하(gate 토글 1~3회·Δτ3 hf 0.06~0.11
Nm·TCP 스텝노이즈 9~26 µm) = 감속 꼬리 lag 만회 스윙, 코스메틱. **운영 수칙: 콜드
세션 첫 2~3회 접근은 워밍업으로 간주**(문턱 근처 miss 정상). 최종 게인 = KD 28/25/20 ·
**KF_3 6.0 / KF_4 3.0**. 부수 사고 1건: 복귀 중 급정지 신고 → 'j'(서보 해제) 키
typematic 입력으로 판명(500 ms 지연 후 33 Hz 반복 = 물리 키 눌림, 로봇·코드 무죄).
**관찰(끝단 미세 흔들림, orange/apple/tennis만)**: 정체 = **감속 꼬리의 j3 lag 재축적 후
만회** — j3 대이동(0.5~1.1 rad) 물체만 감속 구간에서 lag가 53~97 mrad(3~6°)로 부풀었다
마지막 ~0.5 s에 FF+PD가 되감음(공구가 눈에 띄게 몇 도 스윙; banana/mustard는 j3 2~13 mrad라
안 보임). 헌팅 아님(부호 교차 0), gate 플리커 아님(1~9회), settle은 완전 정지(ref 정지 후
TCP 이동 0.0 mm), 과주 0, 최종 정확도 무영향 — **코스메틱, 대응 불요**(원인은 감속에서
속도↓·마찰 상대 우세 + inertia-scaled PD 저권한이라 근본책은 wrist 권한 구조 변경뿐).

### 부록 — #23 판독: settle leash 발견과 3계열 그래프 인프라 노트

**settle leash 발견**: banana #23에서 one-shot plan 끝의 "+6.4 mm 상승"을 DataLog로 추적 →
pass-1 계획은 t=2.97에 목표 0.00 mm 도달, 궤적 시간 종료 순간 `MAX_REF_LEAD 0.15 rad`
(FullDynControllerRT.cpp:1417, 궤적 종료 후 reference가 실측보다 0.15 rad 이상 앞서지
못하게 당기는 토크 상한 leash)가 발동해 **손목 pitch(q4) reference를 1.69° 회수**(레버암
~22 cm → TCP +6.4 mm). 원인은 q4가 stiction으로 **10.3° 뒤처진 채 정지**한 것 — first
miss 46 mm의 주범이 손목 pitch 걸림임을 그래프+로그가 직접 지목. 1차 착지가 목표보다
**위(+36 mm)** 인 것은 중력 과보상 방향과 일치 → 페이로드 A/B 제거 실험(①)으로 연결.

3계열 = **IK 입력**(goal+refine 편향 스텝) / **FK(q_ref)**(레퍼런스) / **FK(q_act)**(실측)
— refine이 왜 2~3 pass 필요한지를 그림 한 장으로 보여주는 도구(ref는 명령점에 도달, act는
못 미침 = CTC 무적분+stiction; 편향 스텝이 ref를 goal 너머로 밀어 act를 goal에 얹는
과정이 수직 점선으로 표시). 변위는 방향을 접으므로 "변위 같음 ≠ 도달"(측면 miss 불가시) —
진짜 오차 판정은 제목의 final miss와 정확도 PNG가 담당하고, 축별 방향 정보는 3D 경로와
DataLog CSV에 남는다.

- **RT 비용 신규 0**: 접근이 **기존 'l'/'a'/RECT 로깅 블록**(FK 1회+락프리 링 push, 실기
  1 kHz 검증 완료)을 켜는 것뿐. 엣지에서 atomic 스토어 2회 추가가 전부. 파일 IO는 prio 30
  Logger 태스크, FK 재계산·작도는 별도 오프라인 프로세스
- **로거 윈도우 변경**: 고정 24 s → 접근은 `m_bLogUntilDone`으로 **완료 에지까지**(실기
  접근 간격 ~15 s라 고정창이면 두 접근이 한 파일에 섞임). 수동 'l'은 24 s 유지. 플래그는
  `exchange()` 소비 + idle-trigger 가드로 다음 로그 오염 차단. 중단 접근도 파일은 닫힘
- **짝 맞추기 = 시각 근접**(DataLog 스탬프 ↔ row unix_s, 15 s 창, 파일당 1회) — 완료
  에지에서 둘 다 ~1 s 안에 찍히므로 배선 불필요. 워처 시그니처에 `rt_log_results` 포함
- `plot_tcp_trajectory.py`(기존 궤적 그리기)는 **레이아웃만 계승** — 보드에서 실행 불가
  (pandas 없음·`plt.show()` 블로킹·세션 단위 대상)라 워처에 numpy+matplotlib로 이식
- 앱 쪽 arm/close 프로토콜(트리거 엣지)·55 s 캡 — 당초 실기 미검증 항목 → 07-31 접근 30건+로
  자동 검증 완료. DataLog는 접근당 수 MB — `rt_log_results/` 주기 정리 필요

## 10. 재개 가이드 (새 세션용)

1. 이 문서 + 메모리 `raon-vs-merge-plan.md`부터. RAON-RT 코드 질문은 재탐색 말고
   `~/RAON-RT_guide_for_CLAUDE.md`의 해당 §. 정밀화 트랙(마찰 FF·KD/KF·E32~E35·원샷)은
   **§9.5**가 정본.
2. 작업은 `~/RAON-RT-Revision` **kv260-merge 브랜치에서만**. lab 저장소 push 금지.
3. EMasterApp 수정 시 실행 전 `sudo make install` (루트에서) — /opt가 런타임 정본.
4. 로봇 전원만 켜면 `ethercat slaves`로 7슬레이브 즉시 확인 가능(일반유저 CLI 가능).
5. sudo가 필요한 단계는 명령을 준비해 사용자에게 요청(비대화형 sudo 불가).
6. **운영 수칙(E24 이후 갱신)**
   - **E34(2026-07-31)**: EtherCAT-OP의 FIFO 40 승격은 **run.sh가 자동 처리**(성공 시 콘솔
     `[run.sh] EtherCAT-OP -> SCHED_FIFO 40 (E34 auto)`). 전제 = **1회 설치**
     `sudo bash tools/install_ecat_op_fifo.sh`(미설치면 WARN + 수동 명령 안내).
     **실기 세션 중 보드에서 빌드·무거운 분석·에이전트 금지** — 위반 증상 = 갭/슬로모션/
     "덜덜"(§9.5 ④). 의심되면 DataLog Δt 갭 체크(§9.5).
   - **HOME을 재기록하면 reach map은 무효**(§0 07-29 앵커 민감도) — 좌표를 뽑아 쓸 땐
     현재 앵커로 재계산.
   - 세션 첫 행동 = **자연 자세에서 HOME 기록**(앵커). 전 생애 1회지만 앵커가 오염되면 재기록.
   - **종료 자세는 이제 무관**(AXIS4/5도 HOMING_METHOD=3 → POS_BEFORE_EXIT를 아무도 안 읽음).
     "팔을 세워두고 종료" 규칙은 폐기.
   - 종료는 **Ctrl-C**. `'q'`는 `DeInit`을 안 타서 종료 진단 로그(`AXIS[n] CurrentRawPos/
     HomePosition/Diff`)와 PBE 갱신이 없음.
   - **접근 정확도 그래프는 자동이다**(2026-07-29~). `run.sh`가 워처를 띄우고 앱 종료 시
     회수하므로 손댈 것 없음. 결과 = `App/Indy7/approach_results/plots/`(`approach_latest.png`,
     `approach_history.png`), 원본 = `approach_results/approach_log.csv`(앱 실행 간 누적).
     끄려면 `INDY7_NO_PLOT=1 ./run.sh`. 수동 재작도 = `python3 tools/approach_plot.py --all`.
     ⚠️ 그래프의 원점은 **물체가 아니라 호버 지점**(물체 z + 0.15 m) — E31.
   - **`s`는 이제 기본이 출력 전용이다(2026-07-29부터, `w` 게이트)**. 예전에는 `s`가
     `App/CalibUtils/kv260/robot_poses.csv`와 `pose_rPe_*.yaml`을 **무조건 덮어썼고**
     (매 실행 첫 `s`는 csv를 **truncate**), 그게 진단용 출력처럼 보였다. 지금은:
     - 평상시 `s` → `[TCP Pose] (print only — 'w' arms capture)`, **파일 무접촉**
     - `w` → 배너와 함께 캡처 무장(다음 N 표시), 이때 `s`가 `*** CAPTURED ***`로 기록
     - `w` 다시 → 해제. **매 실행 OFF로 시작**한다
     - 사고 시 복원은 그대로 `git checkout -- App/CalibUtils/kv260/`
     ⚠️ **정식 캘리브 세션에서는 `w`를 먼저 눌러야 한다** — 잊으면 이미지 16장에
     pose 0개로 끝난다(데이터가 틀리는 게 아니라 없으므로 안전한 실패, 재실행하면 됨).
     `tools/calib_grab.py`가 매 컷마다 `*** CAPTURED ***` 확인을 요구한다.
   - ⚠️ **새 키를 추가할 때 `proc_keyboard_control`을 먼저 볼 것.** 이 스레드가
     `q`(→`StopTasks()`, 그래서 `q`는 **종료가 맞다** — 다만 `DeInit`을 안 탄다)와
     `z`/`Z`(→ISO 큐브 IK 검증)를 **`m_cKeyPress`를 설정하지 않고 소비**한다. 따라서
     `DoInput`에 그 두 키의 `case`를 써도 **도달하지 않는다** — 실제로 `case 'q'`(ISO
     큐브 하드웨어 테스트)는 그렇게 죽어 있다. `w`가 유일하게 남은 자유 알파벳이었다.
   - **★ 6/8 앵커 시드 보존(2026-07-28 03:41:51 기록)**: `q = [0.26, −0.90, −0.87, −0.14, 0.15, −0.56]`
     rad — 커버리지 **6/8**(미달 2점은 z=0.50 최상단 코너 (0.30,0.63,0.50)·(0.76,0.51,0.50)로
     실제 물체와 무관). 이후 다른 자세 실험으로 `~/.indy7_ready_seed`가 덮어써졌으므로
     **`~/.indy7_ready_seed.6of8-20260728T0341`에 별도 보존**했다. 복원 =
     `cp ~/.indy7_ready_seed.6of8-20260728T0341 ~/.indy7_ready_seed` (앱 실행 전에).
     정밀도는 로그 표기대로 소수 2자리(±0.005 rad = ±0.29°) — 시드는 브랜치 선택이 역할이고
     자세 기준선으로도 0.3°는 무시 가능하므로 충분하다. 참고로 현재 파일에 남은 값은
     03:46:34에 기록된 다른 자세 `[0.248, −0.844, −0.719, −0.024, −0.750, −0.560]`이다.
   - **`n/8`은 도달 영역이 아니다** — 워크스페이스 박스 `x[0.30,0.94] y[−0.37,0.63]
     z[0.10,0.50]`의 **꼭짓점 8개**(r>0.92면 반경 클램프)이고, 절반이 z=0.50으로 실제 물체
     높이(goal z≈0.28~0.35)보다 15~20 cm 위다. **체계적으로 가장 어려운 점만 찍는 지표**라
     실제 도달성의 예측자가 못 된다 — 5/8 앵커가 5클래스를 전부 첫 단에 풀었고, 6/8 앵커에서
     banana/orange가 거부된 것이 두 반례. 대체물 = reach map(§9)
   - **시드 백업은 파일을 그대로 `cp`할 것** — 로그의 `[SEED] ready posture q = [...]`는 소수
     2자리 표시라 옮겨 적으면 관절당 ±0.29°가 실린다. 실제로 그렇게 복원한 앵커가 경계선의
     코너 프로브 2개를 뒤집어 6/8이 4/8로 보였다(접근 자체엔 무해했음).
     현재 보관: `~/.indy7_ready_seed.good-20260728T1634`(전 5클래스 첫 단 통과 실적)
   - 앱에는 **로그 싱크가 없다**(`~/.ros/log/Indy7Ctrl.out_*.log`는 전부 0바이트, run.sh는
     stdout으로만). 사고 조사 시 터미널 스크롤백이 유일한 사본이므로 세션 종료 전에 덤프.
