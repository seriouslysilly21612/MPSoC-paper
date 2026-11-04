# 워크스페이스 정리 계획 (분류 결과)

> 생성: 2026-07-15 · 전수 조사(174 항목 분류 / 45 삭제후보 적대적 검증 / 8건 구조)
> **상태: 분류만 완료. 아무것도 삭제되지 않았습니다.** 사용자가 삭제 항목을 지정하면 실행.

---

## 1. 결론 요약

- **🔴 보안 최우선: `/home/ubuntu/ros2_ws/github_personal_access_token.txt` (41B)** — 평문 GitHub PAT(`ghp_gOF6…`)가 repo 루트에 있고, **`.gitignore`에 걸려 있지 않습니다**(`git check-ignore` exit 1, `git status`에 `??`로 노출). `git add .` 한 번이면 public remote(`github.com/seriouslysilly21612/Capstone_Design`)로 푸시됩니다. **파일 삭제는 해결이 아닙니다 — 먼저 github.com/settings/tokens에서 revoke하고, 그다음 삭제하고, `.gitignore`에 규칙을 추가**하세요.
- **총 242M** (ros2_ws 159M + vitis_ai_work 83M). 이 중 **약 88M(36%)이 검증을 통과한 안전 삭제/재생성 대상**입니다. 디스크는 173G 여유라 공간이 목적이 아니라 **탐색 가능성(navigability)** 이 목적입니다.
- **최대 의외의 발견 2가지**:
  1. **실제로 돌아가는 시스템은 아주 작습니다.** ros2_ws 159M 중 117M(74%)은 third-party `src/realsense-ros` 클론이고, 프로덕션 파이프라인이 실제로 쓰는 우리 코드는 **6개 파일 ~110K + config 5개 ~6.5K**가 전부입니다.
  2. **`vitis_ai_work/perf` 35M 중 33M(94%)이 단일 rosbag 하나**(`bags/t3d_input_0.db3`)입니다. 문서엔 "reindex 필요"라 적혀 있지만 실제로는 **btree 손상**(`ros2 bag reindex`가 core dump)이라 복구 불가입니다. 이것 하나 지우면 perf가 35M→2.4M이 됩니다.
- **⚠️ 삭제보다 더 급한 위험 — `yolo_v3_tiny_training/`이 git에 없습니다.** `git status`에 `?? yolo_v3_tiny_training/`. README는 "보드에서 버전관리"라고 주장하지만 실제로는 **한 번도 커밋된 적이 없고**, rsync 방향이 보드→데스크톱이라 **보드가 마스터**입니다. 현재 배포 모델을 재생산할 유일한 경로인데 백업이 0입니다. **개편 착수 전에 커밋하세요.**
- **문서가 거짓말을 하고 있습니다.** Claude Code가 매 세션 자동 로드하고 라우팅 맵이 "항상 우선"이라 표시한 `CLAUDE.md`가 아직도 detector를 `ssd_adas_pruned_0_95` stand-in, worker를 `vitis_ai_worker.py`, D435i FW를 5.16.0.1이라고 적고 있습니다(전부 사실 아님). 삭제보다 **이 갱신이 이번 작업의 최고 가치 항목**입니다.

---

## 2. 절대 삭제 금지 (ACTIVE) — 이게 지금 돌아가는 것

`ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py` 하나가 띄우는 전부입니다.

### ros2_ws — 프로덕션 파이프라인

| 경로 | 크기 | 역할 |
|---|---|---|
| `src/system_bringup_pkg/launch/pick_place_vitis_ai.launch.py` | 3.3K | **유일한 진입점**. 6개 노드 기동 + 63-68행에 "노드 병합 금지" NOTE |
| `src/system_bringup_pkg/config/{realsense_pick_place,vitis_ai_detector,pick_logic,target_3d,target_base}.yaml` | 6.5K | 튜닝값 **+ 그 근거 주석**. 코드에 없는 유일한 기록 |
| `src/my_interfaces/` (msg 4개) | 44K | Detection/DetectionArray/PickTarget/PickTarget3D — 모든 노드의 타입 뼈대 |
| `src/vitis_ai_detector_pkg/…/vitis_ai_detector_node.py` | 31K | worker 서브프로세스 관리 + `/detections` 발행 |
| `src/vitis_ai_detector_pkg/…/vitis_ai_worker_yolo.py` | 16K | **프로덕션 DPU worker**. per-class 임계(person 2.0=검출 OFF) 등 튜닝 내장 |
| `src/pick_logic_pkg/…/pick_logic.py` | 6.8K | 클래스/신뢰도/기하 필터 → `/pick_target` |
| `src/target_3d_pkg/…/pick_target_3d_node.py` | 19K | 역투영 (epipolar) → `/pick_target_3d` |
| `src/target_3d_pkg/…/pick_target_base_node.py` | 8.7K | base_link 변환 → `/pick_target_base` (최종 출력) |
| `src/realsense-ros/` | **117M** | third-party 클론(4.57.7), gitignored. 클러터 아님 — 재빌드 유일 경로 |
| `fastdds_shm_profile.xml` | 4.0K | **`.bashrc:124`가 export 중**. 지우면 512KB 기본 SHM으로 떨어져 1.22MB Image 전송 실패 (-6.6%p 손실) |
| `site_md/reference_0{1..6}_*.md` | 52K | `CLAUDE.md:104,112-117`이 **경로째 하드코딩**한 리서치 룩업 테이블 |
| `rt_verify/{churn,sustained}.sh`, `cyclic_rt.sh`, `soak_rt.sh` | 20K | RT/zocl 재검증 하네스. EtherCAT 단계에서 재사용 예정 |
| `yolo_v3_tiny_training/` | 152K | **모델 재생산 마스터. git 미추적 — 최우선 커밋 대상** |
| `.git/` | 1.7M | ⚠️ `main`과 `origin/main`이 **1:1 diverge** 중 |

### vitis_ai_work — 모델·계측

| 경로 | 크기 | 역할 |
|---|---|---|
| `models/yolov3_tiny_7class.xmodel` | 8.9M | **배포 모델** (md5 9bc6520c, 실제 6-class D14) |
| `models/decode_meta.json` | 631B | worker가 xmodel **옆에서 자동 로드**. 없으면 즉시 abort |
| `arch/arch_b3136.json` | **41B** | fingerprint 0x101000016010406. 모든 recompile 스크립트가 이 파일을 scp로 요구 |
| `smartcam_models/models/ssd_adas_pruned_0_95/` | 1.2M | 문서화된 SSD 롤백 대상 + 노드의 model_path 기본값. **`/opt/xilinx` 없음, apt에도 없음 → 보드 유일 사본** |
| `scripts/{yolov3_tiny_image_test,capture_color_frames,autolabel_single_object,vitis_ai_worker_protocol_test,vart_smoke_test}.py` | 40K | Gate5 격리 테스트 / D14 재현 경로 / DPU liveness 프로브 |
| `perf/{perf_probe.py,summarize.py,run_gate6_perf.sh,run_gate6_perf_3core.sh,thread_sample.py}` | 26K | 계측 하네스. `rt_verify/*.sh`가 DPU 부하원으로 호출 |
| `test_images/topdown_nopeach/` | 328K | **진짜 회귀 fixture** (아래 §4 참고) |
| `perf/runs/{stock_fw51710_baseline,phase1_cpuopt,phase2_final,3core_baseline,gate6_4core_15hz}_*` | 2.1M | 76.8→44% 주장의 증거. 문서가 경로째 인용 |

---

## 3. 삭제 후보 (검증 통과)

크기 내림차순. **누적 회수량**은 오른쪽 끝.

| 경로 | 크기 | 무엇인가 | 왜 삭제해도 되나 | 잃는 것 | 누적 |
|---|---|---|---|---|---|
| `vitis_ai_work/perf/bags/` | **33M** | 2026-07-10 크래시 중 끊긴 rosbag | `metadata.yaml` 없음 + btree 손상(error 11, page 3206-4609). **`ros2 bag reindex`가 core dump** → 복구 불가. 메시지 130개(≈2.9초)뿐이고 33M은 sqlite 예약공간. target_3d는 이미 최적화·검증 완료 | 손상된 2초 데이터. `launch_bag.log`(크래시 기록)는 별도 보존 | 33M |
| `ros2_ws/build/realsense2_camera_msgs` | 13M | src/realsense-ros 오버레이 빌드 중간물 | `src/realsense-ros`에 `COLCON_IGNORE` → **재빌드해도 안 생김**(고아). install 쪽이 self-contained(심링크·RPATH 0) | 없음 | 46M |
| `ros2_ws/log/` | 8.5M | colcon 빌드 로그 61개 (2026-03~07) | gitignored, 인바운드 참조 0. `colcon build` 시 자동 재생성 | ⚠️ **예외 1건**: `log/build_2026-05-05_16-16-01/realsense2_camera/stderr.log`는 realsense-ros 빌드를 포기한 유일 근거("RealSense SDK 2.0 is missing"). 그 6줄만 문서에 옮기거나 그 디렉터리 하나만 남길 것 | 55M |
| `vitis_ai_work/real_apple_frames/viz/` | 7.4M | D14 오토라벨 육안검수 오버레이 68장 | 검수 완료·결과 기록됨(plan:349 "obj_54 1장만 오검출→삭제"). 데스크톱으로 rsync되지 않음(images/labels만), `06_merge_split.py`도 안 읽음. 라벨+원본이 남아 있어 10줄 cv2로 충실 재생성 가능 | 없음 (spent QA 렌더) | 62M |
| `ros2_ws/build/` (나머지) | 6.2M | colcon 중간 빌드 | gitignored. **재생성**: `cd /home/ubuntu/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash` | 없음. ⚠️ install/의 egg-link 5개가 build/를 가리킴 → **build/와 install/은 반드시 함께** 삭제·재빌드 | 68M |
| `ros2_ws/install/realsense2_camera_msgs` | 5.5M | apt 사본을 가리는 오버레이 install | apt `ros-humble-realsense2-camera-msgs` **4.57.7 == 동일 버전, msg 전부 byte-identical**. 지우면 `/opt/ros/humble`로 폴백. 오히려 현재 진행 중인 mixed-copy 상태(typesupport .so가 오버레이 것으로 로드됨) 해소 | 없음. 확인: `ros2 pkg prefix realsense2_camera_msgs` → `/opt/ros/humble` | 74M |
| `vitis_ai_work/test_images/test_set/test.jpg` | 4.9M | SSD 브링업용 3000x2000 파리 거리 스톡사진 | 검증 완료(progress.md:732), SSD 자체가 superseded, pick&place 씬과 무관. 참조 0 | 없음 (교체 가능한 스톡 이미지) | 79M |
| `…/test_images/result_original/ssd_adas_result_original.jpg` | 2.9M | 위 사진의 원본해상도 SSD 오버레이 | 순수 파생물(픽셀 95.3% 동일, bbox 선만 차이). 결론은 progress.md:732에 기록 | 없음 | 82M |
| `vitis_ai_work/realsense_frames/realsense_00*.jpg` (12장) | 2.3M | 2026-05-17 SSD 시절 1280x720 캡처 | **해상도가 배포 프로파일(848x480)과 불일치** → fixture 불가. "도메인 갭 최초 관측" 서사는 progress.md:736-745에 기록, 그 갭은 이미 해소 | ⚠️ `yolov3_tiny_image_test.py:9`의 usage 예시 주석이 dangling → `topdown_nopeach/obj_00.jpg`로 교체 권장 | 84M |
| `ros2_ws/install/` (나머지) | 2.1M | colcon install prefix | gitignored. 위 재생성 명령. **`--symlink-install` 필수** — 빼면 config YAML 실시간 편집 워크플로가 조용히 깨짐 | 없음 | 86M |
| `smartcam_models/models/densebox_640_360` | 920K | 벤더 얼굴검출 모델 | 참조 0, pick&place와 무관. **복구 가능**: `.bash_history`에 레시피 보존 + `xilinx/smartcam:2022.1` 이미지가 보드에 아직 있음(ID da2e5262, md5 일치 확인) | 없음 | 87M |
| `ros2_ws/build/realsense2_camera` | 312K | **실패한** CMake configure 잔해 (rc=1) | 바이너리 생성 안 됨. 프로덕션은 apt 노드 사용 | 없음 | 87M |
| `ros2_ws/src/install` | 260K | src/ 안에서 잘못 돌린 colcon의 install prefix | **git에 커밋됨**(34파일, `.gitignore`가 `/install/` 루트 앵커라 미매치). **SSD 시절 config를 실어 위험**: `source ~/ros2_ws/src/install/setup.bash` 하면 SSD model_path로 파이프라인이 뜸. 제거: `git rm -r --cached src/install` + `.gitignore`에 `src/install/` | 없음 (shadowing 해저드 제거) | 87M |
| `…/realsense_frames/realsense_result_{original,model}.jpg` | 232K | SSD 오버레이 2장 | **박스가 0개** (픽셀 diff max 11, >30인 픽셀 0) = 원본 재인코딩일 뿐 | 없음 | 87M |
| `…/result_model/ssd_adas_result_model.jpg` | 128K | 480x360 SSD 오버레이 | 파생물, 결론 기록됨. 문서의 보존자산 목록에서 명시적으로 제외됨 | 없음 | 87M |
| `ros2_ws/src/log`, `ros2_ws/src/build` | 104K + 92K | 2026-05-21 오폭 빌드 잔해 | 둘 다 git 커밋됨(12+8 파일). COLCON_IGNORE 있어 빌드엔 무해, **git 클러터**. `git rm -r --cached src/log src/build` + `.gitignore` | 없음 | 87M |
| 모든 `__pycache__` (7곳) | 248K | 바이트코드 | **재생성**: `find /home/ubuntu/ros2_ws /home/ubuntu/vitis_ai_work -name __pycache__ -type d -prune -exec rm -rf {} +` | 없음 | 88M |
| `test_set/{car,bicycle,person,background,test2,test3}.jpeg` + `{car,bicycle,person,background}_result.jpeg` | 138K | SSD 웹 썸네일 입력 4장 + 오버레이 4장 + 미문서 2장 | 전부 superseded SSD 클래스(car/bicycle/person). `.bash_history`가 전 생애 기록(2026-05 1회 실행 후 미사용). 문서의 보존자산 목록(topdown/, objects*/)에서 제외 | 없음 (`test1.jpeg`는 §4 참고 — **남기세요**) | 88M |
| `src/camera_source_pkg/` | 56K | legacy USB 카메라 (cv2.VideoCapture) | **가장 고아**. launch/config/harness/memory 어디도 참조 0, stale APU launch조차 미참조. git 추적됨(01da279) → 복구 가능. ⚠️ editable install이라 **재빌드로 안 지워짐** — `install/camera_source_pkg`도 함께 rm | ~83줄 cv2 스니펫 (git 히스토리에 보존) | 88M |
| `src/mock_detection_pkg/` + `src/system_bringup_pkg/launch/pick_place_apu.launch.py` | 56K + 2.9K | pre-DPU mock 파이프라인 (한 단위) | 라이브 참조 0(progress.md의 역사 서술뿐). **origin/main에 byte-identical 존재** → `git checkout origin/main -- src/mock_detection_pkg`로 완전 복구. "no-DPU 폴백" 논거는 zocl 픽스(07-15)로 무효화 — DPU가 이제 RT 커널에서 돕니다 | ⚠️ `system_bringup_pkg/package.xml:14`의 `<exec_depend>mock_detection_pkg</exec_depend>`를 **같은 커밋에서 제거**해야 함 | 88M |
| `vitis_ai_work/scripts/ssd_adas_path_worker.py` | 16K | 폐기된 path-기반 worker 프로토타입 | 참조 1개(progress.md:1383 "root-cause isolation 중 추가된 진단 스크립트" 역사 목록). 유일 통찰(graph 수명 주석)은 3곳에 중복. 문서화된 SSD 롤백은 이 파일이 아니라 in-package `vitis_ai_worker.py`를 가리킴 | ⚠️ vitis_ai_work는 **git repo가 아님** → 되돌릴 수 없음. tarball로 아카이브 권장 | 88M |
| `src/my_interfaces/{include,src}` | 12K | 빈 디렉터리 (`ros2 pkg create` 잔해) | 파일 0개, git 미추적(빈 dir은 추적 안 됨), CMakeLists에 `include_directories()` 없음 | 없음 | 88M |
| `ros2_ws/.agents`, `ros2_ws/.codex` | 4K + 4K | **완전히 빈** 디렉터리 | `stat` 상 Links=2, mtime==btime → 생성 후 아무것도 들어간 적 없음. 도구가 필요하면 `mkdir`로 자동 생성 | 없음. `.gitignore`의 규칙 18-19행은 **남겨두세요** | 88M |
| `perf/threads.csv`, `perf/threads_console.log` | 4K | 루트 스크래치 | `runs/3core_baseline/`의 사본과 **byte-identical**(md5 일치). 하네스가 `cp`로 아카이브 → 루트는 스크래치. 다음 실행 시 어차피 덮어씀 | 없음 | 88M |
| `perf/vitis_ai_detector.yaml.bak` | 2.6K | 하네스 백업 | 라이브 config와 **byte-identical**(= trap 복원 성공 증거). **재생성**: `cp /home/ubuntu/ros2_ws/src/system_bringup_pkg/config/vitis_ai_detector.yaml /home/ubuntu/vitis_ai_work/perf/vitis_ai_detector.yaml.bak` (`run_gate6_perf.sh:60`이 매 실행마다 자동 생성). ⚠️ 계측 실행 중엔 지우지 말 것 | 없음 | 88M |
| `vitis_ai_work/logs/ssd_adas_{xmodel_info,prototxt}.txt` | 6.9K | SSD 덤프 2개 | prototxt는 벤더 파일의 **완전 중복**(diff 결과 동일). **재생성**: `xdputil xmodel -l /home/ubuntu/vitis_ai_work/smartcam_models/models/ssd_adas_pruned_0_95/ssd_adas_pruned_0_95.xmodel` / `cp .../ssd_adas_pruned_0_95.prototxt logs/ssd_adas_prototxt.txt` | 없음 | 88M |
| `ros2_ws/instruction.txt` | 745B | 2026-05-06 수동 실행 치트시트 | 참조 0. git 추적 → `git show HEAD:instruction.txt`로 복구. 모든 값이 이미 config YAML로 이관됨(오히려 divergent: `848x480x30` vs 라이브 depth `848x480x15`). **해저드**: 이름이 범용적이라 신입이 따라 실행하면 mock detector가 뜸 | 없음 | **~88M** |

**누적 회수: 약 88M / 242M (36%)** — 그중 33M이 손상 rosbag 하나, 13+5.5M이 realsense 오버레이 고아입니다.

---

## 4. 삭제하면 안 되는데 삭제 후보처럼 보이는 것 (⚠️ 검증에서 구조됨)

**이 섹션이 이 보고서에서 가장 중요합니다.** 아래는 1차 분류가 "삭제 가능"이라 판정했고 검증 단계가 **뒤집은** 항목입니다.

| 항목 | 왜 삭제로 보였나 | 검증 반박 근거 | 조치 |
|---|---|---|---|
| **`ros2_ws/CODEX.md`** (6.6K) | Codex는 `AGENTS.md`를 읽는데 `.codex/AGENTS.md`는 0바이트. 어떤 config도 이 파일을 로드하지 않음 → "0 참조" | **호출 방식이 기계가 아니라 사람입니다.** Codex 세션 `rollout-2026-05-06T14-19-16`(mtime 2026-06-22, 즉 3주 전 재개)에서 사용자가 직접 *"ros2_ws/CODEX.md 파일을 읽고…"* 라 지시했고 Codex가 49회 참조하며 *"Per `CODEX.md`, I'll answer in English"*, *"Per `CODEX.md`, SSD… is the safest first model choice"* 로 반응 — **역사적 SSD stand-in 결정을 이 파일이 이끌었습니다.** 또 `CODEX.md:117 "Answer in **English**"`는 `CLAUDE.md:156 "Answer in **Korean**"`과 **정면 충돌** → 정보 부분집합이 아님. 제안됐던 "`.codex/AGENTS.md`를 CLAUDE.md로 연결" 픽스는 Codex 출력 언어를 조용히 뒤집습니다 | **보존 + 내용 갱신.** pre-DPU 서술만 고치고 영어 지시는 유지 |
| **`ros2_ws/inst_claude.md`** (17K) | `integrated_progress.md`가 헤더에서 "통합한 원본 문서"로 명시 + 라우팅 맵이 엄밀한 부분집합 + 상태 텍스트가 거짓("RT 커널 패치 진행 중") | **`integrated_progress.md:637` §8.1이 새 세션 온보딩 절차 STEP 1으로 이 파일 읽기를 지정**하고, §2 맵에 라이브 역할 행이 있으며, `progress.md:2431`이 "New session onboarding entry point"라 명명 → 아카이브하면 **처방된 라이브 경로가 끊깁니다**(게다가 `archive/` 디렉터리가 아직 없음). 부실함은 오히려 **과소평가**됨(58행의 md5 `e2ca87c2`는 라이브 파일의 실제 `9bc6520c`와 모순) | **이동/삭제 금지. 제자리 갱신** — stale 5곳 수정 + RT 문서 클러스터 추가 |
| **`node_graph_pic/rosgraph.png`** (128K) | SVG 형제(419K, 11일 더 최신)의 저품질 중복으로 보임 | **두 이미지를 실제로 렌더해 보니 중복이 아닙니다.** PNG는 `/mock_detector` 시대 그래프(mock_detector → pick_logic → pick_target_3d, base 스테이지·TF 없음), SVG는 `/vitis_ai_detector_node` + `/pick_target_base_node` + `/base_to_camera_tf`. **pre-DPU mock 파이프라인의 유일한 시각 자료**이고 그 시대는 문서에 산문으로만 남아 있음 | **KEEP_ARCHIVE** (git 추적됨, blob 25ab66be) |
| **`test_images/objects/`, `objects2/`** (356K+372K) | plan:270의 "재개용 테스트 자산" 줄이 2026-07-07 일시중단 블록에 있고, 그 재개는 이미 일어나 superseded | 1차 분류가 **`plan:289`를 다루지 않았습니다** — `test_images/objects*/`를 **"▶ 다음 작업(2026-07-09 예정)" 블록의 보존 자산**으로 명시. 이건 supersession 사건(07-08) *이후*에, 그 결과를 이미 알던 저자가, 더 강한 단어(보존)로 쓴 줄입니다. 게다가 vitis_ai_work는 **git repo가 아니고**, 카메라는 D10 top-down 스테이션으로 이동했으며 peach는 드롭된 클래스 → **재촬영 불가한 물리 씬** | **KEEP_ARCHIVE** (합 728K) |
| **`test_images/test_set/test1.jpeg`** (9.7K) | "오버레이 출력이 저장된 적 없음 → 결과 없는 미문서 썸네일" | **결정적 증거가 틀렸습니다.** `_result.jpeg` 명명규칙 + test_images/ 내부만 찾은 탓 — 실제 출력은 **`/home/ubuntu/test1_overlay.jpg`**(25,850B, 입력 8분 후 생성)이고 열어보니 정확히 이 프레임의 SSD 오버레이입니다. 즉 살아있는 input/output 쌍의 입력 절반. 지우면 오버레이가 고아가 됨 | **KEEP_ARCHIVE** (`test2/test3`는 삭제 OK — 그쪽은 오버레이가 독립 보존) |
| **`models/decode_meta.OLD7.json`** (644B) | 액티브 meta와 byte-diff가 `peach@index1` + `num_classes 7` 뿐 → 정보 없음. 클래스 순서는 4개 문서에 기록 | **가치가 정보가 아니라 관계입니다.** `yolov3_tiny_7class.OLD7.xmodel`(md5 e2ca87c2)의 **유일한 기계판독 decode 키**이고, 4개 문서가 그 md5를 *"xmodel + decode_meta.json" 쌍*으로 못박아 인용합니다. `--meta`는 1급 인자(`yolov3_tiny_image_test.py:134`는 **required=True**)이므로 이 파일 없이는 OLD7 xmodel을 **실행 자체가 불가**. xmodel엔 클래스명이 없음(`strings` 0건). 손으로 재작성 시 **조용한 실패 모드**: peach는 index 1이지 끝에 붙이는 게 아님 | **KEEP_ARCHIVE.** 9.3M짜리 재생불가 바이너리를 남기면서 644B 키를 지우는 건 최악의 조합 |
| **`smartcam_models/models/refinedet_pruned_0_96`** (916K) | 보행자 검출 벤더 모델, 참조 0(prototxt 자기참조뿐), pick&place와 무관 | 사용 주장은 반박 실패 — 정말 미사용. 하지만 **일방향 문(one-way door)이 확인·강화**됐습니다: `xlnx-firmware-kv260-smartcam`은 모델을 안 실어 나름, apt에 app 패키지 없음, `/opt/xilinx` 부재, `find /` 결과 유일 사본, vitis_ai_work는 git 미추적. **md5 무결(90df5387)한 벤더 아티팩트를 916K 아끼자고 영구 삭제**하는 건 upside가 0. 형제 `ssd_adas_pruned_0_95`가 롤백 경로라 트리는 어차피 남습니다 | **KEEP_ARCHIVE** (densebox는 docker 이미지로 복구 가능해서 삭제 OK — 이 차이가 핵심) |
| **`test_images/topdown_nopeach/`** (328K) | 어떤 문서·스크립트·config도 **이름을 부르지 않음** (rg 0 hits) | **문서가 틀린 자산을 가리키고 있습니다.** 3개 문서(plan:289, inst_claude:78, memory:26)가 `test_images/topdown/`(5장)을 "보존 자산"이라 부르지만, Gate-5 합격 수치(apple 0.876/0.899/0.875)는 거기서 나오지 않았습니다 — plan:356이 "D13과 같은 프레임"이라 했고 **정확히 3개** 점수를 나열, `topdown_nopeach/`는 **정확히 3장**(2026-07-09 21:15, D13 결과일과 일치)이며, 두 obj_00.jpg를 직접 열어보니 동일 씬에서 peach만 물리적으로 제거된 상태. **문서를 따르는 정리 작업은 정확히 이 폴더를 지웁니다** | **KEEP (ACTIVE).** 문서에 이름을 추가하세요 |
| **`src/vitis_ai_detector_pkg/…/vitis_ai_worker.py`** (18K, SSD worker) | SSD는 superseded → 죽은 코드처럼 보임 | **코드 레벨 기본값입니다.** `vitis_ai_detector_node.py:91`이 이 파일을 `worker_script_path`의 **하드코딩 default**로 선언. 지우면 dangling default. **더 중요한 latent 함정**: yaml에서 `worker_script_path`를 빠뜨리거나 오타내면 YOLO xmodel에 대해 **SSD worker가 조용히 실행**됩니다(클린 에러가 아니라 오디코드 실패) | **KEEP.** 별도로 **기본값을 YOLO worker로 뒤집을 것을 권장** |
| **`src/target_3d_pkg/…/pick_post_stack.py`** (1.4K) | setup.py에 등록됐지만 **한 번도 launch되지 않음**(모든 런타임 히트가 방어적 `pkill`) | 측정된 **음성 결과의 실행 가능한 재현체**입니다: 36.5% vs 31.1%(병합 vs 분리), 원인 = rclpy SingleThreadedExecutor의 wait-set 재구성. `memory/perception-cpu-opt-phase1.md:23`이 "사유와 함께 보존"이라 명시하고 "rclpy에서 노드 병합으로 CPU 아끼려 하지 말 것"을 상시 규칙으로 일반화. 액티브 launch의 63-68행 NOTE가 이 경로를 참조 | **KEEP 제자리** (아카이브도 하지 말 것 — launch와 memory가 이 정확한 경로를 가리킴) |
| **`crash_logs/` 전체** (1.4M) | postmortem이 작성됐으니 원본 로그는 superseded로 보임 | **postmortem이 이 로그들을 *가리킵니다*.** `rt_kernel_postmortem.md:231`이 "크래시 원본 로그: ~/ros2_ws/crash_logs/ (BUG 17건 전수, SLUB oops, fpsimd 폭풍 전체)"라 약속하고, `rt_final.md §9-2`가 증거 표에서 **경로째** 인용. `rt_kv260_lazyoff_debug_boot_20260713.log`(253건 위반)는 **5개 문서가 인용**, `e2_poison_report_20260714-2336.log`는 zocl UAF의 유일한 스모킹건(upstream XRT 버그 리포트 첨부물). 전부 **git 미추적 → 복구 불가**, DEBUG 커널은 은퇴 | **KEEP / KEEP_ARCHIVE** |
| **`kernel_configs/vanilla-5.15.199-radix-fix/`** (76K) + **`zocl_patches/`** (8K) | 오래된 커널 잔해처럼 보임 | **바로 이게 하드윈 픽스의 유일한 재현 경로입니다.** 보드에 커널 소스 트리 없음(`/usr/src`에 headers만). 4개 문서가 경로째 인용. zocl 버그는 **upstream XRT master에 아직 미수정** → 향후 모든 커널/zocl 리빌드가 재적용 필요. 미추적 → 복구 불가 | **KEEP** |
| **`perf/` 루트의 loose CSV/launch.log** (78K) | `runs/phase2_final`로 복사된 스크래치처럼 보임 | **md5 불일치 — 다른, 더 나중 실행입니다.** `launch.log` 헤더 2026-07-15 01:31:31, 커널 `5.15.199-rt91-rt-kv260c`(프로덕션 RT). cpu_total **51.0% vs phase2_final 44.6% = +6.4%p** — `integrated_progress.md:22`가 요구한 "RT 위에서 baseline 재측정, RT 오버헤드 +5~10%p 예상"에 정확히 부합. **이 수치가 어떤 문서에도 안 적혀 있습니다** | **KEEP_ARCHIVE → `runs/rt_kv260c_baseline_20260715/`로 승격 + §4.2에 +6.4%p 기록.** ⚠️ 120s 실행이라 vision 섹션 누락(하네스 flush 임계 185s) — 부분 데이터임을 명시 |
| **`perf/run_gate6_perf_3core.sh`** (4.8K) | isolcpus가 현재 cmdline에 없음 → 레거시처럼 보임 | `integrated_progress.md:19`: EtherCAT의 **남은 유일한 전제조건 = "3+1 격리 코어 실측"**. `taskset -c 0-2` + core-3 프로브 핀 + affinity 상속 검증을 구현한 **유일한 스크립트**(isolcpus 없이 taskset으로 에뮬레이션 — 그래서 존재함). **다음에 돌릴 도구** | **KEEP.** ⚠️ `run_gate6_perf.sh`의 2026-07-14 orphan launch-wrapper preflight 클린업이 **아직 이식 안 됨** — EtherCAT 실측 전에 동기화 필요 |

---

## 5. 사용자 판단 필요 (USER_DECIDE)

| 항목 | 크기 | Trade-off | **제 권고** |
|---|---|---|---|
| **`github_personal_access_token.txt`** | 41B | 파일 삭제 ≠ 토큰 무효화. 토큰은 revoke할 때까지 GitHub에서 유효 | **① github.com/settings/tokens에서 revoke → ② `rm` → ③ `.gitignore`에 `*token*.txt`·`*.pat` 추가.** 계속 필요하면 `gh auth login`이나 git credential helper로. 워크스페이스 파일에 두는 건 절대 금지. 이 순서를 지키세요 — 파일만 지우면 가짜 해결입니다 |
| **`real_apple_frames/*.jpg` (68장) + `labels/`** | 7.6M | D14 apple 픽스(0.5→0.88)의 학습 데이터. 문서는 "~54장"이라 하지만 **실제 68장**(obj_00~68, obj_54 결번 = 문서화된 삭제와 일치). plan:349는 데스크톱 `datasets/real_apple_yolo/`로 rsync됐다고 하나 **보드에서 확인 불가**. 라벨은 **덮어써서 사라진 D13 모델**이 만든 것이라 재생성해도 다른 박스가 나옴 | **보존.** 이 프로젝트는 이미 D13 xmodel을 무백업 덮어쓰기로 잃었고(plan:357), 데스크톱은 D14 중 GPU 드라이버 행이 반복됐습니다(plan:350). 7.6M은 보험료로 공짜. **데스크톱 사본(`jaehyeon@jaehyeon-Raimlab:~/capstone_training/datasets/real_apple_yolo/`)을 눈으로 확인하기 전엔 손대지 마세요.** viz/(7.4M)만 지우면 이 폴더는 15M→7.6M |
| **`scripts/ssd_adas_oneshot_json.py`** | 16K | **프로덕션 config의 살아있는(주석 아닌) 줄이 가리킴** (`vitis_ai_detector.yaml:10 script_path:`) + 노드의 declare_parameter 기본값. 하지만 `detector_mode: "worker"`라 실행 안 됨. 게다가 그 폴백은 **이미 깨져 있음**(스크립트는 SSD 480x360/car-bicycle-person 하드코딩, model_path는 YOLO). 문서화된 SSD 롤백 경로도 아님(그건 in-package `vitis_ai_worker.py`) | **협응 정리로 삭제.** 단독 삭제 금지 — 4곳을 한 커밋에서: ① 스크립트 ② `src/…/vitis_ai_detector.yaml:10`의 `script_path:` ③ 빌드 사본 ④ 노드의 `declare_parameter("script_path")` + `detect_with_oneshot()` 분기. **죽은 코드보다 "살아 보이는 깨진 폴백"이 더 위험**합니다 |
| **`src/apu_rpu_bridge_pkg/`** | 40K | 코드 0바이트(`__init__.py` 0B, console_scripts 빈 리스트, package.xml에 TODO 그대로). `ros2 pkg create` 산출물 그 자체. 6개 문서가 이 이름을 계획으로 지목 | **삭제.** "src/에는 진짜 코드만"이 신입 탐색성 목표에 부합하고, git 추적(01da279)이라 복구 가능하며, RPU 트랙 개시 시 한 명령으로 재생성: `cd /home/ubuntu/ros2_ws/src && ros2 pkg create --build-type ament_python apu_rpu_bridge_pkg`. 문서들은 이미 "현재 placeholder, Phase 5에서 구현"이라 쓰고 있어 이름이 없어도 무의미해지지 않습니다. ⚠️ `install/apu_rpu_bridge_pkg`의 egg-link도 함께 제거 |
| **`crash_logs/e2_live_burst1.log`** | 260K | 도메인 최대 미인용 파일. 내용 3633줄이 **netconsole 계측 자체의 노이즈**(`__might_resched → macb_start_xmit → netpoll_send_udp`)이지 zocl 버그가 아님. 실제 발견은 `e2_poison_report`에 있고 그건 모든 문서가 인용 | **삭제.** 단, 미추적이라 되돌릴 수 없으니 — 23:36:46 포획 주변의 burst 타임라인 맥락이 필요했던 기억이 있으면 남기세요. 없으면 지우세요. (계측 노이즈에 260K는 아깝지 않지만, 판단 근거를 가진 건 이 세션을 실제로 돌린 사용자뿐입니다) |
| **`models/yolov3_tiny_7class.OLD7.xmodel`** | 8.9M | "peach 복귀 대비"로 보이지만 — **OLD7이 바로 peach에서 Gate 5를 떨어뜨린 모델**입니다(peach 0.02, apple로 오분류). 게다가 .xmodel은 컴파일된 INT8 프로그램이라 fine-tune도 재학습 seed도 불가. `09_drop_peach_remap.py`가 peach 데이터셋을 파괴적으로 일방 삭제해서 재생산도 불가 | **KEEP_ARCHIVE.** peach 복귀 논쟁은 무의미하니 사용자 판단으로 올릴 필요 없음. 남기는 유일한 이유는 **provenance** — D13(7→6 class) 결정을 정당화한 결정적 Gate-5 측정을 재시연할 수 있는 유일 아티팩트. 173G 여유 대비 8.9M(0.005%) |

---

## 6. 문서 정리 (별도 취급)

문서는 **삭제가 아니라 merge/archive/갱신**이 정답입니다. 이 문서군은 사실 잘 관리돼 있습니다 — `integrated_progress.md §2`와 `inst_claude.md §1`이 명시적 **"정본 라우팅" 표**를 게시하고, `memory/*.md`가 독립적으로 같은 타깃을 가리킵니다. 즉 정본성은 **추측이 아니라 선언**돼 있습니다.

### 6-1. 정본 (그대로 유지)

| 문서 | 정본 역할 | 대체 불가 이유 |
|---|---|---|
| `CLAUDE.md` (12K) | 시스템 전체 규칙·소통, **항상 우선** | 자동 로드되는 에이전트 계약. 인바운드 참조 8개(최다) |
| `integrated_progress.md` (71K) | 통합 허브 + **§2 정본 라우팅 표** | 지우면 ~15개 문서가 색인 없이 고아 |
| `progress.md` (85K) | 시간순 히스토리 | 각 단계의 측정값·root-cause 원본. 요약본들은 결론만 인용 |
| `workflow.md` (23K) | 파이프라인 파라미터 **근거** | **46개 rationale 행**(vision_final은 1행). 코드는 값만 보여주고 트레이드오프는 절대 안 보여줌 |
| `yolov3_tiny_execution_plan.md` (35K) | 비전 명령어·게이트 | Phase 실행 섹션 **127줄의 실제 명령**(vision_final 부록은 24줄). memory가 재개점으로 지목 |
| `yolo_v3_process.md` (22K) | 비전 의사결정 서사 | 부록의 5개 툴체인 함정(Arial.ttf 308, SUDO_USER, pip_constraints, Pillow 9.5.0)이 **여기에만** 존재 |
| `rt_patch.md` (68K) | RT 인수인계 정본 | **memory가 6회 라우팅(최다)**. DTB pre-copy, ARCH_ZYNQMP 게이트(off면 부팅 불가) 등 벽돌화 함정 |
| `rt_kernel_postmortem.md` (33K) | RT 크래시 종합 + **§12 = zocl 인수인계 정본** | §3 "오진의 기록"(5개 틀린 가설) = 희소한 음성 지식 |
| `rt_kernel_fix_plan.md` (29K) | 진단 상세 | `crash_logs/` 원본과 결론을 잇는 **유일한 다리** |
| `rpu_{guide_for_claude,plan,freertos_soem_execution_plan}.md` (53K) | RPU/EtherCAT 진입점·논리·게이트 | 미착수 트랙 → 전방 가치. RT 종결로 **지금 더 중요해짐** |
| `reprojection.md` (5.9K) | 3D 역투영 원리 | 액티브 노드의 알고리즘을 원리부터 설명하는 유일 문서. 미추적 |
| `site_md/reference_0{1..6}` (52K) | 리서치 룩업 | CLAUDE.md가 경로째 하드코딩 |

**RT 4종·비전 3종은 중복이 아닙니다.** `rt_final.md` 헤더가 나머지 셋을 *자기 정본 출처*로 명시하고, `vision_final.md`도 마찬가지입니다 — 이들은 요약본이지 대체재가 아닙니다.

### 6-2. ⚠️ OUT-OF-DATE — 적극적으로 오도 중 (최우선 수정)

| 문서 | 거짓 내용 | 왜 위험한가 |
|---|---|---|
| **`CLAUDE.md`** | L22/L139 detector = `ssd_adas_pruned_0_95` stand-in (실제 yolov3_tiny_7class 6-class D14) · L38/L52 worker = `vitis_ai_worker.py` (실제 `vitis_ai_worker_yolo.py`) · L138 FW 5.16.0.1 (실제 5.17.0.10) · L49 "7-class" (실제 6) + **존재하지 않는 `yolov3_tiny_plan.md` 링크** · L149 완료된 TODO 미체크 · RT 트랙 언급 0 | **자동 로드 + "항상 우선"** = 모든 새 세션이 틀린 모델·틀린 worker 파일로 추론을 시작합니다. **이번 작업 전체에서 최고 가치의 수정** |
| **`workflow.md`** | L7 "현재 모델: ssd_adas_pruned_0_95" · L114 model_path가 SSD xmodel · L29/51/86/116/127/286/292 전부 `vitis_ai_worker.py` · L344 "교체 예정"(이미 완료) | **"비전 변경 전 필독"으로 선언된 문서**가 SSD 시절 사실을 담고 있음 = 안내해야 할 독자를 정확히 오도. **46개 rationale 행은 한 줄도 건드리지 말고**, 모델/worker 식별자만 고칠 것 |
| **`inst_claude.md`** | §0 "현재 물리 상태: RT 커널 패치 진행 중"(종결됨) · "비전 config는 YOLO로 전환, 검증 미완"(Gate5 통과) · L58 md5 `e2ca87c2`(실제 `9bc6520c`) · §1 맵에 **RT 문서 0개**(integrated_progress는 14개) | 온보딩 STEP 1인데 신입이 RT 문서군 전체를 놓칩니다. **§4에서 구조됨 — 이동 말고 제자리 갱신** |
| **`progress.md`** | L3 "Last updated: 2026-06-22"(실제 mtime Jul 14) · L9 "Detector is an SSD ADAS stand-in" | 완화 요인: 문서가 스스로 "맨 아래 최신 날짜 섹션이 현재 상태"라 밝힘. **L3 날짜 + L9 한 문장만 고치면 끝** (저비용·저위험) |
| **`CODEX.md`** | 2026-05-06 고정: "USB RGB camera", "PL→APU: Vision accelerator (if used)" — DPU 활성화·SSD stand-in·YOLO 스왑·RT 트랙 전부 이전 | §4 참고: **사람이 수동 호출하는 라이브 문서**. 삭제 말고 갱신 |

### 6-3. 발견 불가 (라우팅 맵 누락 — 추가 필요)

| 문서 | 상태 |
|---|---|
| **`rt_final.md`** (30K) | 2026-07-15 01:56 작성 — `integrated_progress.md`의 마지막 편집(01:43)보다 **13분 늦어** §2 맵에 없음. 종결된 트랙의 종결 보고서 = 수명이 가장 긴 아티팩트. **맵을 따라 읽는 세션은 영원히 못 봅니다** |
| **`vision_final.md`** (44K) | 02:29 작성 — 동일 사유로 맵 누락. 비전 트랙의 유일한 단일 문서 서술 |
| **`decision_journey.md`** (36K) | 인바운드 참조 0 + 양쪽 맵 부재 + **git 미추적** → 실수로 지우기 가장 쉬움. 두 트랙을 관통하는 유일한 "무엇을 어떤 순서로 왜" 서사 (§1.1 mock-first 원칙 등). **커밋 + 맵 등재** |
| **`yolov8_vitisai35_feasibility.md`** (10K) | 참조 0, 맵 부재, 미추적. **가지 않은 길의 결정 기록** — "그냥 YOLOv8 쓰면 안 되나?"는 자연히 재발하는 질문이고(3.5는 SiLU→Hardswish 자동 매핑이라 순진한 답이 매력적), 답은 "보드 런타임이 2.5.0에 고정"이라는 벽. **`docs/analyses/`로 이관 + §2 등재** |

**요약 액션**: 삭제 0건. **갱신 4건**(CLAUDE.md ★, workflow.md, inst_claude.md, progress.md) + **라우팅 등재 4건**(rt_final, vision_final, decision_journey, yolov8_feasibility) + **커밋 2건**(decision_journey, reprojection — 둘 다 미추적).

---

## 7. 제안 구조 (개편안)

**원칙**: ① `src/`는 colcon이 기대하는 자리에서 절대 이동 금지 ② 진입점 하나(`README.md`, "이걸 실행하세요") ③ code / config / docs / evidence / archive 분리 ④ **도구와 싸우는 구조를 발명하지 않기** ⑤ 경로째 인용된 것은 이동 시 반드시 인용부도 함께 수정.

### Before → After: `ros2_ws`

```
BEFORE (루트에 .md 21개 + 스크립트 + 로그 + 캡처가 뒤섞임)
ros2_ws/
├── github_personal_access_token.txt   🔴 SECRET
├── CLAUDE.md  CODEX.md  instruction.txt
├── integrated_progress.md  progress.md  inst_claude.md  decision_journey.md
├── workflow.md  vision_final.md  yolo_v3_process.md  yolov3_tiny_execution_plan.md
├── rt_final.md  rt_patch.md  rt_kernel_postmortem.md  rt_kernel_fix_plan.md
├── rpu_plan.md  rpu_guide_for_claude.md  rpu_freertos_soem_execution_plan.md
├── reprojection.md  yolov8_vitisai35_feasibility.md
├── cyclic_rt.sh  soak_rt.sh  fastdds_shm_profile.xml
├── rt_verify/  zocl_patches/  kernel_configs/  crash_logs/
├── metrics/  node_graph_pic/  site_md/  yolo_v3_tiny_training/(미추적!)
├── .agents/(빈) .codex/(빈)
├── build/ install/ log/                (재생성 가능)
└── src/
    ├── build/ install/ log/            (git에 커밋된 오폭 잔해)
    ├── apu_rpu_bridge_pkg/ camera_source_pkg/ mock_detection_pkg/   (dead)
    ├── my_interfaces/ pick_logic_pkg/ target_3d_pkg/
    ├── vitis_ai_detector_pkg/ system_bringup_pkg/
    └── realsense-ros/                  (117M, third-party)

AFTER
ros2_ws/
├── README.md                    ★신규: "실행: ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py"
│                                        + 6-노드 그래프 + 빌드 명령 + docs/ 지도
├── CLAUDE.md                    (루트 유지 — 자동 로드 위치. 내용 갱신)
├── CODEX.md                     (루트 유지 — 사용자가 경로로 수동 호출)
├── fastdds_shm_profile.xml      ⚠️ 이동 금지 (.bashrc:124 하드코딩)
├── site_md/                     ⚠️ 이동 금지 (CLAUDE.md:104가 절대경로 하드코딩)
├── crash_logs/                  ⚠️ 이동 금지 (soak_rt.sh:20 + rt_final §9-2 + postmortem §11)
├── docs/
│   ├── STATUS.md                ← integrated_progress.md (허브·§2 맵. rt_final/vision_final 등재)
│   ├── onboarding.md            ← inst_claude.md (제자리 갱신 후 이관)
│   ├── history.md               ← progress.md
│   ├── decision_journey.md      (+ git 커밋)
│   ├── vision/                  ← workflow.md, yolov3_tiny_execution_plan.md,
│   │                              yolo_v3_process.md, vision_final.md, reprojection.md
│   ├── rt/                      ← rt_final.md, rt_patch.md, rt_kernel_postmortem.md,
│   │                              rt_kernel_fix_plan.md
│   ├── rpu/                     ← rpu_plan.md, rpu_guide_for_claude.md,
│   │                              rpu_freertos_soem_execution_plan.md
│   └── analyses/                ← yolov8_vitisai35_feasibility.md
├── tools/
│   ├── rt/                      ← cyclic_rt.sh, soak_rt.sh, rt_verify/
│   └── zocl_patches/            ← (apply_zocl_uaf_fix.py + README.md)
├── evidence/
│   ├── kernel_configs/          ← vanilla-5.15.199-radix-fix/ + 3개 config 백업
│   ├── metrics/                 ← vitis_ai_metrics{,1,2,3}.csv
│   └── node_graph/              ← rosgraph_vitis_ai.svg, rosgraph.png(mock 시대 유일본)
├── yolo_v3_tiny_training/       ★git 커밋 (현재 미추적 마스터)
└── src/                         ← 위치 불변 (colcon 규약)
    ├── my_interfaces/           (include/ src/ 빈 디렉터리 제거)
    ├── pick_logic_pkg/  target_3d_pkg/  vitis_ai_detector_pkg/  system_bringup_pkg/
    └── realsense-ros/
    ✗ build/ install/ log/       삭제 + git rm --cached + .gitignore
    ✗ camera_source_pkg/  mock_detection_pkg/  apu_rpu_bridge_pkg/
```

### Before → After: `vitis_ai_work`

```
BEFORE
vitis_ai_work/                          83M
├── dpu_info.txt  xdputil_query.json     (루트에 흩뿌려진 스펙 캡처)
├── arch/  models/(18M, OLD7 포함)  smartcam_models/(3M)
├── logs/                                (중복 SSD 덤프)
├── scripts/                             (라이브 5 + SSD 4 + pycache 섞임)
├── test_images/(9.7M)  real_apple_frames/(15M)  realsense_frames/(2.5M)
└── perf/(35M — 94%가 손상 rosbag 하나)

AFTER
vitis_ai_work/                          ~28M
├── README.md                    ★신규: 모델 실체(6-class D14) + arch_b3136 + 하네스 사용법
├── arch/arch_b3136.json         ⚠️ 이동 금지 (README/plan의 scp 명령이 경로 하드코딩)
├── models/
│   ├── yolov3_tiny_7class.xmodel + decode_meta.json   ⚠️ 반드시 같은 디렉터리 (worker가 sibling 탐색)
│   └── archive/                 ← OLD7.xmodel + decode_meta.OLD7.json (쌍으로)
├── smartcam_models/models/ssd_adas_pruned_0_95/       ⚠️ 이동 금지 (노드:83 절대경로 default)
│                     └── archive/refinedet_pruned_0_96/   (densebox는 삭제)
├── scripts/                     ← 라이브 5개만
│   └── archive/ssd_era/         ← ssd_adas_image_test.py, ssd_adas_repeat_infer_test.py,
│                                   vitis_ai_worker_direct_image_test.py
├── fixtures/
│   ├── topdown_nopeach/         ★진짜 회귀 fixture — 문서에 이름 추가
│   └── archive/                 ← topdown/, objects/, objects2/, test1.jpeg
├── datasets/real_apple_frames/  ← images + labels (viz/ 삭제)
├── perf/
│   ├── perf_probe.py summarize.py thread_sample.py run_gate6_perf{,_3core}.sh
│   ├── runs/{stock_fw51710_baseline,phase1_cpuopt,phase1_dpunbind,
│   │         phase2_w1,phase2_w2_fastdds_shm,phase2_final,3core_baseline,gate6_4core_15hz}/
│   │   └── rt_kv260c_baseline_20260715/   ★루트 loose 출력을 여기로 승격
│   └── analyses/                ← ANALYSIS_{30fps_bottleneck,gate6_perf}.json
│   ✗ bags/(33M)  threads.csv  threads_console.log  vitis_ai_detector.yaml.bak
└── docs/specs/                  ← dpu_info.txt, xdputil_query.json
    ✗ logs/  realsense_frames/  test_images/test_set/  *_result.jpeg
```

### 🚨 이동 시 반드시 함께 고쳐야 하는 하드코딩 경로

| # | 이동 | 깨지는 것 | 함께 할 수정 |
|---|---|---|---|
| 1 | `fastdds_shm_profile.xml` | `~/.bashrc:124 FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/ros2_ws/fastdds_shm_profile.xml` | **이동 비권장.** 옮기면 .bashrc 수정 + 새 셸에서 `env \| grep FASTRTPS` 확인 |
| 2 | `site_md/` | `CLAUDE.md:104`(디렉터리 절대경로) + `:112-117`(파일명 6개) | **이동 비권장.** 옮기면 CLAUDE.md 7줄 수정 |
| 3 | `crash_logs/` | `soak_rt.sh:20`(`LOG=$HOME/ros2_ws/crash_logs/…`, **`mkdir -p` 없음** → 즉시 실패) + `rt_final.md §9-2` + `rt_kernel_postmortem.md:231,235,279,284` + `rt_kernel_fix_plan.md` + memory 2개 | **이동 비권장.** 옮기면 soak_rt.sh + 4개 문서 + memory 수정 |
| 4 | `test_images/*` → `fixtures/` | `capture_color_frames.py:115` (`--out` default = `test_images/objects`) + `yolov3_tiny_image_test.py:9` usage 주석 + plan:270,289 + inst_claude:78 + memory:26 | 스크립트 2곳 + 문서 3곳 수정. **덤으로 `--out` default를 `runN` 경로로 바꿔** 기존 캡처 덮어쓰기 footgun 제거 |
| 5 | `models/` (decode_meta 분리) | `vitis_ai_worker_yolo.py:374` — meta를 **xmodel의 sibling으로 유도**. 분리 시 startup abort | **분리 절대 금지.** 항상 쌍으로 이동 |
| 6 | `smartcam_models/…/ssd_adas_pruned_0_95` | `vitis_ai_detector_node.py:83` 절대경로 default + `vitis_ai_detector.yaml:4-7` 롤백 주석 | **이동 비권장** |
| 7 | `arch/arch_b3136.json` | `yolo_v3_tiny_training/README.md:156`, `yolov3_tiny_execution_plan.md:70`의 scp 명령 + `13_compile_docker.sh:23`, `12a_inspect_docker.sh:25`가 부재 시 hard-fail | **이동 비권장** (41B) |
| 8 | `.md` → `docs/` | `integrated_progress.md §2` 라우팅표, `inst_claude.md §1`, `CLAUDE.md:47-49`, `memory/*.md`(rt_patch 6회 포함) — 대부분 **파일명만** 참조하지만 memory와 §2는 갱신 필요 | 라우팅표 2개 + `memory/*.md` 4개 갱신. **문서 이동의 유일한 비용이자, 어차피 §6에서 해야 할 일** |
| 9 | `src/` 패키지 이동/삭제 | `install/*/…egg-link` 5개가 `build/<pkg>` 지목 (editable install) | **재빌드 필수.** `camera_source_pkg`는 `install/camera_source_pkg`도 `rm -rf` (재빌드로 안 지워짐) |
| 10 | `yolo_v3_tiny_training/` | `README.md:13-14`가 **이미 stale**(`~/ros2_ws/training/` — 리네임 반영 안 됨). rsync 명령이 실패함 | 이동 여부와 무관하게 **지금 고치세요** |

**설계 판단**: `fastdds_shm_profile.xml` / `site_md/` / `crash_logs/` / `arch/` / `models/` / `smartcam_models/`는 **의도적으로 루트에 남깁니다.** 미관상 이득보다 하드코딩 경로 6곳을 건드리는 위험이 큽니다. `README.md`가 "이 파일들은 왜 여기 있는가"를 한 줄씩 설명하면 탐색성 목표는 충족됩니다.

---

## 8. 실행 순서 제안

각 단계 끝에서 파이프라인을 한 번씩 돌려보세요. 단계 간에 섞지 마세요 — 뭐가 깼는지 모릅니다.

**0단계 — 되돌릴 수 없는 것부터 보호 (삭제 전에)**
```bash
# ⚠️ main과 origin/main이 1:1로 diverge 중 — 먼저 화해시킬 것
cd /home/ubuntu/ros2_ws && git log --oneline --graph --all -10
git rev-list --left-right --count main...origin/main   # → "1 1" 확인

# vitis_ai_work는 git repo가 아님 → 삭제 전 스냅샷
tar czf /home/ubuntu/vitis_ai_work_pre_cleanup_20260715.tar.gz \
  --exclude=perf/bags -C /home/ubuntu vitis_ai_work
```

**1단계 — SECRET (최우선, 다른 무엇보다 먼저)**
```bash
# ① github.com/settings/tokens 에서 ghp_gOF6… REVOKE  ← 이게 진짜 픽스
# ② .gitignore 에 규칙 추가 (파일 삭제보다 먼저 — add . 사고 방지)
printf '\n# credentials\n*token*.txt\n*.pat\n' >> /home/ubuntu/ros2_ws/.gitignore
# ③ 그다음 삭제
rm /home/ubuntu/ros2_ws/github_personal_access_token.txt
git status --porcelain | grep -i token   # → 아무것도 안 나와야 함
```

**2단계 — 미추적 자산 커밋 (삭제 전에! 백업 0인 것들)**
```bash
cd /home/ubuntu/ros2_ws
git add yolo_v3_tiny_training/ decision_journey.md reprojection.md \
        rt_final.md vision_final.md   # 필요 시 나머지 .md도
git status --short | grep -i token    # → 반드시 비어 있어야 함
git commit -m "Track training pipeline and untracked docs before restructure"
```

**3단계 — REGENERABLE (되돌리기 가장 쉬움)**
```bash
find /home/ubuntu/ros2_ws /home/ubuntu/vitis_ai_work -name __pycache__ -type d -prune -exec rm -rf {} +

# src/ 오폭 잔해: git에 커밋돼 있으니 인덱스에서도 빼야 재발 안 함
cd /home/ubuntu/ros2_ws
cp -r log/build_2026-05-05_16-16-01 /home/ubuntu/realsense_build_failure_evidence   # ← 왜 realsense-ros를 포기했는지 유일 근거
git rm -r --cached src/build src/install src/log
printf 'src/build/\nsrc/install/\nsrc/log/\n' >> .gitignore
rm -rf src/build src/install src/log log/
git commit -m "Remove stray in-src colcon output committed on 2026-05-21"

rm -f /home/ubuntu/vitis_ai_work/perf/{threads.csv,threads_console.log,vitis_ai_detector.yaml.bak}
rm -rf /home/ubuntu/vitis_ai_work/logs
```

**4단계 — STALE (§3 표의 나머지)**
```bash
# 최대 회수 — 손상 rosbag (launch_bag.log는 남길 것)
rm -rf /home/ubuntu/vitis_ai_work/perf/bags

# realsense 오버레이 고아 (COLCON_IGNORE 때문에 재빌드로도 안 돌아옴)
rm -rf /home/ubuntu/ros2_ws/{build,install}/realsense2_camera{,_msgs}
ros2 pkg prefix realsense2_camera_msgs    # → /opt/ros/humble 기대

# SSD 시대 이미지 + spent QA 렌더
rm -rf /home/ubuntu/vitis_ai_work/real_apple_frames/viz
rm -rf /home/ubuntu/vitis_ai_work/test_images/{test_set,result_model,result_original}
#   ※ test_set/test1.jpeg 는 §4에서 구조됨 — 지우기 전에 빼두세요
rm -f  /home/ubuntu/vitis_ai_work/test_images/{car,bicycle,person,background}_result.jpeg
rm -rf /home/ubuntu/vitis_ai_work/realsense_frames
rm -rf /home/ubuntu/vitis_ai_work/smartcam_models/models/densebox_640_360
rm -f  /home/ubuntu/vitis_ai_work/scripts/ssd_adas_path_worker.py

# dead ROS 패키지 (mock은 한 단위로)
cd /home/ubuntu/ros2_ws
git rm -r src/camera_source_pkg src/mock_detection_pkg src/apu_rpu_bridge_pkg
git rm src/system_bringup_pkg/launch/pick_place_apu.launch.py instruction.txt
#   ★ 같은 커밋에서: src/system_bringup_pkg/package.xml:14 의
#     <exec_depend>mock_detection_pkg</exec_depend> 제거
#   ★ 덤으로 지금 고칠 것: <exec_depend>vitis_ai_detector_pkg</exec_depend> 가 빠져 있음
rm -rf install/{camera_source_pkg,mock_detection_pkg,apu_rpu_bridge_pkg}   # editable install은 재빌드로 안 지워짐
rmdir src/my_interfaces/include/my_interfaces src/my_interfaces/include src/my_interfaces/src
rmdir .agents .codex
git commit -m "Remove pre-DPU mock pipeline, legacy USB camera path, and RPU placeholder"
```

**5단계 — 문서 갱신 (삭제 0건, 최고 가치)**
- `CLAUDE.md`: L22/L38/L49/L52/L138/L139/L149 + dead link 제거 + rt_final/vision_final 추가
- `workflow.md`: 모델/worker 식별자만 (46개 rationale 행 보존)
- `inst_claude.md`: §0 상태 + L58 md5 + §1에 RT 문서 추가
- `progress.md`: L3 날짜 + L9 한 문장
- `integrated_progress.md §2`: rt_final / vision_final / decision_journey / yolov8_feasibility 등재
- `yolo_v3_tiny_training/README.md:13-14`: `~/ros2_ws/training/` → `~/ros2_ws/yolo_v3_tiny_training/`
- 비전 문서 3곳: 보존 자산을 `topdown/` → **`topdown_nopeach/`** 로 정정 (§4 최대 함정)
- `integrated_progress.md §4.2`: **RT baseline +6.4%p (51.0 vs 44.6) 기록** — 현재 어디에도 없음

**6단계 — 개편 + 경로 수정 (§7 표를 한 항목씩)**
한 번에 한 이동, 이동마다 대응 수정을 같은 커밋에. §7의 6·7번은 이동하지 않는 것을 권장합니다.

**7단계 — 재빌드 + 검증 (파이프라인이 여전히 도는지 증명)**
```bash
cd /home/ubuntu/ros2_ws
rm -rf build install                       # ★ build/와 install/은 반드시 함께 (egg-link 결합)
source /opt/ros/humble/setup.bash
colcon build --symlink-install             # ★ --symlink-install 필수 — 빼면 config 실시간 편집이 조용히 깨짐
source install/setup.bash

# 환경 확인
env | grep -E 'FASTRTPS|RMW'               # → fastdds_shm_profile.xml, rmw_fastrtps_cpp
ros2 pkg prefix realsense2_camera          # → /opt/ros/humble

# 파이프라인 기동
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py

# 다른 터미널에서 — 최종 출력까지 흐르는지 (이게 통과 기준)
ros2 topic hz /detections                  # → ~15 Hz
ros2 topic hz /pick_target_base            # → 최종 스테이지 살아있음
ros2 node list                             # → 6개 노드

# 정량 회귀 (선택, 3분) — phase2_final(44.6%) / RT baseline(51.0%)과 비교
bash /home/ubuntu/vitis_ai_work/perf/run_gate6_perf.sh
```

**8단계 — RT/DPU 안정성 재확인 (커널을 건드렸거나 zocl 근처를 만졌다면)**
```bash
bash /home/ubuntu/ros2_ws/rt_verify/sustained.sh   # 180s 연속 부하, 잔여 worker 0 기대
bash /home/ubuntu/ros2_ws/rt_verify/churn.sh       # 4 사이클 start/stop, Poison/Oops 0 기대
```