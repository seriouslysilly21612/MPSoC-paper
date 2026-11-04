# 파이프라인 성능 개선 — 남은 레버 전수 조사·검증 (2026-09-01)

"파이프라인 성능을 올리기 위해 남은 방법 + 새로운 방법"을 전수 조사한 스냅샷.
4방향 병렬 조사(문서 사다리 / 워커 파이프라이닝 실현성 / 노드 핫패스 / 신규 각도,
원시 아이디어 25건) → 병합·중복 제거 9건 → **후보마다 독립 검증자가 반박 시도**
(detector.csv 6,343프레임 재계산으로 산식 검산, 파일:라인 실사, 기각 이력 대조).

**정본 관계**: 사다리의 정본은 `docs/vision/throughput.md` §4다. 이 문서는 그
사다리의 검증 스냅샷 + **사다리에 없던 신규 레버 3건**(§3.5, §3.6, §3.8)의 추가
제안이다. 어떤 항목이든 착수·완료되면 throughput.md에 반영하고 이 문서는 갱신하지
말 것(스냅샷 고정).

**증보 이력**: §3 상세(현재 구조의 이유·구현 절차·메커니즘)와
§6(벤더 벤치마크 UG1354와의 괴리 분해)은 2026-09-02 추가분이다.

---

## 0. 기준 실측 (p50, `evidence/metrics/runs/shm_gate030_20260805`, 6,343프레임)

```
주기 45.47ms (21.7Hz) ≈ processing 43.01
processing 43.01 = img 1.27 (RGB→BGR) + worker_call 39.49 + publish 0.76
worker_call     = ipc 3.19 (shm+JSON+대기) + worker 35.26
worker 35.26    = pre 11.56 (letterbox+LUT) + dpu 17.35 + post 4.78 + 잔여 1.57
age_in p50 57.87ms · CPU 1.60코어 · 카메라 30fps 공급(33.3ms 격자) · DPU ~60% 유휴
```

주기 법칙: 현 레짐(gate 0.030 상시 통과)은 **주기 ≈ processing 1:1 추종 구간**이다
— processing 절감이 그대로 주기·신선도에 반영되고, 33.3ms 아래로 내려가는 순간
신선도↔속도 트레이드가 소멸하며 30Hz에 안착한다.

## 1. 요약 — 9건, 검증 보정 이득 순

| # | 방법 | 이득(보정) | 판정 | 공수 | 성격 |
|---|---|---|---|---|---|
| 1 | 워커 파이프라이닝 pre‖DPU (window-2) | **~11.8ms → 28~29Hz** | PLAUSIBLE | 중 | throughput+신선도 |
| 2 | 워커 프로세스 2개 (방법 B) | 12.2ms (1과 비가산) | PLAUSIBLE | 중상 | 1의 보험 트랙 |
| 3 | 카메라 60fps (레버 4) | 신선도 −8ms (rate 지금 0) | **CONFIRMED** | 소 | latency 전용 |
| 4 | pp_pipeline_accel PL 전처리 | 8ms (1과 비가산) | PLAUSIBLE | 대 | 장기 |
| 5 | 🆕 모델 416×416→416×256 재컴파일 | ~5.5ms | PLAUSIBLE | 중 | 1과 **가장 크게 가산** |
| 6 | 🆕 RGB→BGR→RGB 왕복 제거 | 1.4ms | **CONFIRMED** | **소** | 거의 공짜 |
| 7 | detector rclcpp 포팅 + composition | ~1ms (2.5에서 감액) | PLAUSIBLE | 대 | 최후순위 |
| 8 | 🆕 리사이즈를 shm 쓰기에 융합 | ~0.5ms | PLAUSIBLE | 중 | 1 착수 시 묶기 |
| 9 | NMS C++ 교체 (cv2.dnn.NMSBoxes) | ~0.5ms (1.5에서 감액) | PLAUSIBLE | 소 | 1 이후 마진용 |

🆕 = 이번 조사에서 새로 발굴 (throughput.md §4 사다리에 없던 것).

## 2. 권장 순서

```
전제. 실기 추적 검증 먼저 — 21.7Hz+skew≈0으로 실제 추종이 좋아졌는지 판정한 뒤,
      "부족하다"가 나와야 아래에 공수를 쓴다 (크리티컬 패스 원칙).
0. §3.6 RGB 왕복 제거    — 1.4ms 공짜, bit-identical, A/B 1회로 종결. 지금 해도 무방
1. §3.1 파이프라이닝     — 단, 코드 전에 go/no-go 게이트(§4) 필수
2. §3.3 60fps           — 파이프라이닝 후 (그때 30Hz 천장을 뚫는 유일한 길)
3. §3.5 416×256         — 파이프라이닝 후 DPU가 병목이 되면 (가장 큰 가산 카드)
장기. §3.4 pp_accel / §3.7 rclcpp — 위가 소진된 뒤
```

## 3. 후보 상세

### 3.1 워커 파이프라이닝 pre‖DPU — window-2 프로토콜 (보정 11.8ms, PLAUSIBLE)

**현재 구조와 그 이유**

워커의 `detect()`는 완전 직렬이다: `letterbox_lut`(worker_yolo.py:307-309) → `execute_async` 후 **즉시** `wait`(:312-313) → decode/NMS(:315-328). 이름은 async지만 submit 직후 wait하므로 사실상 동기 호출이며, 이 관행은 SSD 워커(vitis_ai_worker.py:291-293)에서 JSON contract와 함께 그대로 계승됐다(vision_final.md §8.1의 model-agnostic 교체 설계 — YOLO 워커는 의도적으로 SSD 워커의 골격을 복사했다). 노드 쪽도 마찬가지로 lock-step이다: `detect_with_worker`(node:665)가 send(:709-728) 후 `read_worker_json`의 블로킹 `readline`(:649)에서 응답을 기다리므로 outstanding 요청은 항상 1개다.

이것은 실수가 아니라 이유 있는 선택이었다. ① 워커를 별도 프로세스로 뺀 것 자체가 VART `execute_async` segfault(XIR 객체 GC → runner가 무효 참조, vision_final.md §8.2) 때문의 방어적 조치였고, 그 위의 IPC는 최대한 단순해야 했다 — 요청 1개 outstanding이면 응답이 어느 요청 것인지 ID 없이 자명하고, 타임아웃·워커 사망 시 상태가 명확해 `restart_worker`(:627) 복구가 간단하다. ② shm 핸드오프의 무락(lock-free) 설계도 이 위에서만 성립한다: "node writes only BEFORE sending a request; the worker reads only BETWEEN request and response"(shm_frame.py:11-13, 'strictly serial' 불변식). ③ 파이프라이닝이 아예 없는 것도 아니다 — `image_callback`/`worker_loop` 분리(node:310-343, §8.3)로 카메라↔검출은 이미 겹쳐 놓았고, 15Hz 시절엔 병목이 target_3d·ipc(13.4ms)여서 워커 **내부** 겹침은 우선순위가 아니었다. shm 적용(방법 0)으로 ipc가 3.19ms까지 내려간 지금에야 pre 11.56‖dpu 17.35 겹침이 최대 단일 레버로 부상했다(throughput.md §2단계 "방법 A(권장)"가 이 방향을 기록해 두었다).

**어떻게 바꾸나**

핵심: 워커 단독 수정으로는 안 된다 — lock-step이라 겹칠 N+1 프레임이 워커에 도착하지 않는다. 노드와 워커를 함께 재배열해야 한다.

1. **go/no-go 계측(§4)**: `execute_async` 전후 타임스탬프를 찍어 submit이 즉시(<1ms) 반환하고 시간이 `wait`에서 소비되는지 확인. submit이 블록하면 전체 기각.
2. **워커 재배열**(worker_yolo.py `main`:387~ + `detect`:299 분해): 스레드 없이 단일 스레드 순서만 바꾼다.
   ```
   현재: readline(N)   → pre(N)        → submit(N)+wait(N) → post(N) → respond(N)
   변경: readline(N+1) → pre(N+1, buf B) → wait(N) → submit(N+1) → post(N) → respond(N)
   ```
   `detect()`를 pre_submit / wait_post로 쪼개고, main 루프가 in-flight job 1개(job_id+프레임 메타)를 들고 순환한다. DPU(N)가 하드웨어에서 도는 동안 CPU가 pre(N+1)과 post(N)를 수행 — in-flight DPU job은 항상 1개, GIL 무관.
3. **input_buf 더블버퍼**(:269-270): (416,416,3) int8 = 519KB × 2. DPU가 buf A를 읽는 동안 pre는 buf B에 쓴다. `letterbox_lut`의 패딩 캐시(pad_key, :150-155 / `_letterbox_state` :271)는 버퍼별로 분리 — 각 버퍼가 최초 1회 패딩을 채워야 한다.
4. **노드 window-2**(`worker_loop`:322): send(N) → recv(N−1) → publish(N−1)로 재배열. 램프업 첫 루프는 send만, 종료·타임아웃 시엔 in-flight 1건을 flush(수신 후 폐기)하고 `restart_worker`.
5. **shm 경로 2개 ping-pong**: 램프업에서 send(0), send(1)이 연속되면 워커가 pre(0)을 하기 전에 노드가 슬롯을 프레임 1로 덮을 수 있다 — 'strictly serial' 불변식(:11-13)이 깨지므로 경로를 교대하고 주석도 갱신한다. reader는 `shm_path` 변경 시 자동 remap(:77-95)이라 워커 쪽 추가 코드는 거의 없다.
6. 응답에 seq 필드를 넣어 노드가 요청-응답 매칭을 검증 — lock-step이 주던 "자명함"을 명시적으로 회수.

**왜 빨라지나**

지금 worker 35.26ms는 pre 11.56 + dpu 17.35 + post 4.78 + 잔여 1.57의 직렬합이다. dpu 17.35ms 동안 CPU는 `wait`에서 잠들어 있고, pre 11.56ms 동안 DPU는 논다 — DPU ~60% 유휴 실측과 부합한다. DPU는 PL의 독립 연산기로 input_buf를 DMA로 읽어 자체 실행하므로, CPU의 letterbox/LUT(memcpy·cv2 연산)와 물리적으로 병행 가능하다. 겹치면 워커 1주기 = max(pre, dpu) + post: pre가 dpu의 그늘에 완전히 숨는다.

등가 processing ≈ img 1.27 + ipc 3.19 + **max(11.56, 17.35)** + post 4.78 + 잔여 1.57 + publish 0.76 ≈ **28.9ms** < 33.3ms. §2 주기 법칙에 따라 "주기≈processing 1:1 추종" 레짐이 끝나고 30fps 격자 대기 레짐으로 진입한다. 단 p50끼리의 합산은 낙관이다 — 프레임 단위 재계산은 25.4~30.1ms 분포이고 22~36%의 프레임이 33.3ms를 초과해 격자 한 칸을 놓친다 → 완전 30Hz 안착(이득 12.2ms)에는 못 미쳐 실효 **~28~29.7Hz**(주기 ~33.7~35.5ms), 보정 이득은 §1 정본 **11.8ms**. 프레임 개별의 worker_call은 +10ms(응답이 한 슬롯 늦게 도착)지만, 주기 절감과 fresh-grab(처리 시작 시점 프레임이 방금 도착한 것)이 이를 상쇄해 age_in 순손실 없이 이동체 샘플링이 +33~38% 는다.

기각한 변형: 워커 내 스레드 프리페치(이득 동일한데 GIL·락 리스크만 추가), 3단 submit-first(30Hz 안착 후 추가 이득 0).

**리스크와 검증 방법**

1. **최대 리스크 = VART 세만틱 미검증**: `execute_async`가 submit 시 입력을 확보(복사 또는 DMA 개시)하고 즉시 반환하는지 문서로 확인 안 됨. 이것이 §4 go/no-go — 전후 타임스탬프 측정이 구현 1단계인 이유.
2. submit이 입력을 복사하지 않고 실행 중 buf에서 직접 읽는다면 더블버퍼가 정합성의 전부다. 누락 시 프레임 N+1 픽셀로 N을 추론하는 **조용한 오염**(크래시 없음). 검증: 정지 장면에서 전/후 검출 좌표 비트동일 + 서로 다른 정적 이미지 2장을 교대로 넣어 seq별 응답이 올바른 장면과 일치하는지 확인.
3. VART 안정성 — 프로세스 분리의 원인이 바로 이 API의 segfault였다(vision_final.md §8.2, worker.py:229 주석). in-flight 1개 유지라 러너 증설(방법 B)보다 안전하지만, 장시간(≥1h) 스트레스에서 restart/timeout 카운터 0을 확인해야 한다.
4. 오류·재시작 시 window 꼬임: in-flight flush를 빠뜨리면 이후 모든 응답이 한 칸 밀린다(N 응답을 N+1로 해석). seq 불일치 → 즉시 restart로 방어. 순서 역전 자체는 단일 워커 + window-2라 구조적으로 불가(응답 순서=요청 순서)임을 seq로 상시 검증.
5. dpu_ms 17.35가 순수 하드웨어 점유인지(큐잉·DMA 포함 가능) 불확실 — 겹침 실이득은 실측으로만 확정된다(판정이 PLAUSIBLE인 이유).
6. 판정 절차: `tools/metrics/ab_one_run.sh` 전/후 각 1회, 같은 장면 back-to-back(§1단계에서 확인된 장면 오염 함정 회피). 합격선: throughput ≥28Hz, age_in p50 비악화, 검출 수 동일, restart/timeout/slow 0.

### 3.2 워커 프로세스 2개 — 방법 B (12.2ms, PLAUSIBLE, A 실패 시에만)

**현재 구조와 그 이유**

DPU 추론은 노드 안이 아니라 별도 프로세스(`vitis_ai_worker_yolo.py`) 하나가 전담한다. 이것은 실수가 아니라 사고 수습의 결과다: VART `execute_async()`를 rclpy 노드 in-process에서 부르면 XIR graph/subgraph Python 객체가 GC되는 순간 runner가 무효 메모리를 참조해 segfault/bus error가 났고(`docs/vision/vision_final.md` §8.2), 해법이 "별도 worker 프로세스 + graph 객체를 runner 수명 동안 `self`에 보관"이었다(워커 L248-249 주석에 그대로 남아 있다). 워커가 **정확히 1개**인 것도 같은 보수성의 연장이다 — `docs/onboarding.md` L97은 "DPU는 한 번에 한 프로세스만. 파이프라인(worker)이 떠 있으면 `yolov3_tiny_image_test.py`가 충돌한다"는 운영 실증을 명시한다. 그래서 노드는 단일 `worker_process` 핸들 하나(L568 `Popen`)와 단일 `worker_loop` 스레드(L322)로, 프레임 한 장을 보내고 답을 받을 때까지 블로킹하는 완전 직렬 설계다. 이 직렬성 때문에 DPU가 도는 17.35ms 동안 CPU 구간(pre/post)이 놀고, CPU가 도는 동안 DPU가 논다(DPU 유휴 ~60%).

**어떻게 바꾸나**

3.1(방법 A: 한 워커 안 파이프라이닝)과 같은 천장(45.47→33.3ms)을 노리는 **비가산 대체재**다. A가 go/no-go에 실패했을 때만 착수한다.

1. **go/no-go 선행 시험**: 파이프라인 없이 최소 스크립트로 VART runner 2개를 **서로 다른 프로세스**에서 동시에 열고 각자 수백 회 추론. onboarding의 "1 프로세스" 실증과 정면 충돌하는 미검증 가정이므로 이게 실패하면 여기서 끝.
2. `start_worker()`(L547)를 워커 2개 기동으로 확장. **shm 경로 주의**: 현재 경로가 `/dev/shm/vitis_ai_frame_{os.getpid()}`(L167)로 **노드 PID** 기반이라 두 워커가 같은 파일을 읽게 된다 — `_a`/`_b` 접미사로 `ShmFrameWriter` 2개 분리.
3. `worker_loop`를 디스패치 구조로: 스레드 2개가 각자 워커 하나를 소유하고 `latest_msg` 슬롯에서 프레임을 짝/홀(사실상 "빈 워커가 집어감")로 가져간다. GIL은 문제 없다 — 무거운 일은 워커 프로세스에 있고 노드 쪽은 대부분 I/O 대기.
4. **하류 단조 가드 신설**(`throughput.md` L148-149): 발행 직전 `out_msg.header.stamp`(L361, color 프레임 촬영 시각)를 마지막 발행 stamp와 비교해 더 오래된 결과는 버린다. 소비자(`pick_target_3d_node`)마다 고치지 말고 발행 지점 한 곳에 넣는다.

**왜 빨라지나**

현 주기 45.47ms의 처리 43.01ms는 CPU 구간(img 1.27 + pre 11.56 + post 4.78 + ipc 3.19 + publish 0.76 ≈ 21.6ms)과 DPU 17.35ms가 **한 타임라인에서 교대**한 합이다. 프로세스가 2개면 워커 A의 프레임이 DPU에 들어가 있는 17.35ms 동안 워커 B가 다른 코어에서 다음 프레임의 letterbox/decode를 돌린다(보드는 CPU3 격리 후에도 3코어, 현 사용량 1.60코어라 코어 여유는 있다). 프레임당 자원 점유가 병렬화되면 처리시간이 33.3ms(카메라 30fps 격자) 아래로 내려가고, §1의 주기 법칙대로 주기는 processing 추종을 멈추고 **격자에 안착**한다: 이득 = 45.47 − 33.3 = **12.2ms**, 부수 효과로 신선도↔속도 트레이드가 소멸. 단 이 산식은 `dpu_ms` 17.35가 순수 DPU 점유라는 가정 위에 있다 — 측정점이 `execute_async`→`wait` 벽시계(워커 L312-313)라 XRT 큐잉·DMA가 섞여 있고, 큐잉 몫이 크면 두 워커가 그 큐에서 다시 직렬화되어 이득이 줄어든다. 그래서 PLAUSIBLE.

**리스크와 검증 방법**

- **VART 2-프로세스 동시 오픈 미검증**: 위 go/no-go 시험이 1차 관문. 프로세스 분리의 기원이 segfault였음을 기억할 것 — 러너를 늘리는 건 그 지뢰밭 재진입이다(`throughput.md` L146-147).
- **zocl KDS 경로 스트레스**: 이 보드 커널의 zocl은 submit-후-타임스탬프 UAF를 손으로 순서교체한 패치본이다(`docs/STATUS.md` §5.2). 멀티프로세스 CU 중재는 정확히 그 `kds_core.c` 경로를 더 세게 두드린다. 과거 검증 프로토콜 재사용: 330s+ 스트레스 중 `dmesg`에서 Poison/Oops 0 확인.
- **순서 역전**: 단조 가드의 드롭 카운트를 metrics CSV에 추가해 역전 빈도를 정량화. 드롭이 수 %를 넘으면 구조 재고.
- **메모리 +136MB**: 워커 RSS 실측 ~136-137MB flat — 워커 하나 몫이 그대로 추가된다.
- **판정 A/B**: `bash tools/metrics/ab_one_run.sh` 2회(단일 런 비교는 장면 오염). 합격선 = `detect_period_ms` p50 → 33.3ms, age_in 개선, worker restart/timeout 0. 동시에 워커별 `dpu_ms` 분포를 보라 — 경합 하에서 p50이 17.35보다 뚜렷이 뜨면 큐잉 지배, 즉 이 방법의 이득 자체가 축소된다는 신호다.

### 3.3 카메라 60fps — 레버 4 (신선도 −8ms, CONFIRMED)

`realsense_pick_place.yaml`의 profile 2줄(L54–55, 실코드 재확인 완료)을 `848x480x30` → `848x480x60`으로 바꾸는 설정 레버다. 지금 레짐에서는 **throughput 0, 순수 신선도 레버**(−8.3ms)이고, 3.1/3.5로 processing이 33.3ms 아래로 내려간 뒤에는 **30Hz 천장을 뚫는 유일한 길**이 된다.

**현재 구조와 그 이유**

30fps는 실수가 아니라 세 겹의 보수적 선택이 쌓인 결과다. 이력이 전부 yaml 주석과 `docs/history.md`에 남아 있다.

1. **USB/카메라 안정성 전과**. 1280x720에서 realsense2_camera 노드가 ~70초 만에 100% busy-spin으로 조용히 멈추는 사고(`history.md` L1908, "silent stall")로 해상도를 848x480으로 내렸고, FW 5.16.0.1의 RGB 웨지(dmesg `GET_CUR ... -32`, 스트리밍 수십 초 후 color만 정지)는 FW 5.17.0.10 업데이트 + `initial_reset: true`(yaml L1–5)로 방어했다. 이 보드에서 카메라 대역을 올리는 일은 늘 사고 이력과 함께였다.
2. **CPU 예산**. 카메라 노드는 단일 스레드로 한때 0.91코어(align 시절)를 먹던 최대 소비자였고, 총 CPU ≤60%는 EtherCAT 마진 게이트다. 공급을 올릴 이유가 없는데 비용만 확실했다.
3. **정지 물체 전제**. depth는 2026-07-14에 15fps로 내렸다가("정지 물체면 staleness 33→66ms 무해", yaml L46–53) 08-04 이동 물체 피벗으로 30fps+`enable_sync`로 복귀했다. color 30fps도 같은 전제 위의 선택이다 — 그리고 그 전제가 실시간 추적('o')으로 다시 바뀌었다.

게다가 지금까지 카메라는 병목인 적이 없었다(공급 29.4Hz vs 소비 15~21.7Hz, `throughput.md` §3). 올릴 동기 자체가 없었던 것이다.

**어떻게 바꾸나**

1. **지원 확인 선행**: `rs-enumerate-devices`로 이 개체·FW에서 color 848x480 @60 지원을 확인한다(D435i 스펙상 존재하지만 실기 확인이 먼저다).
2. yaml L54–55 두 줄을 모두 60으로: `depth_module.depth_profile: 848x480x60`, `rgb_camera.color_profile: 848x480x60`. **depth도 반드시 함께** — `enable_sync`(yaml L8–24)는 같은 timestamp 짝을 만들 때 depth가 color만큼 자주 생산되어야 한다. depth를 30에 두면 color 프레임 절반은 짝이 없어 nearest 선택으로 skew ≤16.7ms가 남는다(현재는 89%가 정확히 0.0).
3. **하류 코드 수정 불필요** — 이미 rate 변화를 흡수하도록 설계돼 있다. detector의 `image_callback`(vitis_ai_detector_node.py L311)은 최신 프레임 덮어쓰기라 도착률 배증을 `frames_overwritten` 증가로만 흡수하고, target_3d의 `depth_buf`는 프레임 수가 아니라 **나이(800ms 창)로 prune**한다(pick_target_3d_node.py L246–253 주석: "fps는 config 선택이라 fixed maxlen은 rate가 배가되면 커버 시간을 반토막 낸다" — 정확히 이 변경을 내다본 설계).
4. 게이트 주의: `process_period_sec: 0.030`은 지금은 상시 통과(processing 43.01 > 30)라 무관하지만, 훗날 processing < 30ms가 되면 16.7ms 격자에서 주기를 33.3ms로 올림해 **30Hz에 잠근다**. 30Hz를 넘기려면 그때 게이트도 16.7ms 아래로 내려야 한다.
5. 300s×2 back-to-back A/B(`tools/metrics/ab_one_run.sh`)로 판정.

**왜 빨라지나**

핵심 메커니즘은 **latest_msg 묵힘의 기대값**이다. 워커가 프레임을 집는 시점(직전 처리가 끝나는 순간)은 카메라 도착 격자와 무관하게 정해진다. 서로 독립인 두 주기 프로세스의 위상차는 균일분포이므로, 집는 순간 `latest_msg`에 앉아 있는 프레임의 나이는 **U(0, 격자)** — 기대값은 격자/2다.

```
30fps: E[묵힘] = 33.3/2 = 16.7ms
60fps: E[묵힘] = 16.7/2 =  8.3ms   → Δ = −8.3ms
```

격자를 반으로 줄이면 기대 묵힘이 정확히 반이 된다. 이 −8.3ms는 age_in(p50 57.87ms)에서 빠져 ~49.5ms가 되어야 하고, E2E·신선도 p50에 그대로 전사된다.

- **throughput이 0인 이유**: 주기 법칙 `max(공급 16.7, processing 43.01, gate 30)`에서 구속조건은 여전히 processing이다. 현 레짐은 주기≈processing 1:1 추종 구간(21.7Hz)이라 공급을 2배→4배로 늘려도 rate는 1Hz도 안 움직인다.
- **미계상 보너스 (−15ms까지)**: 센서 쪽 exposure→readout→USB 전송 체인도 프레임 주기에 비례해 짧아진다(`docs/STATUS.md` 레버 표의 "latency −15ms(readout 반감)"이 이것). 다만 age_in 측정에는 두 메커니즘이 겹쳐 들어와 독립 합산하면 이중 계상 위험이 있어 **정본 수치에서는 −8.3ms만 계상**한다. 실측이 −8보다 크게 나오면 이 몫이다.
- **미래 가치**: 파이프라이닝/416×256으로 processing < 33.3ms가 되는 순간 워커는 항상 다음 도착을 기다리는 쪽으로 레짐이 바뀌고(§주기 법칙), 그때 천장은 카메라 격자 그 자체다. 그 천장을 30→60Hz로 올릴 수 있는 레버는 이것 하나뿐이다.

**리스크와 검증 방법**

- **USB 대역**: 61 → ~98MB/s. 이 보드는 STALL(-32) UVC 웨지·FW 5.16 RGB 웨지·1280x720 busy-spin 전력이 있다. 전부 해결·방어됐고 과거 웨지가 대역폭 원인이라는 증거는 없지만(`sync30_compare_20260804.md`), 61MB/s에서의 누적 무사고 근거도 ~10분뿐이라 98MB/s는 미답 영역이다. → **300s×2 소크 필수**, dmesg `-32` 감시, `depth_valid_rate 1.0`·worker restart/timeout 0 확인.
- **CPU**: realsense 노드 +10pt± 예상. target_3d는 depth 콜백이 다시 배증 — 15→30 때 실측 +0.088코어였으니 30→60도 비슷한 규모를 예산에 넣어야 한다(콜백 자체는 store-only지만 DDS 역직렬화 814KB×60Hz가 비용). 현재 총 1.60코어(40%)라 +0.2~0.3코어를 얹어도 EtherCAT 게이트(≤60%) 안이지만 마진이 얇아진다. → per-process CPU 재측정으로 게이트 판정.
- **sync 페어링 열화**: depth를 60으로 못 올리는 상황(대역/CPU)이면 color만 60으로 가되, skew가 "89% 정확히 0" → "절반은 ≤16.7ms"로 후퇴함을 받아들이는 결정이 필요하다. skew 분포를 반드시 재측정해 0.0 비율을 확인할 것.
- **판정 기준**: 같은 장면 back-to-back A/B에서 ① age_in p50 −8ms 안팎 하락, ② throughput 불변(21.7Hz — 변하면 다른 것이 움직인 것), ③ skew 0.0 비율 유지, ④ CPU 합 ≤60%. 실패 시 롤백은 yaml 2줄 원복으로 끝난다.


### 3.4 pp_pipeline_accel PL 전처리 오프로드 — 4단계 (8ms, PLAUSIBLE, 장기)

**현재 구조와 그 이유** — 이 보드의 PL(FPGA fabric)에는 CU(Compute Unit)가 2개 실려 있다. `xdputil query`가 "DPU Core Count: 2"라고 보고하지만 실제 DPU는 cu_idx 0(`DPUCZDX8G_1`, B3136) 하나뿐이고, cu_idx 1이 `pp_pipeline_accel` — DPU가 아닌 전처리 가속 IP다(`DPU Arch` 비어 있음, throughput.md L67-70 · vision_final.md L54). 이게 실려 있는 이유는 단순하다: 우리는 DPU를 얻으려고 Xilinx 공식 앱 오버레이 `kv260-smartcam`을 `xmutil loadapp`으로 통째로 로드했고(history.md), smartcam의 원래 용도인 VVAS/GStreamer 카메라 파이프라인이 쓰는 pp IP가 같은 xclbin에 딸려 온 것이다. 우리 파이프라인은 이 IP를 한 번도 구동한 적이 없다 — `xdputil`은 이 CU에 `fingerprint 0x0` 경고까지 띄운다(history.md L662).

전처리를 CPU에 둔 것도 당시엔 합리적 선택이었다. `pre`는 순수 numpy 42ms에서 출발해 LUT화(§9.1(1), 42→12ms), cv2 경로+pad 캐시(P3, 17.5→9.0ms)로 **두 차례에 걸쳐 bit-identical 검증을 통과하며** 깎아 내린 역사가 있고(vision_final.md), 그 결과가 지금의 `letterbox_lut()`(vitis_ai_worker_yolo.py:125 — letterbox+BGR→RGB+`cv2.LUT` 한 패스, `input_buf`에 직기록)이다. 반면 pp IP를 쓰려면 Petalinux 전제인 VVAS/xfOpenCV 스택을 Ubuntu에 새로 붙여야 했으니, CPU에 여유가 있던 시절엔 손댈 이유가 없었다. 참고로 throughput.md §4 4단계의 "pre_ms 8.9"는 옛 15.3Hz 스냅샷 수치다 — 본 조사의 정본은 **pre 11.56ms**(§1).

**어떻게 바꾸나** — 4단계, 앞 단계가 go/no-go 게이트다.

1. **IP 규명(코드 0줄)**: `xclbinutil --info`로 xclbin에서 `pp_pipeline_accel`의 커널 시그니처·레지스터 맵을 뽑아, 이 IP가 정확히 무엇을 하는지(통상 smartcam pp = resize+mean/scale+CSC) 확인. fingerprint 0x0이 "DPU가 아니라서 뜨는 무해한 메타데이터 경고"인지 "동작 불능"인지 판별.
2. **최소 구동 테스트**: XRT(pyxrt)로 CU를 열고 고정 이미지 1장을 DMA in→kernel→DMA out. **저장소에 이 CU를 구동한 실측이 0건**이므로 여기가 진짜 관문이다. 출력이 나오면 CPU `letterbox_lut` 결과와 diff.
3. **기능 매핑**: 우리 전처리의 3요소 — 비대칭 letterbox 패딩(416×235 콘텐츠+회색), BGR 채널 순서, `v/255×2^fix_point` int8 양자화(`build_input_lut`, :118) — 를 IP 파라미터(scale/mean/출력 포맷)로 표현 가능한지 확인. 패딩 미지원이면 pad는 CPU 캐시(현행 유지)로 두고 콘텐츠 영역만 IP 출력을 받는 절충.
4. **워커 통합**: `YoloWorker.load_model()`에서 XRT BO(buffer object) 할당, `detect()`(:299)의 `letterbox_lut` 호출을 pp 호출+wait로 교체. `input_buf`를 BO와 zero-copy로 묶으면 memcpy 1회도 절약.

**왜 빨라지나** — pre 11.56ms의 정체는 A53 코어 1개가 848×480×3=1.22MB를 읽어 bilinear resize(픽셀당 곱셈-누산 수 회)+채널 재배열+표 조회를 하고 519KB를 쓰는 **픽셀 순회**다. A53은 DDR 대역이 좁고 NEON 폭이 얕아 이 순회가 ms 단위로 나온다. PL에서는 같은 일이 다르게 처리된다: HLS 커널은 line buffer를 두고 픽셀 스트림을 **클럭당 1픽셀(II=1) 파이프라인**으로 통과시키므로, 848×480 ≈ 40.7만 픽셀은 ~300MHz에서 **~1.4ms**, 여기에 DMA 왕복(1.22MB in+0.5MB out, DDR 수 GB/s → sub-ms)을 더해도 'HW 호출+DMA ≤3.5ms' 가정이 선다. CPU가 하는 일은 DMA descriptor 세팅뿐이다. 절감 = 11.56−3.5 ≈ **8ms**: 단독 적용 시 processing 43.01→~35ms(~26-28Hz)로 **30Hz엔 미달**하고, 3.1 파이프라이닝 후엔 pre가 이미 DPU 17.35ms 뒤에 숨으므로 throughput 이득 ~0, **레이턴시 ~8ms만 남는 비가산 카드**다(§1 표와 동일). 장기 순위가 낮은 이유가 이것이다.

**리스크와 검증 방법** —
- **동작 여부 자체가 미검증**(구동 실측 0건 + fingerprint 0x0). 2단계에서 죽으면 전체 폐기 — 그래서 통합 코드를 쓰기 전에 단독 테스트가 먼저다.
- **기능 비대칭**: smartcam pp는 VVAS/NV12 경로용이라 비대칭 letterbox+BGR+`2^fix_point` 양자화 지원이 불명. 미지원 항목마다 CPU 보정 패스가 붙어 8ms가 잠식된다 — 3단계에서 잔여 CPU 몫을 먼저 계산해 이득이 5ms 밑이면 중단.
- **통합 난이도 최고**: VVAS/xfOpenCV의 Ubuntu 신규 통합. 9건 중 유일한 '공수 대'.
- **정확도 검증 원점 재시작**: HW의 fixed-point bilinear는 `cv2.INTER_LINEAR`와 라운딩이 달라 기존 bit-identical 관행이 깨질 수 있다 → 고정 이미지 세트 max-abs-diff 허용치 검증 + Gate cosine 재실행으로 대체하고, **D7 calibration 이미지도 같은 경로로 재정합**해야 한다.
- **DPU와 AXI/DDR 대역 공유**: pp 구동이 `dpu_ms`를 밀어 올릴 수 있다 → A/B에서 detector.csv의 pre/dpu/processing p50을 각각 비교(pre만 줄고 dpu 불변이어야 합격).

### 3.5 🆕 모델 입력 416×416 → 416×256 직사각 재컴파일 (보정 5.5ms, PLAUSIBLE)

**현재 구조와 그 이유**

카메라는 848×480(16:9)을 내보내는데 모델 입력은 416×416 정사각이다. letterbox는 종횡비를 보존하는 축소라 스케일 비율이 `r = min(416/480, 416/848) = 0.4906`으로 결정되고, 실제 콘텐츠는 416×235 영역에만 들어간다. 나머지 세로 181행(416행의 43.5%)은 회색 상수(`PAD_VALUE = 114`, worker L50 — yolov5 letterbox 기본값)로 채워지며, DPU는 매 프레임 이 정보량 0인 픽셀에도 콘텐츠와 똑같은 convolution을 수행한다.

정사각이 "실수"였던 건 아니다. ① ultralytics YOLO 학습 관행이 정사각 `imgsz`이고 COCO pretrained 가중치도 그 기준이며, ② D7 원칙(train/calib/worker 세 곳 전처리 통일, `vision_final.md` §5.2)의 최우선 목표가 **분포 정합**이었지 종횡비 최적화가 아니었다. ③ 당시 병목은 pre 42ms(LUT 도입 전, §9.1)였고 dpu가 아니었다. ④ worker↔node의 model-agnostic JSON contract(§8.1)에서 meta의 `input_size`를 스칼라 하나로 두는 게 가장 단순했다. 합리적 우선순위의 산물이고, dpu 17.35ms가 worker 최대 항목이 된 지금에서야 재검토 가치가 생긴 것이다.

**어떻게 바꾸나**

핵심: 코드의 decode 경로는 이미 직사각을 소화한다. `decode_head`는 `_, H, W, C = raw.shape`로 출력 텐서에서 H/W를 따로 읽고(worker L175, L181) grid 좌표 `xs`/`ys`에 각각 곱하므로 무수정. 노드 쪽도 ready handshake에서 `input_shape[1]`/`[2]`를 height/width로 분리 저장하고 있어(node L594-595) **노드는 수정 불요** — 브리프의 "노드측 handshake도 H/W 분리"는 "이미 되어 있음"이 코드상 사실이다. 정사각 가정은 worker 4곳뿐이다.

1. **사전 게이트(먼저)**: Vitis-AI Inspector로 직사각 매핑 검증 — Gate 0 플로우(`12a_inspect_docker.sh`, vision_final.md L272-277) 그대로 `inspector.inspect(model, torch.randn([1, 3, 256, 416]))`. 통과 못 하면 여기서 중단(비용 수 분).
2. **재양자화·재컴파일**: 문서화된 플로우(vision_final.md 부록, Gate 3-4) 재수행. 단 D7대로 **calibration 500장도 416×256 letterbox로** 다시 — `yolov3_tiny_quant.py --quant_mode calib --subset_len 500` → `vai_c_xir -x ... -a arch_b3136.json`. 모델은 FCN이라 재학습 불요(anchor는 픽셀 단위고 `r`이 동일하므로 그대로 유효).
3. **worker 코드** (`vitis_ai_worker_yolo.py`):
   - meta `"input_size": 416` → `[256, 416]` 허용(L225에서 스칼라/리스트 분기하면 구 xmodel과 하위호환).
   - `letterbox_lut(image, input_size, ...)`(L125) → `(in_h, in_w)` 튜플: `r = min(in_h/h, in_w/w)`(L136), `top = (in_h-nh)//2; left = (in_w-nw)//2`(L147-148).
   - `load_model`: 입력 검사 L262-266을 `shape[1] != in_h or shape[2] != in_w`로, **브리프에 없던 2곳 추가** — `input_buf` 할당 `(1, in_h, in_w, 3)`(L270), stride 도출 `stride = in_h // shape[1]`(L278, `in_w // shape[2]`와 일치 검증 추가).
4. **모델 교체**: xmodel+meta 동시 교체. L262 검사가 fail-fast 가드라 짝이 안 맞으면 기동 시 RuntimeError.

**왜 빨라지나**

conv 한 층의 연산량은 `H_out × W_out × C_out × C_in × k²` — 출력 spatial 크기에 **선형 비례**한다. 입력 세로를 416→256으로 줄이면 모든 층의 H가 256/416 = 0.615배가 되고 W는 불변이므로 네트워크 전체 MAC이 ~38% 준다. DPU(B3136, 3136 MAC/cycle)는 층별 tile을 순회하며 계산하므로 compute-bound 층의 실행 시간도 거의 그 비율로 준다: dpu 17.35 × 0.615 ≈ 10.7ms. 단 VART `execute_async`/`wait` 호출, interrupt, DMA 셋업, memory-bound 층 등 고정분 ~1-2ms는 안 줄어 **실효 −5.5ms**(dpu ≈ 11.9ms)로 보정한다. 핵심 메커니즘은 "패딩 픽셀은 정보 0인데 연산 비용은 콘텐츠와 동일"이라는 것 — 회색 상수 181행을 convolution이 알아서 건너뛸 방법은 없고, 입력에서 제거하는 수밖에 없다.

부수 이득: post의 후보 셀이 `3×(26²+13²) = 2535` → `3×(16·26+8·13) = 1560`으로 줄어 pre-filter 스캔(worker L182-184)과 int8 출력 버퍼(27.9KB→17.2KB)도 비례 감소 — post 4.78ms에서 소폭(~1ms 미만) 추가 절감. 콘텐츠는 416×235로 **변경 전과 픽셀 단위 동일**(r 동일)이므로, 물체당 해상도를 깎는 320² 축소와 달리 정확도 리스크가 근본적으로 작다.

단독 적용 시 processing 43.01 → ~37.5ms(≈26.7Hz)로 30Hz 격자(33.3ms)에는 못 미친다. 이 항목의 진짜 가치는 **3.1(파이프라이닝)에 가장 크게 가산**된다는 점 — 3.1 이후 DPU 단계가 병목이 되는 레짐에서 dpu 17.35→11.9ms 절감이 그대로 주기 단축으로 직결된다.

**리스크와 검증 방법**

- **컴파일러의 직사각 지원 (게이트 ①)**: DPUCZDX8G가 416×256을 매핑한다는 저장소 직접 근거는 없다(현재까지 전부 416² 실적뿐). 그래서 판정이 PLAUSIBLE이고, 1단계 Inspector 검사("All the operators are assigned to the DPU" + CPU subgraph 0)를 학습·양자화 어떤 투자보다 먼저 둔다. §6.5의 SiLU 교훈(사후 발견 비용 2시간) 재현 방지.
- **재양자화 ≠ bit-identical (게이트 ②)**: PTQ calibration을 다시 돌리므로 per-tensor `fix_point`가 달라질 수 있고 mAP는 재평가 필수. 판정: Gate 2 val 세트 mAP@0.5를 기존 표(§5.3)와 클래스별 비교 → Gate 5 보드 정지 이미지 격리 검증(`yolov3_tiny_image_test.py`) → Gate 6 풀 파이프라인.
- **fix_point 변화의 파급**: `input_fix`가 바뀌면 LUT(L118)와 `thr_q = min_obj_logit × 2^fix`(L288)가 달라지는데, 둘 다 `load_model`에서 텐서로부터 재계산하므로 코드 수정 불요 — 다만 pre-filter 임계 동작이 달라질 수 있어 Gate 5에서 검출 수 비교로 확인.
- **하위호환**: meta 스칼라/튜플 분기를 넣으면 기존 416² xmodel로 즉시 롤백 가능. L262 검사가 짝 안 맞는 배포를 기동 시점에 잡는다.
- **성능 A/B**: `run_gate6_perf.sh 180`로 detector.csv 재수집, `dpu_ms` p50 17.35 → 11.9±1 확인. worker p50 35.26 → ~29.8, processing 43.01 → ~37.5 연쇄 확인. 30fps 격자상 주기는 여전히 processing 1:1 추종 구간이므로 주기 p50도 같은 폭으로 내려와야 한다 — 안 내려오면 고정분 추정(1-2ms)이 틀렸다는 뜻이고 실효치를 재보정한다.

### 3.6 🆕 RGB→BGR→RGB 왕복 채널 스왑 제거 (1.4ms, CONFIRMED)

**현재 구조와 그 이유**

카메라(D435i realsense 노드)는 **rgb8**로 발행한다. 실측 근거는 launch 콘솔 로그다 — `evidence/metrics/runs/shm_gate030_20260805/launch_console.log:57`에 `Format: RGB8, Width: 848, Height: 480, FPS: 30`이 찍혀 있다. `docs/vision/workflow.md:23`(및 :66)은 BGR8로 기재하고 있는데 **이는 코드·로그와 불일치하며, 정본은 런타임 로그다**. 문서만 믿고 encoding을 하드코딩하면 안 되고, 지금 코드처럼 런타임 `msg.encoding` 분기가 맞다.

이 rgb8 프레임이 겪는 일: 노드의 `ros_image_to_bgr()`(`vitis_ai_detector_node.py:58-101`)이 `np.frombuffer` 제로카피 뷰(:75)로 848×480 프레임을 잡은 뒤, rgb8이면 **전 해상도에서 `cv2.cvtColor(RGB2BGR)`**(:94-95)를 돌려 BGR로 "정규화"한다 — 이것이 §1의 img_ms p50 **1.27ms**의 정체다. 그런데 워커의 `letterbox_lut()`(`vitis_ai_worker_yolo.py:125`)은 리사이즈 직후 416×235에서 **`cv2.cvtColor(BGR2RGB)`**(:156)로 도로 뒤집는다 — 모델 입력이 RGB이기 때문이다. 즉 프레임마다 채널 스왑 2회가 **정확히 상쇄되는 왕복**이다.

이게 실수가 아니라 각자 합리적이었던 선택의 잔재라는 점이 중요하다. ① OpenCV의 픽셀 포맷 관행이 역사적으로 BGR이라, "노드 안에서는 무조건 BGR"이라는 정규화 규칙은 표준적인 방어 설계다(함수 위 주석 :57이 정확히 이 논리를 적고 있다). ② 초기 파이프라인은 USB 카메라가 **BGR8을 직접 발행**했고(docs/history.md 초반 `USB Camera (V4L2, 640x480 BGR8)`), 그때는 이 분기가 conversion 없는 경로였다. ③ 당시 노드는 `/vitis_ai_detector/overlay`로 bbox를 그려 발행했는데(history.md 곳곳), cv2 그리기·JPEG 인코딩은 BGR을 전제하므로 **BGR을 실제로 소비하는 코드가 노드 안에 있었다**. 이후 카메라가 rgb8을 내는 D435i로 바뀌며 분기가 조용히 "실제 변환"이 되었고, overlay가 데스크톱 뷰잉으로 이관·사장(:371-396, 플래그 False 고정 — 주석 스스로 "절대 들어오지 않는다"고 명시)되면서 마지막 BGR 소비자가 사라졌는데, 정규화 규칙만 남은 것이다.

**어떻게 바꾸나**

계약(노드→워커 request JSON) 양측을 함께 고친다. 한 패키지 안의 두 파일이고 워커는 노드가 같은 디렉터리에서 spawn하므로(`_default_worker_script_path()`) 배포 스큐는 사실상 없다.

1. **노드**: `ros_image_to_bgr()`를 `(image, channel_order)` 반환으로 확장. rgb8이면 cvtColor를 생략하고 frombuffer 뷰를 그대로 반환 + `"rgb"` 태그. 그 외 인코딩(bgra8, mono8 등)은 기존 cvtColor 폴백 유지 + `"bgr"`.
   ```python
   if encoding == "rgb8":
       return image, "rgb"            # 제로카피 뷰, cvtColor 생략
   if encoding in ("bgr8", "8uc3"):
       return image.copy(), "bgr"     # 기존 경로 그대로
   ```
2. **노드** `detect_with_worker()`(:665): request dict에 `request["channel_order"]` 추가. 뷰는 `np.ascontiguousarray`(:667)를 무변화 통과하고 `shm_writer.write()`가 msg 버퍼→/dev/shm memcpy 1회로 끝난다.
3. **워커** `main()` 루프: `order = request.get("channel_order", "bgr")` — **필드 부재 시 "bgr" 기본값**이라 구버전 노드와도 안전. `detect()` 경유로 `letterbox_lut()`에 전달.
4. **워커** `letterbox_lut()`: `order == "rgb"`면 :156의 `cv2.cvtColor(BGR2RGB)` 생략, 아니면 기존대로.
5. ~~검증 후 workflow.md:23의 BGR8 기재를 rgb8로 정정.~~ **✅ 완료(2026-09-08)** — `workflow.md`(데이터 흐름·§1.1), `STATUS.md`, `vision_final.md`, `detector_worker_walkthrough.md`의 BGR8 기재를 전부 RGB8로 정정했다. 1~4번(전처리에서 `cvtColor` 제거)은 **미착수**로 남아 있다.

**bit-identical 보장 근거** (스왑을 없애도 모델 입력 텐서가 비트 단위로 같은 이유): 현 경로는 "스왑→resize→역스왑", 새 경로는 "resize만"인데, 경로상 모든 연산이 채널 독립이다 — INTER_LINEAR 보간은 채널별로 따로 계산하고, LUT는 전 채널 공통 단일 테이블이며(`build_input_lut` :117-122, docstring "전 채널 공통: v/255 * 2^fix"), 패딩 fill은 단일값 `lut[PAD_VALUE]`(:152, PAD_VALUE=114)다. 채널 순서에 의존하는 연산이 경로에 하나도 없으므로 스왑과 resize는 교환 가능하고, 최종 텐서는 동일하다.

**왜 빨라지나**

cvtColor는 계산이 아니라 **메모리 트래픽**이다. 848×480×3 = 1.22MB를 읽고 새 배열 1.22MB를 할당·쓰기 — 프레임당 ~2.4MB의 DDR 왕복을 Cortex-A53에서 수행한 값이 img_ms p50 1.27이다. 제거하면 frombuffer 헤더 파싱·reshape 잔여 ~0.05만 남아 **노드 −1.17ms**. 워커 쪽 cvtColor는 리사이즈 후 416×235(원본 면적의 24%)라 **~0.25-0.3ms** — 합쳐 **~1.4ms**다. 현 레짐은 주기≈processing 1:1 추종 구간이므로 processing 43.01→~41.6, 주기도 그만큼 준다. 단독으로 33.3ms 격자에 못 넣지만, §3의 다른 레버와 합산되는 순수 감산이며 DPU·모델과 완전히 무관해 부작용 채널이 없다. 부수 효과로 img 꼬리(p90 5.1 / p99 12.5)도 비례 축소된다 — cvtColor는 매 호출 새 출력 배열을 할당하므로 allocator·page fault 지터가 꼬리에 실리는데, 그 자체가 사라진다.

**리스크와 검증 방법**

- **계약 양측 동시 수정**: 한쪽만 바뀌면 모델이 채널 반전 입력을 받아 **조용히** 정확도만 떨어진다(에러 없음 — R/B 민감 클래스가 먼저 죽는다). 워커의 `.get("channel_order", "bgr")` 기본값이 구노드+신워커 조합을 방어하고, 신노드+구워커는 같은 패키지 동시 배포라 실질 불가.
- **frombuffer 뷰는 read-only**: `msg.data`(bytes) 기반이라 이후 코드가 `image`를 in-place 수정하면 ValueError — 소비자는 shm memcpy(읽기만)와 사장된 overlay뿐이라 현재 안전하고, 위반 시 조용한 오염이 아니라 즉시 예외라 오히려 안전장치다.
- **step 패딩 프레임**: `msg.step > width*3`이면 슬라이스 뷰가 비연속 → `:667`의 `ascontiguousarray`가 자동으로 복사 폴백. 현 848×480은 step=2544(무패딩)라 발동 안 하지만, 별도 코드 없이 이미 안전하다.
- **overlay 재활성 시**: `image`가 RGB이므로 그려진 JPEG의 R/B가 반전된다. 사장 코드라 지금은 무관하나, 재활성하려면 그 지점에서 1회 cvtColor를 넣어야 함을 :371 주석에 추기할 것.
- **검증**: ① 오프라인 bit-identity — 같은 프레임을 구/신 경로로 넣어 `letterbox_lut` 출력 int8 텐서를 `np.array_equal`로 비교(파이프라인 기동 불필요, 이것으로 정확도 논쟁 종결). ② A/B 1회 — 같은 장면에서 detector.csv의 img_ms/pre_ms/processing p50·p90 비교 + 검출 수·좌표 일치 확인. 파이프라이닝(§3.1)과 계약 변경이 겹치므로 **이 항목을 먼저 단독 투입**해 A/B를 오염 없이 끝낸 뒤 다음 레버로 넘어가는 순서를 권장한다.

### 3.7 detector rclcpp 포팅 + camera composition — 레버 6 (보정 ~1ms, 최후순위)

**현재 구조와 그 이유** — detector 노드는 Python(rclpy)이다. 이것은 실수가 아니라 세 가지 이유가 겹친 선택이다. ① Vitis-AI 런타임을 Python `vart.Runner` API로 쓰는 워커·전처리(numpy)·디코드가 전부 Python 생태계라, 노드만 C++로 만들면 언어 경계가 하나 늘 뿐이었다. ② 무거운 일(DPU)은 어차피 노드 안에 있지 않다 — VART `execute_async()`가 rclpy in-process에서 segfault를 내는 문제(XIR 객체 GC, vision_final.md L383·STATUS.md §4.3.1) 때문에 워커가 별도 프로세스로 분리됐고, 노드는 "구독→shm 쓰기→JSON 왕복→publish"만 하는 얇은 껍데기다. ③ 노드 간 대용량 전송 비용은 W2(FastDDS SHM transport)가 이미 커널 네트워크 스택을 제거해 크게 줄였다. 카메라 쪽도 third-party `realsense2_camera`(C++)를 launch에서 별도 프로세스로 포함하는 표준 구성이다(`pick_place_vitis_ai.launch.py` L101–105). 그런데도 이 레버가 카탈로그(STATUS.md L290, 표기상 "CPU·latency 동시 대폭")에 남아 있는 이유는 zero-copy 분석(STATUS.md L254–266)의 결론 때문이다: `sensor_msgs/Image`는 unbounded 필드 3곳 때문에 DDS Data Sharing이 불가능하고, **rclpy에는 intra-process comm도 loaned message API도 없다**(실측: loan 메서드 0개). 즉 Image의 진짜 zero-copy 경로는 rclcpp 포팅 + composition **하나뿐**이다 — "유일한 경로"라는 사실과 "그 경로의 이득이 크다"는 별개이고, 이번 검증에서 후자가 감액됐다.

**어떻게 바꾸나** — 워커 프로세스는 그대로 두고 노드만 옮긴다.
1. `vitis_ai_detector_node.py`를 rclcpp 노드로 포팅: 구독 콜백의 최신-프레임 latch(L311–320)와 전용 처리 스레드(L322 `worker_loop`) 구조를 그대로 재현하고, shm 쓰기 + stdin/stdout JSON 헤더 왕복(L710–723) 계약을 C++ 클라이언트로 재구현한다. 계약(JSON 스키마·shm 레이아웃)은 바꾸지 않는다 — 워커는 Python으로 남는다.
2. 노드를 `rclcpp_components` 컴포넌트로 빌드한다.
3. launch에서 `rs_launch.py` include를 `ComposableNodeContainer`로 교체해 realsense 컴포넌트와 detector 컴포넌트를 **한 프로세스**에 싣고 `use_intra_process_comms:=true`를 준다. 이때 카메라→detector의 Image 전달이 직렬화 없이 `shared_ptr` 이동이 된다.
4. (선택) publish 쪽에 loaned message를 적용한다 — 단 아래 검증대로, 소비자가 별도 프로세스라 실익이 없다.

**왜 빨라지나** — intra-process comm의 메커니즘 자체는 실재한다: 같은 프로세스 안에서는 publisher가 메시지의 `shared_ptr`를 in-process 버퍼에 넣고 subscriber가 그 포인터를 받으므로, CDR 직렬화→SHM segment memcpy(쓰기/읽기)→역직렬화→rmw C 구조체→Python 객체 변환의 사슬 전체가 소멸한다. 1.22MB 프레임 기준 memcpy 대역만 따져도 왕복 수 ms급이다. 문제는 **그 사슬이 지금 우리 processing_ms 안에 거의 없다**는 것이다(검증자 감액 2.5→1ms의 근거):
- (a) img 1.27ms는 DDS 역직렬화가 아니다 — 역직렬화·Python 변환은 rclpy rmw 계층에서 **콜백 진입 전**에 일어나 processing_ms 계측 밖이고, 콜백이 받은 `msg.data`에 대한 L75 `frombuffer`는 이미 zero-copy view다. 1.27의 정체는 L95 `cvtColor(RGB2BGR)`이며 이것은 C++로 포팅해도 잔존한다(제거는 §3.6의 몫). 회수분은 reshape/slice 파이썬 오버헤드 ~0.2–0.3ms.
- (b) publish 0.76ms의 소비자(pick_logic)는 **별도 프로세스**라 composition으로 intra-process화되지 않는다. rclpy→rclcpp의 publish 경로 오버헤드 절감 ~0.5ms.
- (c) ipc 3.19ms는 노드↔워커 프로세스 경계(shm+JSON) 비용이라 포팅과 무관하게 잔존.
합계 ~1ms. 주 이득 후보는 오히려 processing 밖의 age_in(p50 57.87ms)이다 — 콜백 전 역직렬화·변환·큐 대기가 사라지면 신선도가 수 ms 좋아질 수 있으나 미정량이고, 캡처→도착 40.7ms의 대부분은 센서 물리(30fps readout)라 소프트웨어로 못 줄인다. W2가 네트워크 비용을 이미 제거한 뒤라 남은 살이 얇다.

**리스크와 검증 방법**
- **W3 교훈이 직접 적용된다**: rclpy 노드 병합은 "당연히 이득"으로 보였지만 실측 +5.4pt 역효과로 기각됐다(STATUS.md L243). 병합·포팅류는 실측 없이 확신 금지 — 착수하더라도 먼저 detector만 포팅한 A/B(composition 없이)와 composition A/B를 분리 측정한다.
- **언어 교차 계약 유지보수**: C++ 노드 ↔ Python 워커의 JSON/shm 계약이 두 언어로 이중화된다. 스키마 변경마다 양쪽 수정 + bit-identity 전면 재검증(전 클래스 검출 A/B) 비용이 영구적으로 붙는다.
- **장애 격리 상실**: composition은 카메라와 detector를 한 프로세스에 넣으므로 한쪽 crash가 카메라까지 데려간다. 현재 노드는 워커 사망을 감지해 재시작하는 감시자 역할(§4.3.2)도 하므로 이 로직의 이식 누락이 곧 가용성 회귀다.
- **판정 방법**: detector.csv 동일 하네스로 processing 분해(img/ipc/publish)·age_in·노드별 CPU를 6,000프레임급 A/B. 성공 기준은 "age_in p50 −3ms 이상 또는 CPU −3pt 이상"처럼 사전에 수치로 박아두고, 미달이면 되돌린다.
- **착수 게이트**: §3.1/3.3/3.5/3.6이 소진되고도 신선도가 부족하다는 실기 판정이 나온 뒤에만. 이득 ~1ms 대비 공수(포팅+재검증)가 9건 중 최악이라 **최후순위**가 결론이다.

### 3.8 🆕 letterbox 리사이즈를 shm 쓰기에 융합 (보정 ~0.5ms, PLAUSIBLE)

**현재 구조와 그 이유**

- 현 흐름: 노드가 848×480×3 = 1.22MB BGR 원본을 `ShmFrameWriter.write()`로 /dev/shm 매핑에 memcpy 1회(`shm_frame.py:49`의 `self._np[:n] = arr.reshape(-1)`, 호출부 `vitis_ai_detector_node.py:713`) → 워커가 zero-copy numpy 뷰로 받아 `letterbox_lut()` 안에서 416×235로 `cv2.resize`(`vitis_ai_worker_yolo.py:141`; r = min(416/480, 416/848) = 0.4906) 후 패딩·BGR2RGB·LUT를 수행한다.
- 왜 풀프레임이 넘어가는가: shm 채널(2026-08-05, throughput.md §4 방법 0)은 stdin pipe(복사 3-4회, ipc 9-12ms)를 대체하는 **순수 전송 최적화**로 설계됐다. 페이로드 내용은 그대로 두고 전송 수단만 바꾸는 최소 변경이었고, `shm_frame.py` 모듈 docstring이 그 계약을 못 박는다: "ONE memcpy", 노드는 요청 전만 쓰고 워커는 요청–응답 사이만 읽는 strictly serial 프로토콜, 성장 시 inode 유지.
- 노드측 resize의 선례와 폐기 이유: SSD 시절 `send_resized_input: true`가 정확히 노드측 resize였다(pipe payload −81%, workflow.md §최적화). YOLO 전환 때 **의도적으로** false로 내렸는데(node:683-694에 코드가 살아 있고 주석 "현재 비활성화됨"), 실행계획 문서의 근거는 "node의 plain resize는 비율을 깨므로"(yolov3_tiny_execution_plan.md) — 그 경로는 aspect를 무시하는 stretch라 letterbox 좌표계와 충돌했다. 즉 지금 구조는 실수가 아니라 "letterbox 정합성 > IPC 절감"이라는 당시의 합리적 선택이고, 본 후보는 워커와 동일 산식의 **letterbox-aware 축소**라 그 반례에 해당하지 않는다.

**어떻게 바꾸나**

1. `ShmFrameWriter`의 성장 로직(shm_frame.py:39-48, ftruncate+remap)을 `_ensure(nbytes)` 헬퍼로 추출하고, 이를 재사용하는 `view(shape)`를 추가: 매핑 확보 후 `self._np[:nbytes].reshape(shape)`(C-contiguous 뷰)를 반환.
2. 노드 전송 경로(node:697 부근): 워커와 동일 산식으로 축소 치수 계산 — `r = min(W/h, W/w)`(W=416, ready 응답의 `worker_input_width` 재사용), `nw, nh = round(w*r), round(h*r)` → `dst = writer.view((nh, nw, 3))`; `cv2.resize(image, (nw, nh), dst=dst, interpolation=cv2.INTER_LINEAR)`. request의 `height/width`는 축소 치수(235/416)로 바꾸고, `source_height/width`는 원본 유지 — 이 필드는 이미 있다(node:704-705).
3. 워커: `letterbox_lut`은 `(nh, nw) == (h, w)`면 resize를 건너뛰므로(worker:140-143) 전처리 코드는 거의 무변경. 단 **ratio를 source 치수로 재계산** — `detect()`에 source 치수를 넘겨 `r_true = min(416/source_h, 416/source_w)`로 역변환하고, `make_detections`의 sx·sy 2단 스케일 대신 단일 r_true 나눗셈으로 통일. source 치수는 이미 request에 흐른다(worker:445-446).
4. `cv2.resize` 반환값이 넘긴 dst와 동일 객체인지 assert — 아래 리스크 1의 가드.

**왜 빨라지나**

- 이동과 소거를 구분해야 한다. resize 연산 자체는 워커 pre(11.56ms)에서 노드 img 단으로 **자리만 옮긴다** — 프로토콜이 strictly serial이라 노드·워커가 동시에 일하지 못하고, 직렬 총합은 불변이다. 순이득은 1.22MB memcpy 소거분뿐.
- 산식: 현재 `write()`는 1.22MB를 읽고 1.22MB를 쓴다. A53의 실효 memcpy 대역 ~2-3GB/s 기준 **≈ 0.4-0.6ms**(ipc p50 3.19ms의 일부). 융합 후 노드는 resize 출력 287KB(416×235×3 = 293,280B)만 shm에 쓰고, 원본 1.22MB read는 resize가 어차피 하던 접근이다. 메모리 서브시스템 관점에서 "1.22MB를 한 번 더 왕복시키던" 트래픽이 사라지는 것이 전부다.
- 부수 효과로 워커의 cvtColor·LUT 이전 cross-process cache-cold read가 1.22MB→287KB로 줄지만, 보정치(~0.5ms)에는 포함하지 않는다.

**리스크와 검증 방법**

1. **조용한 재할당(검증 라운드 발굴, 최대 함정)**: `cv2.resize(dst=뷰)`는 dst의 형상·dtype·연속성이 하나라도 안 맞으면 예외 없이 새 배열을 할당해 반환한다 — shm에는 아무것도 안 써지고 워커는 이전 프레임을 계속 읽는다. docstring이 경고하는 inode 함정과 동급의 침묵 고장. 가드: 반환 객체 identity assert(단계 4) + 프레임 카운터를 픽셀에 새겨 워커가 에코하는 스모크를 `test/test_shm_frame.py`에 추가.
2. **성장/재매핑 복제 누락**: `view()`가 ftruncate+remap 경로를 안 타면 첫 프레임 또는 해상도 변경 시 실패한다. `_ensure()` 단일화(단계 1)로 원천 차단.
3. **bit-identity 이탈**: 축소본에서 r=1.0으로 계산하면 역변환 y스케일이 2.0426 vs 2.0385로 갈려 가장자리 ~0.5px 오차. 단계 3의 ratio 재계산으로 해소한 뒤, 동일 프레임 세트에 대해 구경로/신경로 detection 좌표 diff == 0을 확인(resize 자체는 동일 연산·동일 치수라 픽셀 bit-identical).
4. **A/B 판정**: detector.csv에서 ipc p50 3.19 → 2.6-2.8ms, img 단은 resize 이동분만큼 증가, worker pre는 같은 만큼 감소, worker_call 총합 −0.4~0.6ms가 기대치. 주기 45.47ms 대비 ~1%라 단독으로는 레짐(주기≈processing 1:1 추종)을 바꾸지 못한다.
5. **착수 조건**: 단독 착수 비권장. shm 계약(치수 협상·request 필드·reader remap)을 어차피 여는 **3.1에 묶는 조건부** 후보다. SSD 폴백(`send_resized_input`)과의 상호작용은 없음 — 그 플래그는 pipe 경로 전용 유물로 그대로 둔다.

### 3.9 NMS를 C++ 경로로 — cv2.dnn.NMSBoxes (보정 ~0.5ms, PLAUSIBLE)

**현재 구조와 그 이유**

`make_detections()`(`src/vitis_ai_detector_pkg/vitis_ai_detector_pkg/vitis_ai_worker_yolo.py:339`)는 클래스마다 Python 루프를 돌며(:349–355) 순수 numpy greedy-NMS(`nms()` :102–114, `iou()` :83)를 호출하고, keep된 박스마다 letterbox 역변환·클램프·dict 생성을 검출 1건당 Python 산술로 수행한다(:356–378). NMS를 손으로 쓴 것은 실수가 아니라 이 코드의 출신 때문이다: worker의 postprocess는 standalone SSD 테스트 스크립트 시절(docs/history.md의 "Standalone SSD ADAS Test") softmax·prior decode·NMS까지 전 단계를 직접 구현해 DPU 파이프라인을 검증하던 코드가 이식된 것이고, 학습용 주석이 달린 채(docs/vision/detector_worker_walkthrough.md §2가 이 nms를 교재로 씀) 유지됐다. 이후 최적화도 NMS 자체가 아니라 **입구를 좁히는 방향**이었다 — int8 objectness pre-filter(vision_final.md, `decode_head` :183–186)가 후보를 소수로 줄이면서 NMS는 한 번도 hot spot으로 측정된 적이 없어, 굳이 건드릴 이유가 없었다.

**어떻게 바꾸나**

1. `make_detections` 안의 `for k in nms(cb, cs, NMS_THRESHOLD):`를 `cv2.dnn.NMSBoxes` 호출로 교체. OpenCV는 박스를 `(x, y, w, h)`로 받으므로 xyxy → xywh 변환 1줄이 필요하다:
   ```python
   xywh = np.column_stack([cb[:, 0], cb[:, 1], cb[:, 2] - cb[:, 0], cb[:, 3] - cb[:, 1]])
   keep = np.asarray(cv2.dnn.NMSBoxes(xywh, cs, 0.0, NMS_THRESHOLD)).flatten()
   ```
   `score_threshold=0.0`이 핵심 — per-class threshold(:350의 mask, person 2.0=OFF vs 기본 0.50)는 이미 선적용됐으므로 NMSBoxes에 또 넘기면 이중 적용이 된다.
2. 역변환 벡터화: keep된 박스에 대해 `(cb[keep] - [pad_x, pad_y, pad_x, pad_y]) / ratio * [sx, sy, sx, sy]`를 numpy 한 방으로 계산하고 `np.clip`으로 클램프한 뒤, 최소 크기 필터도 boolean mask로. dict 생성 루프만 Python에 남긴다.
3. 기존 `nms()`/`iou()`는 A/B 검증이 끝날 때까지 남겨두고, 검증 후 삭제.

**왜 빨라지나**

p50에서 NMS 입력은 이미 소수(pre-filter 덕분에 6,343프레임 전부 count=5 상수)라 알고리즘 복잡도가 아니라 **인터프리터 dispatch 오버헤드**가 비용의 정체다. 배열이 5개짜리일 때 numpy 연산 하나는 실제 계산(수십 ns)보다 호출 고정비(수 µs)가 지배하는데, 현재 경로는 클래스 루프 × (`argsort` + while 반복마다 `iou`의 numpy 연산 ~6회 + fancy indexing) + 검출별 Python float 산술 8줄로 이런 소형 호출이 프레임당 수백 회 쌓인다. `cv2.dnn.NMSBoxes`는 이 전체를 C++ 함수 1회로 접고, 역변환 벡터화는 검출별 산술을 배열 연산 몇 회로 접는다. 다만 post p50 4.78ms의 대부분은 `decode_head`(:164–200)의 numpy 체인(reshape/nonzero/sigmoid/stack/concat ×2헤드)이고 greedy NMS는 sub-ms 추정이라, 이전 라운드의 1.5ms 견적은 0.5ms로 감액됐다. 클래스 루프 골격과 dict 생성은 교체 후에도 잔존한다. 단독 효과는 43.01→~42.5ms, **~+0.2Hz** — 이 후보의 실질 가치는 파이프라이닝(§3.1) 이후 CPU 레그가 병목이 될 때의 시너지다. 참고로 "p95 꼬리(검출 폭증) 방어" 논거는 데이터와 모순된다: count=5 상수인데 post p95=11.77ms이므로 꼬리는 검출 수가 아니라 CPU 경합성이다.

**리스크와 검증 방법**

- **동등성**: OpenCV의 greedy-NMS는 같은 알고리즘이지만 confidence 동점 시 정렬 순서와 부동소수 연산 순서가 달라 bit-identical은 보장되지 않는다. pre-filter 때 쓴 방식(history.md: LUT는 bit-for-bit, pre-filter는 랜덤 30회 동일 확인) 그대로, 랜덤/실측 프레임에 대해 구·신 경로의 keep 집합과 최종 박스 좌표를 비교하는 A/B 스크립트로 판정한다 — 기준은 "keep 집합 동일 또는 동점 케이스에서만 상이".
- **이중 threshold**: `score_threshold=0.0`을 빠뜨리고 단일 값을 넣으면 per-class 임계가 이중 적용된다(현재는 person 2.0=OFF 외 전부 0.50이라 실질 영향은 없지만 계약 위반). A/B에 임계 근처 conf 케이스를 포함.
- **좌표 포맷**: xyxy를 그대로 넘기면 w·h 자리에 x2·y2가 들어가 IoU가 조용히 틀어진다(에러 없이 결과만 다름). A/B가 이를 잡는다.
- **효과 측정**: detector.csv의 post_ms 컬럼 p50 전후 비교(pre-filter 최적화 때와 동일한 방법), 목표 −0.5ms 내외.

## 4. 착수 전 게이트 (§3.1의 go/no-go)

파이프라이닝 이득 전체가 **VART `execute_async`가 submit 시점에 입력을 내부
복사하고 즉시 반환한다**는 미검증 전제에 걸려 있다. 현 코드는 submit 직후 즉시
wait라(`:312-313`) dpu_ms 17.35가 submit+wait 합산 — 기존 데이터로 판별 불가.

**구현 1단계 = 코드가 아니라 측정**: execute_async 전후 타임스탬프를 찍는 최소
스크립트로 (a) submit이 즉시(≪17ms) 반환하는지 (b) submit 후 입력 버퍼를 덮어써도
결과가 안 깨지는지(더블버퍼 필요성 판정) 확인. no-go면 §3.1·§3.3의 throughput
트랙은 §3.2(방법 B) 또는 §3.4로 우회.

이력 주의: 워커의 segfault 전력은 rclpy 충돌이 아니라 **XIR graph/subgraph 파이썬
wrapper의 GC(수명 버그)**였음이 규명되어 있다(docs/history.md 2026-05-21). 파이프
라이닝 구현 시에도 XIR 객체 수명·장시간 스트레스(1000회+)는 필수 검증 항목.

## 5. 검증에서 깎이거나 걸러진 것 (기록)

- 파이프라이닝 12.2→**11.8ms**: p50 합산 산식이 낙관 — 프레임 단위 재계산 시 22~36%
  프레임이 33.3 초과라 완전 30Hz 안착은 안 됨(28~29Hz).
- 416×256 6.5→**5.5ms**: dpu_ms 선형 스케일 가정에서 VART 호출·DMA 고정분 차감.
- rclcpp 2.5→**1ms**: img_ms는 DDS 역직렬화가 아니었음(frombuffer는 이미 뷰).
- NMS 1.5→**0.5ms**: p95 꼬리 논거가 count=5 상수 데이터와 모순.
- REJECTED 0건 — 병합 단계에서 기각 이력 중복(rclpy 노드 병합 W3, 보드측 overlay,
  align_depth, 워커↔노드 프로세스 병합)을 사전 제거했기 때문.

## 6. 벤더 벤치마크(UG1354 Table 65)와의 괴리 분해 (2026-09-02 추가)

> 질문: "UG1354 p.145 No.122 `tiny_yolov3_vmss`가 KV260에서 **130.9 fps**(single) /
> **167.2 fps**(multi)인데, 우리는 왜 21.7 Hz인가?"

**결론: 성능 미달이 아니라 측정 대상이 다르다.** 다만 "보드가 달라서"는 답이 아니다 —
같은 KV260 표(Table 65)가 맞고, 입력 크기 416×416·5.46 GOPS도 우리 모델과 사실상 같다.

### 6.1 그 수치가 재는 것

| | UG1354 130.9 fps | 우리 21.7 Hz |
|---|---|---|
| 이미지 출처 | **메모리 사전 적재** | 카메라 30fps 실스트림 |
| 전·후처리 | C++ (NEON, ~2ms 추정) | Python (LUT 최적화, **16.34ms**) |
| 실행 형태 | 타이트 루프, 버퍼 재사용 | ROS 노드 + 워커 프로세스 + IPC |
| 측정 범위 | **라이브러리 함수의 반복 속도** | **카메라→발행 시스템 전체** |

fps 대 fps 비교는 성립하지 않는다. 공정한 축은 프레임당 시간:
**7.64ms(벤치) vs 35.26ms(우리 worker_ms) = 4.6배.**

### 6.2 갭 폭포수 (전부 §0 실측 기반)

```
131 fps (7.64ms)  ← 벤치: 메모리 이미지 + C++ + 타이트 루프
  │ ① DPU 실행 자체        ~6ms → 17.35ms      [+11.4ms]
58 fps 상당 (dpu_ms 단독)
  │ ② Python 전·후처리     pre 11.56 + post 4.78 [+16.3ms]
28.4 fps (worker 35.26ms)
  │ ③ 시스템 배관          img 1.27 + ipc 3.19 + pub 0.76 + 잔여 1.57 [+6.8ms]
23.3 fps (processing 43.01ms)
  │ ④ 도착 격자 대기                            [+2.5ms]
21.7 Hz (주기 45.47ms)   ← 현재
```

### 6.3 ①의 정체 — DPU 구성이 다를 가능성이 높다 (추정)

산수가 맞지 않는다:

- 5.46 GOPS × 130.9 fps = **715 GOPS/s 지속** 필요.
- B3136 피크 = 3136 ops/cycle × 300 MHz ≈ **941 GOPS/s** → 전·후처리 포함 피크의
  76~97%를 지속해야 함(비현실적).
- 결정적 반증: **ZCU104(Table 64 No.122, p.140)의 싱글스레드가 132.2**로 KV260의
  130.9와 사실상 동일. 싱글스레드는 DPU 1코어만 쓰므로 **두 보드의 DPU 코어가 같다**는
  뜻이고, ZCU104는 B4096이다.

→ **UG1354의 KV260 행은 B4096 벤치마크 플랫폼(Vitis AI 공식 보드 이미지)일 가능성이
높고, 우리가 쓰는 `kv260-smartcam` 오버레이의 B3136이 아니다.** B4096/B3136 = 1.31배.
⚠️ 문서가 표에 DPU 구성을 적지 않아 **확증은 아니다**(추정).

나머지 ~1.7배: 우리 `dpu_ms 17.35`는 순수 HW 시간이 아니라 **Python VART 바인딩의
submit+wait 벽시계**(호출 오버헤드·버퍼 복사·DMA 왕복 포함, throughput.md 위험 3과
동일 지적). 실효 이용률 5.46 GOPS/17.35ms ≈ **317 GOPS/s = B3136 피크의 34%**.

### 6.4 가장 근본적인 사실 — 천장은 벤치마크가 아니라 카메라다

**벤치의 131 fps 라이브러리를 통째로 이식해도 우리 출력은 30 Hz다.** 카메라가 30fps이기
때문이다. 벤치는 메모리에서 읽으므로 이 천장이 없다.

→ 의미 있는 괴리는 "131 vs 21.7"이 아니라 **"30(천장) vs 21.7"**이고, 그 8.3 Hz의
회수 경로가 이 문서 §1~§3이다.

### 6.5 이 표에서 실제로 얻을 것

1. **"Multiple thread" 167.2 fps는 §3.1의 벤더판 증거다.** 벤치의 single→multi가
   **1.28배**인데, 우리 §3.1(pre‖DPU 파이프라이닝) 추정이 45.47→~33.7ms = **1.35배**로
   같은 물리 현상이다. 벤더도 DPU 유휴를 CPU 단계 겹침으로 메운다.
2. **층별로 카드가 이미 매핑된다**: ① → §3.5(416×256, DPU 연산량 자체를 축소),
   ② → §3.4(PL 전처리 오프로드)·§3.9(NMS C++화), ③ → §3.8·§3.7, ④ → §3.3(60fps).
3. **병목이 DPU가 아니라는 throughput.md의 결론이 벤더 문서로도 재확인된다** — 같은
   실리콘에서 DPU가 한참 남아 있고(우리 실측 ~60% 유휴), 갭의 대부분은 DPU 바깥이다.

⚠️ 이 절의 ①은 **정적 추론**이다. 확정하려면 `xdputil query`로 실제 DPU arch(B3136)를
확인하고, 벤치와 같은 조건(메모리 이미지 + 반복 추론)으로 우리 워커의 DPU 단독 fps를
재야 한다 — §4의 go/no-go 계측에 이 항목을 같이 넣으면 한 번에 끝난다.
