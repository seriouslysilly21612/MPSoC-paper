# 파이프라인 metrics — performance · resource · yield

perception→pick 파이프라인 end-to-end 계측. **기본값 off** (CSV 경로가 비면 →
buffering·I/O 전혀 없음, detector의 원래 `metrics_csv_path`와 동일). run마다
켜서 in-memory로 buffering하다가, **window-close 또는 shutdown 시 CSV 한 번**만
씁니다.

## 레이아웃 — run마다 디렉토리 하나 (기록 보존)

**매 run이 자기 디렉토리를 갖습니다. 덮어쓰지 않습니다.**

```
evidence/metrics/
├── latest -> runs/20260722_231246      # 최신 run 을 가리키는 symlink (상대경로)
├── runs/
│   ├── 20260722_231246/                # 라벨 = 기동 시각 (또는 metrics_run:= 로 지정)
│   │   ├── performance/  resource/  yield/     # 측정 CSV
│   │   └── plots/  plots_paper/                # 그래프 (plots.sh 가 생성)
│   └── 20260730_150410/
└── vitis_ai_metrics*.csv               # 2026-06 detector 단독 시절 유물 (구 포맷)
```

- **파일 이름은 고정**(`performance/detector.csv` …)입니다 — `join_perf.py`·`plot_metrics*.py`가
  `--base <dir>` 아래 이 구조를 전제하므로, **파일명에 날짜를 붙이는 대신 디렉토리를 분리**해서
  기록을 보존하면서 도구를 그대로 쓸 수 있게 했습니다.
- 같은 라벨을 재사용하면 그 run은 **덮어써집니다**(의도적 — 재측정 용도).
- `latest`가 실제 디렉토리면 launch가 건드리지 않고 경고만 냅니다.

## 세 가지 산출물 (각 run 디렉토리 안)

| Dir | 내용 | 쓰는 주체 |
|---|---|---|
| `performance/` | frame별 stage latency + **true E2E** (capture → base publish) | 4개 노드가 각자 자기 CSV |
| `resource/` | process별 CPU cores / memory + system + temp/power | `tools/metrics/resource_sampler.py` (standalone) |
| `yield/` | reject-reason tally + valid-rate + worker 신뢰성 (노드당 1 row) | 각 노드가 shutdown 시 자기 CSV |

## Clock discipline (왜 이게 맞나)

- **Intra-node stage duration** → `time.perf_counter_ns()` (monotonic). duration은
  한 프로세스 안에서 끝난 self-contained delta라, 프로세스가 달라도 비교 가능
  (예: detector의 `ipc_overhead_ms`).
- **Cross-node age / E2E** → `get_clock().now()` − 전파된 **`capture_stamp`**
  (원본 color capture 시각). 모든 노드가 같은 KV260 system clock을 공유하고
  (`use_sim_time=false`), 따라서 이 뺄셈이 valid.

`capture_stamp`는 `PickTarget.msg`·`PickTarget3D.msg`에 추가한 **전용 필드**입니다.
pick_logic → 3d → base로 **덮어쓰지 않고** 흘려보냅니다. (`PickTarget3D.header.stamp`는
*depth* stamp 그대로 둡니다 — base 노드가 TF lookup에 `header.frame_id`를 쓰기
때문. 그래서 header에 얹지 않고 capture_stamp를 따로 둔 것.)

## 켜는 법 — launch 플래그 하나

```bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py metrics:=true
```

이거면 끝입니다. launch가 **4개 노드에 performance+yield CSV 경로를 주입**하고
**resource_sampler를 자동으로 띄웁니다**. `metrics_duration`(기본 300초) 뒤 자동
저장되고, 그다음 Ctrl-C 하면 됩니다 (Ctrl-C는 sampler에도 전달돼 resource CSV까지
저장). 플래그 없이 실행하면 아무것도 기록 안 하고 평소 동작 그대로입니다.

선택 override:
```bash
ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py \
    metrics:=true metrics_duration:=300 metrics_dir:=/some/abs/dir
```

| arg | 기본값 | 의미 |
|---|---|---|
| `metrics` | `false` | true = 기록 on (4 노드 CSV + sampler) |
| `metrics_dir` | `/home/ubuntu/ros2_ws/evidence/metrics` | base dir — 실제 기록은 그 아래 `runs/<label>/` |
| `metrics_run` | `''` (= 기동 시각 `YYYYmmdd_HHMMSS`) | 이 run의 라벨. 예: `metrics_run:=baseline` → `runs/baseline/` |
| `metrics_duration` | `300.0` | 첫 frame부터 N초 수집 후 자동 저장; `0` = shutdown까지 |
| `sampler_script` | `.../tools/metrics/resource_sampler.py` | sampler 경로 |

> **왜 sampler는 노드 param이 아닌가**: sampler는 ROS 노드가 아니라, worker
> 서브프로세스를 포함한 7개 OS 프로세스를 `/proc`로 보는 외부 관찰자라 노드 param에
> 못 넣습니다. 대신 launch가 대신 띄워주므로 CLI 인자를 직접 칠 필요가 없습니다.

> yaml에 직접 경로를 박아 **항상** 켜둘 수도 있지만(각 노드 `metrics_csv_path` /
> `yield_csv_path` / `metrics_duration_sec`), 그러면 매 run마다 기록돼 CSV가 계속
> 덮어써집니다. 위 플래그가 기본 off라 더 낫습니다. (경로는 항상 **절대경로** —
> ROS param YAML은 `~`도 env var도 확장 안 함.)

## 읽어내는 법

4개 performance CSV를 capture stamp로 join해서 tail stat + inter-node handoff 3개를
뽑습니다 (handoff는 offline 도출이라 runtime 비용 0):

```bash
python3 tools/metrics/join_perf.py \
    --detector evidence/metrics/latest/performance/detector.csv \
    --pick-logic evidence/metrics/latest/performance/pick_logic.csv \
    --target-3d evidence/metrics/latest/performance/target_3d.csv \
    --base evidence/metrics/latest/performance/base.csv \
    --out-merged evidence/metrics/latest/performance/merged.csv \
    --out-summary evidence/metrics/latest/performance/summary.csv
```

## 그래프로 보기

> **한 방에**: `~/ros2_ws/launch_script/plots.sh` — **최신 run**(`latest`)을 자동으로 찾고
> 의존성 확인까지 대신 해줍니다.
> `--list` run 목록 / `--run <라벨>` 특정 run / `--all` 전체 run /
> `--quick` 간이 3장 / `--both` 둘 다 / `--base <dir>` 임의 디렉토리.
> 아래는 그 스크립트가 실제로 호출하는 것들입니다.

렌더러가 **두 개**입니다 — 용도가 다릅니다. 둘 다 `join_perf.py`를 안 돌렸어도 4개
노드 CSV로 스스로 join하니 그냥 실행하면 됩니다. (차트 텍스트는 English, 터미널
요약은 한글 — matplotlib 기본 폰트에 한글 glyph가 없어서 차트 안 텍스트만 English.)

### 1) `plot_metrics.py` — 보드에서 빠른 확인 (stdlib + matplotlib)

추가 의존성 `matplotlib` 하나뿐(보드에 이미 3.5.1 설치됨). numpy/pandas 안 씀 →
Kria에서 바로 돌아갑니다. 3-panel 대시보드 PNG 3개(`<base>/plots/`)를 뽑습니다.

```bash
python3 tools/metrics/plot_metrics.py --base evidence/metrics/latest   # 보드에서 바로
```

### 2) `plot_metrics_paper.py` — 논문/발표용 (numpy + matplotlib)

지표를 **논문/시스템즈-페이퍼에서 쓰는 형태**로 그립니다. `<base>/plots_paper/`에
figure 8장을 저장합니다.

**보드에서 바로 돌립니다** — numpy 1.21.5 / matplotlib 3.5.1이 이미 설치돼 있고,
**ROS와 무관한 오프라인 변환**이라 `setup.bash` source도 파이프라인 실행도 필요 없습니다
(`env -i` 로 환경을 완전히 비운 상태에서도 8장 생성 확인).

```bash
python3 tools/metrics/plot_metrics_paper.py --base evidence/metrics/latest
#  또는 그냥:  ~/ros2_ws/launch_script/plots.sh
```

PNG를 PC에서 보고 싶으면 **생성은 보드에서 하고 PNG만** 가져오는 게 가볍습니다
(CSV 전체보다 훨씬 작음):
```bash
scp -r kria:~/ros2_ws/evidence/metrics/latest/plots_paper .
```

| figure | 폼 | 읽는 법 |
|---|---|---|
| `1_latency_distribution` | **ECDF + CCDF(log-y)** | latency 분포의 표준형. **ECDF** = "x ms 이하로 도착한 프레임 비율"(y=0.5→p50, 0.95→p95). **CCDF** = "x ms 이상 걸린 비율"(로그축, tail 확대). 퍼센타일이 높이+실제값으로 벌어져 **선이 안 겹침** |
| `2_latency_breakdown` | **stacked contribution bar + per-stage boxplot(log-y)** | 왼쪽=E2E budget이 어느 stage로 가나(capture→detect+detector가 ~90%), 오른쪽=어느 stage가 tail을 만드나(target_3d reproj의 outlier가 E2E spike 주범) |
| `3_latency_timeline` | **running median line + p10–p90 band** | 시간 흐름에 지연이 일정한가. 선=매 순간 대표(중앙) 지연, 띠=중간 80% 범위(좁으면 안정적). 드문 spike는 우상단 annotate(꼬리는 CCDF에서) |
| `3b_throughput` | **inter-output gap ECDF + rate-over-time** | latency의 짝 — *얼마나 자주* 결과가 나오나. 왼쪽=출력 간격 분포(66.6ms 선 왼쪽 = 그 순간 15Hz 충족, 여기선 71%), 오른쪽=시간별 출력 rate(≈16.9Hz, 15Hz 목표 충족). **latency는 나이, throughput은 빈도** |
| `4_cpu` | **stacked area + mean/peak bar** (프로세스 뷰) | 어느 process가 core를 먹나. total 선 = 전체 core 사용량(mean 1.9 / peak 4.1 of 4) |
| `4b_cpu_cores` | **per-core small multiples (2×2)** | 4개 A53 코어를 **각각 개별 패널**로. 코어별 부하 균형 확인(mean 54~61%) |
| `5_memory_thermal` | **RSS/free-RAM/temp/power 시계열** | RSS·free RAM 평평 = **memory leak 없음**, 온도·전력 |
| `6_yield_funnel` | **frame-survival funnel + valid-rate** | 왼쪽=단계별 프레임 생존 수(30fps 입력을 **의도적으로** 15Hz로 gate → 다운샘플, 손실 아님), 오른쪽=처리된 프레임의 valid rate(전부 100%, worker restart 0) |

> 왜 두 개냐: `plot_metrics.py`는 **의존성 0**로 보드에서 즉시 sanity-check용,
> `plot_metrics_paper.py`는 numpy로 ECDF/CCDF/rolling 같은 제대로 된 통계 폼을 그려
> 보고서/논문에 바로 넣는 용도. 소스 CSV는 동일합니다.

## E2E latency ladder (performance CSV)

```
[capture] ─ capture_stamp (join key: capture_sec, capture_nanosec)
  age_in_ms                    detector: capture → process_frame 시작
  processing_ms + dpu_ms/pre_ms/post_ms/worker_ms/ipc_overhead_ms/...   detector 내부
  detect_period_ms             detector: /detections publish 간격 (jitter)
  handoff_det_to_pick_ms       offline: pick_logic_in_age − detector frame_age
  pick_logic_compute_ms        pick_logic: filter 루프
  handoff_pick_to_3d_ms        offline
  target3d_compute_ms          3d: reverse-projection (tail 의심 1순위)
  depth_vs_color_skew_ms       3d: 융합된 depth stamp − color capture (부호 있음)
  handoff_3d_to_base_ms        offline
  base_compute_ms              base: TF matmul
  true_e2e_ms                  base: capture → /pick_target_base publish  ★headline
  output_period_ms             base: 최종 publish 간격 (robot 소비자가 보는 jitter)
```

## 참고 / deferred

- **latency ≠ throughput (중요)**: `true_e2e_ms`(≈117ms median)는 *한 프레임의
  나이*(capture→base)이고, 15Hz budget(66.6ms)은 *출력 주기* 목표입니다. 파이프라인은
  stage들이 서로 다른 프레임을 동시 처리하므로 **출력은 ~59ms마다(≈17Hz, 목표 충족)**
  나오지만 **한 프레임의 E2E latency는 ~2 프레임 주기**가 됩니다 — 조립라인처럼 정상.
  그래서 latency가 66.6ms를 넘는 건 위반이 아닙니다. (그림 3의 그 선은 latency 한계가
  아니라 throughput 기준선.)
- `true_e2e_ms`의 **deadline SLO는 일부러 안 잡았습니다** — 아직 로봇이 없어서
  max-staleness budget을 정할 수 없음. 지금은 raw `true_e2e_ms` + percentile만
  기록하고, pass/fail threshold는 나중에 얹으면 됩니다.
- resource 단위는 raw sysfs (`temp*=milli-°C`, `power*=microW`).
- person은 detector worker threshold 2.0으로 차단되어 pick_logic yield tally에
  나타나지 않습니다. `reject_class_not_allowed`에 person이 보이면 worker 임계가
  되돌려진 것입니다.
- deferred (더 무거운 것, 나중에): load 중 `cyclictest`, depth/color arrival-rate,
  DDS SHM-vs-UDP health, executor→worker wake latency.
- capture window 동안 보드는 **NTP-quiet**로 유지 (cross-node age가 `CLOCK_REALTIME`을
  타므로, run 중 NTP step이 들어가면 age ladder가 오염됨).
