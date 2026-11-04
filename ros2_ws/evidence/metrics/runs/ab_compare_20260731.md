# A/B 실측: production vs SDK align_depth (2026-07-31)

교수님 지적 확인 실험. 각 300초 metrics window, 동일 장면(정적), 뷰어 OFF, 로봇 앱 OFF.

| | **A_production** | **B_aligned15** |
|---|---|---|
| 카메라 | color 848×480×**30**, depth ×15, align **OFF** | color·depth 둘 다 848×480×**15**, align **ON** |
| 3D 방식 | 단일점 reverse projection (raw depth) | SDK aligned depth 직접 픽셀 조회 |
| config | `realsense_pick_place.yaml` + `target_3d.yaml` | `realsense_aligned_15fps.yaml` + `target_3d_aligned.yaml` (launch `camera_config:=aligned target3d_config:=aligned`) |

## 결과

| 지표 | A | B | Δ |
|---|---|---|---|
| **E2E latency p50 / p95 / p99 (ms)** | **113.0** / 143.7 / 157.9 | **355.3** / 420.0 / 445.8 | **3.1×** 악화 |
| └ age_in (capture→detector, 평균) | 53.3 | **298.5** | +245 ← 악화 전량 |
| └ detector 연산 (평균) | 48.0 | 46.0 | ≈동일 |
| └ 3D 변환 (평균) | 3.7 | **2.5** | 개선 (per-pick은 aligned가 쌈) |
| **throughput (output p50)** | **17.0 Hz** | **14.2 Hz** | −16% |
| **CPU 합 (mean cores)** | **1.31** | **1.69** | +29% |
| └ camera 노드 | 0.28 | **0.91** | **+0.63 = align 비용** |
| └ detector+worker | 0.66 | 0.47 | −0.19 (처리 프레임 감소 덕) |
| sysCPU / power | 45.5% / 7.20 W | 53.8% / 7.01 W | +8.3%p / ≈ |
| **depth-color 시각차 p50 (ms)** | 52.5 | **0.0** | ← B의 유일한 실질 이점 |
| valid rate (pick/depth/base) | ≈100% 전부 | ≈100% 전부 | 동일 |

## 해석

- **레이턴시 3.1× 악화의 전량이 `age_in`**: full-frame align이 단일 스레드인 카메라
  노드에서 돌아 노드가 포화(0.91 cores) → 프레임이 detector에 닿기 전 **~300 ms
  정상상태 큐잉**(300초 내내 안정, 발산 아님; min도 185 ms = 구조적 지연).
  detector·다운스트림은 그대로 → 병목은 순수하게 카메라 노드.
- **스루풋**: A는 30fps 입력을 게이트로 17 Hz(skip 3392+overwrite 2634/14371),
  B는 15fps 입력이 그대로 상한(게이트 사실상 통과: skip 34) → 14.2 Hz.
- **B의 실질 이점**: depth가 color 시각에 정합(skew 0 ms) + 3D 변환 p99 10.1→6.9 ms.
  정적 pick 장면에선 z 정확도 영향 없음(기존 검증), 동적 물체라면 의미 있음.
- **결론**: 2026-07-14의 align 제거 결정을 정량 재확인. 레이턴시가 지배 요구인 이
  시스템에선 production 구성이 명확히 우위.

## 부산물 (이 실험이 남긴 것)

- 3D 노드 `use_aligned_depth` 파라미터 + aligned config 2종 + launch A/B 스위치
  (`camera_config`/`target3d_config`, 불일치 조합은 기동 거부).
  **→ 실험 종료 후 코드에서 원복** — 전체 구현은 커밋 `52f2731`에 보존.
  재실험 시 그 커밋에서 launch/노드/config 2종을 복원하면 된다.
- `tools/metrics/ab_one_run.sh` — 무인 run 오케스트레이터(헬스 게이트 → 300 s
  window → 단일 SIGINT 정상 종료 → 산출물 검증). production run에도 그대로 사용 가능.
- **metrics 저장 무결성 픽스**(모든 run에 적용): ① CSV 원자적 저장(temp+`os.replace`)
  ② 노드 finally 진입 시 후속 시그널 무시 ③ (수동 Ctrl-C에도 있던) 2차 SIGINT가
  finally의 재저장을 절단하던 경쟁 제거. B 1·2차 실패(base 3972→1086행 절단)의 근본 원인.

데이터: `runs/A_production/`, `runs/B_aligned15/` (각 performance/resource/yield + plots_paper 8장).
