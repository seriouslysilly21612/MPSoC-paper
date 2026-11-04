import math
import signal
import time

import rclpy
from rclpy.node import Node

from my_interfaces.msg import Detection, DetectionArray, PickTarget
from pick_logic_pkg.metrics_recorder import (
    MetricsRecorder,
    age_ms,
    write_summary_csv,
    _round,
)


class PickLogicNode(Node):
    """타깃 선택 v3 (2026-09-04): 오퍼레이터가 지정한 클래스의 인스턴스 1개.

    v1은 게이트를 통과한 첫 검출(decode 순서 = 임의)을 그대로 썼다.
    v2(2026-07-26)는 모든 후보에 점수(conf / 화면 중앙 근접 / 크기)를 매기고
    점수가 더 높은 물체가 hysteresis를 넘으면 갈아탔다 — "가장 좋은 물체를
    집는다"는 pick-and-place 시절의 발상. 앱이 오퍼레이터가 지정한 물체의
    접근 + 추종으로 바뀌어 v3에서 점수식과 자동(AUTO) 모드를 삭제했다:

      L1 게이트     — 불변: confidence / class / bbox 크기 / edge / 면적.
      L2 선택       — `desired_class`는 필수. 로봇측 오퍼레이터 메뉴가 세팅하는
                      LIVE 파라미터(`ros2 param set /pick_logic_node
                      desired_class apple`)이며 ''이면 invalid로 발행
                      (사유 no_class_selected). 후보 = L1을 통과한 그 클래스의
                      검출. 트랙이 없으면 confidence 최대 후보를 채택한다.
      L3 안정성     — 채택된 인스턴스가 stability_min_frames 연속 보여야
                      target_valid, 그 뒤엔 고정(sticky): hysteresis도 "더 좋은
                      후보" 전환도 없다. 타깃이 바뀌는 경우는 둘뿐이다.
                      (a) lost_timeout_frames를 넘겨 연속으로 사라졌을 때
                      (N+1번째 연속 미검출 프레임에서 해제, 이후 새 채택과
                      새 워밍업) (b) desired_class가 바뀌었을 때(트랙 리셋).
                      이어받기 = 최근접 후보. 같은 클래스 인스턴스가 2개
                      이상일 때의 규칙과, 실험은 클래스당 1개인데도 그 규칙을
                      두는 이유는 _update_track에 적었다.

    출력 계약은 불변: DetectionArray 1건당 PickTarget 1건, target_valid
    true/false(invalid 발행 여부는 publish_invalid_targets). v1 이후 추가된
    reject 사유: no_class_selected, desired_class_not_visible, target_unstable.
    """

    def __init__(self):
        super().__init__('pick_logic_node')

        self.declare_parameter('input_topic', '/detections')
        self.declare_parameter('output_topic', '/pick_target')

        self.declare_parameter('min_confidence', 0.3)
        self.declare_parameter('allowed_classes',
                               ['apple', 'orange', 'banana',
                                'tennis_ball', 'mustard_bottle'])

        self.declare_parameter('image_width', 848)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('edge_margin_px', 30)

        self.declare_parameter('min_bbox_area_px', 400.0)
        self.declare_parameter('max_bbox_area_ratio', 0.5)

        # --- 선택 / 안정성 ---
        self.declare_parameter('desired_class', '')       # LIVE: '' = 선택 없음

        self.declare_parameter('stability_min_frames', 3)
        self.declare_parameter('stability_match_px', 60.0)
        self.declare_parameter('lost_timeout_frames', 5)

        self.declare_parameter('publish_invalid_targets', True)
        self.declare_parameter('log_period_sec', 1.0)

        # Metrics(기본 off). performance CSV = DetectionArray 1건당 1행,
        # yield CSV = 종료 시 reject 집계 1행. 둘 다 종료 시 한 번만 쓰고
        # 프레임마다 I/O는 없다 — vitis_ai_detector_node와 같은 방식.
        self.declare_parameter('metrics_csv_path', '')
        self.declare_parameter('metrics_duration_sec', 0.0)
        self.declare_parameter('yield_csv_path', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.allowed_classes = list(self.get_parameter('allowed_classes').value)

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.edge_margin_px = int(self.get_parameter('edge_margin_px').value)

        self.min_bbox_area_px = float(self.get_parameter('min_bbox_area_px').value)
        self.max_bbox_area_ratio = float(self.get_parameter('max_bbox_area_ratio').value)

        self.stability_min_frames = int(self.get_parameter('stability_min_frames').value)
        self.stability_match_px = float(self.get_parameter('stability_match_px').value)
        self.lost_timeout_frames = int(self.get_parameter('lost_timeout_frames').value)

        self.publish_invalid_targets = bool(
            self.get_parameter('publish_invalid_targets').value
        )
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)

        self.last_accept_log_time_ns = None
        self.last_reject_log_time_ns = None

        # --- 런타임 상태 ---
        self._desired_class_cached = str(self.get_parameter('desired_class').value)
        # 현재 타깃 트랙: dict(cx, cy, stable, missing) 또는 None.
        # 클래스는 항상 desired_class(바뀌면 리셋).
        self._track = None

        self.metrics = MetricsRecorder(
            self.get_logger(),
            str(self.get_parameter('metrics_csv_path').value),
            float(self.get_parameter('metrics_duration_sec').value),
        )
        self.yield_csv_path = str(self.get_parameter('yield_csv_path').value)
        self.frames_total = 0
        self.frames_accepted = 0
        self.frames_empty = 0
        self.reject_tally = {}

        self.subscription = self.create_subscription(
            DetectionArray,
            input_topic,
            self.detections_callback,
            10
        )

        self.publisher = self.create_publisher(
            PickTarget,
            output_topic,
            10
        )

        self.get_logger().info(
            f'Pick logic node (v3) started: {input_topic} -> {output_topic}'
        )

    # ------------------------------------------------------------------ #
    # 메인 callback
    # ------------------------------------------------------------------ #

    def detections_callback(self, msg: DetectionArray):
        compute_start_ns = time.perf_counter_ns()
        # DetectionArray.header.stamp가 원본 color 캡처 시각이다(detector가
        # 전파). 모든 PickTarget에 실어 보내야 하류 노드가 진짜 E2E 지연을
        # 계산할 수 있다.
        capture_stamp = msg.header.stamp
        in_age = (age_ms(self.get_clock().now().nanoseconds, capture_stamp)
                  if self.metrics.enabled else None)
        self.frames_total += 1

        # desired_class는 LIVE 파라미터(로봇측 오퍼레이터 메뉴가 세팅).
        # 매 프레임 다시 읽고, 바뀌면 추적 상태를 리셋한다.
        desired = str(self.get_parameter('desired_class').value)
        if desired != self._desired_class_cached:
            self.get_logger().info(
                f"desired_class: '{self._desired_class_cached}' -> '{desired}'"
                f' — tracking state reset')
            self._desired_class_cached = desired
            self._reset_track()

        if len(msg.detections) == 0:
            self.frames_empty += 1
            self._step_track_missing()
            self._finish_invalid('empty_detection_array', capture_stamp,
                                 compute_start_ns, in_age, 0)
            return

        # v3: 선택된 클래스가 없으면 따라갈 것도 없다. 오퍼레이터가 'p' 메뉴로
        # 먼저 고르고, 단독 실행은 launch에 desired_class:=<class>를 넘긴다.
        if not desired:
            self._finish_invalid('no_class_selected', capture_stamp,
                                 compute_start_ns, in_age, len(msg.detections))
            return

        # L1 게이트 -> 후보 목록 (desired_class만 살아남는다)
        candidates = []
        first_reject_reason = None
        first_rejected_detection = None
        for det in msg.detections:
            accepted, reason = self.is_detection_acceptable(det)
            if accepted and det.class_name != desired:
                accepted, reason = False, 'desired_class_not_visible'
            if accepted:
                candidates.append(det)
            elif first_reject_reason is None:
                first_reject_reason = reason  # accepted되지 않은 물체 중 conf가 가장 높은 물체의 reject 사유가 저장됨
                first_rejected_detection = det

        if not candidates:
            self._step_track_missing()
            reason = first_reject_reason or 'no_acceptable_detection'
            self._finish_invalid(reason, capture_stamp, compute_start_ns,
                                 in_age, len(msg.detections),
                                 first_rejected_detection)
            return

        # L2 + L3: 인스턴스 1개를 채택/이어받기(sticky 트랙). None =
        # 추적 중인 인스턴스가 이번 프레임에 없음(아직 lost_timeout 안),
        # stable < min_frames(= 3frames) = 워밍업 중.
        det = self._update_track(candidates)
        if det is None or self._track['stable'] < self.stability_min_frames:
            self._finish_invalid('target_unstable', capture_stamp,
                                 compute_start_ns, in_age,
                                 len(msg.detections),
                                 det if det is not None else candidates[0])
            return

        target = self.make_pick_target(det)
        target.target_valid = True
        target.capture_stamp = capture_stamp
        self.publisher.publish(target)
        self.frames_accepted += 1

        if self.should_log_accept():
            self.get_logger().info(
                f'Accepted target: class={target.class_name}, '
                f'conf={target.confidence:.2f}, '
                f'cx={target.center_x:.1f}, cy={target.center_y:.1f}, '
                f'stable={self._track["stable"]}, '
                f'candidates={len(msg.detections)}, desired={desired}'
            )
        self._record_metrics(capture_stamp, compute_start_ns, in_age,
                             n_candidates=len(msg.detections),
                             target_valid=True)

    # ------------------------------------------------------------------ #
    # 트랙 helper
    # ------------------------------------------------------------------ #

    def _reset_track(self):
        self._track = None

    # lost_timeout_frames까지는 버티고, 그 다음(N+1번째) 연속 미검출
    # 프레임에서 트랙 해제
    def _step_track_missing(self):
        if self._track is None:
            return
        self._track['missing'] += 1
        if self._track['missing'] > self.lost_timeout_frames:
            self._reset_track()

    def _update_track(self, candidates):
        """이번 프레임에서 desired_class 인스턴스 1개를 채택하거나 이어받는다.

        candidates = 이번 프레임에 L1을 통과한 desired_class 검출(비어 있지
        않음). 발행할 Detection을 돌려주고, 추적 중인 인스턴스가 이번 프레임에
        없으면(아직 lost_timeout 안) None.

        같은 클래스 인스턴스가 2개 이상일 때의 동작을 여기서 정의한다.

        실험 프로토콜은 클래스당 물체 1개다. 그런데도 2개 이상을 정의해 두는
        이유는, "클래스당 1개"가 테이블 배치(장면)의 가정이지 검출기의 보장이
        아니기 때문이다. /detections에 같은 클래스 박스가 2개 오는 일은 배치와
        무관하게 생긴다:
          - banana처럼 긴 물체가 두 박스로 쪼개져 NMS(IoU 0.45)를 둘 다 통과
          - 배경의 비슷한 물체가 같은 클래스로 오검출
        규칙 없이 decode 순서를 따르면 타깃 중심이 프레임마다 두 박스 사이를
        오가고, 브리지(v2)엔 jump 게이트가 없어 TCP가 그 사이를 왕복한다.
        사람이 옆에 서 있는 추종 앱이라 정의되지 않은 동작은 남기지 않는다.

        규칙: 채택 = confidence 최대 / 이어받기 = 최근접 (match_px 거리
        게이트는 2개 이상 보일 때만) / 상실 전까지 전환 없음. appearance
        기반 re-identification은 하지 않는다 — 프로토콜이 클래스당 1개라
        이것으로 충분하고, 그 이상은 이 노드의 범위가 아니다.
        """
        # 1. 트랙 없음 -> confidence 최대 채택
        if self._track is None:
            best = max(candidates, key=lambda d: float(d.confidence))
            self._track = {'cx': float(best.center_x), 'cy': float(best.center_y),
                           'stable': 1, 'missing': 0}
            return best

        # 2. 직전 프레임 타깃의 bbox와 현재 후보들의 bbox간 거리를 구함
        # 만약 후보가 하나라면 그대로
        # 만약 후보가 두개 이상일 때 경합(qa 문서 참조)
        # dist = 거리, cont = Detection msg
        dist, cont = min(
            ((math.hypot(float(d.center_x) - self._track['cx'],
                         float(d.center_y) - self._track['cy']), d)
             for d in candidates),
            key=lambda t: t[0])
        
        if len(candidates) > 1 and dist > self.stability_match_px:
            cont = None

        # 3. 타깃이 이번 프레임에 없음 — lost_timeout 넘기면 처음부터
        if cont is None:
            self._step_track_missing()
            if self._track is None:      # 타임아웃 -> 새로 채택
                return self._update_track(candidates)
            return None                  # 대기 (이번 프레임 invalid)

        # 4. 추적 계속 (sticky: 더 좋은 후보가 나와도 갈아타지 않는다)
        self._track.update({'cx': float(cont.center_x),
                            'cy': float(cont.center_y),
                            'stable': self._track['stable'] + 1,
                            'missing': 0})
        return cont

    def _finish_invalid(self, reason, capture_stamp, compute_start_ns,
                        in_age, n_candidates, det=None):
        self._tally(reason)
        self.publish_invalid_target(reason, capture_stamp, det)
        self._record_metrics(capture_stamp, compute_start_ns, in_age,
                             n_candidates=n_candidates, target_valid=False)

    # ------------------------------------------------------------------ #
    # 배관: metrics / 발행 / L1 게이트 / 로깅 / yield
    # ------------------------------------------------------------------ #

    # reject 사유 집계
    def _tally(self, reason):
        self.reject_tally[reason] = self.reject_tally.get(reason, 0) + 1

    def _record_metrics(self, capture_stamp, compute_start_ns, in_age,
                        n_candidates, target_valid):
        if not self.metrics.enabled:
            return
        now_ns = self.get_clock().now().nanoseconds
        compute_ms = (time.perf_counter_ns() - compute_start_ns) / 1e6
        self.metrics.add({
            'capture_sec': capture_stamp.sec,
            'capture_nanosec': capture_stamp.nanosec,
            'pick_logic_in_age_ms': _round(in_age),
            'pick_logic_compute_ms': _round(compute_ms),
            'pick_logic_out_age_ms': _round(age_ms(now_ns, capture_stamp)),
            'n_candidates': n_candidates,
            'target_valid': int(target_valid),
        })

    def publish_invalid_target(self, reason, capture_stamp, det=None):
        target = PickTarget()
        target.target_valid = False
        target.capture_stamp = capture_stamp

        if det is not None:
            target.class_id = det.class_id
            target.class_name = det.class_name
            target.confidence = det.confidence
            target.center_x = det.center_x
            target.center_y = det.center_y
            target.width = det.width
            target.height = det.height

        if self.should_log_reject():
            self.get_logger().warn(
                f'Rejected detection array: reason={reason}'
            )

        if self.publish_invalid_targets:
            self.publisher.publish(target)

    def make_pick_target(self, msg: Detection):
        target = PickTarget()
        target.class_id = msg.class_id
        target.class_name = msg.class_name
        target.confidence = msg.confidence
        target.center_x = msg.center_x
        target.center_y = msg.center_y
        target.width = msg.width
        target.height = msg.height
        return target

    def is_detection_acceptable(self, msg: Detection):
        # min_confidence = 0.3 (yaml). worker가 이미 0.50으로 걸러 주므로
        # 이 게이트는 배포 경로에선 통과만 시킨다 — /detections를 다른
        # 소스(합성테스트·리플레이)가 쏠 때의 방어선이다.
        if msg.confidence < self.min_confidence:
            return False, 'confidence_below_threshold'

        # allowed_classes 밖의 클래스 제외 (5종 화이트리스트)
        if self.allowed_classes and msg.class_name not in self.allowed_classes:
            return False, 'class_not_allowed'

        if msg.width <= 0.0 or msg.height <= 0.0:
            return False, 'invalid_bbox_size'

        left = msg.center_x - msg.width / 2.0
        right = msg.center_x + msg.width / 2.0
        top = msg.center_y - msg.height / 2.0
        bottom = msg.center_y + msg.height / 2.0

        # edge_margin_px = 30px
        if left < self.edge_margin_px:
            return False, 'bbox_too_close_to_left_edge'

        if right > self.image_width - self.edge_margin_px:
            return False, 'bbox_too_close_to_right_edge'

        if top < self.edge_margin_px:
            return False, 'bbox_too_close_to_top_edge'

        if bottom > self.image_height - self.edge_margin_px:
            return False, 'bbox_too_close_to_bottom_edge'

        bbox_area = float(msg.width * msg.height)
        image_area = float(self.image_width * self.image_height)
        max_bbox_area = image_area * self.max_bbox_area_ratio

        # min_bbox_area_px = 400
        if bbox_area < self.min_bbox_area_px:
            return False, 'bbox_area_too_small'

        # max_bbox_area = 0.5 * image_area
        if bbox_area > max_bbox_area:
            return False, 'bbox_area_too_large'

        return True, 'accepted'

    def should_log_accept(self):
        should_log, now_ns = self.should_log(self.last_accept_log_time_ns)
        if should_log:
            self.last_accept_log_time_ns = now_ns
        return should_log

    def should_log_reject(self):
        should_log, now_ns = self.should_log(self.last_reject_log_time_ns)
        if should_log:
            self.last_reject_log_time_ns = now_ns
        return should_log

    def should_log(self, last_log_time_ns):
        now_ns = self.get_clock().now().nanoseconds

        if last_log_time_ns is None:
            return True, now_ns

        elapsed_sec = (now_ns - last_log_time_ns) / 1e9
        return elapsed_sec >= self.log_period_sec, now_ns

    def write_yield_summary(self):
        if not self.yield_csv_path:
            return
        total = max(self.frames_total, 1)
        row = {
            'node': 'pick_logic',
            'frames_total': self.frames_total,
            'frames_accepted': self.frames_accepted,
            'target_valid_rate': round(self.frames_accepted / total, 4),
            'frames_empty': self.frames_empty,
        }
        for reason, count in sorted(self.reject_tally.items()):
            row[f'reject_{reason}'] = count
        write_summary_csv(self.get_logger(), self.yield_csv_path, [row])


def main(args=None):
    rclpy.init(args=args)
    node = PickLogicNode()

    # SIGTERM(launch 에스컬레이션, pkill)도 Ctrl-C처럼 동작시켜 finally 블록이
    # 종료 시 metrics/yield CSV를 쓰게 한다.
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 2차 시그널 차단: 그룹 Ctrl-C + launch 자체 전파로 SIGINT가 한 번 더
        # 들어와 finally 안의 CSV 재저장을 도중 절단시켰다(2026-07-31 실증).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.metrics.save()
        node.write_yield_summary()
        node.destroy_node()
        # SIGINT 시 rclpy의 전역 signal handler가 이미 context를 shutdown 시켰으므로
        # 여기서 또 부르면 "rcl_shutdown already called" 예외가 난다. ok()로 가드.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
