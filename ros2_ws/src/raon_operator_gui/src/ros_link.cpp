#include "ros_link.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace raon_gui
{

// Keyed by class_name, never class_id: the id ordering comes from
// decode_meta.json and would silently mis-label if it ever changed.
// (Duplicated with detection_viewer_pkg on purpose — see README §8.)
static const std::map<std::string, cv::Scalar> kClassColors = {
  {"apple", {60, 60, 220}}, {"orange", {0, 150, 255}},
  {"banana", {60, 220, 220}}, {"tennis_ball", {60, 220, 60}},
  {"mustard_bottle", {200, 140, 60}},
};
static const cv::Scalar kUnknownColor{200, 200, 200};

static int64_t stampNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

RosLink::RosLink(
  const std::string & image_topic, const std::string & detections_topic,
  const std::string & key_topic, const std::string & state_topic,
  const std::string & msg_topic)
: rclcpp::Node("raon_operator_gui")
{
  // BEST_EFFORT on the image: the publisher is RELIABLE (compatible), and
  // retransmitting a frame we are about to overwrite only adds latency.
  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  // RELIABLE for the small topics — and for the outgoing keys: a command that
  // moves an arm must not be silently dropped by the transport.
  auto small_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  sub_image_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    image_topic, image_qos,
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {onImage(m);});
  sub_dets_ = create_subscription<my_interfaces::msg::DetectionArray>(
    detections_topic, small_qos,
    [this](my_interfaces::msg::DetectionArray::ConstSharedPtr m) {onDetections(m);});
  sub_state_ = create_subscription<std_msgs::msg::String>(
    state_topic, small_qos,
    [this](std_msgs::msg::String::ConstSharedPtr m) {onState(m);});
  // depth 20 to match the publisher: menu/gate lines are events, keep them all
  sub_msg_ = create_subscription<std_msgs::msg::String>(
    msg_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) {
      emit signals_.robotMsg(QString::fromStdString(m->data));
    });
  pub_key_ = create_publisher<std_msgs::msg::String>(key_topic, small_qos);
  pub_gains_ = create_publisher<std_msgs::msg::String>("/operator_gains", small_qos);
  pub_jog_ = create_publisher<std_msgs::msg::String>("/operator_jog", small_qos);

  // transient_local + publish once: the RViz Marker display initializes only
  // when the Workspace tab is first shown, and still gets the box replayed.
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/operator_ws_markers", rclcpp::QoS(1).transient_local());
  publishWorkspaceMarkers();
  pub_path_ = create_publisher<nav_msgs::msg::Path>(
    "/operator_tcp_path", rclcpp::QoS(1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
}

void RosLink::publishWorkspaceMarkers()
{
  // Copied from the robot app's WorkspaceBox.h (kv260::WS_BOX / WS_RMAX_XY,
  // 2026-07-27 hand-eye box, E21 re-base) — one side changes, change the
  // other. Frame 'world' == base_link (identity static TF in the launch).
  static constexpr double kBox[6] = {0.30, 0.94, -0.37, 0.63, 0.10, 0.50};
  static constexpr double kRmaxXY = 0.92;

  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker box;
  box.header.frame_id = "world";
  box.ns = "workspace";
  box.id = 0;
  box.type = visualization_msgs::msg::Marker::CUBE;
  box.pose.position.x = (kBox[0] + kBox[1]) / 2.0;
  box.pose.position.y = (kBox[2] + kBox[3]) / 2.0;
  box.pose.position.z = (kBox[4] + kBox[5]) / 2.0;
  box.pose.orientation.w = 1.0;
  box.scale.x = kBox[1] - kBox[0];
  box.scale.y = kBox[3] - kBox[2];
  box.scale.z = kBox[5] - kBox[4];
  box.color.r = 0.24f; box.color.g = 0.86f; box.color.b = 0.52f;
  box.color.a = 0.15f;   // goals only — keep the robot visible through it
  arr.markers.push_back(box);

  visualization_msgs::msg::Marker ring;
  ring.header.frame_id = "world";
  ring.ns = "workspace";
  ring.id = 1;
  ring.type = visualization_msgs::msg::Marker::LINE_STRIP;
  ring.pose.orientation.w = 1.0;
  ring.scale.x = 0.008;   // line width
  ring.color.r = 0.63f; ring.color.g = 0.63f; ring.color.b = 0.66f;
  ring.color.a = 0.8f;
  for (int i = 0; i <= 48; ++i) {
    const double a = 2.0 * M_PI * i / 48.0;
    geometry_msgs::msg::Point pt;
    pt.x = kRmaxXY * std::cos(a);
    pt.y = kRmaxXY * std::sin(a);
    pt.z = 0.005;
    ring.points.push_back(pt);
  }
  arr.markers.push_back(ring);

  pub_markers_->publish(arr);
}

void RosLink::sendGains(
  const std::vector<double> & kp, const std::vector<double> & kd,
  const std::vector<double> & kf)
{
  if (kp.size() != 6 || kd.size() != 6 || kf.size() != 6) {return;}
  char buf[512];
  int len = snprintf(buf, sizeof(buf), "{");
  const struct {const char * key; const std::vector<double> * v;} parts[] =
  {{"kp", &kp}, {"kd", &kd}, {"kf", &kf}};
  for (int p = 0; p < 3 && len < static_cast<int>(sizeof(buf)); ++p) {
    len += snprintf(buf + len, sizeof(buf) - len, "%s\"%s\":[",
      p ? "," : "", parts[p].key);
    for (int i = 0; i < 6 && len < static_cast<int>(sizeof(buf)); ++i) {
      len += snprintf(buf + len, sizeof(buf) - len, "%s%.3f",
        i ? "," : "", (*parts[p].v)[i]);
    }
    if (len < static_cast<int>(sizeof(buf))) {
      len += snprintf(buf + len, sizeof(buf) - len, "]");
    }
  }
  if (len < static_cast<int>(sizeof(buf))) {
    len += snprintf(buf + len, sizeof(buf) - len, "}");
  }
  if (len >= static_cast<int>(sizeof(buf))) {return;}
  std_msgs::msg::String msg;
  msg.data = buf;
  pub_gains_->publish(msg);
  RCLCPP_INFO(get_logger(), "gains -> /operator_gains: %s", buf);
}

void RosLink::sendJog(int axis, double q_rad)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"axis\":%d,\"q\":%.4f}", axis, q_rad);
  std_msgs::msg::String msg;
  msg.data = buf;
  pub_jog_->publish(msg);
}

bool RosLink::tcpPose(double out[6])
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform("world", "tcp", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;   // tree not up yet — the label shows "waiting"
  }
  out[0] = tf.transform.translation.x;
  out[1] = tf.transform.translation.y;
  out[2] = tf.transform.translation.z;
  const auto & r = tf.transform.rotation;
  tf2::Matrix3x3(tf2::Quaternion(r.x, r.y, r.z, r.w))
  .getRPY(out[3], out[4], out[5]);

  // TCP trail: ~60 s at the caller's 5 Hz. Republishing the whole path each
  // sample is how the rviz Path display expects updates.
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "world";
  ps.header.stamp = now();
  ps.pose.position.x = out[0];
  ps.pose.position.y = out[1];
  ps.pose.position.z = out[2];
  ps.pose.orientation = r;
  tcp_path_.poses.push_back(ps);
  if (tcp_path_.poses.size() > 300) {
    tcp_path_.poses.erase(tcp_path_.poses.begin());
  }
  tcp_path_.header = ps.header;
  pub_path_->publish(tcp_path_);
  return true;
}

void RosLink::sendKey(char key)
{
  std_msgs::msg::String msg;
  msg.data = std::string(1, key);
  pub_key_->publish(msg);
  RCLCPP_INFO(get_logger(), "key '%c' -> %s", key, pub_key_->get_topic_name());
}

void RosLink::onDetections(my_interfaces::msg::DetectionArray::ConstSharedPtr msg)
{
  last_dets_ = msg;
}

void RosLink::onState(std_msgs::msg::String::ConstSharedPtr msg)
{
  State s;
  if (parseState(msg->data, s)) {
    emit signals_.stateReady(s);
  } else {
    // a parse failure is version skew worth shouting about, not crashing over
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "bad /robot_state JSON (app/console version skew?)");
  }
}

void RosLink::onImage(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  if (frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "imdecode failed");
    return;
  }
  const int64_t key = stampNs(msg->header.stamp);
  // Monotonic guard: a reordered/duplicate image must not redraw an older
  // frame over a newer one, which reads as a stutter.
  if (have_rendered_ && key <= last_rendered_ns_) {return;}
  have_rendered_ = true;
  last_rendered_ns_ = key;

  // Boxes older than kMaxBoxAgeS relative to THIS frame are not drawn: a
  // stalled detector must go visibly blank rather than leave its last boxes
  // frozen on live video.
  const my_interfaces::msg::DetectionArray * dets = nullptr;
  if (last_dets_ &&
    (key - stampNs(last_dets_->header.stamp)) <= static_cast<int64_t>(kMaxBoxAgeS * 1e9))
  {
    dets = last_dets_.get();
  }

  if (dets) {
    for (const auto & det : dets->detections) {
      // Coords are already pixels in THIS frame — the worker undid the
      // 416x416 letterbox. Never rescale here.
      const int xmin = static_cast<int>(std::lround(det.center_x - det.width / 2.0));
      const int ymin = static_cast<int>(std::lround(det.center_y - det.height / 2.0));
      const int xmax = static_cast<int>(std::lround(det.center_x + det.width / 2.0));
      const int ymax = static_cast<int>(std::lround(det.center_y + det.height / 2.0));
      const auto it = kClassColors.find(det.class_name);
      const cv::Scalar color = (it != kClassColors.end()) ? it->second : kUnknownColor;
      cv::rectangle(frame, {xmin, ymin}, {xmax, ymax}, color, 2);
      char label[64];
      snprintf(label, sizeof(label), "%s %.2f", det.class_name.c_str(), det.confidence);
      int baseline = 0;
      const cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      const int ytop = std::max(0, ymin - ts.height - 4);
      cv::rectangle(frame, {xmin, ytop}, {xmin + ts.width + 4, ytop + ts.height + 4}, color, -1);
      cv::putText(frame, label, {xmin + 2, ytop + ts.height},
        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 0}, 1, cv::LINE_AA);
    }
  }

  // Format_BGR888 means NO channel swap — the JPEG payload really is BGR
  // (verified in the field; a cvtColor here turns the orange blue).
  // .copy() detaches the QImage from the cv::Mat buffer before it dies.
  emit signals_.frameReady(
    QImage(frame.data, frame.cols, frame.rows,
    static_cast<int>(frame.step), QImage::Format_BGR888).copy());
}

}  // namespace raon_gui
