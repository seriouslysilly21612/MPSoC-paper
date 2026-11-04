// ROS 2 side of the operator console — runs OFF the Qt GUI thread.
//
// Everything expensive (JPEG decode, bbox drawing) happens on the executor
// thread; only a finished QImage / parsed State crosses into the GUI thread
// via queued Qt signals. Qt widgets are never touched from here. sendKey()
// goes the other way and is the exception that proves it: rclcpp publish is
// thread-safe and touches no Qt.
//
// This is the C++ port of the former Python ros_link.py; the QoS choices and
// the bbox conventions are carried over verbatim (they came from
// detection_viewer_pkg, see docs/vision/desktop_viewer_plan.md §4).
#ifndef RAON_GUI_ROS_LINK_HPP
#define RAON_GUI_ROS_LINK_HPP

#include <QImage>
#include <QObject>
#include <memory>
#include <string>

#include "my_interfaces/msg/detection_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "state.hpp"

namespace raon_gui
{

// The only bridge from ROS threads to the GUI thread. Qt delivers these as
// queued connections, so slots run on the GUI thread even though emit()
// happens on the executor thread. QImage is implicitly shared + deep-copied
// at creation, so passing it by value across threads is safe.
class RosSignals : public QObject
{
  Q_OBJECT

public:
signals:
  void frameReady(QImage frame);
  void stateReady(raon_gui::State state);
  void robotMsg(QString line);   // /operator_msg — the robot's terminal lines
};

class RosLink : public rclcpp::Node
{
public:
  RosLink(
    const std::string & image_topic, const std::string & detections_topic,
    const std::string & key_topic, const std::string & state_topic,
    const std::string & msg_topic = "/operator_msg");

  // "signals" itself is a Qt macro, hence the short name
  RosSignals * sig() {return &signals_;}

  // One character to the robot app. Called from the GUI thread.
  void sendKey(char key);

  // Admin gains → /operator_gains as one JSON line ({"kp":[6],...}); the
  // bridge clamps and idle-gates before anything reaches the RT loop.
  // Called from the GUI thread (publish is thread-safe, like sendKey).
  void sendGains(
    const std::vector<double> & kp, const std::vector<double> & kd,
    const std::vector<double> & kf);

  // Joint-tab jog → /operator_jog ({"axis":N,"q":rad}). The bridge gates
  // (FullDyn/CT + servo + idle) and clamps; the RT side walks q_ref to the
  // target at 0.5 rad/s. Deliberately not logged — a drag is a stream.
  void sendJog(int axis, double q_rad);

  // Latest world→tcp as x,y,z [m] + roll,pitch,yaw [rad]. False until the
  // TF tree exists (robot app + robot_state_publisher up). GUI thread; the
  // tf2 buffer takes its own lock. Each successful lookup also extends the
  // TCP trail on /operator_tcp_path (the RViz trajectory display) — the
  // window's 5 Hz poll is the one sampling clock, so it lives here.
  bool tcpPose(double out[6]);

private:
  void onImage(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void onDetections(my_interfaces::msg::DetectionArray::ConstSharedPtr msg);
  void onState(std_msgs::msg::String::ConstSharedPtr msg);

  RosSignals signals_;

  void publishWorkspaceMarkers();

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<my_interfaces::msg::DetectionArray>::SharedPtr sub_dets_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_key_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_gains_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_jog_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

  // world→tcp for the workspace tab; the listener's subscriptions ride our
  // executor, lookups come from the GUI thread (buffer is internally locked)
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::Path tcp_path_;   // GUI-thread-only (tcpPose caller)

  // Callback-thread-only state (the node's default callback group is
  // mutually exclusive, so callbacks never run concurrently — no lock).
  my_interfaces::msg::DetectionArray::ConstSharedPtr last_dets_;
  bool have_rendered_{false};
  int64_t last_rendered_ns_{0};   // monotonic guard vs reordered images

  static constexpr double kMaxBoxAgeS = 0.5;
};

}  // namespace raon_gui

Q_DECLARE_METATYPE(raon_gui::State)

#endif  // RAON_GUI_ROS_LINK_HPP
