// Entry point. Qt owns the main thread; our executor thread spins ONLY our
// RosLink node. The rviz node is NEVER added to it: VisualizationManager owns
// a SingleThreadedExecutor of its own — it add_node()s the rviz node in its
// constructor and spin_some()s it from the 30 Hz update timer
// (rviz_common/visualization_manager.cpp:159,252,406). Adding the same node
// to a second executor corrupts the shared wait set and segfaults on startup
// (field-diagnosed 2026-08-04); this is also why stock rviz2's main never
// spins its node.
//
//   ros2 launch raon_operator_gui operator.launch.py     (the normal way —
//        also starts robot_state_publisher, without which the RViz robot
//        has no TF and renders nothing)
//   operator_console --no-rviz                           (video+buttons only,
//        no GL needed; what the offscreen smoke test exercises)
//   ros2 launch raon_operator_gui operator.launch.py admin:=true
//        (adds the Kp/Kd/KF tab; plain runs never show it)

#include <QApplication>
#include <csignal>
#include <cstring>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

#include "main_window.hpp"
#include "ros_link.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  raon_gui::Topics topics{
    "/camera/camera/color/image_raw/compressed", "/robot_state", "/operator_key"};
  std::string detections_topic = "/detections";
  bool with_rviz = true;
  bool admin = false;   // --admin: adds the Kp/Kd/KF tab (see gains_pane.hpp)
  for (int i = 1; i < argc; ++i) {
    auto arg = [&](const char * name) -> const char * {
        return (std::strcmp(argv[i], name) == 0 && i + 1 < argc) ? argv[++i] : nullptr;
      };
    if (std::strcmp(argv[i], "--no-rviz") == 0) {with_rviz = false;} else if (std::strcmp(argv[i], "--admin") == 0) {admin = true;} else if (const char * v = arg("--image-topic")) {topics.image = v;} else if (const char * v = arg("--detections-topic")) {detections_topic = v;} else if (const char * v = arg("--key-topic")) {topics.key = v;} else if (const char * v = arg("--state-topic")) {topics.state = v;}
  }

  auto node = std::make_shared<raon_gui::RosLink>(
    topics.image, detections_topic, topics.key, topics.state);

  QApplication app(argc, argv);
  // Qt swallows SIGINT while its event loop runs; the default handler makes
  // Ctrl-C in the launching terminal work as expected.
  std::signal(SIGINT, SIG_DFL);

  qRegisterMetaType<QImage>();
  qRegisterMetaType<raon_gui::State>();

  // Created BEFORE the window (RvizPane needs it), destroyed after app.exec.
  // ONE abstraction for the ONE VisualizationManager — the workspace tab's
  // top-down view is a second RenderPanel on the same manager, not a second
  // manager (rviz_pane.hpp records the RVIZ/Red crash a second one causes).
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node;
  if (with_rviz) {
    rviz_node =
      std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_embed");
  }

  raon_gui::MainWindow window(node.get(), topics, rviz_node, admin);
  window.show();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);   // ONLY ours — see the header comment
  std::thread spin_thread([&executor]() {executor.spin();});

  const int rc = app.exec();

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return rc;
}
