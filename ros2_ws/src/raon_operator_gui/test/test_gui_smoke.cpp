// Build the real window offscreen (--no-rviz path: Ogre has no GL there),
// drive it through the gating states, click every button, count what came out
// on /operator_key. C++ port of the former test_gui_smoke.py — same phases.
// Run:  QT_QPA_PLATFORM=offscreen ./build/raon_operator_gui/test_gui_smoke
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QImage>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QTabWidget>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "../src/gains_pane.hpp"
#include "../src/joint_pane.hpp"
#include "../src/main_window.hpp"
#include "../src/ros_link.hpp"
#include "../src/state.hpp"

using raon_gui::State;

static State full()
{
  State s;
  s.q = {0, 0, 0, 0, 0, 0};
  s.servo = s.ctrl = s.grav = s.home = s.lock = s.menu = true;
  return s;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<raon_gui::RosLink>(
    "/img", "/detections", "/operator_key", "/robot_state");

  QApplication app(argc, argv);
  qRegisterMetaType<QImage>();
  qRegisterMetaType<raon_gui::State>();

  raon_gui::MainWindow w(
    node.get(), {"/img", "/robot_state", "/operator_key"},
    std::weak_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>(),
    true /*admin — the gains tab is asserted below*/);

  std::vector<std::string> seen;
  auto echo = std::make_shared<rclcpp::Node>("echo");
  auto sub = echo->create_subscription<std_msgs::msg::String>(
    "/operator_key", 10,
    [&seen](std_msgs::msg::String::ConstSharedPtr m) {seen.push_back(m->data);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(echo);

  const auto & btns = w.buttons();
  assert(btns.size() == 19);   // 2 stop + 17 letters; digits → class buttons,
                               // g/f/c → the mode dropdown (tested below)

  // 1) before any /robot_state: EVERYTHING disabled, E-STOP included —
  //    the per-tab stop copies too
  for (const auto & [k, b] : btns) {assert(!b->isEnabled());}
  assert(w.extraStopButtons().size() == 6);   // 3 non-운전 tabs × (e + x)
  for (const auto & [k, b] : w.extraStopButtons()) {assert(!b->isEnabled());}

  // frames drive the labels (two: FPS is an interval)
  w.onFrame(QImage(848, 480, QImage::Format_BGR888));
  assert(w.resLabel()->text() == "848 × 480");
  w.onFrame(QImage(848, 480, QImage::Format_BGR888));
  assert(w.fpsLabel()->text() != "NO SIGNAL");

  // 2) fully permissive: all enabled; click all, one spin burst per click
  // (/operator_key QoS is depth 1 on purpose — a burst legitimately drops)
  w.onState(full());
  for (const auto & [k, b] : btns) {assert(b->isEnabled());}
  for (const auto & [k, b] : btns) {
    b->click();
    for (int i = 0; i < 6; ++i) {exec.spin_some(std::chrono::milliseconds(20));}
  }
  assert(seen.size() == btns.size());
  const char * allowed = "rhjgfcitkbpvoxensmadlw0123456789";
  for (const auto & k : seen) {
    assert(k.size() == 1 && std::strchr(allowed, k[0]) != nullptr);
    assert(k != "q");
  }

  // 3) tracking: exactly DoInput's whitelist stays enabled
  State tr = full();
  tr.track = true;
  w.onState(tr);
  const char * track_ok = "ojexgp";
  for (const auto & [k, b] : btns) {
    assert(b->isEnabled() == (std::strchr(track_ok, k) != nullptr));
  }

  // 4) no lock → v/o die
  State s4 = full();
  s4.menu = false;
  s4.lock = false;
  w.onState(s4);
  assert(!btns.at('v')->isEnabled() && !btns.at('o')->isEnabled() &&
    btns.at('h')->isEnabled());

  // 4b) object-selection gating + digit mapping. Menu open with two classes
  // (menu order = bridge numbering): only those two light up; clicks send the
  // POSITION digit; AUTO = N+1; 취소 = 0; HOME 기록 never enables.
  State sm = full();
  sm.menu_items = {"apple", "tennis_ball"};
  w.onState(sm);
  assert(w.classButtons().at("apple")->isEnabled());
  assert(w.classButtons().at("tennis_ball")->isEnabled());
  assert(!w.classButtons().at("orange")->isEnabled());
  assert(!w.classButtons().at("banana")->isEnabled());
  assert(!w.classButtons().at("mustard_bottle")->isEnabled());
  assert(w.autoButton()->isEnabled() && w.cancelButton()->isEnabled());
  assert(!w.homeRecordButton()->isEnabled());

  seen.clear();
  auto click_spin = [&](QPushButton * b) {
      b->click();
      for (int i = 0; i < 6; ++i) {exec.spin_some(std::chrono::milliseconds(20));}
    };
  click_spin(w.classButtons().at("apple"));
  click_spin(w.classButtons().at("tennis_ball"));
  click_spin(w.autoButton());
  click_spin(w.cancelButton());
  assert((seen == std::vector<std::string>{"1", "2", "3", "0"}));

  // menu closed again → every selection control dies
  w.onState(s4);
  for (const auto & [cls, b] : w.classButtons()) {assert(!b->isEnabled());}
  assert(!w.autoButton()->isEnabled() && !w.cancelButton()->isEnabled());

  // 5) robot link dead: everything off again — but the QUIT button (outside
  // buttons_) must survive: closing a dead console is always legitimate.
  w.forceStateStale();
  w.checkLink();
  for (const auto & [k, b] : btns) {assert(!b->isEnabled());}
  {
    int quit_alive = 0;
    for (auto * b : w.findChildren<QPushButton *>()) {
      if (b->text() == "콘솔 종료") {quit_alive += b->isEnabled() ? 1 : 0;}
    }
    assert(quit_alive == 1);
  }

  // 6) /operator_msg lines land in the log pane, menu multi-line intact
  w.onRobotMsg("[PickBridge] ---- visible objects ----\n[PickBridge]   1) apple");
  assert(w.robotLog()->toPlainText().contains("1) apple"));

  // 7) tabs: --admin gives 운전/Workspace/Joint/게인; the per-tab stop pairs
  //    gate exactly like the main pair and publish the same keys
  assert(w.tabWidget()->count() == 4);
  w.onState(full());
  for (const auto & [k, b] : w.extraStopButtons()) {assert(b->isEnabled());}
  seen.clear();
  assert(w.extraStopButtons()[0].first == 'e');
  click_spin(w.extraStopButtons()[0].second);
  assert((seen == std::vector<std::string>{"e"}));

  // 8) gains tab: blocked until the robot REPORTS gains (old binary = no
  //    readback = no send); readback seeds the editors; idle-only apply;
  //    apply publishes ONE JSON line on /operator_gains
  raon_gui::GainsPane * gp = w.gainsPane();
  assert(gp != nullptr);
  assert(!gp->applyButton()->isEnabled());   // full() reports no gains
  State g = full();
  g.kp = {800, 600, 400, 200, 150, 100};
  g.kd = {80, 60, 40, 20, 15, 10};
  g.kf = {8, 6, 4, 2, 1.5, 1};
  w.onState(g);
  assert(gp->applyButton()->isEnabled() && gp->loadButton()->isEnabled());
  assert(gp->spin(0, 0)->value() == 800.0 && gp->spin(5, 2)->value() == 1.0);
  State gb = g;
  gb.busy = true;
  w.onState(gb);
  assert(!gp->applyButton()->isEnabled());   // mirror of the bridge's idle gate
  w.onState(g);
  std::vector<std::string> gains_seen;
  auto gsub = echo->create_subscription<std_msgs::msg::String>(
    "/operator_gains", 10,
    [&gains_seen](std_msgs::msg::String::ConstSharedPtr m) {
      gains_seen.push_back(m->data);
    });
  gp->applyButton()->click();
  for (int i = 0; i < 6; ++i) {exec.spin_some(std::chrono::milliseconds(20));}
  assert(gains_seen.size() == 1);
  assert(gains_seen[0].find("\"kp\":[800.000") != std::string::npos);
  assert(gains_seen[0].find("\"kd\":[") != std::string::npos);
  assert(gains_seen[0].find("\"kf\":[") != std::string::npos);

  // 8b) mode dropdown: 전환 sends the selected mode's KEY, gated per key —
  //     'g' is always-valid, 'f' dies while tracking (DoInput's whitelist)
  w.onState(full());
  seen.clear();
  w.modeCombo()->setCurrentIndex(1);   // 풀 다이내믹스 [f]
  assert(w.modeApplyButton()->isEnabled());
  click_spin(w.modeApplyButton());
  assert((seen == std::vector<std::string>{"f"}));
  State trm = full();
  trm.track = true;
  w.onState(trm);
  assert(!w.modeApplyButton()->isEnabled());   // 'f' swallowed while tracking
  w.modeCombo()->setCurrentIndex(0);           // 중력보상 [g]
  assert(w.modeApplyButton()->isEnabled());    // 'g' always valid

  // 8c) joint jog: sliders live ONLY in FullDyn(1)/CT(2) + ctrl+servo+idle;
  //     releasing a slider publishes the axis target on /operator_jog
  std::vector<std::string> jog_seen;
  auto jsub = echo->create_subscription<std_msgs::msg::String>(
    "/operator_jog", 10,
    [&jog_seen](std_msgs::msg::String::ConstSharedPtr m) {
      jog_seen.push_back(m->data);
    });
  w.onState(full());   // mode -1 → jog dead even fully permissive
  assert(!w.jointPane()->slider(0)->isEnabled());
  State jog = full();
  jog.mode = 2;        // ComputedTorque
  w.onState(jog);
  assert(w.jointPane()->slider(0)->isEnabled());
  auto * sl = w.jointPane()->slider(0);
  sl->setSliderDown(true);
  sl->setValue(30);
  sl->setSliderDown(false);   // emits sliderReleased → publish
  for (int i = 0; i < 6; ++i) {exec.spin_some(std::chrono::milliseconds(20));}
  assert(jog_seen.size() == 1);
  assert(jog_seen[0].find("\"axis\":0") != std::string::npos);
  assert(jog_seen[0].find("\"q\":0.5236") != std::string::npos);   // 30°
  State jog_busy = jog;
  jog_busy.busy = true;
  w.onState(jog_busy);
  assert(!w.jointPane()->slider(0)->isEnabled());   // idle-only mirror

  // 9) stale link also kills the per-tab stops, the apply button and jog
  w.forceStateStale();
  w.checkLink();
  for (const auto & [k, b] : w.extraStopButtons()) {assert(!b->isEnabled());}
  assert(!gp->applyButton()->isEnabled());
  assert(!w.jointPane()->slider(0)->isEnabled());

  // 10) without --admin the gains tab does not exist at all
  raon_gui::MainWindow w2(
    node.get(), {"/img", "/robot_state", "/operator_key"},
    std::weak_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>(),
    false);
  assert(w2.tabWidget()->count() == 3);
  assert(w2.gainsPane() == nullptr);

  printf("PASS: 19 key buttons + object selection; init/full/tracking/menu/"
    "dead gating match DoInput; digit mapping 1/2/AUTO/cancel exact; "
    "no 'q'; HOME-record dead; quit ungated; robot log live; res/fps live; "
    "4 tabs w/ admin (3 without); per-tab stops gate+publish; gains tab "
    "readback-gated, idle-gated, publishes JSON; mode dropdown sends g/f/c "
    "per-key gated; jog sliders mode(1|2)+idle gated, publish axis targets\n");
  rclcpp::shutdown();
  return 0;
}
