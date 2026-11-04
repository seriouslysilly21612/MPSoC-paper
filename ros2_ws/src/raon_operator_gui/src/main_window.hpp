// Operator console — ONE window, now FOUR tabs: 운전 (the original video +
// RViz + buttons layout), Workspace (top-down RViz + TCP numbers), Joint
// (gauge bars), 게인 (admin only, --admin). Tabs 2–4 carry their own stop
// row: the tab bar must never put 비상 정지 a click away. The button set, the
// Korean labels, the gating rules and the link-dead behaviour are carried
// over 1:1 (the gating itself lives in state.hpp so test_logic can hit it
// without Qt).
#ifndef RAON_GUI_MAIN_WINDOW_HPP
#define RAON_GUI_MAIN_WINDOW_HPP

#include <QImage>
#include <QMainWindow>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include "state.hpp"

class QComboBox;
class QLabel;
class QPushButton;
class QGroupBox;
class QPlainTextEdit;
class QTabWidget;

namespace raon_gui
{

class GainsPane;
class JointPane;
class RosLink;
class RvizPane;
class VideoPane;

struct Topics
{
  std::string image, state, key;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  // rviz_node empty (expired weak_ptr) => no RViz panes. That is the offscreen
  // test path — Ogre needs a GL context the offscreen platform cannot give —
  // and the --no-rviz escape hatch on machines with broken GL. The workspace
  // tab's top-down view is a second RenderPanel on the SAME
  // VisualizationManager (rviz_pane.hpp explains why there can only be one).
  // admin adds the 게인 tab.
  MainWindow(
    RosLink * ros, const Topics & topics,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node,
    bool admin);

  // exposed for the smoke test
  const std::map<char, QPushButton *> & buttons() const {return buttons_;}
  const std::map<std::string, QPushButton *> & classButtons() const {return class_btns_;}
  const std::vector<std::pair<char, QPushButton *>> & extraStopButtons() const
  {
    return extra_stop_btns_;
  }
  QTabWidget * tabWidget() {return tabs_;}
  GainsPane * gainsPane() {return gains_pane_;}
  JointPane * jointPane() {return joint_pane_;}
  QComboBox * modeCombo() {return mode_combo_;}
  QPushButton * modeApplyButton() {return btn_mode_apply_;}
  QPushButton * autoButton() {return btn_auto_;}
  QPushButton * cancelButton() {return btn_cancel_;}
  QPushButton * homeRecordButton() {return btn_home_rec_;}
  QLabel * resLabel() {return lbl_res_;}
  QLabel * fpsLabel() {return lbl_fps_;}
  QPlainTextEdit * robotLog() {return log_;}

public slots:
  void onFrame(QImage frame);
  void onState(raon_gui::State state);
  void onRobotMsg(QString line);
  void checkLink();     // watchdog: public so the test can force a timeout
  void forceStateStale() {last_state_ms_ = 0;}

private:
  QWidget * buildOperationTab(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node);
  QWidget * buildWorkspaceTab();
  QWidget * buildJointTab();
  QWidget * buildGainsTab();
  QWidget * makeStopRow();   // 비상 정지 + 동작 정지, duplicated on tabs 2–4
  QGroupBox * buildVideoBox();
  QGroupBox * buildRobotBox();
  QGroupBox * buildModeBox();   // 현재 모드 + dropdown 전환 + i/k
  QGroupBox * buildStopBox();
  QGroupBox * buildKeyBox(const QString & title,
    const std::vector<std::pair<char, QString>> & keys);
  QGroupBox * buildSelectBox();
  QPushButton * makeBtn(char key, const QString & text);
  void sendKey(char key);
  void sendMenuSelection(const std::string & cls);   // "" = AUTO
  void applyGating();
  void showRobotLive(const State & s);
  void showRobotDead();
  void pollTcp();

  RosLink * ros_;
  VideoPane * video_;
  QTabWidget * tabs_{nullptr};
  RvizPane * rviz_pane_{nullptr};   // primary pane; owns THE manager
  JointPane * joint_pane_{nullptr};
  GainsPane * gains_pane_{nullptr};   // nullptr unless --admin
  QLabel * lbl_tcp_{nullptr};
  std::map<char, QPushButton *> buttons_;
  // stop buttons repeated on tabs 2–4; same keys, same gating as buttons_
  // (kept out of buttons_, which the tests treat as one-button-per-key)
  std::vector<std::pair<char, QPushButton *>> extra_stop_btns_;
  // menu-selection buttons — gated on the menu, not on a fixed key
  std::map<std::string, QPushButton *> class_btns_;   // class_name → button
  QPushButton * btn_auto_{nullptr};
  QPushButton * btn_cancel_{nullptr};
  QPushButton * btn_home_rec_{nullptr};   // permanently disabled, see README
  QComboBox * mode_combo_{nullptr};
  QPushButton * btn_mode_apply_{nullptr};   // sends g/f/c per combo choice
  QLabel * lbl_mode_cur_{nullptr};
  QLabel * lbl_res_{nullptr};
  QLabel * lbl_fps_{nullptr};
  QLabel * lbl_robot_{nullptr};
  QPlainTextEdit * log_{nullptr};

  std::optional<State> state_;
  qint64 last_state_ms_{0};
  qint64 last_frame_ms_{0};
  qint64 prev_frame_ms_{0};
  double fps_ema_{0.0};
  bool fps_valid_{false};
};

}  // namespace raon_gui

#endif  // RAON_GUI_MAIN_WINDOW_HPP
