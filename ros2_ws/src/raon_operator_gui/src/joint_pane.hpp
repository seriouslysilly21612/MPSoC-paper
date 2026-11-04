// Joint-space tab, patterned on the lab's UiCtrlNG.py rows:
//   [Jn] [−] [========slider========] [+] [target] [actual]
// The slider is a JOG commander: releasing it (or ±1°) publishes an absolute
// per-axis target on /operator_jog; the robot walks its reference there at
// 0.5 rad/s. Enabled ONLY when the bridge would accept it anyway (FullDyn/CT
// mode + ctrl + servo + idle) — the UI mirrors the gate, the robot enforces
// it. The actual column is the live q from /robot_state, coloured by limit
// proximity.
#ifndef RAON_GUI_JOINT_PANE_HPP
#define RAON_GUI_JOINT_PANE_HPP

#include <QWidget>

#include "state.hpp"

class QLabel;
class QPushButton;
class QSlider;

namespace raon_gui
{

class RosLink;

class JointPane : public QWidget
{
  Q_OBJECT

public:
  explicit JointPane(RosLink * ros, QWidget * parent = nullptr);

  // exposed for the smoke test
  QSlider * slider(int i) {return sliders_[i];}
  QPushButton * plusButton(int i) {return btn_plus_[i];}

public slots:
  void onState(raon_gui::State state);   // fresh /robot_state
  void setDead();                        // link watchdog fired

private:
  void sendAxis(int i);                  // slider value → /operator_jog
  void setJogEnabled(bool on);

  RosLink * ros_;
  QSlider * sliders_[6] = {};
  QPushButton * btn_minus_[6] = {};
  QPushButton * btn_plus_[6] = {};
  QLabel * target_[6] = {};
  QLabel * actual_[6] = {};
  QLabel * hint_{nullptr};
  bool jog_enabled_{false};
  bool seeded_{false};   // sliders take the live posture on (re)enable
};

}  // namespace raon_gui

#endif  // RAON_GUI_JOINT_PANE_HPP
