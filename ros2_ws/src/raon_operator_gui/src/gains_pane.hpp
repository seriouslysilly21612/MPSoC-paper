// Admin tab: per-joint Kp/Kd/KF editing over /operator_gains, with the live
// values read back from /robot_state. Shown only with --admin. The apply
// button is gated like every other command: link fresh, robot idle, AND the
// robot actually reporting gains (an old binary without the channel reports
// none — sending into the void must not look like tuning).
#ifndef RAON_GUI_GAINS_PANE_HPP
#define RAON_GUI_GAINS_PANE_HPP

#include <QWidget>
#include <vector>

#include "state.hpp"

class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace raon_gui
{

class RosLink;

class GainsPane : public QWidget
{
  Q_OBJECT

public:
  explicit GainsPane(RosLink * ros, QWidget * parent = nullptr);

  // exposed for the smoke test
  QPushButton * applyButton() {return btn_apply_;}
  QPushButton * loadButton() {return btn_load_;}
  QDoubleSpinBox * spin(int row, int col) {return spins_[row][col];}

public slots:
  void onState(raon_gui::State state);
  void setDead();

private:
  void loadFromReadback();
  void apply();

  RosLink * ros_;
  QDoubleSpinBox * spins_[6][3] = {};   // [joint][kp,kd,kf]
  QLabel * current_[6] = {};
  QLabel * status_{nullptr};
  QPushButton * btn_apply_{nullptr};
  QPushButton * btn_load_{nullptr};
  std::vector<double> rb_kp_, rb_kd_, rb_kf_;   // last readback
  bool loaded_once_{false};
};

}  // namespace raon_gui

#endif  // RAON_GUI_GAINS_PANE_HPP
