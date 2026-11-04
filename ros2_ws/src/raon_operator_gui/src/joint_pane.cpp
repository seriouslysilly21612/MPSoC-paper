#include "joint_pane.hpp"

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <cmath>

#include "ros_link.hpp"

namespace raon_gui
{

// urdf/indy7_mesh.urdf limits: J1–J5 ±3.0543 rad (175°), J6 ±3.7525 rad
// (215°). The robot side clamps to the same numbers (bridge + controller).
static constexpr int kLimitDeg[6] = {175, 175, 175, 175, 175, 215};
// same palette as the status badges in main_window.cpp
static const char * kOkStyle = "color:#3ddc84; font-family:monospace;";
static const char * kWarnStyle = "color:#ffb454; font-family:monospace;";
static const char * kBadStyle = "color:#ff5f56; font-family:monospace;";
static const char * kDimStyle = "color:#808088;";

JointPane::JointPane(RosLink * ros, QWidget * parent)
: QWidget(parent), ros_(ros)
{
  auto * outer = new QVBoxLayout(this);
  auto * grid = new QGridLayout;
  const char * heads[] = {"", "", "", "", "Target", "Actual"};
  for (int c = 0; c < 6; ++c) {
    auto * h = new QLabel(heads[c]);
    h->setStyleSheet(kDimStyle);
    grid->addWidget(h, 0, c);
  }
  for (int i = 0; i < 6; ++i) {
    grid->addWidget(new QLabel(QString("J%1").arg(i + 1)), i + 1, 0);

    btn_minus_[i] = new QPushButton("−");
    btn_minus_[i]->setFixedWidth(32);
    connect(btn_minus_[i], &QPushButton::clicked, this, [this, i]() {
        sliders_[i]->setValue(sliders_[i]->value() - 1);
        sendAxis(i);
      });
    grid->addWidget(btn_minus_[i], i + 1, 1);

    sliders_[i] = new QSlider(Qt::Horizontal);
    sliders_[i]->setRange(-kLimitDeg[i], kLimitDeg[i]);
    sliders_[i]->setSingleStep(1);
    // live target label while dragging; the jog is sent on RELEASE — the
    // robot walks at 0.5 rad/s, so a commit per gesture beats a stream of
    // half-dragged targets
    connect(sliders_[i], &QSlider::valueChanged, this, [this, i](int v) {
        target_[i]->setText(QString::asprintf("%+4d°", v));
      });
    connect(sliders_[i], &QSlider::sliderReleased, this, [this, i]() {
        sendAxis(i);
      });
    grid->addWidget(sliders_[i], i + 1, 2);

    btn_plus_[i] = new QPushButton("+");
    btn_plus_[i]->setFixedWidth(32);
    connect(btn_plus_[i], &QPushButton::clicked, this, [this, i]() {
        sliders_[i]->setValue(sliders_[i]->value() + 1);
        sendAxis(i);
      });
    grid->addWidget(btn_plus_[i], i + 1, 3);

    target_[i] = new QLabel("–");
    target_[i]->setStyleSheet("font-family:monospace;");
    target_[i]->setMinimumWidth(56);
    grid->addWidget(target_[i], i + 1, 4);

    actual_[i] = new QLabel("–");
    actual_[i]->setStyleSheet(kDimStyle);
    actual_[i]->setMinimumWidth(72);
    grid->addWidget(actual_[i], i + 1, 5);
  }
  grid->setColumnStretch(2, 1);
  outer->addLayout(grid);

  hint_ = new QLabel;
  hint_->setWordWrap(true);
  outer->addWidget(hint_);
  outer->addStretch(1);

  setDead();
}

void JointPane::sendAxis(int i)
{
  if (!jog_enabled_) {return;}
  ros_->sendJog(i, sliders_[i]->value() * M_PI / 180.0);
}

void JointPane::setJogEnabled(bool on)
{
  jog_enabled_ = on;
  for (int i = 0; i < 6; ++i) {
    sliders_[i]->setEnabled(on);
    btn_minus_[i]->setEnabled(on);
    btn_plus_[i]->setEnabled(on);
  }
}

void JointPane::onState(State state)
{
  const bool have = state.q.size() >= 6;
  for (int i = 0; i < 6 && have; ++i) {
    const double deg = state.q[i] * 180.0 / M_PI;
    actual_[i]->setText(QString::asprintf("%+8.1f°", deg));
    const double lim = kLimitDeg[i];
    actual_[i]->setStyleSheet(
      std::abs(deg) > lim - 15 ? kBadStyle :
      (std::abs(deg) > lim - 45 ? kWarnStyle : kOkStyle));
  }

  // mirror of the bridge's OnJog gate — the robot still enforces it
  const bool allow = have && state.ctrl && state.servo && !state.track &&
    !state.busy && (state.mode == 1 || state.mode == 2);
  if (allow && !jog_enabled_) {
    // (re)entering jog: sliders take the live posture so the first drag
    // starts from where the arm actually is, not a stale target
    for (int i = 0; i < 6; ++i) {
      sliders_[i]->setValue(
        static_cast<int>(std::lround(state.q[i] * 180.0 / M_PI)));
    }
    seeded_ = true;
  }
  setJogEnabled(allow);

  if (allow) {
    hint_->setText(
      "<span style='color:#3ddc84'>jog 가능</span>"
      "<span style='color:#808088'> — 슬라이더를 놓는 순간 해당 축이 0.5 rad/s로 이동</span>");
  } else if (state.mode != 1 && state.mode != 2) {
    hint_->setText(
      "<span style='color:#ffb454'>jog 비활성 — 모드가 풀 다이내믹스('f') 또는 "
      "계산 토크('c')일 때만</span>"
      "<span style='color:#808088'> (+ 컨트롤러/서보 ON, 유휴)</span>");
  } else {
    hint_->setText(
      "<span style='color:#ffb454'>jog 비활성 — 컨트롤러('r')/서보('h') ON + "
      "유휴 상태 필요</span>");
  }
}

void JointPane::setDead()
{
  setJogEnabled(false);
  for (int i = 0; i < 6; ++i) {
    actual_[i]->setText("–");
    actual_[i]->setStyleSheet(kDimStyle);
  }
  hint_->setText(
    "<span style='color:#ff5f56; font-weight:bold'>NO LINK</span>"
    "<span style='color:#808088'> — /robot_state 응답 없음</span>");
}

}  // namespace raon_gui
