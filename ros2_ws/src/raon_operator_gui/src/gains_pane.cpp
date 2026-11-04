#include "gains_pane.hpp"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "ros_link.hpp"

namespace raon_gui
{

// Spinbox ceilings mirror the bridge's clamps (OnGains) — the UI must not
// offer what the robot will clip. Kf's 12 Nm is the controller's own clamp.
static constexpr double kMax[3] = {2000.0, 200.0, 12.0};
static constexpr double kStep[3] = {10.0, 1.0, 0.1};
static constexpr int kDecimals[3] = {1, 1, 2};

GainsPane::GainsPane(RosLink * ros, QWidget * parent)
: QWidget(parent), ros_(ros)
{
  auto * outer = new QVBoxLayout(this);
  auto * grid = new QGridLayout;
  const char * heads[] = {"", "Kp", "Kd", "KF [Nm]", "현재 (Kp / Kd / KF)"};
  for (int c = 0; c < 5; ++c) {
    auto * h = new QLabel(heads[c]);
    h->setStyleSheet("color:#808088;");
    grid->addWidget(h, 0, c);
  }
  for (int r = 0; r < 6; ++r) {
    grid->addWidget(new QLabel(QString("J%1").arg(r + 1)), r + 1, 0);
    for (int c = 0; c < 3; ++c) {
      auto * sp = new QDoubleSpinBox;
      sp->setRange(0.0, kMax[c]);
      sp->setSingleStep(kStep[c]);
      sp->setDecimals(kDecimals[c]);
      spins_[r][c] = sp;
      grid->addWidget(sp, r + 1, c + 1);
    }
    current_[r] = new QLabel("–");
    current_[r]->setStyleSheet("color:#808088; font-family:monospace;");
    grid->addWidget(current_[r], r + 1, 4);
  }
  outer->addLayout(grid);

  auto * row = new QHBoxLayout;
  btn_load_ = new QPushButton("현재값 불러오기");
  btn_load_->setToolTip("로봇이 보고한 현재 게인을 입력칸으로 복사");
  connect(btn_load_, &QPushButton::clicked, this, [this]() {loadFromReadback();});
  btn_apply_ = new QPushButton("적용");
  btn_apply_->setToolTip(
    "필요: 링크 정상 + 유휴(비추종·비busy) + 로봇의 게인 보고.\n"
    "브리지가 범위 clamp 후 다음 1 kHz 사이클에 반영");
  btn_apply_->setStyleSheet(
    "background-color:#1d4f8c; color:#ffffff; font-weight:bold;");
  connect(btn_apply_, &QPushButton::clicked, this, [this]() {apply();});
  row->addWidget(btn_load_);
  row->addWidget(btn_apply_);
  outer->addLayout(row);

  status_ = new QLabel;
  status_->setWordWrap(true);
  outer->addWidget(status_);
  outer->addStretch(1);

  setDead();   // starts gated off, like everything else
}

void GainsPane::onState(State state)
{
  rb_kp_ = state.kp;
  rb_kd_ = state.kd;
  rb_kf_ = state.kf;
  const bool reported = rb_kp_.size() == 6 && rb_kd_.size() == 6 && rb_kf_.size() == 6;

  for (int r = 0; r < 6; ++r) {
    current_[r]->setText(
      reported ?
      QString::asprintf("%7.1f / %6.1f / %5.2f", rb_kp_[r], rb_kd_[r], rb_kf_[r]) :
      QStringLiteral("–"));
  }
  // first readback seeds the editors — applying untouched zeros by accident
  // would drop every gain to 0 on a live arm
  if (reported && !loaded_once_) {
    loadFromReadback();
  }

  btn_load_->setEnabled(reported);
  const bool idle = !state.track && !state.busy;
  btn_apply_->setEnabled(reported && idle);
  if (!reported) {
    status_->setText(
      "<span style='color:#ffb454'>로봇이 게인을 보고하지 않음 — "
      "/operator_gains 미지원(구 바이너리)이므로 전송 차단</span>");
  } else if (!idle) {
    status_->setText("<span style='color:#ffb454'>추종/동작 중 — 유휴 상태에서만 적용 가능</span>");
  } else {
    status_->setText("<span style='color:#808088'>적용 시 브리지가 clamp 후 즉시 반영</span>");
  }
}

void GainsPane::setDead()
{
  btn_apply_->setEnabled(false);
  btn_load_->setEnabled(false);
  status_->setText(
    "<span style='color:#ff5f56; font-weight:bold'>NO LINK</span>"
    "<span style='color:#808088'> — /robot_state 응답 없음</span>");
}

void GainsPane::loadFromReadback()
{
  if (rb_kp_.size() != 6 || rb_kd_.size() != 6 || rb_kf_.size() != 6) {return;}
  for (int r = 0; r < 6; ++r) {
    spins_[r][0]->setValue(rb_kp_[r]);
    spins_[r][1]->setValue(rb_kd_[r]);
    spins_[r][2]->setValue(rb_kf_[r]);
  }
  loaded_once_ = true;
}

void GainsPane::apply()
{
  std::vector<double> kp(6), kd(6), kf(6);
  for (int r = 0; r < 6; ++r) {
    kp[r] = spins_[r][0]->value();
    kd[r] = spins_[r][1]->value();
    kf[r] = spins_[r][2]->value();
  }
  ros_->sendGains(kp, kd, kf);
  status_->setText(
    "<span style='color:#3ddc84'>전송됨 — 적용/거부 결과는 로봇 메시지 로그와 '현재' 열 확인</span>");
}

}  // namespace raon_gui
