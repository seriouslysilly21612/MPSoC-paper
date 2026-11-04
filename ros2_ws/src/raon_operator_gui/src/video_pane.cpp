#include "video_pane.hpp"

namespace raon_gui
{

VideoPane::VideoPane(QWidget * parent)
: QLabel(parent)
{
  setAlignment(Qt::AlignCenter);
  setMinimumSize(424, 240);
  setStyleSheet("background-color: #101014; color: #808088;");
  setText("waiting for the compressed stream…");
}

void VideoPane::setFrame(QImage frame)
{
  pixmap_ = QPixmap::fromImage(frame);
  rescale();
}

void VideoPane::resizeEvent(QResizeEvent * event)
{
  QLabel::resizeEvent(event);
  rescale();
}

void VideoPane::rescale()
{
  if (pixmap_.isNull()) {return;}
  // Scale for display only — never draw on a resized frame. The boxes were
  // computed in 848x480 space and the 1.767 aspect makes a single uniform
  // scale wrong for y.
  setPixmap(pixmap_.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

}  // namespace raon_gui
