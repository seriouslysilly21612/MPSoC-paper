// The video widget. GUI-thread only. Port of the former video_pane.py.
#ifndef RAON_GUI_VIDEO_PANE_HPP
#define RAON_GUI_VIDEO_PANE_HPP

#include <QLabel>
#include <QPixmap>

namespace raon_gui
{

class VideoPane : public QLabel
{
  Q_OBJECT

public:
  explicit VideoPane(QWidget * parent = nullptr);

public slots:
  void setFrame(QImage frame);

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  void rescale();
  QPixmap pixmap_;
};

}  // namespace raon_gui

#endif  // RAON_GUI_VIDEO_PANE_HPP
