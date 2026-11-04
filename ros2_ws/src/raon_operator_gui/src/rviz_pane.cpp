#include "rviz_pane.hpp"

#include <QShowEvent>
#include <QTimer>
#include <QVBoxLayout>

#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"

namespace raon_gui
{

RvizPane::RvizPane(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node,
  const RvizOptions & options, QWidget * parent)
: QWidget(parent), ros_node_(ros_node), options_(options)
{
  setMinimumSize(280, 170);
}

void RvizPane::showEvent(QShowEvent * event)
{
  QWidget::showEvent(event);
  if (!init_scheduled_) {
    init_scheduled_ = true;
    // One event-loop turn later: the show cascade has finished and every
    // native window exists, so Ogre binds to the FINAL X window (see .hpp).
    QTimer::singleShot(0, this, [this]() {initNow();});
  }
}

void RvizPane::initNow()
{
  render_panel_ = new rviz_common::RenderPanel(this);
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(render_panel_);
  render_panel_->show();   // parent is already visible; new children are not

  // Init order is load-bearing (rviz_common's own visualizer_app does the
  // same dance): render window first, then the manager, then panel->manager,
  // then manager init + update timer.
  render_panel_->getRenderWindow()->initialize();
  auto clock = ros_node_.lock()->get_raw_node()->get_clock();
  manager_ = new rviz_common::VisualizationManager(
    render_panel_, ros_node_, nullptr /*window manager*/, clock);
  render_panel_->initialize(manager_);
  manager_->initialize();
  manager_->startUpdate();

  // A bare VisualizationManager has NO tools (stock rviz gets them from its
  // display config): without a current tool every mouse event dies in
  // handleMouseEvent and the view cannot be orbited/panned at all
  // (2026-08-05 field report). MoveCamera is the stock left-drag camera tool;
  // it drives whichever panel's ViewController the event came from, so the
  // top-down panel gets pan/zoom from the same tool.
  auto * tools = manager_->getToolManager();
  tools->setCurrentTool(tools->addTool("rviz_default_plugins/MoveCamera"));

  // The robot URDF roots at 'world' (world→link0 fixed identity); the
  // perception pipeline's frames hang under base_link, stitched to world by
  // the static TF in operator.launch.py.
  manager_->setFixedFrame("world");

  auto * grid = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
  grid->subProp("Cell Size")->setValue(0.25);
  grid->subProp("Plane Cell Count")->setValue(12);

  auto * robot =
    manager_->createDisplay("rviz_default_plugins/RobotModel", "RobotModel", true);
  robot->subProp("Description Source")->setValue("Topic");
  robot->subProp("Description Topic")->setValue("/robot_description");
  // robot_state_publisher latches robot_description as transient-local; a
  // volatile subscriber would only see it if we started first — this makes
  // start order not matter.
  robot->subProp("Description Topic")->subProp("Durability Policy")
  ->setValue("Transient Local");

  if (!options_.marker_topic.empty()) {
    auto * markers = manager_->createDisplay(
      "rviz_default_plugins/MarkerArray", "Workspace", true);
    markers->subProp("Topic")->setValue(QString::fromStdString(options_.marker_topic));
    // publisher is transient_local + publishes once at console start; the
    // display may initialize later, so the subscription must replay history
    // (same pattern as /robot_description above)
    markers->subProp("Topic")->subProp("Durability Policy")
    ->setValue("Transient Local");
  }
  if (!options_.path_topic.empty()) {
    auto * path = manager_->createDisplay(
      "rviz_default_plugins/Path", "TCP trail", true);
    path->subProp("Topic")->setValue(QString::fromStdString(options_.path_topic));
    path->subProp("Color")->setValue("255; 180; 84");
  }

  auto * views = manager_->getViewManager();
  views->setCurrentViewControllerType("rviz_default_plugins/Orbit");
  if (auto * v = views->getCurrent()) {
    v->subProp("Distance")->setValue(2.5);
    v->subProp("Pitch")->setValue(0.5);
    v->subProp("Yaw")->setValue(0.8);
    v->subProp("Focal Point")->subProp("Z")->setValue(0.4);
  }
}

// Destruction order is the constructor's reverse; the manager owns the
// displays and must die before the panel it draws into.
RvizPane::~RvizPane()
{
  delete manager_;
}

// ---------------- secondary top-down panel ----------------

RvizTopDownPanel::RvizTopDownPanel(
  RvizPane * primary, double focus_x, double focus_y, double scale,
  QWidget * parent)
: QWidget(parent), primary_(primary),
  focus_x_(focus_x), focus_y_(focus_y), scale_(scale)
{
  setMinimumSize(280, 170);
}

void RvizTopDownPanel::showEvent(QShowEvent * event)
{
  QWidget::showEvent(event);
  if (!init_scheduled_) {
    init_scheduled_ = true;
    QTimer::singleShot(0, this, [this]() {initNow();});
  }
}

void RvizTopDownPanel::initNow()
{
  // The primary pane initializes one event turn after ITS first show (tab 0,
  // so before any human reaches this tab) — but don't bet on it: retry.
  auto * mgr = primary_->manager();
  if (mgr == nullptr) {
    QTimer::singleShot(100, this, [this]() {initNow();});
    return;
  }

  render_panel_ = new rviz_common::RenderPanel(this);
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(render_panel_);
  render_panel_->show();

  // Same dance as the primary pane: create the Ogre window FIRST, then bind.
  // Without this the scene/camera set below land on an uninitialized window
  // and the panel stays blank (2026-08-05 field report: "숫자만 나오고 3D가
  // 없다"). Then CameraDisplay's second-viewport pattern: use_main_scene
  // binds this window to the ONE existing scene — never a second
  // VisualizationManager, whose ctor re-registers the global Ogre materials
  // and dies (RVIZ/Red ItemIdentityException).
  render_panel_->getRenderWindow()->initialize();
  render_panel_->initialize(mgr, true /*use_main_scene*/);

  // Private camera for this panel. create() instantiates + initializes but
  // does NOT touch the current view; add() hands ownership to the manager
  // (outlives us — never delete it here).
  auto * views = mgr->getViewManager();
  auto * v = views->create("rviz_default_plugins/TopDownOrtho");
  v->subProp("X")->setValue(focus_x_);
  v->subProp("Y")->setValue(focus_y_);
  v->subProp("Scale")->setValue(scale_);
  v->subProp("Angle")->setValue(0.0);
  views->add(v);
  render_panel_->setViewController(v);
}

}  // namespace raon_gui
