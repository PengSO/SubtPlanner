#include "subt_planner_rviz_panel/subt_planner_control.h"

#include <QPushButton>
#include <QVBoxLayout>
#include <QSizePolicy>
#include <pluginlib/class_list_macros.h>

namespace subt_planner_rviz_panel {
namespace {
constexpr int kPanelMargin = 4;
constexpr int kPanelSpacing = 4;
constexpr int kButtonHeight = 24;

bool callTrigger(ros::ServiceClient& client, const char* tag) {
  std_srvs::Trigger srv;
  if (!client.call(srv)) {
    ROS_ERROR("[%s] service call failed: %s", tag, client.getService().c_str());
    return false;
  }
  return true;
}

void configureButton(QPushButton* button) {
  button->setFixedHeight(kButtonHeight);
  button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}
}  // namespace

SubtPlannerControl::SubtPlannerControl(QWidget* parent) : rviz::Panel(parent) {
  start_client_ = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/automatic_planning");
  stop_client_ = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/stop");
  init_client_ = nh_.serviceClient<planner_msgs::pci_initialization>("pci_initialization_trigger");

  auto* layout = new QVBoxLayout;
  layout->setContentsMargins(kPanelMargin, kPanelMargin, kPanelMargin,
                             kPanelMargin);
  layout->setSpacing(kPanelSpacing);

  init_button_ = new QPushButton("Initialize UAV");
  start_button_ = new QPushButton("Start Exploration");
  stop_button_ = new QPushButton("Stop Exploration");
  configureButton(init_button_);
  configureButton(start_button_);
  configureButton(stop_button_);

  layout->addWidget(init_button_);
  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);

  setLayout(layout);

  connect(init_button_, SIGNAL(clicked()), this, SLOT(onInitialize()));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartPlanner()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopPlanner()));
}

void SubtPlannerControl::onStartPlanner() {
  callTrigger(start_client_, "SubtPlanner RViz Panel");
}

void SubtPlannerControl::onStopPlanner() {
  callTrigger(stop_client_, "SubtPlanner RViz Panel");
}

void SubtPlannerControl::onInitialize() {
  planner_msgs::pci_initialization srv;
  if (!init_client_.call(srv)) {
    ROS_ERROR("[SubtPlanner RViz Panel] service call failed: %s", init_client_.getService().c_str());
  }
}

void SubtPlannerControl::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void SubtPlannerControl::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

}  // namespace subt_planner_rviz_panel

PLUGINLIB_EXPORT_CLASS(subt_planner_rviz_panel::SubtPlannerControl, rviz::Panel)
