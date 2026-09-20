#include "subt_planner_rviz_panel/subt_planner_control.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>
#include <stdexcept>

namespace subt_planner_rviz_panel {
namespace {
bool callTrigger(ros::ServiceClient& client, const char* tag) {
  std_srvs::Trigger srv;
  if (!client.call(srv)) {
    ROS_ERROR("[%s] service call failed: %s", tag, client.getService().c_str());
    return false;
  }
  return true;
}
}  // namespace

SubtPlannerControl::SubtPlannerControl(QWidget* parent) : rviz::Panel(parent) {
  start_client_ = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/automatic_planning");
  single_client_ = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/single_planning");
  stop_client_ = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/stop");
  home_client_ = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/homing_trigger");
  init_client_ = nh_.serviceClient<planner_msgs::pci_initialization>("pci_initialization_trigger");
  frontier_client_ = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/go_to_waypoint");
  global_client_ = nh_.serviceClient<planner_msgs::pci_global>("pci_global");
  mode_client_ = nh_.serviceClient<std_srvs::SetBool>("subt_planner/switch_operation_mode");

  auto* layout = new QVBoxLayout;

  auto* title = new QLabel("Exploration Task Control");
  title->setStyleSheet("font-weight: 600; font-size: 14px; margin-bottom: 6px;");

  start_button_ = new QPushButton("Start Exploration");
  single_button_ = new QPushButton("Plan One Step");
  stop_button_ = new QPushButton("Stop Exploration");
  home_button_ = new QPushButton("Return Home");
  init_button_ = new QPushButton("Initialize Vehicle");
  frontier_button_ = new QPushButton("Go to Frontier");
  global_button_ = new QPushButton("Run Repositioning");
  mode_button_ = new QPushButton("Mode: Exploration");

  layout->addWidget(title);
  layout->addWidget(start_button_);
  layout->addWidget(single_button_);
  layout->addWidget(stop_button_);
  layout->addWidget(home_button_);
  layout->addWidget(init_button_);
  layout->addWidget(frontier_button_);

  auto* global_row = new QHBoxLayout;
  global_row->addWidget(new QLabel("Frontier ID:"));
  frontier_id_edit_ = new QLineEdit;
  global_row->addWidget(frontier_id_edit_);
  global_row->addWidget(global_button_);
  layout->addLayout(global_row);
  layout->addWidget(mode_button_);

  setLayout(layout);

  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartPlanner()));
  connect(single_button_, SIGNAL(clicked()), this, SLOT(onRunSingle()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopPlanner()));
  connect(home_button_, SIGNAL(clicked()), this, SLOT(onHome()));
  connect(init_button_, SIGNAL(clicked()), this, SLOT(onInitialize()));
  connect(frontier_button_, SIGNAL(clicked()), this, SLOT(onPlanToFrontier()));
  connect(global_button_, SIGNAL(clicked()), this, SLOT(onRunGlobal()));
  connect(mode_button_, SIGNAL(clicked()), this, SLOT(onToggleMode()));
}

void SubtPlannerControl::onStartPlanner() {
  callTrigger(start_client_, "SubtPlanner RViz Panel");
}

void SubtPlannerControl::onRunSingle() {
  callTrigger(single_client_, "SubtPlanner RViz Panel");
}

void SubtPlannerControl::onStopPlanner() {
  callTrigger(stop_client_, "SubtPlanner RViz Panel");
}

void SubtPlannerControl::onHome() {
  callTrigger(home_client_, "SubtPlanner RViz Panel");
}

void SubtPlannerControl::onInitialize() {
  planner_msgs::pci_initialization srv;
  if (!init_client_.call(srv)) {
    ROS_ERROR("[SubtPlanner RViz Panel] service call failed: %s", init_client_.getService().c_str());
  }
}

void SubtPlannerControl::onPlanToFrontier() {
  callTrigger(frontier_client_, "SubtPlanner RViz Panel");
}

void SubtPlannerControl::onRunGlobal() {
  const std::string text = frontier_id_edit_->text().toStdString();
  int id = 0;

  if (!text.empty()) {
    try {
      id = std::stoi(text);
    } catch (const std::exception&) {
      ROS_ERROR("[SubtPlanner RViz Panel] invalid frontier id: %s", text.c_str());
      return;
    }
  }

  if (id < 0) {
    ROS_ERROR("[SubtPlanner RViz Panel] frontier id must be non-negative");
    return;
  }

  planner_msgs::pci_global srv;
  srv.request.id = id;
  if (!global_client_.call(srv)) {
    ROS_ERROR("[SubtPlanner RViz Panel] service call failed: %s", global_client_.getService().c_str());
  }
}

void SubtPlannerControl::onToggleMode() {
  waypoint_mode_ = !waypoint_mode_;
  std_srvs::SetBool srv;
  srv.request.data = waypoint_mode_;
  if (!mode_client_.call(srv)) {
    ROS_ERROR("[SubtPlanner RViz Panel] service call failed: %s", mode_client_.getService().c_str());
  }
  mode_button_->setText(waypoint_mode_ ? "Mode: Waypoint" : "Mode: Exploration");
}

void SubtPlannerControl::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void SubtPlannerControl::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

}  // namespace subt_planner_rviz_panel

PLUGINLIB_EXPORT_CLASS(subt_planner_rviz_panel::SubtPlannerControl, rviz::Panel)
