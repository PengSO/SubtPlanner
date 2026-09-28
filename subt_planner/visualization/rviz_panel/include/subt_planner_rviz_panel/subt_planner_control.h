#ifndef SUBT_PLANNER_UI_SUBT_PLANNER_CONTROL_H
#define SUBT_PLANNER_UI_SUBT_PLANNER_CONTROL_H

#include <planner_msgs/pci_initialization.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#ifndef Q_MOC_RUN
#include <QPushButton>
#include <QVBoxLayout>
#include <rviz/panel.h>
#endif

class QPushButton;

namespace subt_planner_rviz_panel {

class SubtPlannerControl : public rviz::Panel {
  Q_OBJECT
 public:
  explicit SubtPlannerControl(QWidget* parent = nullptr);

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

 public Q_SLOTS:
  void onStartPlanner();
  void onStopPlanner();
  void onInitialize();

 private:
  QPushButton* init_button_ = nullptr;
  QPushButton* start_button_ = nullptr;
  QPushButton* stop_button_ = nullptr;

  ros::ServiceClient init_client_;
  ros::ServiceClient start_client_;
  ros::ServiceClient stop_client_;

  ros::NodeHandle nh_;
};

}  // namespace subt_planner_rviz_panel

#endif  // SUBT_PLANNER_UI_SUBT_PLANNER_CONTROL_H
