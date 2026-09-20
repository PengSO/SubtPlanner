#ifndef SUBT_PLANNER_UI_SUBT_PLANNER_CONTROL_H
#define SUBT_PLANNER_UI_SUBT_PLANNER_CONTROL_H

#include <planner_msgs/pci_global.h>
#include <planner_msgs/pci_initialization.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#ifndef Q_MOC_RUN
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <rviz/panel.h>
#endif

class QLineEdit;
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
  void onRunSingle();
  void onStopPlanner();
  void onHome();
  void onInitialize();
  void onPlanToFrontier();
  void onRunGlobal();
  void onToggleMode();

 private:
  QPushButton* start_button_ = nullptr;
  QPushButton* single_button_ = nullptr;
  QPushButton* stop_button_ = nullptr;
  QPushButton* home_button_ = nullptr;
  QPushButton* init_button_ = nullptr;
  QPushButton* frontier_button_ = nullptr;
  QPushButton* global_button_ = nullptr;
  QPushButton* mode_button_ = nullptr;
  QLineEdit* frontier_id_edit_ = nullptr;

  ros::ServiceClient start_client_;
  ros::ServiceClient single_client_;
  ros::ServiceClient stop_client_;
  ros::ServiceClient home_client_;
  ros::ServiceClient init_client_;
  ros::ServiceClient frontier_client_;
  ros::ServiceClient global_client_;
  ros::ServiceClient mode_client_;

  bool waypoint_mode_ = false;
  ros::NodeHandle nh_;
};

}  // namespace subt_planner_rviz_panel

#endif  // SUBT_PLANNER_UI_SUBT_PLANNER_CONTROL_H
