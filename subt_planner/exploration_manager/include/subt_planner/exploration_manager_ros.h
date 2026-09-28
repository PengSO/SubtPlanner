#pragma once

#include <ros/package.h>
#include "subt_planner/exploration_coordinator.h"
#include "subt_planner/exploration_state_nodes.h"

class ExplorationManagerRos
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer planner_service_;
  ros::ServiceServer planner_homing_service_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<SubtPlanner> subt_planner_;
public:
  ExplorationManagerRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  bool plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                              planner_msgs::planner_srv::Response& res);
  bool plannerHomingServiceCallback(planner_msgs::planner_homing::Request& req,
                              planner_msgs::planner_homing::Response& res);

  void registerTree();

};
