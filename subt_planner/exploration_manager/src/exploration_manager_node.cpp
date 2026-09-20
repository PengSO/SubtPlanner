
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "subt_planner/exploration_manager_ros.h"
#include "subt_planner/terminal_view.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "subt_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  bool show_startup_animation = true;
  nh_private.param("show_startup_animation", show_startup_animation, true);
  if (show_startup_animation) {
    subt_terminal::printStartupAnimation();
  }

  ExplorationManagerRos planner(nh, nh_private);

  ros::spin();

  return 0;
}
