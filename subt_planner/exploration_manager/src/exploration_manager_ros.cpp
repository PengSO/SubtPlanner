
#include "subt_planner/exploration_manager_ros.h"
#include "subt_planner/terminal_view.h"

ExplorationManagerRos::ExplorationManagerRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  :nh_(nh), nh_private_(nh_private)
{
  planner_service_ = nh_.advertiseService(
      "subt_planner_ros", &ExplorationManagerRos::plannerServiceCallback, this);
  
  planner_homing_service_ = nh_.advertiseService(
      "subt_planner_ros_homing", &ExplorationManagerRos::plannerHomingServiceCallback, this);
  
  subt_planner_.reset(new SubtPlanner(nh_, nh_private_));

  registerTree();
}

bool ExplorationManagerRos::plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                              planner_msgs::planner_srv::Response& res)
{
  subt_planner_->setPlannerSrvReq(req);
  
  tree_.tickOnce();

  subt_planner_->getPlannerSrvRes(res);
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kMission, "Exploration Manager", "Response sent to execution bridge: status=" + std::to_string(res.status)));

  return true;
}

bool ExplorationManagerRos::plannerHomingServiceCallback(planner_msgs::planner_homing::Request& req,
                              planner_msgs::planner_homing::Response& res)
{
  ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Requested by mission tree."));
  subt_planner_->bt_states_.homing_required = true;

  // planner_msgs::planner_srv::Request req_planner;
  // planner_msgs::planner_srv::Response res_planner;
  // req_planner.header = req.header;
  // req_planner.bound_mode = planner_msgs::BoundMode::kExtendedBound;

  // subt_planner_->setPlannerSrvReq(req_planner);
  // tree_.tickOnce();
  // subt_planner_->getPlannerSrvRes(res_planner);

  // res.path = res_planner.path;

  return true;
}

void ExplorationManagerRos::registerTree()
{
  factory_.registerNodeType<LocalExploration>("LocalExploration", subt_planner_);
  factory_.registerNodeType<GlobalExploration>("GlobalExploration", subt_planner_);
  factory_.registerNodeType<LocalExpExhaustedCheck>("LocalExpExhaustedCheck", subt_planner_);
  factory_.registerNodeType<GlobalExpExhaustedCheck>("GlobalExpExhaustedCheck", subt_planner_);
  factory_.registerNodeType<Inspection>("Inspection", subt_planner_);
  factory_.registerNodeType<CompartmentTransition>("CompartmentTransition", subt_planner_);
  factory_.registerNodeType<Homing>("Homing", subt_planner_);
  factory_.registerNodeType<HomingCheck>("HomingCheck", subt_planner_);
  factory_.registerNodeType<OPENINGPhase1>("OPENINGPhase1", subt_planner_);
  factory_.registerNodeType<OPENINGPhaseCheck>("OPENINGPhaseCheck", subt_planner_);
  factory_.registerNodeType<OPENINGPhase2>("OPENINGPhase2", subt_planner_);
  factory_.registerNodeType<LocalExpExhaustedReset>("LocalExpExhaustedReset", subt_planner_);
  factory_.registerNodeType<Idle>("Idle", subt_planner_);
  factory_.registerNodeType<OPENINGP1FailCheck>("OPENINGP1FailCheck", subt_planner_);
  factory_.registerNodeType<SetNextCompartment>("SetNextCompartment", subt_planner_);
  factory_.registerNodeType<AllCompartmentsInspectedCheck>("AllCompartmentsInspectedCheck", subt_planner_);
  factory_.registerNodeType<LocalNavigation>("LocalNavigation", subt_planner_);
  factory_.registerNodeType<LocalNavigationExhaustedCheck>("LocalNavigationExhaustedCheck", subt_planner_);
  factory_.registerNodeType<LocalNavigationExhaustedReset>("LocalNavigationExhaustedReset", subt_planner_);
  factory_.registerNodeType<CalculateHomingPath>("CalculateHomingPath", subt_planner_);
  factory_.registerNodeType<UpdateHomingGoal>("UpdateHomingGoal", subt_planner_);
  factory_.registerNodeType<SwitchToLocalNavigation>("SwitchToLocalNavigation", subt_planner_);
  factory_.registerNodeType<CalculateGlobalPath>("CalculateGlobalPath", subt_planner_);
  factory_.registerNodeType<UpdateGlobalGoal>("UpdateGlobalGoal", subt_planner_);

  std::string tree_path = ros::package::getPath("subt_planner") + "/configs/behavior_trees/main_tree.xml";
  
  if(!ros::param::get(ros::this_node::getName() + "/behavior_tree_path", tree_path))
  {
    tree_path = ros::package::getPath("subt_planner") + "/configs/behavior_trees/main_tree.xml";
  }

  std::string trial_tree_path;
  ros::param::get("~tree_path", trial_tree_path);

  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kMission, "Exploration Manager", "Behavior tree: " + tree_path));
  
  factory_.registerBehaviorTreeFromFile(tree_path);
  tree_ = factory_.createTree("MainTree");
	ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kMission, "Exploration Manager", "Behavior tree ready."));

  // BT::Groot2Publisher publisher(tree_);
}
