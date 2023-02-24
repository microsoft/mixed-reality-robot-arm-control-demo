#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/String.h>
#include <shape_msgs/Mesh.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "mrirac_msgs/TrajectoryPlan.h"
#include "mrirac_msgs/MeshObstacle.h"
#include "mrirac_msgs/MeshObstacles.h"
#include "mrirac_msgs/PoseCorrectionAction.h"
#include "mrirac_lib/mrirac_lib.h"

class TrajectoryPlannerNode
{
private:
  ros::NodeHandle node_handle_;

  ros::ServiceServer trajectory_server_;
  ros::ServiceServer execute_server_;
  ros::ServiceServer clear_obstacles_server_;
  ros::ServiceServer home_service_;

  ros::Subscriber target_pose_subscriber_;
  ros::Subscriber hologram_obstacle_sub_;
  ros::Subscriber spatial_obstacle_sub_;

  ros::Publisher n_obstacles_pub_;

  const std::string kPlanningGroup_ = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  bool trajectory_planned_ = false;
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;

  std::vector<std::string> hologram_mesh_obstacles_;

  const std::string remove_string_ = "remove";

  geometry_msgs::Pose target_pose_;
  geometry_msgs::Pose pre_grasp_pose_;
  geometry_msgs::Pose grasp_pose_;

  bool simulation;

  actionlib::SimpleActionClient<mrirac_msgs::PoseCorrectionAction> pose_correction_action_client_;

  bool PlanTrajectory(mrirac_msgs::TrajectoryPlan::Request &req, mrirac_msgs::TrajectoryPlan::Response &res);
  bool ExecuteTrajectory(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool ClearObstacles(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  void TargetPoseCallback(geometry_msgs::Pose target_pose);
  void UpdateHologramObstacles(const mrirac_msgs::MeshObstacles msg);
  void UpdateSpatialObstacles(const mrirac_msgs::MeshObstacle msg);
  bool HomeArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

public:
  TrajectoryPlannerNode(const ros::NodeHandle &node_handle);
  ~TrajectoryPlannerNode();
};

TrajectoryPlannerNode::TrajectoryPlannerNode(const ros::NodeHandle &node_handle) : node_handle_(node_handle), pose_correction_action_client_("/mrirac_pose_correction/pose_correction", true), move_group_interface_(kPlanningGroup_)
{
  ros::param::param<bool>("~simulation", simulation, false);

  if (!simulation)
  {
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    pose_correction_action_client_.waitForServer(); // will wait for infinite time
    ROS_INFO("Action server started");
  }

  trajectory_server_ = node_handle_.advertiseService("plan_trajectory", &TrajectoryPlannerNode::PlanTrajectory, this);
  execute_server_ = node_handle_.advertiseService("execute_trajectory", &TrajectoryPlannerNode::ExecuteTrajectory, this);

  clear_obstacles_server_ = node_handle_.advertiseService("clear_obstacles", &TrajectoryPlannerNode::ClearObstacles, this);
  home_service_ = node_handle_.advertiseService("home_arm", &TrajectoryPlannerNode::HomeArm, this);

  target_pose_subscriber_ = node_handle_.subscribe("unity_target_pose", 100, &TrajectoryPlannerNode::TargetPoseCallback, this);
  hologram_obstacle_sub_ = node_handle_.subscribe("unity_hologram_obstacles", 100, &TrajectoryPlannerNode::UpdateHologramObstacles, this);
  spatial_obstacle_sub_ = node_handle_.subscribe("unity_spatial_obstacles", 100, &TrajectoryPlannerNode::UpdateSpatialObstacles, this);

  n_obstacles_pub_ = node_handle_.advertise<std_msgs::String>("n_obstacles", 100);
}

TrajectoryPlannerNode::~TrajectoryPlannerNode()
{
}

bool TrajectoryPlannerNode::PlanTrajectory(mrirac_msgs::TrajectoryPlan::Request &req, mrirac_msgs::TrajectoryPlan::Response &res)
{
  ROS_INFO("received pose");
  target_pose_ = req.target_pose;
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

  bool success = RobotMovements::PlanMovementToPose(req.target_pose, move_group_interface_, motion_plan);

  if (success)
  {
    res.trajectory = motion_plan.trajectory_.joint_trajectory;
    res.success = true;

    current_plan_ = motion_plan;
    trajectory_planned_ = true;
    return true;
  }
  else
  {
    res.trajectory = trajectory_msgs::JointTrajectory();
    res.success = false;
    return true;
  }
}

bool TrajectoryPlannerNode::ExecuteTrajectory(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (trajectory_planned_)
  {
    RobotMovements::ExecutePlannedTrajectory(move_group_interface_, current_plan_, target_pose_, !simulation, pose_correction_action_client_);
    trajectory_planned_ = false;
  }
  else
  {
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    RobotMovements::PlanMovementToPose(target_pose_, move_group_interface_, motion_plan);
    RobotMovements::ExecutePlannedTrajectory(move_group_interface_, motion_plan, target_pose_, !simulation, pose_correction_action_client_);
  }
  return true;
}

bool TrajectoryPlannerNode::ClearObstacles(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("clearing planning scene");
  auto obstacles = planning_scene_interface_.getObjects();

  std::vector<std::string> to_remove;

  for (auto const &obstacle : obstacles)
  {
    to_remove.push_back(obstacle.first);
  }

  planning_scene_interface_.removeCollisionObjects(to_remove);

  return true;
}

void TrajectoryPlannerNode::TargetPoseCallback(geometry_msgs::Pose target_pose)
{
  target_pose_ = target_pose;
}

void TrajectoryPlannerNode::UpdateHologramObstacles(const mrirac_msgs::MeshObstacles msg)
{

  planning_scene_interface_.removeCollisionObjects(hologram_mesh_obstacles_);

  std::vector<std::string> new_obstacles;
  hologram_mesh_obstacles_ = new_obstacles;

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  for (auto mesh_obstacle : msg.mesh_obstacles)
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_.getPlanningFrame();
    collision_object.id = mesh_obstacle.name;
    hologram_mesh_obstacles_.push_back(mesh_obstacle.name);
    collision_object.meshes.push_back(mesh_obstacle.mesh);
    collision_object.mesh_poses.push_back(mesh_obstacle.mesh_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
  }

  planning_scene_interface_.addCollisionObjects(collision_objects);

  std_msgs::String n_obstacles_msg;
  n_obstacles_msg.data = std::to_string(planning_scene_interface_.getObjects().size());
  n_obstacles_pub_.publish(n_obstacles_msg);
}

void TrajectoryPlannerNode::UpdateSpatialObstacles(const mrirac_msgs::MeshObstacle msg)
{

  if (msg.action == remove_string_)
  {
    std::vector<std::string> to_remove = {msg.name};
    planning_scene_interface_.removeCollisionObjects(to_remove);
    return;
  }

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface_.getPlanningFrame();
  collision_object.id = msg.name;
  collision_object.meshes.push_back(msg.mesh);
  collision_object.mesh_poses.push_back(msg.mesh_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  ROS_INFO("adding spatial obstacle %s", msg.name.c_str());
  planning_scene_interface_.addCollisionObjects(collision_objects);
}

bool TrajectoryPlannerNode::HomeArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("sending to home");
  move_group_interface_.setNamedTarget("Home");
  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  bool success = (move_group_interface_.plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("planning complete");
  if (success)
  {
    move_group_interface_.execute(motion_plan);
    return true;
  }
  else
  {
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrirac_trajectory_planner");
  ros::NodeHandle node_handle("~");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  TrajectoryPlannerNode node(node_handle);

  ros::waitForShutdown();

  return 0;
}
