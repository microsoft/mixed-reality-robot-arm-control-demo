#ifndef MRIRAC_LIB_H
#define MRIRAC_LIB_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "mrirac_msgs/PickAndPlaceAction.h"
#include "mrirac_msgs/PoseCorrectionAction.h"

class RobotMovements
{
public:
    static bool GripperAction(double turn, actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> *finger_client);

    static bool PickAndPlaceMovement(const geometry_msgs::Pose &target_pose,
                                     moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                     actionlib::SimpleActionServer<mrirac_msgs::PickAndPlaceAction> &pick_and_place_server,
                                     bool use_pose_correction,
                                     actionlib::SimpleActionClient<mrirac_msgs::PoseCorrectionAction> &pose_correction_client);

    static bool PlanMovementToPose(const geometry_msgs::Pose &target_pose,
                                   moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                   moveit::planning_interface::MoveGroupInterface::Plan &motion_plan);

    static void SetPlannerStartState(const std::vector<double> &joint_angles,
                                     const std::vector<std::string> &joint_names,
                                     moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                     moveit::core::RobotState &robot_state);

    static void ExecutePlannedTrajectory(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                         moveit::planning_interface::MoveGroupInterface::Plan &motion_plan,
                                         const geometry_msgs::Pose &target_pose,
                                         bool use_pose_correction,
                                         actionlib::SimpleActionClient<mrirac_msgs::PoseCorrectionAction> &pose_correction_client);
};

#endif