#include "mrirac_lib/mrirac_lib.h"

bool RobotMovements::GripperAction(double turn, actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> *finger_client)
{
    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client->sendGoal(goal);

    if (finger_client->waitForResult(ros::Duration(5.0)))
    {
        finger_client->getResult();
        return true;
    }
    else
    {
        finger_client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

bool RobotMovements::PickAndPlaceMovement(const geometry_msgs::Pose &target_pose,
                                          moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                          actionlib::SimpleActionServer<mrirac_msgs::PickAndPlaceAction> &pick_and_place_server,
                                          bool use_pose_correction,
                                          actionlib::SimpleActionClient<mrirac_msgs::PoseCorrectionAction> &pose_correction_client)
{
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

    bool success;
    mrirac_msgs::PickAndPlaceFeedback feedback;
    mrirac_msgs::PickAndPlaceResult result;

    if (pick_and_place_server.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("Pose Correction Action: Preempted");
        // set the action state to preempted
        pick_and_place_server.setPreempted();
        return false;
    }

    move_group_interface.setPoseTarget(target_pose);
    ROS_INFO("planning");
    success = (move_group_interface.plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("planning over");
    if (success)
    {
        feedback.trajectory = motion_plan.trajectory_.joint_trajectory;
        pick_and_place_server.publishFeedback(feedback);

        move_group_interface.execute(motion_plan);

        if (use_pose_correction)
        {
            ROS_INFO("calling pose correction action");

            mrirac_msgs::PoseCorrectionGoal pose_correction_goal;
            pose_correction_goal.target_pose = target_pose;
            pose_correction_client.sendGoal(pose_correction_goal);
            // wait for the action to return
            bool finished_before_timeout = pose_correction_client.waitForResult(ros::Duration(30.0));
            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = pose_correction_client.getState();
                ROS_INFO("Action finished: %s", state.toString().c_str());
            }
            else
            {
                ROS_INFO("Action did not finish before the time out.");
            }
        }
    }
    else
    {
        result.success = false;
        ROS_INFO("Pick and Place Action: Failed");
        // set the action state to failed
        pick_and_place_server.setAborted(result);
        return false;
    }
    return true;
}

bool RobotMovements::PlanMovementToPose(const geometry_msgs::Pose &target_pose,
                                        moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                        moveit::planning_interface::MoveGroupInterface::Plan &motion_plan)
{

    move_group_interface.setPoseTarget(target_pose);
    ROS_INFO("planning");
    bool success = (move_group_interface.plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("planning complete");

    return success;
}

void RobotMovements::SetPlannerStartState(const std::vector<double> &joint_angles,
                                          const std::vector<std::string> &joint_names,
                                          moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                          moveit::core::RobotState &robot_state)
{
    for (size_t i = 0; i < joint_angles.size(); i++)
    {
        double joint_angle = joint_angles.at(i);
        if (joint_angle < 0)
        {
            joint_angle = joint_angle + 2 * M_PI;
        }

        const std::vector<double> value = {joint_angle};
        robot_state.setJointPositions(joint_names.at(i), value);
    }

    move_group_interface.setStartState(robot_state);
}

void RobotMovements::ExecutePlannedTrajectory(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                              moveit::planning_interface::MoveGroupInterface::Plan &motion_plan,
                                              const geometry_msgs::Pose &target_pose,
                                              bool use_pose_correction,
                                              actionlib::SimpleActionClient<mrirac_msgs::PoseCorrectionAction> &pose_correction_client)
{
    ROS_INFO("executing planned trajectory");
    move_group_interface.execute(motion_plan);

    ROS_INFO("calling pose correction action");
    mrirac_msgs::PoseCorrectionGoal goal;
    goal.target_pose = target_pose;

    if (use_pose_correction)
    {
        pose_correction_client.sendGoal(goal);
        // wait for the action to return
        bool finished_before_timeout = pose_correction_client.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = pose_correction_client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
        }
    }
}