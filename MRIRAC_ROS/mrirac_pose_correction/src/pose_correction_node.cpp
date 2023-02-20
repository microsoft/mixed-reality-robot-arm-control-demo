#include <cmath>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "mrirac_msgs/PoseCorrectionAction.h"
#include "mrirac_lib/mrirac_lib.h"

class PoseCorrectionNode
{
private:
    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<mrirac_msgs::PoseCorrectionAction> pose_correction_action_server_;

    const double tolerance_m = 0.01;

    geometry_msgs::Pose target_pose_;
    geometry_msgs::Pose current_pose_;

    ros::ServiceServer pose_correction_server_;
    ros::Subscriber tool_pose_sub_;

    const std::string tool_pose_topic_ = "/j2n6s300_driver/out/tool_pose";

    const std::string kPlanningGroup_ = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface_;

    mrirac_msgs::PoseCorrectionFeedback pose_correction_feedback_;
    mrirac_msgs::PoseCorrectionResult pose_correction_result_;

    void ToolPoseCallback(geometry_msgs::PoseStamped msg);
    void ExecuteCallback(const mrirac_msgs::PoseCorrectionGoalConstPtr &goal);

public:
    PoseCorrectionNode(const std::string &pose_correction_server_name, const ros::NodeHandle &node_handle);
    ~PoseCorrectionNode();
};

PoseCorrectionNode::PoseCorrectionNode(const std::string &pose_correction_server_name, const ros::NodeHandle &node_handle) : node_handle_(node_handle), pose_correction_action_server_(node_handle_, pose_correction_server_name, boost::bind(&PoseCorrectionNode::ExecuteCallback, this, _1), false), move_group_interface_(kPlanningGroup_)
{
    tool_pose_sub_ = node_handle_.subscribe(tool_pose_topic_, 200, &PoseCorrectionNode::ToolPoseCallback, this);

    ROS_INFO("starting action server");
    pose_correction_action_server_.start();
    ROS_INFO("started action server");
}

PoseCorrectionNode::~PoseCorrectionNode()
{
}

void PoseCorrectionNode::ExecuteCallback(const mrirac_msgs::PoseCorrectionGoalConstPtr &goal)
{
    bool action_success = true;
    target_pose_ = goal->target_pose;

    geometry_msgs::Pose correction_pose;
    correction_pose.orientation = target_pose_.orientation;
    correction_pose.position = target_pose_.position;

    geometry_msgs::Vector3 initial_difference;
    geometry_msgs::Vector3 current_difference;
    initial_difference.x = target_pose_.position.x - current_pose_.position.x;
    initial_difference.y = target_pose_.position.y - current_pose_.position.y;
    initial_difference.z = target_pose_.position.z - current_pose_.position.z;

    double magnitude = sqrt(pow(initial_difference.x, 2) + pow(initial_difference.y, 2) + pow(initial_difference.z, 2));
    ROS_INFO("initial pose error: %s", std::to_string(magnitude).c_str());
    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

    int count = 0;

    while (magnitude > tolerance_m && count < 5)
    {
        if (pose_correction_action_server_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Pose Correction Action: Preempted");
            // set the action state to preempted
            pose_correction_action_server_.setPreempted();
            action_success = false;
            break;
        }
        correction_pose.position.x += initial_difference.x * 0.2;
        correction_pose.position.y += initial_difference.y * 0.2;
        correction_pose.position.z += initial_difference.z * 0.2;

        success = RobotMovements::PlanMovementToPose(correction_pose, move_group_interface_, motion_plan);
        if (success && motion_plan.trajectory_.joint_trajectory.points.size() < 5)
        {
            move_group_interface_.execute(motion_plan);
        }
        else
        {
            break;
        }

        current_difference.x = target_pose_.position.x - current_pose_.position.x;
        current_difference.y = target_pose_.position.y - current_pose_.position.y;
        current_difference.z = target_pose_.position.z - current_pose_.position.z;
        magnitude = sqrt(pow(current_difference.x, 2) + pow(current_difference.y, 2) + pow(current_difference.z, 2));
        count++;
        pose_correction_feedback_.pose_error = magnitude;
        pose_correction_action_server_.publishFeedback(pose_correction_feedback_);
    }

    if (action_success)
    {
        pose_correction_result_.pose_error = pose_correction_feedback_.pose_error;
        ROS_INFO("Pose Correction Action: Succeeded");
        // set the action state to succeeded
        pose_correction_action_server_.setSucceeded(pose_correction_result_);
    }
}

void PoseCorrectionNode::ToolPoseCallback(geometry_msgs::PoseStamped msg)
{
    current_pose_ = msg.pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mrirac_pose_correction");
    ros::NodeHandle node_handle("~");

    ros::AsyncSpinner spinner(0);
    spinner.start();
    std::string pose_correction_server_name = "pose_correction";
    PoseCorrectionNode node(pose_correction_server_name, node_handle);

    ros::waitForShutdown();

    return 0;
}