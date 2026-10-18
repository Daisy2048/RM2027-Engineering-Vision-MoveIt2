#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "panthera_interfaces/msg/arm_pose.hpp"



using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using BoolMsg = example_interfaces::msg::Bool;
using ArmJoint = example_interfaces::msg::Float64MultiArray;
using ArmPose = panthera_interfaces::msg::ArmPose;



class CommanderTemplate
{
public:
    CommanderTemplate(const rclcpp::Node::SharedPtr& node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
        gripper_->setMaxVelocityScalingFactor(1.0);
        gripper_->setMaxAccelerationScalingFactor(1.0);

        gripper_success_sub_ = node_->create_subscription<BoolMsg>(
            "open_gripper", 10,
            std::bind(&CommanderTemplate::GripperSuccessCallback, this, std::placeholders::_1));



        arm_joint_sub_ = node_->create_subscription<ArmJoint>(
            "arm_joint", 10,
            std::bind(&CommanderTemplate::ArmJointCallback, this, std::placeholders::_1));
        arm_pose_sub_ = node_->create_subscription<ArmPose>(
            "arm_pose", 10,
            std::bind(&CommanderTemplate::ArmPoseCallback, this, std::placeholders::_1));
    }

    void goToNamedTarget(const std::string& target)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(target);
        playAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double>& joint_values)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joint_values);
        playAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path = false)
    {
        arm_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();
        pose.pose.orientation.x = q.getX();
        pose.pose.orientation.y = q.getY();
        pose.pose.orientation.z = q.getZ();
        pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if (cartesian_path)
        {

            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.01;
            const double jump_threshold = 0.0;
            const bool avoid_collisions = true;
            double fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);
            RCLCPP_INFO(node_->get_logger(), "Cartesian path fraction: %f", fraction);
            if (fraction == 1.0)
            {
                arm_->execute(trajectory);
            } else
            {
                RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning failed");
            }
        }
        else
        {
            arm_->setStartStateToCurrentState();
            arm_->setPoseTarget(pose);
            playAndExecute(arm_);
        }
    }

    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("open");
        playAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("close");
        playAndExecute(gripper_);
    }

private:

    void playAndExecute(const std::shared_ptr<MoveGroupInterface>& group)
    {
        MoveGroupInterface::Plan plan;
        if (group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            group->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Plan failed");
        }
    }

    void GripperSuccessCallback(const BoolMsg::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(node_->get_logger(), "Gripper open");
            openGripper();
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Gripper close");
            closeGripper();
        }
    }

    void ArmJointCallback(const ArmJoint::SharedPtr msg)
    {
        auto data = msg->data;
        if (data.size() != 6)
        {
            RCLCPP_ERROR(node_->get_logger(), "Arm joint data size error");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Arm joint: %f, %f, %f, %f, %f, %f",
            data[0], data[1], data[2], data[3], data[4], data[5]);
        goToJointTarget(data);
    }

    void ArmPoseCallback(const ArmPose::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Arm pose: %f, %f, %f, %f, %f, %f, %d",
            msg->x, msg->y, msg->z,
            msg->roll, msg->pitch, msg->yaw, msg->cartesian_path);
        goToPoseTarget(msg->x, msg->y, msg->z,
            msg->roll, msg->pitch, msg->yaw, msg->cartesian_path);
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<BoolMsg>::SharedPtr gripper_success_sub_;

    rclcpp::Subscription<ArmJoint>::SharedPtr arm_joint_sub_;

    rclcpp::Subscription<ArmPose>::SharedPtr arm_pose_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander_template");
    auto commander = std::make_shared<CommanderTemplate>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
