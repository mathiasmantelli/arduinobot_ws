#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <arduinobot_msgs/action/arduinobot_task.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

using namespace std::placeholders;

namespace arduinobot_remote{
class TaskServer: public rclcpp::Node{
public:
    explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("task_server", options){
        action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
            this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2), 
            std::bind(&TaskServer::cancelCallback, this, _1), 
            std::bind(&TaskServer::acceptedCallback, this, _1));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting the Action Server");
    }

private:
    rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;

    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received goal request with ID: %d", goal->task_number);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle){
        std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal");
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

        std::vector<double> arm_joint_goal, gripper_joint_goal;

        if(goal_handle->get_goal()->task_number == 0){
            arm_joint_goal = {0.0, 0.0, 0.0};
            gripper_joint_goal = {-0.7, 0.7};
        }else if(goal_handle->get_goal()->task_number == 1){
            arm_joint_goal = {-1.14, -0.6, -0.07};
            gripper_joint_goal = {0.0, 0.0};
        }else if(goal_handle->get_goal()->task_number == 2){
            arm_joint_goal = {-1.57, 0.0, -1.0};
            gripper_joint_goal = {0.0, 0.0};
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid task number");
            return;
        }

        bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
        bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

        if(!arm_within_bounds || !gripper_within_bounds){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Target joint position were outside limits");
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan, gripper_plan;

        bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        bool gripper_plan_success = gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS;

        if(arm_plan_success && gripper_plan_success){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner succeed, moving the arm and the gripper");
            arm_move_group.move();
            gripper_move_group.move();
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed");
            return;
        }

        auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
    }

    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel the goal");
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

        arm_move_group.stop();
        gripper_move_group.stop();
        return rclcpp_action::CancelResponse::ACCEPT;

    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)

