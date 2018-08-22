#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

void chatterCallback(const moveit_msgs::RobotState& robot_state)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    robot_state::RobotState current_state(kinematic_model);
    robot_state::robotStateMsgToRobotState(robot_state,current_state);

    ros::WallDuration sleep_time(5.0);
    sleep_time.sleep();

    std::vector<double> joint_values;
    current_state.copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_state_subscriber");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);

    ros::Subscriber sub = node_handle.subscribe("/my_robot_state", 1, chatterCallback);
  
    ros::spin();

    return 0;
}

