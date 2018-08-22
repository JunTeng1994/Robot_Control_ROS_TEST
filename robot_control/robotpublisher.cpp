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

int main(int argc, char **argv)
{
    ros::init (argc, argv, "robot_state_publisher");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;

    //ros::Rate loop_rate(10);

    moveit::planning_interface::MoveGroup move_group("manipulator");
    move_group.setPoseReferenceFrame("base");

    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

    /* Publisher for display */
    ros::Publisher Robotstate_publisher = node_handle.advertise<moveit_msgs::RobotState>("/my_robot_state", 1);

    while (ros::ok())
    {  
        /* Construct a robot state message */
        moveit_msgs::RobotState robot_state;
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        robot_state::robotStateToRobotStateMsg(*current_state, robot_state);
    
        Robotstate_publisher.publish(robot_state);

        ros::spinOnce();

        //loop_rate.sleep();
  }
}