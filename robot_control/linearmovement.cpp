#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "robot_linear_movement");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup move_group("manipulator");

  // set waypoints for which to compute path
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose;
  target_pose = move_group.getCurrentPose().pose;
  target_pose.position.z += 0.1;
  waypoints.push_back(target_pose);
  target_pose.position.z += 0.1;
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;

  // compute cartesian path
  double ret = move_group.computeCartesianPath(waypoints, 0.01, 0.0 , trajectory);
  if(ret < 0)
  {
    // no path could be computed
    ROS_ERROR("Unable to compute Cartesian path!");
  }
  else if (ret < 1)
  {
    // path started to be computed, but did not finish
    ROS_WARN_STREAM("Cartesian path computation finished " << ret * 100 << "% only!");
  }

  // execute trajectory
  moveit::planning_interface::MoveGroup::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);
  
  ros::shutdown(); 
  return 0;
}