#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  moveit::planning_interface::MoveGroup move_group("manipulator");
  move_group.setPoseReferenceFrame("base");

  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("manipulator");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
  
  // get current robot state
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
  // retreive the current set of joint values stored in the state for the manipulator.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_group_positions[i]);
  }

  // set target pose matrix base on the world coordinate
  Eigen::Matrix4d end_effector_state_matrix;
  end_effector_state_matrix <<  1.37387, 0.0021328, 0.504446, -0.22598,
        -0.575761, 0.0181216, 1.20353, -0.769617,
        -0.00572053, -1.31168, 0.0174477, 0.305557,
        0, 0, 0, 1;
  //end_effector_state_matrix <<  0.690744, 0.0112346, 0.72301, -0.223933,
  //      -0.721009, 0.0866272, 0.687487, -0.760089,
  //      -0.0549096, -0.996176, 0.0679396, 0.314363,
  //      0, 0, 0, 1;
  const Eigen::Affine3d& transFrame = current_state->getFrameTransform("base");
  std::cout << transFrame.matrix() << std::endl;
  Eigen::Affine3d end_effector_state;
  end_effector_state.matrix() = transFrame.matrix()*end_effector_state_matrix;
  std::cout <<  end_effector_state.matrix() << std::endl;

  // solve inverse kinematics (IK) for the robot
  bool found_ik = current_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
  std::vector<double> joint_values;
  if (found_ik)
  {
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  
  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First, we will define the collision object message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "box";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 8.0;
  primitive.dimensions[1] = 8.0;
  primitive.dimensions[2] = 1.0;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0;
  box_pose.position.y =  0;
  box_pose.position.z =  -0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);  

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);

  // Planning with collision detection can be slow.  Lets set the planning time
  // to be sure the planner has enough time to plan around the box.  10 seconds
  // should be plenty.
  move_group.setPlanningTime(10.0);


  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setJointValueTarget(joint_values);
  
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = move_group.plan(my_plan);
 
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); 
  sleep(5.0); 

  if(success)
    move_group.execute(my_plan);
    

  ros::shutdown(); 
  return 0;
}