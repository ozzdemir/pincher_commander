#include <ros/ros.h>
#include "tf/transform_datatypes.h"

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_ros_control_interface/ControllerHandle.h>
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>

#define PLANNING_INTERFACE_ON 1
#define ROS_CONTROL_INTERFACE_ON !PLANNING_INTERFACE_ON

int main( int argc, char** argv )
{
  ros::init(argc, argv, "pincher_commander_moveit");
  ros::NodeHandle n("~");

  // Start a ROS spinning thread (required for processing callbacks while moveit is blocking)
  ros::AsyncSpinner spinner(1);
  spinner.start();
#if PLANNING_INTERFACE_ON
  moveit::planning_interface::MoveGroup group("arm");
  moveit::planning_interface::MoveGroup gripper("gripper");
#endif
#if ROS_CONTROL_INTERFACE_ON
  moveit_ros_control_interface::MoveItControllerManager
#endif
  geometry_msgs::PoseStamped arm_pose;
  geometry_msgs::Pose current_position = arm_pose.pose; //To be able to retrieve positions from joint
  geometry_msgs::Point exact_pose = current_position.position;
  geometry_msgs::Quaternion exact_orientation = current_position.orientation;

  group.setPoseReferenceFrame("base_link"); // plan w.r.t. base_link frame by default
//   group.setEndEffectorLink();

  ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Endeffector frame: %s", group.getEndEffectorLink().c_str());

  // Set values for your gripper here
  const double gripper_open = 0.7;
  const double gripper_closed = -0.4;

  group.setGoalPositionTolerance(0.04);
  group.setGoalOrientationTolerance(0.1);
  group.setMaxVelocityScalingFactor(0.4);
  group.allowReplanning(true);

  ROS_INFO("Moving into default configuration");
  group.setJointValueTarget({0,0,0,0,0});
  group.move(); // we do not test before, just move if possible ;-)
                // move is a blocking call
  ros::Duration(2).sleep();
#if 1
  ROS_INFO("Going to the target in order to pick it.");
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.627 ;
  target_pose1.orientation.y = 0.360;
  target_pose1.orientation.z = -0.561;
  target_pose1.orientation.w = 0.402;
  target_pose1.position.x = 0.186;
  target_pose1.position.y = 0;
  target_pose1.position.z = -0.048 ;
  group.setPoseTarget(target_pose1);
  // here we first check if we can find a plan
  moveit::planning_interface::MoveGroup::Plan my_plan1;
  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan1);
  if (success){
      group.move();
  }
  else{
      ROS_ERROR("Cannot find a valid plan...");
  }
  ROS_INFO("Holding the object.");
  gripper.setJointValueTarget({gripper_closed});
  ros::Duration(1).sleep();
  gripper.move();
  ros::Duration(1).sleep();
#if 0 // transition is not needed right now
  ROS_INFO("Going to the transition pose.");
  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = -0.192 ;
  target_pose2.orientation.y = 0.247;
  target_pose2.orientation.z = 0.050;
  target_pose2.orientation.w = 0.948;
  target_pose2.position.x = 0.197;
  target_pose2.position.y = 0;
  target_pose2.position.z = 0.149;
  group.setPoseTarget(target_pose2);
  // here we first check if we can find a plan
  moveit::planning_interface::MoveGroup::Plan my_plan2;
  success = group.plan(my_plan2);
  if (success){
      group.move();
  }
  else{
      ROS_ERROR("Cannot find a valid plan...");
  }

  ros::Duration(2).sleep();
#endif

  ROS_INFO("Going to the goal in order to place object.");
  geometry_msgs::Pose target_pose3;
  target_pose3.orientation.x = 0.377;
  target_pose3.orientation.y = 0.437;
  target_pose3.orientation.z = -0.620;
  target_pose3.orientation.w = 0.531;
  target_pose3.position.x = -0.093;
  target_pose3.position.y = -0.203;
  target_pose3.position.z = -0.012;
  group.setPoseTarget(target_pose3);
  // here we first check if we can find a plan
  moveit::planning_interface::MoveGroup::Plan my_plan3;
  success = group.plan(my_plan3);
  if (success){
      group.move();
  }
  else{
      ROS_ERROR("Cannot find a valid plan...");
  }
  ROS_INFO("Releasing the object.");
  ros::Duration(1).sleep();
  gripper.setJointValueTarget({gripper_open});
  gripper.move();
  ros::Duration(1).sleep();
#endif
#if 0//This group is predefined pick and place
  ROS_INFO("Going to pick the object");
  group.setJointValueTarget({0.0, 0.95, 1.21, 0.87,2});
  group.move();
  arm_pose = group.getCurrentPose();
  current_position = arm_pose.pose;
  exact_pose = current_position.position;
  exact_orientation = current_position.orientation;
  std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
  std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;
  ros::Duration(1).sleep();
  ROS_INFO("Object is now getting caught");
  gripper.setJointValueTarget({gripper_closed});
  gripper.move();
  ros::Duration(1).sleep();
  ROS_INFO("Arm going to base position");
  group.setJointValueTarget({0.0, 0, 1.21, 0.87,gripper_closed});
  group.move();
  arm_pose = group.getCurrentPose();
  current_position = arm_pose.pose;
  exact_pose = current_position.position;
  exact_orientation = current_position.orientation;
  std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
  std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;
  ROS_INFO("Arm going to goal");
  group.setJointValueTarget({-2, 0.9, 1.0, 0.87,gripper_closed});
  group.move();
  arm_pose = group.getCurrentPose();
  current_position = arm_pose.pose;
  exact_pose = current_position.position;
  exact_orientation = current_position.orientation;
  std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
  std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;
  ros::Duration(1).sleep();
  ROS_INFO("Arm releasing object");
  gripper.setJointValueTarget({gripper_open});
  gripper.move();
  ros::Duration(2).sleep();
  ROS_INFO("Arm going to home position");
  group.setJointValueTarget({0,0,0,0,0});
  group.move();
  arm_pose = group.getCurrentPose();
  current_position = arm_pose.pose;
  exact_pose = current_position.position;
  exact_orientation = current_position.orientation;
  std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
  std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;
#endif

  ROS_INFO("Demo completed. Waiting for user shutdown (ctrl-c).");
  ros::waitForShutdown();

  return 0;
}
