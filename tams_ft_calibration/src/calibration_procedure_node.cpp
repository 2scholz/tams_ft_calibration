#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/Trigger.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_procedure_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("toggle_ft_calibration_logging");
  std_srvs::Trigger srv;

  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::PlanningSceneInterface psi;

  std::string initial_pose; // The initial starting pose
  std::vector<std::string> joint_names; // to retrieve the joint names as parameter
  std::vector<double> joint_angles; // to retrieve the joint angles as parameter

  ros::NodeHandle pn("~");

  pn.param("initial_pose", initial_pose, std::string("ft_calibration_pose"));
  if(!pn.getParam("joint_names", joint_names))
  {
    ROS_ERROR("No joint names given as paramters.");
    return -1;
  }
  if(!pn.getParam("joint_angles", joint_angles))
  {
    ROS_ERROR("No joint angles given as paramters.");
    return -1;
  }
  if(joint_names.size() != joint_angles.size())
  {
    ROS_ERROR("You need to provide the same number of joint angles and joint names");
    return -1;
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  arm.setNamedTarget(initial_pose);
  while(!arm.move())
    ROS_ERROR("Failed to move arm to initial pose.");



  for(int i = 0; i < joint_angles.size(); i++)
  {
    arm.setJointValueTarget(joint_names[i], joint_angles[i]);

    if (!client.call(srv))
    {
      ROS_ERROR("Failed to call toggle_ft_calibration_logging.");
      return 1;
    }

    while(!arm.move())
      ROS_ERROR("Failed to move joint to provided angle.");

    if (!client.call(srv))
    {
      ROS_ERROR("Failed to call toggle_ft_calibration_logging.");
      return 1;
    }

    arm.setNamedTarget(initial_pose);
    while(!arm.move())
      ROS_ERROR("Failed to move arm to initial pose.");
  }

  return 0;
}
