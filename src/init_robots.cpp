#include <ros/ros.h>
#include <ros/console.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "InitRobots");
  ros::NodeHandle node;
  trajectory_msgs::JointTrajectory right_robot_msgs, left_robot_msgs;

  double time_to_arrive = 2;
  node.param<double>("time_to_arrive", time_to_arrive, 2.0);

  ros::Publisher pub_right_arm, pub_left_arm;

  pub_right_arm = node.advertise<trajectory_msgs::JointTrajectory>("/right_arm/joint_trajectory_controller/command", 100);
  pub_left_arm  = node.advertise<trajectory_msgs::JointTrajectory>("/left_arm/joint_trajectory_controller/command", 100);

  // right_robot_msgs.header.stamp = ros::Time::now();
  right_robot_msgs.points.resize(1);
  right_robot_msgs.joint_names.resize(7);
  right_robot_msgs.points[0].positions.resize(7);

  right_robot_msgs.points[0].positions[0] = -13.0 * M_PI / 180.0;
  right_robot_msgs.points[0].positions[1] = 28.0  * M_PI / 180.0;
  right_robot_msgs.points[0].positions[2] = 0.0   * M_PI / 180.0;
  right_robot_msgs.points[0].positions[3] = -26.0 * M_PI / 180.0;
  right_robot_msgs.points[0].positions[4] = 0.0   * M_PI / 180.0;
  right_robot_msgs.points[0].positions[5] = 32.0  * M_PI / 180.0;
  right_robot_msgs.points[0].positions[6] = 98.0  * M_PI / 180.0;
  

  right_robot_msgs.joint_names[0] = "right_arm_0_joint";
  right_robot_msgs.joint_names[1] = "right_arm_1_joint";
  right_robot_msgs.joint_names[2] = "right_arm_2_joint";
  right_robot_msgs.joint_names[3] = "right_arm_3_joint";
  right_robot_msgs.joint_names[4] = "right_arm_4_joint";
  right_robot_msgs.joint_names[5] = "right_arm_5_joint";
  right_robot_msgs.joint_names[6] = "right_arm_6_joint";

  right_robot_msgs.points[0].time_from_start = ros::Duration(time_to_arrive);


  // left_robot_msgs.header.stamp = ros::Time::now();
  left_robot_msgs.points.resize(1);
  left_robot_msgs.joint_names.resize(7);
  left_robot_msgs.points[0].positions.resize(7);

  left_robot_msgs.points[0].positions[0] = 13.0 * M_PI / 180.0;
  left_robot_msgs.points[0].positions[1] = 28.0  * M_PI / 180.0;
  left_robot_msgs.points[0].positions[2] = 0.0   * M_PI / 180.0;
  left_robot_msgs.points[0].positions[3] = -26.0 * M_PI / 180.0;
  left_robot_msgs.points[0].positions[4] = 0.0   * M_PI / 180.0;
  left_robot_msgs.points[0].positions[5] = 32.0  * M_PI / 180.0;
  left_robot_msgs.points[0].positions[6] = 98.0  * M_PI / 180.0;

  left_robot_msgs.joint_names[0] = "left_arm_0_joint";
  left_robot_msgs.joint_names[1] = "left_arm_1_joint";
  left_robot_msgs.joint_names[2] = "left_arm_2_joint";
  left_robot_msgs.joint_names[3] = "left_arm_3_joint";
  left_robot_msgs.joint_names[4] = "left_arm_4_joint";
  left_robot_msgs.joint_names[5] = "left_arm_5_joint";
  left_robot_msgs.joint_names[6] = "left_arm_6_joint";

  left_robot_msgs.points[0].time_from_start = ros::Duration(time_to_arrive);


  double spin_rate = 100;
  // ros::param::get("~spin_rate", spin_rate);
  // ROS_DEBUG( "Spin Rate %lf", spin_rate);
  ros::Rate rate(spin_rate);

  ros::spinOnce();
  rate.sleep();

  pub_right_arm.publish(right_robot_msgs);
  pub_left_arm.publish(left_robot_msgs);

  double sample = 0;
  while (node.ok())
  {
    pub_right_arm.publish(right_robot_msgs);
    pub_left_arm.publish(left_robot_msgs);
    double time_from_start = time_to_arrive - sample;
    right_robot_msgs.points[0].time_from_start = ros::Duration(time_from_start) ;
    right_robot_msgs.points[0].time_from_start = ros::Duration(time_from_start);
    sample = sample + 1.0 / spin_rate;
    if (time_from_start <= 0)
    {
      return 0;
    }
    
    // ROS_INFO_STREAM(sample);
    ros::spinOnce();
    // sleep(1000.0 * .01);
    rate.sleep();
  }
  return 0;
}
