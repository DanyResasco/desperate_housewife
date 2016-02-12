#ifndef Vito_home_H
#define Vito_home_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
//Eigen
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry> 
#include <desperate_housewife/handPoseSingle.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>

class HomeVitoPosition
{
	public:

	  ros::NodeHandle nh;
	  ros::Publisher desired_hand_right_pose_publisher_, desired_hand_left_pose_publisher_;
	  std::string desired_hand_right_pose_topic_, desired_hand_left_pose_topic_;
	  ros::Subscriber left_home_subscribe_, right_home_subscribe_;
	  std::string home_left_topic_, home_right_topic_;
	  tf::TransformBroadcaster tf_desired_hand_pose;
	  std::string base_frame_;

	    desperate_housewife::handPoseSingle home_robot_right;
	    desperate_housewife::handPoseSingle home_robot_left;
	std::string left_hand_frame_, right_hand_frame_;    
	  tf::TransformListener listener_info; 

	  HomeVitoPosition();
	  ~HomeVitoPosition(){};

  private:

	void SendHomeRobot_left(const std_msgs::Bool::ConstPtr& home_msg);
	void SendHomeRobot_right(const std_msgs::Bool::ConstPtr& home_msg);


};

#endif
