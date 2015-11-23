#ifndef DESPERATE_MIND_H
#define DESPERATE_M_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames_io.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <desperate_housewife/Error_msg.h>
#include <desperate_housewife/handPoseSingle.h>
#include <desperate_housewife/fittedGeometriesSingle.h>
#include <std_msgs/Bool.h>
#include <desperate_housewife/Start.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/UInt16.h>




class DesperateDecisionMaker
{
	public:

  ros::NodeHandle nh;
  ros::Subscriber error_sub_left, error_sub_right;
  std::string error_topic_left, error_topic_right, base_frame_;
  std::string home_left_topic_, home_right_topic_;
  ros::Publisher left_home_publisher_, right_home_publisher_;
  ros::Publisher hand_publisher_left, hand_publisher_right;
  std::string hand_close_left, hand_close_right;
  std::string right_hand_synergy_joint, left_hand_synergy_joint;
  ros::Publisher Reject_obstacles_publisher_left, Reject_obstacles_publisher_right;
  std::string Reject_obstalces_topic_left, Reject_obstalces_topic_right;
  std::string desired_hand_left_pose_topic_, desired_hand_right_pose_topic_;
  ros::Subscriber pose_sub_left, pose_sub_right;
  int home_r = 1;
  int home_l = 1;
 
  //error threshold
  double x = 0.02;
  double y = 0.005;
  double z = 0.04;
  double rot_x = 0.03;
  double rot_y = 0.005;
  double rot_z = 0.005;
  
  std::string start_topic_left, start_topic_right;
  ros::Publisher left_start_controller_pub, right_start_controller_pub;

  desperate_housewife::Start start_controller;
  int start_controller_left = 0;
  int start_controller_right = 0;

  // std::string desired_hand_right_pose_topic_, desired_hand_left_pose_topic_;
  ros::Publisher desired_hand_publisher_left, desired_hand_publisher_right;
  // tf::TransformBroadcaster tf_desired_hand_pose;

  geometry_msgs::Pose pose_obj;
  int whichArm;
  int ObjOrObst;
  int arrived_l = 0;
  int arrived_r = 0;
  // int home  = 1;
  int restart = 0;
  int stop_home = 1;
  int stop_home_r = 1;
  geometry_msgs::Pose pose_removed;

  ros::Subscriber objects_info_right_sub, objects_info_left_sub;
  std::string obj_info_topic_r, obj_info_topic_l;

  ros::Publisher stop_publisher_r, stop_publisher_l;
  std::string stop_pub_filter_topic_r, stop_pub_filter_topic_l;


  //home
  ros::Publisher desired_hand_right_pose_publisher_, desired_hand_left_pose_publisher_;
    // std::string desired_hand_right_pose_topic_, desired_hand_left_pose_topic_;
    ros::Subscriber left_home_subscribe_, right_home_subscribe_;
    // std::string home_left_topic_, home_right_topic_;
    tf::TransformBroadcaster tf_desired_hand_pose;
    // std::string base_frame_;

      // desperate_housewife::handPoseSingle home_robot_right;
      // desperate_housewife::handPoseSingle home_robot_left;
  std::string left_hand_frame_, right_hand_frame_;    
    tf::TransformListener listener_info; 

  DesperateDecisionMaker();
    ~DesperateDecisionMaker(){};
     
  private:


    void Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg);
    void Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg);
    void ControllerStartAndNewPOse(const desperate_housewife::Error_msg::ConstPtr& error_msg);
   
    void hand_left(const desperate_housewife::handPoseSingle::ConstPtr& left_msg);
    void hand_right(const desperate_housewife::handPoseSingle::ConstPtr& right_msg);
    void ObjOrObst_right(const std_msgs::UInt16::ConstPtr& obj_msg);
    void ObjOrObst_left(const std_msgs::UInt16::ConstPtr& obj_msg);
    void SendHomeRobot_right();
    void SendHomeRobot_left();
};


#endif
