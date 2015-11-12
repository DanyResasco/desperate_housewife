#ifndef HAND_POSE_FILTERED_H
#define HAND_POSE_FILTERED_H

#include <ros/ros.h>
#include <ros/console.h>


#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
//Tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <math.h>
#include <std_msgs/UInt16.h>

#include <desperate_housewife/handPoseSingle.h>



/*** Code used to filtered the hand_pose. It's usefull when the cylinder's fitting is not stable
*/

class HandPoseFIltered{

private:

  std::string desired_hand_pose_left_topic_, desired_hand_pose_right_topic_, desired_hand_right_pose_topic_, desired_hand_left_pose_topic_,desired_hand_frame_ ;
  std::string base_frame_;
  tf::TransformBroadcaster tf_desired_hand_pose;

  ros::Subscriber sub_command_left,sub_command_right ;
  int test_read = 0;
  KDL::Frame pose_obj, pose_last;
  desperate_housewife::handPoseSingle Pose_obj_stable;

  int first_step = 0;
  int dot_step = 0;
  int stop = 0;
  ros::Publisher desired_hand_right_pose_publisher_, desired_hand_left_pose_publisher_;
  ros::Subscriber  stop_subscribe_r, stop_subscribe_l;
  std::string stop_pub_filter_topic_r, stop_pub_filter_topic_l;


public:

 ros::NodeHandle nh;



  HandPoseFIltered();
  ~HandPoseFIltered(){};

  /**callback for the left arm
  */
  void HandPoseFIlteredCallback_left(const desperate_housewife::handPoseSingle::ConstPtr& msg);
  /**callback for the right arm
  */
  void HandPoseFIlteredCallback_right(const desperate_housewife::handPoseSingle::ConstPtr& msg);
  /**controll to stabilize the and pose.
  */
  void Controll(const desperate_housewife::handPoseSingle::ConstPtr& msg) ;

  // void Stop_left(const std_msgs::UInt16::ConstPtr& stop_left_msg);
  // void Stop_right(const std_msgs::UInt16::ConstPtr& stop_right_msg);




};

#endif