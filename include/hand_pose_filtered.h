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

#include <desperate_housewife/handPoseSingle.h>


/*** Code used to filtered the hand_pose. It's usefull when the cylinder's fitting is not stable
*/

class HandPoseFIltered{

private:

  std::string desired_hand_pose_topic_, desired_hand_right_pose_topic_, desired_hand_left_pose_topic_;

public:

  ros::Subscriber sub_command_;
  ros::NodeHandle nh;
  int test_read = 0;
  KDL::Frame pose_obj, pose_last;
   desperate_housewife::handPoseSingle Pose_obj_stable;

  int first_step = 0;
  ros::Publisher desired_hand_right_pose_publisher_, desired_hand_left_pose_publisher_;




  HandPoseFIltered();
  ~HandPoseFIltered(){};

  void HandPoseFIlteredCallback(const desperate_housewife::handPoseSingle::ConstPtr& msg);



};

#endif