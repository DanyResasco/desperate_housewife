#ifndef HAND_POSE_GENERATOR_H
#define HAND_POSE_GENERATOR_H

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <desperate_housewife/obstacleSingle.h>
#include <desperate_housewife/obstacleArray.h>
#include <desperate_housewife/handPoseSingle.h>

class HandPoseGenerator{

private:

  std::string geometries_topic_, desired_hand_right_pose_topic_, desired_hand_left_pose_topic_, obstalces_topic_, desired_hand_pose_topic_;
  std::string base_frame_, desired_hand_frame_, right_hand_frame_, left_hand_frame_;
  std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;

public:

  ros::Subscriber stream_subscriber_;
  ros::NodeHandle nh;
  ros::Publisher desired_hand_publisher_, desired_hand_right_pose_publisher_, desired_hand_left_pose_publisher_, obstacles_publisher_;
  tf::TransformBroadcaster tf_desired_hand_pose;
  tf::TransformListener listener_info;

  HandPoseGenerator();
  ~HandPoseGenerator(){};

  void HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  desperate_housewife::handPoseSingle generateHandPose( desperate_housewife::fittedGeometriesSingle geometry );
  bool isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry );

  ////////////////////////
  geometry_msgs::Pose placeHand ( desperate_housewife::fittedGeometriesSingle geometry, int whichArm );
  int whichArm( geometry_msgs::Pose object_pose );
  void fromEigenToPose (Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &Hand_pose);
  Eigen::Matrix4d FromMsgtoEigen(geometry_msgs::Pose &object);

};

#endif