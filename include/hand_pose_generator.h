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
#include <desperate_housewife/Error_msg.h>

class HandPoseGenerator{

private:

  std::string geometries_topic_, desired_hand_right_pose_topic_, desired_hand_left_pose_topic_, obstacles_topic_left,obstacles_topic_right, desired_hand_pose_topic_;
  std::string Reject_obstalces_topic_left, Reject_obstalces_topic_right, error_topic_left, error_topic_right;
  std::string base_frame_, desired_hand_frame_, right_hand_frame_, left_hand_frame_;
  std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;
  int step_obstacle = 1;

  geometry_msgs::Pose retta_hand_obj;
  double dist_to_left_hand;
    double dist_to_right_hand;

public:

  ros::Subscriber stream_subscriber_, error_sub_left, error_sub_right;
  ros::NodeHandle nh;
  ros::Publisher desired_hand_publisher_, desired_hand_right_pose_publisher_, desired_hand_left_pose_publisher_, obstacles_publisher_right, obstacles_publisher_left;
  ros::Publisher Reject_obstacles_publisher_left, Reject_obstacles_publisher_right ; 
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


  geometry_msgs::Pose ObstacleReject( desperate_housewife::fittedGeometriesSingle Pose_rej_obs);
  void Error_info(const desperate_housewife::Error_msg::ConstPtr& error_msg);

};

#endif