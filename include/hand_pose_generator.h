#ifndef HAND_POSE_GENERATOR_H
#define HAND_POSE_GENERATOR_H

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <desperate_housewife/obstacleSingle.h>
#include <desperate_housewife/obstacleArray.h>
#include <desperate_housewife/handPoseSingle.h>

class HandPoseGenerator{

private:

  std::string geometries_topic_, desired_hand_pose_topic_, obstalces_topic_;
  std::string ref_frame_, desired_hand_frame_;
  std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;

public:

  ros::Subscriber stream_subscriber_;
  ros::NodeHandle nh;
  ros::Publisher desired_hand_pose_publisher_, obstacles_publisher_;
  tf::TransformBroadcaster tf_desired_hand_pose;

  HandPoseGenerator();
  ~HandPoseGenerator(){};

  void HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  desperate_housewife::handPoseSingle generateHandPose( desperate_housewife::fittedGeometriesSingle geometry );
  bool isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry );
  geometry_msgs::Pose placeHand ( desperate_housewife::fittedGeometriesSingle geometry );

};

#endif