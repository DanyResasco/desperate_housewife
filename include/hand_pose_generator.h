#ifndef HAND_POSE_GENERATOR_H
#define HAND_POSE_GENERATOR_H

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>

#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <desperate_housewife/obstacleSingle.h>
#include <desperate_housewife/obstacleArray.h>
#include <desperate_housewife/handPoseSingle.h>
#include <desperate_housewife/Error_msg.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <desperate_housewife/Start.h>
#include <std_msgs/UInt16.h>
#include <visualization_msgs/Marker.h>



class HandPoseGenerator{

private:

  std::string geometries_topic_, desired_hand_right_pose_topic_, desired_hand_left_pose_topic_, obstacles_topic_left,obstacles_topic_right, desired_hand_pose_topic_;
  std::string Reject_obstalces_topic_left, Reject_obstalces_topic_right, error_topic_left, error_topic_right;
  std::string base_frame_, desired_hand_frame_, right_hand_frame_, left_hand_frame_;
  std::string start_topic_left, start_topic_right;
  ros::Subscriber left_start_controller_sub, right_start_controller_sub;

  Eigen::Vector3d retta_hand_obj;
  int start_controller_left = 0;
  int start_controller_right = 0;
  int demo;
  int Number_obj;
  int flag_obj = 0;
  int flag_remove = 0;
  int step_grasp = 0;
  int stop;


public:

  ros::Subscriber stream_subscriber_, error_sub_left, error_sub_right;
  ros::NodeHandle nh;
  ros::Publisher desired_hand_publisher_, desired_hand_right_pose_publisher_, desired_hand_left_pose_publisher_, obstacles_publisher_right, obstacles_publisher_left;
  ros::Publisher Reject_obstacles_publisher_left, Reject_obstacles_publisher_right ; 
  ros::Publisher home_robot_pub, desired_hand_publisher_right, desired_hand_publisher_left;
  std::string desired_hand_pose_left_topic_, desired_hand_pose_right_topic_;
  tf::TransformBroadcaster tf_desired_hand_pose;
  tf::TransformListener listener_info;
  ros::Publisher objects_info_right_pub, objects_info_left_pub;
  std::string obj_info_topic_r, obj_info_topic_l;
  ros::Publisher vis_pub;
 
  tf::TransformListener listener_object;

  HandPoseGenerator();
  ~HandPoseGenerator(){};

  /** Caalback: HandPoseGeneratorCallback
  * input: ros message
  * Description: callback that called all function for make a desired pose for each object 
  */
  void HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);

  desperate_housewife::handPoseSingle generateHandPose( desperate_housewife::fittedGeometriesSingle geometry, int cyl_nbr );
  /** Function: isGeometryGraspable
  *input: cylinder
  *output: bool
  *Description: Function to decide if object is graspable. If the ratio (number of inlier/number of point of cluster) >50 and the radius is minus than a threshold 
  *the object is grasbale, otherwise is obstacle. 
  */
  bool isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry );

  /**Function: placeHand
  *input: cylinder, integrer 1 left arm 0 right arm
  *output: pose
  *Description:Function that calculates the hand Pose. Depending on cylinder as is put the pose change. We consider if cylinder is:
  *Lying --> hand is put in the middle and moved up by 5 cm
  *Full and standing --> hand is put on the top
  *Empty and standign --> hand is put on the top and moved along x axis of the quantity of the radius
  */
  geometry_msgs::Pose placeHand ( desperate_housewife::fittedGeometriesSingle geometry, int whichArm );
  /** Function: whichArm
  *input: pose
  *output: integrer 1 left arm 0 right arm
  *Description:Function that calulates whichArm use. It's calculate looking the shortes distance between the object and the arm.  
  */
  int whichArm( geometry_msgs::Pose object_pose, int cyl_nbr );
   /** Function: fromEigenToPose
  *input: eigen matrix, geoemtry_pose to save the hand pose 
  *output: void
  *Description:convert eigen matrix to geometry_pose  
  */
  void fromEigenToPose (Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &Hand_pose);
   /** Function: FromMsgtoEigen
  *input: geometry pose
  *output: eigen matrix
  *Description:convert geometry_pose to eigen matrix 
  */
  Eigen::Matrix4d FromMsgtoEigen(geometry_msgs::Pose &object);

  /** Function: ObstacleReject
  *input: cylinder,integrer 1 left arm 0 right arm
  *output: geometry pose
  *Description:Function that calulates the hand pose to remove the obstacle 
  */
  geometry_msgs::Pose ObstacleReject( desperate_housewife::fittedGeometriesSingle Pose_rej_obs, int arm_);
 
  /** Function for send the obstacle messages
  */
  // void SendObjRejectMsg(desperate_housewife::fittedGeometriesSingle obj_msg, int arm_);

  /** FUnction: Start_left and Start_right
  *input: start msgs
  *output: void
  *Description:Function for move vito in the desired pose before control start   
  */
  void Start_left(const desperate_housewife::Start::ConstPtr& msg);
  void Start_right(const desperate_housewife::Start::ConstPtr& msg);
  /** FUnction: DesperateDemo2
  *input: cylinder
  *output: void
  *Description:Second Demo. grasp object without obstacles avoidance. if item isn't graspable will be removed 
  */
  void  DesperateDemo2(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
   /** FUnction: DesperateDemo1
  *input: cylinder
  *output: void
  *Description:First Demo. grasp object with obstacles avoidance. 
  */
  void  DesperateDemo1( const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
   /** FUnction: Overturn
  *input: void
  *output: void
  *Description:If there are more than 5 objects or all objects aren't graspable, overturn table 
  */
  void Overturn();

   /** Function: CheckRealTimeObstacleMovements
  *input: cylinder
  *output: void
  *Description:Send a obstacles at real time 
  */
  void CheckRealTimeObstacleMovements(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);


// void DrawStraingLIne( Eigen::Vector3d &rett_pos );

};

#endif