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
// #include <desperate_housewife/Start.h>
#include <std_msgs/UInt16.h>
#include <visualization_msgs/Marker.h>
// #include <transition.h>
#include "state.h"


class HandPoseGenerator  : public state<transition>
{

private:

  std::string geometries_topic_, desired_hand_right_pose_topic_, desired_hand_left_pose_topic_, obstacles_topic_left,obstacles_topic_right, desired_hand_pose_topic_;
  std::string Reject_obstalces_topic_left, Reject_obstalces_topic_right, error_topic_left, error_topic_right;
  std::string base_frame_, desired_hand_frame_, right_hand_frame_, left_hand_frame_;
  std::string start_topic_left, start_topic_right;
  ros::Subscriber left_start_controller_sub, right_start_controller_sub;
  ros::Publisher hand_publisher_right, hand_publisher_left;
  std::string hand_close_right, hand_close_left; 

  Eigen::Vector3d retta_hand_obj;
  int demo;
  int Number_obj;
  int id_arm;
  double SoftHandDistanceFrame;
  bool use_both_arm;
  bool failed;


public:

  ros::Subscriber stream_subscriber_, error_sub_left, error_sub_right;
  ros::NodeHandle nh;
  ros::Publisher desired_hand_publisher_, desired_hand_right_pose_publisher_, desired_hand_left_pose_publisher_, obstacles_publisher_right, obstacles_publisher_left;
  // ros::Publisher Reject_obstacles_publisher_left, Reject_obstacles_publisher_right ; 
  ros::Publisher home_robot_pub, desired_hand_publisher_right, desired_hand_publisher_left;
  std::string desired_hand_pose_left_topic_, desired_hand_pose_right_topic_;
  tf::TransformBroadcaster tf_desired_hand_pose;
  tf::TransformListener listener_info;
  ros::Publisher objects_info_right_pub, objects_info_left_pub;
  std::string obj_info_topic_r, obj_info_topic_l;
  std::string right_hand_synergy_joint, left_hand_synergy_joint;
  tf::TransformListener listener_object;
  std::vector<KDL::Twist> vect_error;
  // int index;


  desperate_housewife::fittedGeometriesArray cylinder_geometry;
  int ObjorObst;
  int id_class;
  KDL::Twist e_;
  KDL::Twist e_l, e_r;
  KDL::Twist E_t;
  int id_msgs;
  bool finish;
  int id_arm_msg;
  // int step;
  shared& data;
  bool Arm_r, Arm_l;
  int send_msg ;
  // std_msgs::UInt16 Obj_info;



  bool IsEqual(KDL::Twist E_pf);
  virtual std::map< transition, bool > getResults();
  virtual void run();
  virtual bool isComplete();
  virtual std::string get_type();


  HandPoseGenerator(shared& data);
  // ~HandPoseGenerator(){};

  void Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg);

  void Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg);

  void HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  /*! 
  * \fn HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  * \brief callback that called all function for make a desired pose for each object 
  * \param  ros message
  * \return void
  */

  desperate_housewife::handPoseSingle generateHandPose( desperate_housewife::fittedGeometriesSingle geometry, int cyl_nbr );
  /*! 
  * \fn generateHandPose( desperate_housewife::fittedGeometriesSingle geometry, int cyl_nbr )
  * \brief  function that call the function for generate the hand pose
  * \param cylinder, index of cylinder
  * \return desperate_housewife::handPoseSingle
  */
  
  
  bool isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry );
  /*!
  * \fn isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry );
  * \brief  Function to decide if object is graspable. If the ratio (number of inlier/number of point of cluster) >50 and the radius is minus than a threshold 
            *the object is grasbale, otherwise is obstacle. 
  * \param cylinder
  * \return void
  */
 


  geometry_msgs::Pose placeHand ( desperate_housewife::fittedGeometriesSingle geometry, int whichArm );
  /*!
  * \fn placeHand ( desperate_housewife::fittedGeometriesSingle geometry, int whichArm );
  * \brief Function that calculates the hand Pose. Depending on cylinder as is put the pose change. We consider if cylinder is:
          *Lying --> hand is put in the middle and moved up by 5 cm
          *Full and standing --> hand is put on the top
          *Empty and standign --> hand is put on the top and moved along x axis of the quantity of the radius
  * \param cylinder, integrer 1 left arm 0 right arm
  * \return pose
  */

  int whichArm( geometry_msgs::Pose object_pose, int cyl_nbr );
  /*!
  * \fn whichArm( geometry_msgs::Pose object_pose, int cyl_nbr );
  * \brief Function that calulates whichArm use. It's calculate looking the shortes distance between the object and the arm.  
  * \param pose
  * \return integrer 1 left arm 0 right arm
  */


  void fromEigenToPose (Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &Hand_pose);
  /*!
  * \fn: fromEigenToPose (Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &Hand_pose);
  * \brief convert eigen matrix to geometry_pose  
  * \param eigen matrix, geoemtry_pose to save the hand pose 
  * \return void
  */


  Eigen::Matrix4d FromMsgtoEigen(geometry_msgs::Pose &object);
  /*!
  * \fn FromMsgtoEigen(geometry_msgs::Pose &object);
  * \brief convert geometry_pose to eigen matrix 
  * \param geometry pose
  * \return eigen matrix
  */


  geometry_msgs::Pose ObstacleReject( desperate_housewife::fittedGeometriesSingle Pose_rej_obs, int arm_);
  /*! 
  * \fn ObstacleReject( desperate_housewife::fittedGeometriesSingle Pose_rej_obs, int arm_);
  * \brief Function that calulates the hand pose to remove the obstacle 
  * \param cylinder,integrer 1 left arm 0 right arm
  * \return geometry pose
  */
 
  /*! Function for send the obstacle messages
  */
  // void SendObjRejectMsg(desperate_housewife::fittedGeometriesSingle obj_msg, int arm_);


  // void Start_left(const desperate_housewife::Start::ConstPtr& msg);
  // void Start_right(const desperate_housewife::Start::ConstPtr& msg);
  /*! 
  * \fn Start_left and Start_right (const desperate_housewife::Start::ConstPtr& msg);
  * \brief Function for move vito in the desired pose before control start   
  * \param: start msgs
  * \return void
  */
  
  void  DesperateDemo2(std::vector<desperate_housewife::fittedGeometriesSingle> msg);
  /*! 
  * \fn: DesperateDemo2(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  * \brief Second Demo. grasp object without obstacles avoidance. if item isn't graspable will be removed 
  * \param cylinder
  * \return void
  */
  

  void  DesperateDemo1(std::vector<desperate_housewife::fittedGeometriesSingle> msg);
  /*! 
  * \fn DesperateDemo1( const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  * \brief First Demo. grasp object with obstacles avoidance. 
  * \param cylinder
  * \return void
  */

  double GetDistanceForHand(double radius);

  
  void Overturn();
   /*! 
  * \fn Overturn
  * \brief If there are more than 5 objects or all objects aren't graspable, overturn table 
  * \param void
  * \return void
  */


  // std::vector< desperate_housewife::fittedGeometriesSingle > SortCylinder(std::vector< desperate_housewife::fittedGeometriesSingle > VECT_cyl1);

   /*! Function: CheckRealTimeObstacleMovements
  *input: cylinder
  *output: void
  *Description:Send a obstacles at real time 
  */
  // void CheckRealTimeObstacleMovements(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);


// void DrawStraingLIne( Eigen::Vector3d &rett_pos );

   int CheckWhichArmIsActive();

  void OnlyRight(int cyl_nbr);
  void OnlyLeft(int cyl_nbr);
  int SendBoth(geometry_msgs::Pose object_pose, int cyl_nbr);


};

    /*! Function: SetObstacleMsg
  *input: obstacle
  *output: obstacle in
  *Description:Send a obstacles at real time 
  */
  // desperate_housewife::fittedGeometriesSingle SetObstacleMsg(desperate_housewife::fittedGeometriesSingle geo_obst);

#endif