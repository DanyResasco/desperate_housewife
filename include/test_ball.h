#ifndef TEST_BALL_H
#define TEST_B_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <potential_field_control.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>


  class ball
  {
    public:
      ros::NodeHandle nh;
     
      void DrawSphere( Eigen::Matrix<double,6,1> &pos_ball );
     
      void InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
      ball();
      ~ball(){};
      Eigen::Matrix<double,6,1> GetFieldRep(Eigen::Matrix<double,6,1> &point_pos);
      //to visualize in rviz
      void SeeMarker(Eigen::Matrix<double,6,1> &Pos, std::string obst_name);
      //integration
      void GetVelocityAndPosition(Eigen::Matrix<double,6,1> &Force,  double delta);
      bool Update(double delta);
      void ballInfo(const std_msgs::Float64MultiArray::ConstPtr &msg);
      void DrawCylinder();

      bool check_sms;

    private:
      ros::Subscriber sub_grid_,obstacles_subscribe_;
      std::vector<double> Object_radius;
      std::vector<double> Object_height;
      ros::Publisher marker_pub,marker_publisher_ ;
      std::string obstacle_avoidance;
      desperate_housewife::PotentialFieldControl pfc;
      std::vector<KDL::Frame> Object_position;
      ros::Publisher vis_pub;
      // KDL::Vector POS_init;
      Eigen::Matrix<double,6,1> pos_des;
      Eigen::Matrix<double,6,1> ball_pos;
      double radius,mass;
      Eigen::Matrix<double,6,1> Force_attractive;
      Eigen::Matrix<double,6,1> x_dot;
      Eigen::Matrix<double,6,1> F_repulsive;
      // KDL::Frame frame_des,frame_init;
      Eigen::Matrix<double,6,1> x_err_;
      // Eigen::Matrix<double,6,1> velocity,pos; 
      tf::TransformBroadcaster tf_geometriesTransformations_;
      int count;
      ros::Publisher pub_Fa_, pub_Fr_;
      // bool arrived = false;


    };

  KDL::Frame FromEigenToKdl(Eigen::Matrix<double,6,1> Force);




#endif