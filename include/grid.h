#ifndef GRID_H
#define GRID_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry> 
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <visualization_msgs/Marker.h>
//Tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>

#include <potential_field_control.h>

class grid
{
public:
  ros::NodeHandle nh;
  void gridspace(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void  DrawArrow( KDL::Vector &gridspace_Force, KDL::Vector &gridspace_point );  
  Eigen::Quaterniond RotationMarker(KDL::Vector &ris_Force, KDL::Vector &point);
  void InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  grid();
  ~grid(){};
    void grid::GetForceAndDraw(KDL::Vector point_pos);
private:
  ros::Subscriber sub_grid_,obstacles_subscribe_;
  std::vector<double> Object_radius;
  std::vector<double> Object_height;
  ros::Publisher marker_pub;
  std::string obstacle_avoidance;
  desperate_housewife::PotentialFieldControl pfc;
  std::vector<KDL::Frame> Object_position;
  ros::Publisher vis_pub;
  ros::ServiceClient client;


};


#endif

