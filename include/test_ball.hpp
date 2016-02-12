#ifndef GRID_H
#define GRID_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

class ball
{
public:
  ros::NodeHandle nh;
  void ballInfo(const std_msgs::Float64MultiArray::ConstPtr &msg);
  KDL::Vector GetRepulsiveWithTable();
  KDL::Vector GetRepulsiveWithObstacles();
  void InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  ball();
  ~ball(){};
private:
  ros::Subscriber sub_grid_,obstacles_subscribe_;
  std::vector<double> Object_radius;
  std::vector<double> Object_height;
  std::vector<KDL::Frame> Object_position;
  ros::Publisher marker_pub;


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "desperate_ball_test_node");
  ball node;
  // node.SendVitoHome();

  ROS_INFO("[ball test] Node is ready");

  double spin_rate = 10;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);

  ros::Rate rate(spin_rate); 

  while (node.nh.ok())
  {
    ros::spinOnce(); 
    rate.sleep();
  }
  return 0;
}

ball::ball()
{
    //grid
    sub_grid_ = nh.subscribe("gridspace", 1, &ball::gridspace, this);
    obstacles_subscribe_ = nh.subscribe(obstacle_avoidance.c_str(), 1, &ball::InfoGeometry, this); 
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void ball::ballInfo(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  KDL::Vector ball_pos;
  double radius;
  ball_pos.x() = msg->data[0];
  ball_pos.y() = mag->data[1];
  ball_pos.z() = msg->data[2];
  radius = msg->data[3];




}

void ball::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
      Object_radius.clear();
      Object_height.clear();
      Object_position.clear();
      Time_traj_rep = 0;
      
      //get info for calculates objects surface
      for(unsigned int i=0; i < msg->geometries.size(); i++)
      {
        KDL::Frame frame_obj;
        Object_radius.push_back(msg->geometries[i].info[0]);  //radius
        Object_height.push_back(msg->geometries[i].info[1]);  //height

        tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
        Object_position.push_back(frame_obj); 
      }
}
























#endif