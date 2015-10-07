
#include <ros/ros.h>
#include <ros/console.h>
#include <fit_geometries.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Fit Geometries_node");
  BasicGeometriesNode node;
  ROS_INFO("[fitGeometries] Node is ready");

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