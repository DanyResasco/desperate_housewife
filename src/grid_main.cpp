#include <grid.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "desperate_grid_node");
  grid node;
  // node.SendVitoHome();

  ROS_INFO("[grid] Node is ready");

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