#include <test_ball.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "desperate_ball_node");
  ball node;
 
  ROS_INFO("[ball] Node is ready");

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