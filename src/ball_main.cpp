#include <test_ball.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "desperate_ball_node");
  ball node;

  ROS_INFO("[ball] Node is ready");

  double spin_rate = 1000;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);

  ros::Rate rate(spin_rate);
  bool arrived = false;

  //time and delta_time
  ros::Time initial_time = ros::Time::now();
  ros::Time actual_time = ros::Time::now();
  ros::Duration delta_time = ros::Time::now() - ros::Time::now();

  while( arrived  != true)
    {
      if(node.check_sms == true)
        {
          arrived = node.Update( delta_time.toSec() );
        }
      delta_time = ros::Time::now() - actual_time ; //because there is a sleep
      actual_time = ros::Time::now();

      ros::spinOnce();
      rate.sleep();
    }
  ROS_INFO( "finished desperate_ball_node");
  return 0;
}
