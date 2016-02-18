#include <ros/ros.h>
#include <desperate_State_machine.h>

int main(int argc, char *argv[])
{
  // if( !ros::isInitialized() )
  // {
  ros::init(argc, argv, "desperate_grafo");
  // }
  ROS_INFO("[desperate_grafo] node is ready");
  Desp_state_server machine_s;
  //  while (machine_s.node.ok())
  // {

  machine_s.join();
  // ros::spinOnce();
  // rate.sleep();
  // }


  return 0;
}
