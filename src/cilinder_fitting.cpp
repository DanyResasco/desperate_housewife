#include "phobic_camera.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "PhobicDemo_CylinderFittingInfo");

  ros::Subscriber reader;

  ros::NodeHandle nodeH;
  phobic_scene phobic_scene_local(nodeH, true);

  double spin_rate = 10;
  ros::param::get("~spin_rate",spin_rate);
  ROS_INFO( "Spin Rate %lf", spin_rate);

  std::string camera_topic;
  ros::param::get("~camera_topic",camera_topic);
  ROS_INFO( "camera_topic %s", camera_topic.c_str());

  reader = nodeH.subscribe(nodeH.resolveName(camera_topic), 1, &phobic_scene::pointcloudCallback, &phobic_scene_local);

  ros::Rate loop_rate( spin_rate );

  while (ros::ok())
    {

      loop_rate.sleep();
      ros::spinOnce();

    }
  return 0;

}
