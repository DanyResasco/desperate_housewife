#include <phobic_camera.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "Phobic_CylinderFitting");

  ros::Subscriber reader;
  ros::NodeHandle nodeH;
  phobic_scene phobic_scene_local(nodeH, true); 

  double spin_rate;
  ros::param::get("~spin_rate",spin_rate);
  ROS_INFO( "Spin Rate %lf", spin_rate);

  ros::param::get("~cylinders_topic", phobic_scene_local.cylinders_topic);
  ROS_INFO( "Cylinder Topic %s", phobic_scene_local.cylinders_topic.c_str());

  ros::param::get("~camera_topic", phobic_scene_local.camera_topic);
  ROS_INFO( "Camera Topic %s", phobic_scene_local.camera_topic.c_str());

  ros::param::get("~camera_frame", phobic_scene_local.camera_frame);
  ROS_INFO( "Camera Frame %s", phobic_scene_local.camera_frame.c_str());

  reader = nodeH.subscribe(nodeH.resolveName(phobic_scene_local.camera_topic), 1, &phobic_scene::pointcloudCallback, &phobic_scene_local);
  
  ros::Rate loop_rate( spin_rate ); // 5Hz

  while (ros::ok())
  {

    loop_rate.sleep();
    ros::spinOnce();

  }

  return 0;  
}
