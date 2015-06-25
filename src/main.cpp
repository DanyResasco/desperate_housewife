#include "phobic_camera.h"

int main(int argc, char** argv)
{
	//std::cout<<"Sono nel main"<<std::endl<<std::flush;
	ros::init(argc, argv, "Phobic_whife_scene");

	ros::Subscriber reader;
	// std::cout<<"Sono nel main prima di aver creato ogetto classe"<<std::endl;
	
	ros::NodeHandle nodeH;
	phobic_scene phobic_scene_local(nodeH, true); 
	// std::cout<<"Sono nel main dopo aver creato ogetto classe"<<std::endl;

	reader = nodeH.subscribe(nodeH.resolveName("camera/depth_registered/points"), 1, &phobic_scene::pointcloudCallback, &phobic_scene_local);
	//std::cout<<"Sono nel main dopo aver letto"<<std::endl;

	
	ros::Rate loop_rate(2); // 5Hz

  while (ros::ok())
  {

    loop_rate.sleep();
    ros::spinOnce();
    
  }

	return 0;

	
}