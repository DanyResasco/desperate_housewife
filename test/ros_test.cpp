#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv)
{
	std::cout<<"Sono nel main"<<std::endl;
	ros::init(argc, argv, "Phobic_whife_scene");

	
	ros::Publisher Scena_info;
	ros::Subscriber reader;
	std::cout<<"Sono nel main prima di aver creato ogetto classe"<<std::endl;
	//leggo();
	ros::NodeHandle nodeH;
	//phobic_scene phobic_scene_local(nodeH); 
	std::cout<<"Sono nel main dopo aver creato ogetto classe"<<std::endl;

	ros::spin();
	return 0;

	
}