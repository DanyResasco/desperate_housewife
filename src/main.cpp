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

	double spin_rate;
	ros::param::get("~spin_rate",spin_rate);
	ROS_INFO( "Spin Rate %lf", spin_rate);

	std::string camera_topic;
	ros::param::get("~camera_topic",camera_topic);
	ROS_INFO( "camera_topic %s", camera_topic.c_str());

	reader = nodeH.subscribe(nodeH.resolveName(camera_topic), 1, &phobic_scene::pointcloudCallback, &phobic_scene_local);
	//std::cout<<"Sono nel main dopo aver letto"<<std::endl;

	
	ros::Rate loop_rate( spin_rate ); // 5Hz

	while (ros::ok())
	{

		loop_rate.sleep();
		ros::spinOnce();

	}
	// ros::spin();
	return 0;

	
}