#include "phobic_camera.h"

int main(int argc, char** argv)
{
	std::cout<<"Sono nel main"<<std::endl<<std::flush;
	ros::init(argc, argv, "Phobic_whife_scene");

	// ros::Publisher Scena_info;
	ros::Subscriber reader;
	// std::cout<<"Sono nel main prima di aver creato ogetto classe"<<std::endl;
	
	ros::NodeHandle nodeH;
	phobic_scene phobic_scene_local(nodeH, true); 
	// std::cout<<"Sono nel main dopo aver creato ogetto classe"<<std::endl;

	// //Subscribe to the camera/deepregistered/poins topic with the master. ROS will call 
	// //the pointcloudCallback() function whenever a new message arrives. 
	// //The 2nd argument is the queue size
	reader = nodeH.subscribe(nodeH.resolveName("camera/depth_registered/points"), 10, &phobic_scene::pointcloudCallback, &phobic_scene_local);
	// //  Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>("desperate_camera", "100");
	std::cout<<"Sono nel main dopo aver letto"<<std::endl;

	//boost::this_thread::sleep(boost::posix_time::seconds(5));
	// std::cout<<"Sono nel main dopo aver dormito 2s"<<std::endl;

	
	// //ros::Rate loop_rate(10);   	


	ros::spin();
	return 0;

	
}