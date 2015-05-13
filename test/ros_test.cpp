#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <chrono>
// #include <thread>
// #include <boost/thread/thread.hpp>
//#include "test_classe.h"
pcl::PointCloud<pcl::PointXYZ>::Ptr erase_environment(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud );

class phobic_scene
{
	
	private:
		ros::NodeHandle nodeH;
		ros::Publisher Scena_info;
		//ros::Subscriber reader;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ;
		std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_temp;
		tf::Transform hand_tr;
	
	
	public:
		
		void pointcloudCallback(sensor_msgs::PointCloud2 msg);
		//void pub_transf();
		//tf::Transform fitting (pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud );
		tf::Transform fitting ();
		void send_msg();

		// phobic_scene(){}
		phobic_scene(ros::NodeHandle NodeH): nodeH(NodeH)
		{
			pcl::PointCloud<pcl::PointXYZ> cloud2;
			cloud=cloud2.makeShared();
			//Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>("desperate_camera", "100");
			//Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>(nodeH.resolveName("markers_out"), 10);
			
		}

		~phobic_scene(){}


};

int main(int argc, char **argv)
{
	std::cout<<"Sono nel main"<<std::endl;
	ros::init(argc, argv, "Phobic_whife_scene");

	//ros::Publisher Scena_info;
	ros::Subscriber reader;
	ros::NodeHandle nodeH;
	phobic_scene phobic_scene_local(nodeH); 
	std::cout<<"Sono nel main prima di aver creato ogetto classe"<<std::endl;
	//leggo();
	//ros::NodeHandle nodeH;
	//phobic_scene phobic_scene_local(nodeH); 
	std::cout<<"Sono nel main dopo aver creato ogetto classe"<<std::endl;
	reader = nodeH.subscribe(nodeH.resolveName("camera/depth_registered/points"), 10, &phobic_scene::pointcloudCallback, &phobic_scene_local);

	std::cout<<"Sono nel main dopo aver letto"<<std::endl;
	//std::this_thread::sleep_for(std::chrono::seconds(2));
	ros::spin();
	return 0;

	
}


void phobic_scene::pointcloudCallback(sensor_msgs::PointCloud2 msg)
{
	std::cout<<"sono nella callback"<<std::endl;
		// create a new pointcloud from msg
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (msg, *scene);
	
	// phobic_scene_local.save_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  scene );
	// temp_point_cloud
	//pcl::PointCloud<pcl::PointXYZ> temp (new pcl::PointCloud<pcl::PointXYZ>);
	// std::pair< pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ> > pc_temp;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_table (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_object (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc1 (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc2 (new pcl::PointCloud<pcl::PointXYZ>);

	cloud=erase_environment(scene);
	
	std::cout<<"finito tolgo tavolo"<<std::endl;


}

pcl::PointCloud<pcl::PointXYZ>::Ptr erase_environment(pcl::PointCloud<pcl::PointXYZ>::Ptr  original_pc )
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
	// copies all inliers of the model computed to another PointCloud
	//pcl::copyPointCloud<pcl::PointXYZ>(*original_pc, inliers, *environment);
	pcl::PassThrough<pcl::PointXYZ> Filter_env;
		
	// DEVI METTERE ANCORA LE MISURE DEL TAVOLO 1.0 è un metro

	// x
	Filter_env.setInputCloud (original_pc);
	Filter_env.setFilterFieldName ("x");
	Filter_env.setFilterLimits (-1.0, 1.0);
	Filter_env.filter (*cloud_filtered);
	// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
	//indices_rem = Filter_env.getRemovedIndices ();
	
	//y
	Filter_env.setInputCloud (cloud_filtered);
	Filter_env.setFilterFieldName ("y");
	Filter_env.setFilterLimits (-1.0, 1.0);
	// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
	Filter_env.filter (*cloud_temp);
	// The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
	//indices_rem = Filter_env.getRemovedIndices ();
	
	//z
	Filter_env.setInputCloud (cloud_temp);
	Filter_env.setFilterFieldName ("z");
	Filter_env.setFilterLimits (0.0, 1.0);
	//Filter_env.filter (*indices_z);
	// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
	//indices_rem = Filter_env.getRemovedIndices ();
	Filter_env.filter (*cloud_filtered);

	return cloud_filtered;


}