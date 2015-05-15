#ifndef dany_bicchi_test_classe_h
#define dany_bicchi_test_classe_h

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

//PCL
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <utility>
#include <list>
// #include <chrono>
// #include <thread>


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


class phobic_scene
{
	
	private:
		ros::NodeHandle nodeH;
		ros::Publisher Scena_info;
		//ros::Subscriber reader;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ;
		tf::Transform hand_tr;
		std::list<pcl::PointCloud<pcl::PointXYZRGBA> > object_cluster;
		std::list<pcl::PointCloud<pcl::PointXYZRGBA> > cyl_list;
		bool start;


	
	public:
		
		void pointcloudCallback(sensor_msgs::PointCloud2 msg);
		//void pub_transf();
		//tf::Transform fitting (pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud );
		//tf::Transform fitting ();
		void fitting ();
		void send_msg();
		void erase_environment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pc);
		bool check_change_pc();
		void getcluster();
		void erase_table();
		//void visualization();

		// phobic_scene(){}
		phobic_scene(ros::NodeHandle NodeH): nodeH(NodeH)
		{
			pcl::PointCloud<pcl::PointXYZRGBA> cloud2;
			cloud=cloud2.makeShared();
			start=true;
			//Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>("desperate_camera", "100");
			//Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>(nodeH.resolveName("markers_out"), 10);
			
		}

		~phobic_scene(){}


};



//function:pointcloudCallback
//input: sensor_msgs by kinect
//Description: this function creates a poincloud whit kinect's sensor_msgs and called other functions
//void pointcloudCallback(sensor_msgs::PointCloud2 msg);

//void pointcloudCallback(sensor_msgs::PointCloud2 msg);


//Fucntion: erase_table
//input: ptr point cloud
// output: couple of point cloud
//Description: this function fitting a plane into point cloud and remove it. The inlier into plane will be save into another point cloud
			// In the end we have two point cloud, first with the object anc second with table
//std::pair <pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointXYZ> > erase_table (pcl::PointCloud<pcl::PointXYZ>::Ptr PC_w_env);

//Fucntion: erase_environment
//input: ptr point cloud
// output:  point cloud
//Description: this function utilize a passthrough filter for cancel the environment
//pcl::PointCloud<pcl::PointXYZ>::Ptr erase_environment(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud );


//Function: fitting
//input: object point cloud
//Description: this function fitting a cylinder into point cloud and publish to Ros topic a hand-transformation
//tf::Transform fitting (pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud , ros::NodeHandle nodeH, ros::Publisher Scena_info);

//Funtion: CylToHand_Transform 
//input: cylinder's coefficients
//ouput: hand transformation
//Description: this function return a hand-transformation
tf::Transform CylToHand_Transform (const std::vector<float>  coeff);

//Function: visualization
//input: point cloud
//Description: this funciont called the pcl visualizer
void visualization(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr room_object);





#endif
// dany_bicchi__test_classe_h