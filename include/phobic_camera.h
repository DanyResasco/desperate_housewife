#ifndef dany_bicchi_test_classe_h
#define dany_bicchi_test_classe_h

//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <desperate_housewife/cyl_info.h>
#include <visualization_msgs/Marker.h>

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
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/norms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree.h>


#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <utility>
#include <list>

//Eigen
#include <Eigen/Dense>
#include <Eigen/LU>

//Tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


class phobic_scene
{
	
private:
	ros::NodeHandle nodeH;
	ros::Publisher Scena_info;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ;
	std::vector<float> Plane_coeff;
	std::list<pcl::PointCloud<pcl::PointXYZRGBA> > object_cluster;
	bool testing;
	ros::Publisher phobic_talk;

	struct Mod_cylinder
	{
		 	// double tetha;
		double height;
		double radius;
		pcl::ModelCoefficients cylinder_coeff;
		pcl::PointXYZ info_disegno_cyl_up;
		pcl::PointXYZ info_disegno_cyl_dw;
		pcl::PointXYZ center;
		Eigen::Matrix<double,4,4> Matrix_transform_inv;
		geometry_msgs::Pose Cyl_pose;

	} CYLINDER;

	std::vector<Mod_cylinder> cyl_list; 

	
public:

	void pointcloudCallback(sensor_msgs::PointCloud2 msg);
	void fitting ();
	void send_msg( int cyl_id = 0 );
	void erase_environment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pc);
	void getcluster();
	void erase_table();
	void makeInfoCyl(std::vector<float> coeff, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_cyl);
	bool fromEigenToPose(Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &pose);
	void visualization( bool testing, bool circle );

	phobic_scene(ros::NodeHandle NodeH, bool test): nodeH(NodeH)
	{

		pcl::PointCloud<pcl::PointXYZRGBA> cloud2;
		cloud=cloud2.makeShared();
		phobic_talk = nodeH.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

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
Eigen::Matrix4d Cyl_Transform (const std::vector<float>  coeff);

//Function: visualization
//input: point cloud
//Description: this funciont called the pcl visualizer
// void visualization(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr room_object, bool window );







#endif
// dany_bicchi__test_classe_h