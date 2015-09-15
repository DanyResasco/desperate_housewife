#ifndef PHOBIC_CAMERA_H
#define PHOBIC_CAMERA_H

//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/UInt32.h"

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
#include <pcl/sample_consensus/lmeds.h>
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

// #include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <utility>
#include <list>
#include <string>
#include <math.h> 

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
  tf::TransformBroadcaster tf_br;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ;
  std::vector<float> Plane_coeff;
  std::list<pcl::PointCloud<pcl::PointXYZRGBA> > object_cluster;
  bool testing;
  ros::Publisher phobic_talk;
  ros::Publisher pub_cloud_object;
  ros::Publisher tf_pose_cyl;
  ros::Publisher num_cyl;
  double down_sample_size;
  double pc_save;
  Eigen::Matrix4d T_k_g;


  struct Mod_cylinder
  {
    double Info;
    double height;
    double radius;
    double vol;
    pcl::ModelCoefficients cylinder_coeff;
    pcl::PointXYZ info_disegno_cyl_up;
    pcl::PointXYZ info_disegno_cyl_dw;
    pcl::PointXYZ center;
    Eigen::Matrix<double,4,4> Matrix_transform_inv;
    geometry_msgs::Pose Cyl_pose;
    tf::Transform cyl_tf_pose;

  } CYLINDER;

  std::vector<Mod_cylinder> cyl_list;


public:

  void pointcloudCallback(sensor_msgs::PointCloud2 msg);
  void fitting ();
  void send_msg( int cyl_id = 0 );
  void erase_environment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pc);
  void getcluster();
  void erase_table();
  void makeInfoCyl(std::vector<float> coeff, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_cyl, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scene);
  bool fromEigenToPose(Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &pose);
  void visualization( bool testing, bool circle );
  Eigen::Matrix4d Cyl_Transform (const std::vector<float>  coeff);
  int FullorEmpty(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2);
  void StandingOrLying(Eigen::Matrix4d &T_G_K);
  std::string cylinders_topic;
  std::string camera_topic;
  std::string camera_frame;

  phobic_scene(ros::NodeHandle NodeH, bool test): nodeH(NodeH)
  {

    pcl::PointCloud<pcl::PointXYZRGBA> cloud2;
    cloud=cloud2.makeShared();
    phobic_talk = nodeH.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    pub_cloud_object = nodeH.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >( "object_cloud", 1 );
    num_cyl = nodeH.advertise<desperate_housewife::cyl_info>( "INFO_CYLINDER", 1 );
    ros::param::get("~down_sample_size", down_sample_size);
    ROS_INFO("Down sample size %lf", down_sample_size);
    ros::param::get("~pc_save", pc_save);


  }

  ~phobic_scene(){}


};

std::vector<double> findHeight(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_t);

#endif
