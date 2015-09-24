#ifndef FIT_GEOMETRIES_H
#define FIT_GEOMETRIES_H

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/Marker.h>

#include <desperate_housewife/fittedGeometriesArray.h>


class BasicGeometriesNode {

  public:

  ros::NodeHandle nh;
  std::string camera_topic_, objects_topic_, camera_frame_, object_names_, geometries_topic_;

  ros::Publisher cloud_object_publisher_, marker_publisher_, geometries_publisher_;
  ros::Subscriber stream_subscriber_;
  tf::TransformBroadcaster tf_geometriesTransformations_;


  struct geometry
  {
    int geom_type;
    Eigen::Matrix4d geom_transformation;
    std::vector<double> geom_info;
    std::vector<double> geom_info_marker;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr geom_points_global;
    geometry(){geom_type = -1;}
  };

  BasicGeometriesNode();
    ~BasicGeometriesNode(){};

  private:
  
    void BasicGeometriesNodeCallback(sensor_msgs::PointCloud2 msg);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > getClusters( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OriginalScene );
    geometry fitGeometry( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OriginalCluster );
    void generateMarkerMessages( std::vector<geometry> geometries );
    void generateGeometriesMessages( std::vector<geometry> geometries );
    geometry_msgs::Pose fromEigenMatrix4x4ToPose( Eigen::Matrix4d &tranfs_matrix );
    void printGometriesInfo( std::vector<geometry> geometries );

    int min_cluster_size_;
    double cluster_tolerance_;

};

#endif
