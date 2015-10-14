#ifndef SPHERE_FITTING_H
#define SPHERE_FITTING_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <iterator> 

namespace BasicGeometries{

  class sphere{

  public:

    std::vector<double> getInfo(){return info_;};
    Eigen::Matrix4d getTransformation(){return transformation_;};
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPoints(){return sphere_points_;};
    bool fitSphere();
    

    sphere( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original_cloud )
    {
      original_cloud_ = original_cloud; 
    };
    ~sphere(){};
    
  private:

    Eigen::Matrix4d transformation_;
    std::vector<double> info_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sphere_points_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original_cloud_;
    
  };


  bool sphere::fitSphere(){    

    transformation_ = Eigen::Matrix4d::Identity();

    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne; //
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg; //
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;//
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);//
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);

    ne.setSearchMethod (tree);
    ne.setInputCloud (original_cloud_);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.5);
    seg.setInputCloud (original_cloud_);
    seg.setInputNormals (cloud_normals);

    // Obtain the sphere inliers and coefficients
    seg.segment (*inliers_sphere, *coefficients_sphere);

    extract.setInputCloud (original_cloud_);
    extract.setIndices (inliers_sphere);
    extract.setNegative (true);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sphere_points (new pcl::PointCloud<pcl::PointXYZRGBA>);
    extract.filter ( *sphere_points );

    sphere_points_ = sphere_points;

    if (sphere_points_->points.empty ()) 
    {
      return false;
    }

    double radius = coefficients_sphere->values[3];
    info_.push_back( radius ); //get radius

    double x = coefficients_sphere->values[0], y = coefficients_sphere->values[1], z = coefficients_sphere->values[2];

    transformation_(0,3) = x;
    transformation_(1,3) = y;
    transformation_(2,3) = z;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sphere_points_local (new pcl::PointCloud<pcl::PointXYZRGBA>);
    Eigen::Matrix4d transformation_i = transformation_.inverse();
    pcl::transformPointCloud(*original_cloud_, *sphere_points_local, transformation_i);
    sphere_points_ = sphere_points_local;
    //
    int index = info_.size()-1;
    info_.push_back((((double)inliers_sphere->indices.size()) / ((double)sphere_points->points.size())));
  

    return true;
  }

}

#endif