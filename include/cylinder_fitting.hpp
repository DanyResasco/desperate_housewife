#ifndef CYLINDER_FITTING_H
#define CYLINDER_FITTING_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

namespace BasicGeometries{

  class cylinder{

  public:

    std::vector<double> getInfo(){return info_;};
    Eigen::Matrix4d getTransformation(){return transformation_;};
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPoints(){return cylinder_points_;};
    bool fitCylinder();
    

    cylinder( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original_cloud )
      {
        original_cloud_ = original_cloud; 
      };
    ~cylinder(){};
    
  private:

    Eigen::Matrix4d transformation_;
    std::vector<double> info_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cylinder_points_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original_cloud_;
    
  };


  bool cylinder::fitCylinder(){    

    transformation_ = Eigen::Matrix4d::Identity();

    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne; //
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg; //
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;//
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);//
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    ne.setSearchMethod (tree);
    ne.setInputCloud (original_cloud_);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.5);
    seg.setInputCloud (original_cloud_);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    extract.setInputCloud (original_cloud_);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cylinder_points (new pcl::PointCloud<pcl::PointXYZRGBA>);
    extract.filter ( *cylinder_points );
    cylinder_points_ = cylinder_points;

    if (cylinder_points_->points.empty ()) 
    {
      return false;
    }

    double radius = coefficients_cylinder->values[6];
    info_.push_back( radius ); //get radius

    // Find transformation

    double x = coefficients_cylinder->values[0], y = coefficients_cylinder->values[1], z = coefficients_cylinder->values[2];
    double p_x = coefficients_cylinder->values[3], p_y = coefficients_cylinder->values[4], p_z = coefficients_cylinder->values[5];

    Eigen::Vector3d position(x,y,z); //random position on the axis
    // w is the axis of the cylinder which will be aligned with the z reference frame of the cylinder
    Eigen::Vector3d w(p_x, p_y, p_z);
    Eigen::Vector3d u(0, 0, 1); // -y kinect

    // is object if twisted the z axis is the same to x axis. we change the x axis with a orthogonal vector
    
    if((u.dot(w) < 0))
      {
        w = w * -1; //change sign
      }

    Eigen::Vector3d x_new = (w.cross(u)).normalized();
    Eigen::Vector3d y_new = (x_new.cross(w)).normalized();
    
    Eigen::Matrix3d rotation;
    rotation.col(0) = - x_new;  // x
    rotation.col(1) = y_new;  // y
    rotation.col(2) = w;  // z

    // Compose the transformation and return it


    transformation_.row(0) << rotation.row(0), position[0];
    transformation_.row(1) << rotation.row(1), position[1];
    transformation_.row(2) << rotation.row(2), position[2];
    transformation_.row(3) << 0,0,0, 1;

    // Find Height

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cylinder_points_local (new pcl::PointCloud<pcl::PointXYZRGBA>);
    Eigen::Matrix4d transformation_i = transformation_.inverse();
    pcl::transformPointCloud(*cylinder_points_, *cylinder_points_local, transformation_i);

    double z_min, z_max;

    z_min= cylinder_points_local->points[0].z;
    z_max= cylinder_points_local->points[0].z;
    
    for (unsigned int i=1; i< cylinder_points_local->points.size(); i++)
    {
      if(z_min > cylinder_points_local->points[i].z )
      { 
        z_min = cylinder_points_local->points[i].z;
        
      }

      if(z_max < cylinder_points_local->points[i].z)
      { 
        z_max = cylinder_points_local->points[i].z;
        
      }
    }

    double height = z_max - z_min;
    info_.push_back( height );

    // find cylinder center since 
    double x_center = 0. , y_center = 0., z_center = z_min + ( height ) / 2.;

    Eigen::Matrix4d centerTrasformation_local = Eigen::Matrix4d::Identity();

    centerTrasformation_local(0,3) = x_center;
    centerTrasformation_local(1,3) = y_center;
    centerTrasformation_local(2,3) = z_center;

    Eigen::Matrix4d centerTrasformation_global = transformation_ * centerTrasformation_local;
    transformation_ = centerTrasformation_global;
    transformation_i = transformation_.inverse();

    pcl::transformPointCloud(*original_cloud_, *cylinder_points_, transformation_i);

    // Identify if the cylinder is standing or lying 

    // // Eigen::Vector3d u(0,-1,0);
    // Eigen::Vector3d plane_coeff_kinect_frame, temp_n2;
    // Eigen::Vector4d normal_plane_cyl_frame, temp_normal_plane;
    // plane_coeff_kinect_frame(0) = Plane_coeff[0];
    // plane_coeff_kinect_frame(1) = Plane_coeff[1];
    // plane_coeff_kinect_frame(2) = Plane_coeff[2];


    // if ((u.dot(plane_coeff_kinect_frame)) <0) // in kinect frame
    // {
    //   plane_coeff_kinect_frame = plane_coeff_kinect_frame * -1;
    // }

    // // in cyl frame
    // temp_normal_plane << plane_coeff_kinect_frame, 1;
    // normal_plane_cyl_frame = T_G_K * temp_normal_plane;
    // normal_plane_cyl_frame.normalize();

    Eigen::Vector3d nomal_plani_in_cyl_frame = transformation_i.block<3,3>(0,0) * Eigen::Vector3d( 0, 0, 1);
    nomal_plani_in_cyl_frame.normalize();

    Eigen::Vector3d vec_temp;
    vec_temp = nomal_plani_in_cyl_frame;
    // Eigen::Vector3d axis_cyl_frame(0,0,1);
    // vec_temp(0) = nomal_plani_in_cyl_frame(0);
    // vec_temp(1) = nomal_plani_in_cyl_frame(1);
    // vec_temp(2) = nomal_plani_in_cyl_frame(2);

    vec_temp.normalize();
    double dotproduct = vec_temp.dot(Eigen::Vector3d(0,0,1));

    double theta = std::acos(dotproduct);

    double isLaying = 0.0;
    // if(((theta >= 0.) && (theta<45.*(3.14/180.))) || ((theta <0) && (theta > -45.*(3.14/180.))))
    if(((theta >= -45.*(3.14/180.)) && (theta <= 45.*(3.14/180.))))
    {
      isLaying = 0.;
    }
    else
    {
      isLaying = 1.0;
    }

    info_.push_back( isLaying );

    double isFull = 1.;

    if ( isLaying == 0.)
    {
      // Check if the cylinder is empty

      pcl::PointXYZRGBA point_center; //punto in mezzo

      z_max= cylinder_points_->points[0].z;
      for (unsigned int i=1; i< cylinder_points_->points.size(); i++)
      {
        if(z_max < cylinder_points_->points[i].z)
        { 
          z_max = cylinder_points_->points[i].z;
          
        }
      }

      point_center.x = 0.; // TO Check
      point_center.y = 0.;
      point_center.z = z_max;

      std::vector< int >  k_indices;
      std::vector< float > k_sqr_distances;
      double dim_s;

      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
      tree->setInputCloud ( cylinder_points_ );

      dim_s = radius*0.6; //less than half

      tree->radiusSearch( point_center, dim_s, k_indices, k_sqr_distances,0 );

      if(k_indices.size() < (dim_s*300))
        {
          isFull = 0.; // empty
        }

    }

    info_.push_back( isFull );
    /* info contains: radius, height, isLying, isfull */

    return true;
  }

}

#endif