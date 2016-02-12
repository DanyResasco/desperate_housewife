#include <fit_geometries.h>

#include <cylinder_fitting.hpp>
#include <sphere_fitting.hpp>

BasicGeometriesNode::BasicGeometriesNode()
{
  // initialize all topic and frame with file vito_controllers.yaml
  nh = ros::NodeHandle("BasicGeometriesNode_node");
  nh.param<std::string>("/BasicGeometriesNode/camera_topic", camera_topic_, "/scene_filter/scene_filtered");

  nh.param<std::string>("/BasicGeometriesNode/objects_topic", objects_topic_, "/BasicGeometriesNode/object_cloud");
  cloud_object_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >( objects_topic_.c_str(), 1 );

  nh.param<std::string>("/BasicGeometriesNode/makers_topic", objects_topic_, "/BasicGeometriesNode/markers");
  marker_publisher_ = nh.advertise<visualization_msgs::Marker >( objects_topic_.c_str(), 0 );

  std::string topic = nh.resolveName(camera_topic_.c_str());
  stream_subscriber_ = nh.subscribe(topic, 1, &BasicGeometriesNode::BasicGeometriesNodeCallback, this);

  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  geometries_publisher_ = nh.advertise<desperate_housewife::fittedGeometriesArray>( geometries_topic_.c_str(), 1 );

  nh.param<std::string>("/BasicGeometriesNode/cylinder_names", object_names_, "object");
  nh.param<int>("/BasicGeometriesNode/cluster_tolerance", min_cluster_size_, 200);
  nh.param<double>("/BasicGeometriesNode/min_cluster_size", cluster_tolerance_, 0.07);


}

void BasicGeometriesNode::BasicGeometriesNodeCallback(sensor_msgs::PointCloud2 msg)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OriginalScene (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromROSMsg (msg, *OriginalScene);
  camera_frame_ = OriginalScene->header.frame_id;

  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > cluster_vector = getClusters( OriginalScene );

  std::vector<geometry> geometries;

  if ( cluster_vector.size() > 0)
    {
      for (unsigned int i=0; i < cluster_vector.size(); i++)
      {
        geometry new_geometry = fitGeometry( cluster_vector[i] );
        if (new_geometry.geom_type >= 0)
        {
          geometries.push_back( new_geometry );
        }
      }
    }

  // ROS_DEBUG("Number of geometries %lu", geometries.size());

  if ( geometries.size() > 0 )
  {
    generateMarkerMessages( geometries );
    generateGeometriesMessages( geometries );
    // printGometriesInfo( geometries );
  }
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > BasicGeometriesNode::getClusters( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OriginalScene )
{
  // Make a cluster with euclidean clusters
  // Creating the KdTree object for the search method of the extraction
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > cluster_vector;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud (OriginalScene);

  // ROS_DEBUG("PointCloud in getClusters() has: %lu data points.", cloud_filtered->points.size ());
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> euclidean_cluster;
  euclidean_cluster.setClusterTolerance (cluster_tolerance_); // 5cm
  euclidean_cluster.setMinClusterSize (min_cluster_size_);
  euclidean_cluster.setMaxClusterSize (OriginalScene->points.size());
  euclidean_cluster.setSearchMethod (tree);
  euclidean_cluster.setInputCloud (OriginalScene);
  euclidean_cluster.extract (cluster_indices);

  ROS_DEBUG("Number of clusters %lu", cluster_indices.size());

  // from cluster[i] to point_cloud
  if (cluster_indices.size() > 0)
    {

      cluster_vector.clear();

      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);

          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;
          cloud_cluster->header.frame_id = OriginalScene->header.frame_id;

          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            {

              cloud_cluster->points.push_back (OriginalScene->points[*pit]);

            }
           
          cluster_vector.push_back(cloud_cluster);
        }
    }

    // ROS_INFO("Size cluster_vector %lu", cluster_vector.size());
    // for (unsigned int i = 0; i < cluster_vector.size() ; ++i)
    // {
    //   ROS_INFO("Num of points in cluster %d is %lu", i, cluster_vector[i]->points.size());
    // }

  return cluster_vector;
}

BasicGeometriesNode::geometry BasicGeometriesNode::fitGeometry( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OriginalCluster ){

  //std::vector<BasicGeometriesNode::geometry> new_geometry;
  //new_geometry.resize(2); // 2 is the number of geometry that i can try to fit
   BasicGeometriesNode::geometry cyl_geometry;
  //  double ratio_cyl;
  //  double ratio_sphere;
   // BasicGeometriesNode::geometry sphere_geometry;

  // TODO Here we can implement more BAsic Geometries.
  BasicGeometries::cylinder cylinder_local( OriginalCluster );

  if( cylinder_local.fitCylinder() )
    {
    //Cube = 1, Sphere = 2, Cylinder = 3, Cone = 11
      cyl_geometry.geom_type = 3;
      std::vector<double> cyl_info = cylinder_local.getInfo();
      double radius = cyl_info[0];
      cyl_geometry.geom_info = cyl_info;
      cyl_geometry.geom_info_marker.push_back( radius * 2 );
      cyl_geometry.geom_info_marker.push_back( radius * 2 );
      double height = cyl_info[1];
      cyl_geometry.geom_info_marker.push_back( height );
      cyl_geometry.geom_transformation = cylinder_local.getTransformation();
      cyl_geometry.geom_info.push_back(cyl_info[cyl_info.size() -1]); //ratio
      return cyl_geometry;
    }
  
  // BasicGeometries::sphere sphere_local( OriginalCluster );
  // if( sphere_local.fitSphere() )
  //   {
  //     sphere_geometry.geom_type = 2;
  //     std::vector<double> cyl_info = sphere_local.getInfo();
  //     double radius = cyl_info[0];
  //     sphere_geometry.geom_info = cyl_info;
  //     sphere_geometry.geom_info_marker.push_back( radius );
  //     sphere_geometry.geom_info_marker.push_back( radius );
  //     sphere_geometry.geom_info_marker.push_back( radius );
  //     sphere_geometry.geom_transformation = sphere_local.getTransformation();
  //     // ratio_sphere = cyl_info[cyl_info.size() -1];
  //     return sphere_geometry;
  //   }

  // if(ratio_cyl > ratio_sphere)
  // {    
     return cyl_geometry;
  // }
  // else
  // {
  //   return sphere_geometry;
  // }

 
}

void BasicGeometriesNode::generateMarkerMessages( std::vector<geometry> geometries )
{
  for (unsigned int i = 0; i < geometries.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = camera_frame_.c_str();
    marker.header.stamp = ros::Time();
    marker.ns = "";
    marker.id = i;
    marker.type = geometries[i].geom_type;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Pose pose = fromEigenMatrix4x4ToPose ( geometries[i].geom_transformation );
    marker.pose.position.x = pose.position.x;
    marker.pose.position.y = pose.position.y;
    marker.pose.position.z = pose.position.z;
    marker.pose.orientation.x = pose.orientation.x;
    marker.pose.orientation.y = pose.orientation.y;
    marker.pose.orientation.z = pose.orientation.z;
    marker.pose.orientation.w = pose.orientation.w;

    marker.scale.x = geometries[i].geom_info_marker[0];
    marker.scale.y = geometries[i].geom_info_marker[1];
    marker.scale.z = geometries[i].geom_info_marker[2];

    marker.color.a = 1.0; // for the clearness
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(1);
    marker_publisher_.publish(marker); 

    std::string cyl_name= object_names_.c_str() + std::string("_") + std::to_string(i);

    tf::Transform tfGeomTRansform;
    tf::poseMsgToTF( fromEigenMatrix4x4ToPose( geometries[i].geom_transformation ), tfGeomTRansform );
    tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), camera_frame_.c_str(), cyl_name.c_str()) );
  }
}

void BasicGeometriesNode::generateGeometriesMessages( std::vector<geometry> geometries){

  //send a geometry information (type, pose, info)  
  desperate_housewife::fittedGeometriesArray fittedGeometriesArrayMsg;

  for (unsigned int i=0; i < geometries.size(); i++)
  {
    desperate_housewife::fittedGeometriesSingle fittedGeometriesSingleMsg;
    fittedGeometriesSingleMsg.type = geometries[i].geom_type;
    fittedGeometriesSingleMsg.pose = fromEigenMatrix4x4ToPose( geometries[i].geom_transformation );
    // std::cout<<"pose in fitting: "<< geometries[i].geom_transformation <<std::endl;
    for (unsigned j=0; j < geometries[i].geom_info.size(); j++)
    {
      fittedGeometriesSingleMsg.info.push_back(geometries[i].geom_info[j]);
    }
    fittedGeometriesArrayMsg.geometries.push_back( fittedGeometriesSingleMsg );
  }

  geometries_publisher_.publish(fittedGeometriesArrayMsg);
  return;
}

geometry_msgs::Pose BasicGeometriesNode::fromEigenMatrix4x4ToPose( Eigen::Matrix4d &tranfs_matrix )
{

  geometry_msgs::Pose pose;
  Eigen::Quaterniond eugen_quat(tranfs_matrix.block<3,3>(0,0));
  pose.orientation.x = eugen_quat.x();
  pose.orientation.y = eugen_quat.y();
  pose.orientation.z = eugen_quat.z();
  pose.orientation.w = eugen_quat.w();
  pose.position.x = tranfs_matrix(0,3);
  pose.position.y = tranfs_matrix(1,3);
  pose.position.z = tranfs_matrix(2,3);

  return pose;
}

  
void BasicGeometriesNode::printGometriesInfo ( std::vector<geometry> geometries )
{
  ROS_INFO("Number of geometries %lu", geometries.size());
  //only per print the cylinder's informations
  for (unsigned int i = 0; i < geometries.size(); ++i)
  {
    if (geometries[i].geom_type == 3)
    {
      ROS_INFO("Number of parameters %lu in geometry %u", geometries[i].geom_info.size(), i);
      ROS_INFO("radius = %f", geometries[i].geom_info[0]);
      ROS_INFO("height = %f", geometries[i].geom_info[1]);
      ROS_INFO("isLying = %f", geometries[i].geom_info[2]);
      ROS_INFO("isFull = %f", geometries[i].geom_info[3]);
      ROS_INFO("ratio = %f", geometries[i].geom_info[4]);
      ROS_INFO(" ");
    }
    else
    {
      ROS_INFO("Geometry %u is not a cylinder", i);
    }
  }
}