//#include "camera_desperatehousewife.h"
#include "phobic_camera.h"
using Eigen::VectorXf ;



int main(int argc, char** argv)
{
	std::cout<<"Sono nel main"<<std::endl<<std::flush;
	ros::init(argc, argv, "Phobic_whife_scene");

	// ros::Publisher Scena_info;
	ros::Subscriber reader;
	// std::cout<<"Sono nel main prima di aver creato ogetto classe"<<std::endl;
	
	ros::NodeHandle nodeH;
	phobic_scene phobic_scene_local(nodeH); 
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


void phobic_scene::pointcloudCallback(sensor_msgs::PointCloud2 msg)
{
	std::cout<<"sono nella callback"<<std::endl;
	// create a new pointcloud from msg
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::fromROSMsg (msg, *scene);

	// visualization(scene);

	// phobic_scene_local.save_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  scene );
	// temp_point_cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// std::pair< pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ> > pc_temp;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_table (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_object (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc1 (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc2 (new pcl::PointCloud<pcl::PointXYZ>);

	erase_environment(scene);
	erase_table();
	getcluster();
  	
	fitting();
	
	// send_msg();
		//Move iterator forward to next label
    	// label_itr = supervoxel_clusters.upper_bound(supervoxel_label);	
    	// send_msg();
	//cloud_temp.push_back(cloud);
	// pc_temp=erase_table(cloud);
	
	// pcl::copyPointCloud<pcl::PointXYZ>( pc_temp.first, *pc_object);
	// pcl::copyPointCloud<pcl::PointXYZ>( pc_temp.second, *pc_table);

	// pcl::copyPointCloud<pcl::PointXYZ>( *pc_object, *cloud_temp);

}



void phobic_scene::erase_environment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  original_pc)
{
	std::cout<<"funzione togli ambiente"<<std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	//pcl::copyPointCloud<pcl::PointXYZ>(*original_pc, inliers, *environment);
	pcl::PassThrough<pcl::PointXYZRGBA> Filter_env;
		
	// x
	Filter_env.setInputCloud (original_pc);
	Filter_env.setFilterFieldName ("x");
	Filter_env.setFilterLimits (-1.0, 1.0);
	Filter_env.filter (*cloud_filtered);
	// The indices_x array indexes all points of cloud_in that have x between -1.0, 1.0
	
	//y
	Filter_env.setInputCloud (cloud_filtered);
	Filter_env.setFilterFieldName ("y");
	Filter_env.setFilterLimits (-1.0, 1.0);
	// The indices_x array indexes all points of cloud_in that have x between -1.0, 1.0
	Filter_env.filter (*cloud_temp);
		
	//z
	Filter_env.setInputCloud (cloud_temp);
	Filter_env.setFilterFieldName ("z");
	Filter_env.setFilterLimits (0.0, 1.0);
	// The indices_x array indexes all points of cloud_in that have x between 0.0, 1.0
	Filter_env.filter (*cloud_filtered);

	// copies point clound into another PointCloud (cloud is the class pc )
	pcl::copyPointCloud<pcl::PointXYZRGBA>( *cloud_filtered, *cloud);
	
	//visualization(cloud_filtered);

	//std::cout<<"finito funzione togli ambiente"<<std::endl;

}


void phobic_scene::fitting ()
{	
	std::cout<<"sono nel fitting"<<std::endl;
	
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree< pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree< pcl::PointXYZRGBA> ());
	pcl::NormalEstimation< pcl::PointXYZRGBA, pcl::Normal> ne;
	pcl::ExtractIndices< pcl::PointXYZRGBA> extract;
	pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_first (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	pcl::copyPointCloud<pcl::PointXYZRGBA>( *object_cluster.begin(), *cloud_first);
	std::cout<<"dimensione punti della prima pc"<<cloud_first->points.size()<<std::endl;
	// visualization(cloud_first);
	
	if (cloud_first->points.empty ())
	{
		std::cout<<"empty pointcloud"<<std::endl;

	}

	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_first);
	ne.setKSearch (100);
	ne.compute (*cloud_normals);

    std::cout << " number of input cloud" << cloud_normals->points.size() << std::endl;

    if (cloud_normals->points.empty ())
    {
        std::cout<<"empty normal pointcloud"<<std::endl;

    }

	// Create the segmentation object for cylinder segmentation and set all the parameters
	pcl::SACSegmentationFromNormals< pcl::PointXYZRGBA, pcl::Normal> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.2);
	seg.setInputCloud (cloud_first);
	seg.setInputNormals (cloud_normals);

	// // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
	std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    std::cout << "Number of inliers: " << inliers_cylinder->indices.size() << std::endl;
	
	if (inliers_cylinder->indices.size() == 0)
	{
	 	std::cout << "Could not find a cylinder in the scene." << std::endl;
	}
	
	else
	{
		// Write the cylinder inliers to disk
		std::cout << "cylinder found, extracting inliers" << std::endl;
 		extract.setInputCloud (cloud_first);
	 	extract.setIndices (inliers_cylinder);
	 	extract.setNegative (false);
	 	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGBA> ());
	 	extract.filter (*cloud_cylinder);

	 	if (cloud_cylinder->points.empty ())
		{
			std::cout<<"empty pointcloud"<<std::endl;

	 	}

	 	cyl_list.push_back(*cloud_cylinder);
 

	    //hand's info

	    //tf::Transform CylToHand_tr;    	
		hand_tr=CylToHand_Transform (coefficients_cylinder->values);

		std::cout<< "hand transformation rot "<< *hand_tr.getRotation()<< std::endl;
		std::cout<< "hand transformation rot "<< *hand_tr.getOrigin()<< std::endl;

		// remove the first elements in the list
		object_cluster.pop_front();
		//hand_tr=CylToHand_Transform (info_cil);
		
	  	//visualization(cylinder); 
		
		//return(CylToHand_tr);
	
	}
}


//tf::Transform CylToHand_Transform (const Eigen::VectorXf coeff)
tf::Transform CylToHand_Transform (const std::vector<float> coeff)
{
 	std::cout<<"creo matrice"<<std::endl;
  double x = coeff[0], y = coeff[1], z = coeff[2];
  double p_x = coeff[3], p_y = coeff[4], p_z = coeff[5];
  double r = coeff[6]; 

  // The position is directly obtained from the coefficients, and will be corrected later
  tf::Vector3 position(x,y,z);
  
  // w is the axis of the cylinder which will be aligned with the z reference frame of the cylinder
  tf::Vector3 w(p_x, p_y, p_z);
  tf::Vector3 u(1, 0, 0);
  tf::Vector3 v = w.cross(u).normalized();
  u = v.cross(w).normalized();
  tf::Matrix3x3 rotation;
  rotation[0] = u;  // x
  rotation[1] = v;  // y
  rotation[2] = w;  // z
  rotation = rotation.transpose(); //colum

  // Compose the transformation and return it
  return tf::Transform(rotation, position);
}


void  phobic_scene::send_msg()
{
 	std::cout<<"sono in send_msg"<<std::endl;
 	//create a msg
	//std_msgs::Float32 date_cyl;
	std_msgs::Float32MultiArray array; // pos_ori

    	for (int i=0; i<=2; i++)
    	{
    		//date_cyl=hand_tr.getOrigin()[i];
			array.data.push_back(hand_tr.getOrigin()[i]);
    	}

      	
    //date_cyl=hand_tr.getRotation().getX();
	array.data.push_back(hand_tr.getRotation().getX());
	//date_cyl=hand_tr.getRotation().getY();
	array.data.push_back(hand_tr.getRotation().getY());
	//date_cyl=hand_tr.getRotation().getZ();
	array.data.push_back(hand_tr.getRotation().getZ());
    //date_cyl=hand_tr.getRotation().getW();
	array.data.push_back(hand_tr.getRotation().getW());
		
	ROS_INFO("Info cylinder");
	//send a msg 
	Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>(nodeH.resolveName("markers_out"), 10);	
	//Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>("desperate_camera", "100");
	Scena_info.publish(array); 

}


void phobic_scene::getcluster()
{
	std::cout<<"sono dentro a getcluster"<<std::endl;
	bool check=true;

	// try with euclidean clusters
	// Creating the KdTree object for the search method of the extraction
  	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  	tree->setInputCloud (cloud);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  	ec.setClusterTolerance (0.02); // 2cm
  	ec.setMinClusterSize (100);
  	ec.setMaxClusterSize (25000);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);

  	
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
  		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
  		
  		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      	{	
      		cloud_cluster->points.push_back (cloud->points[*pit]); 
      		cloud_cluster->width = cloud_cluster->points.size ();
    		cloud_cluster->height = 1;
    		cloud_cluster->is_dense = true;

    	}
	 	

    		object_cluster.push_back(*cloud_cluster);
    }

	// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_first (new pcl::PointCloud<pcl::PointXYZRGBA>);
	// pcl::copyPointCloud<pcl::PointXYZRGBA>( *object_cluster.begin(), *cloud_first);
    // visualization(cloud_first);
    //object_cluster.pop_front();

	// if (start!=true)
	// {
	// 	check=check_change_pc();

	// }

	// create a cluster if only the point cloud is change
	// if (check == true)
	// {
	// 	// date
	// 	float color_importance=2.0f;
	// 	float spatial_importance=0.4f;
	// 	float normal_importance=1.0f;
	// 	float seed_resolution = 0.1f;
	// 	float voxel_resolution = 0.08f;

	// 	pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution);
	// 	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr>  supervoxel_clusters;
	  
	//    	super.setInputCloud (cloud);
	// 	super.setColorImportance (color_importance);
	//   	super.setSpatialImportance (spatial_importance);
	//   	super.setNormalImportance (normal_importance);
	// 	super.extract (supervoxel_clusters);

		
	// 	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	//   	super.getSupervoxelAdjacency (supervoxel_adjacency);
	//   	//To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
	//   	std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
	  	
	//   	for ( ; label_itr != supervoxel_adjacency.end (); )
	//   	{
	//   		 //First get the label
	//    		uint32_t supervoxel_label = label_itr->first;
	//    		//Now get the supervoxel corresponding to the label
	//    		pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

	//     	//Now we need to iterate through the adjacent supervoxels and make a point cloud of them
	//     	pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
	//     	std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
	    	
	//     	for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
	//     	{
	//     		  pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
	//     		  adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid);
	//    		}
	   	
	//    		// adds supervoxel in the object class list
	//    		object_cluster.push_back(adjacent_supervoxel_centers);
	// 		//Move iterator forward to next label
	//    		label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
	//    	}

	//    	std::cout<<"finito di creare cluster"<<std::endl;
	//    	std::cout<<"numero di cluster"<<object_cluster.size()<<std::endl;

	//    	start=false;
	   
	// }
}

// bool phobic_scene::check_change_pc()
// {

// }

void visualization(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr room_object)
//void  phobic_scene::visualization()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(room_object, 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGBA> (room_object, single_color, "sample cloud");
	// viewer.->addPointCloud<pcl::PointXYZ> (room_object, "nome");

  	while (!viewer->wasStopped ())
	{
   	viewer->spinOnce (100);
   	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}


}


//std::pair<pcl::PointCloud<pcl::PointXYZRGBA>,pcl::PointCloud<pcl::PointXYZRGBA> > erase_table ()
void phobic_scene::erase_table()
{
	// Create the segmentation object for the planar model and set all the parameters
  	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA> ());
 
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_PLANE);
 	seg.setMethodType (pcl::SAC_RANSAC);
 	seg.setMaxIterations (100);
 	seg.setDistanceThreshold (0.02);

  	// Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud);
    
  	//visualization(cloud);
 	
 // 	std::pair<pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointXYZ> > object_table;
 // 	pcl::copyPointCloud<pcl::PointXYZ>( *PC_w_env, object_table.first);
	// pcl::copyPointCloud<pcl::PointXYZ>( *table, object_table.second);

	// return 	object_table;

}
