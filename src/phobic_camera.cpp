//#include "camera_desperatehousewife.h"
#include "phobic_camera.h"
using Eigen::VectorXf ;



void phobic_scene::pointcloudCallback(sensor_msgs::PointCloud2 msg)
{
	std::cout<<"sono nella callback"<<std::endl;
	// create a new pointcloud from msg
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::fromROSMsg (msg, *scene);
	
	pcl::copyPointCloud<pcl::PointXYZRGBA>( *scene, *cloud);
	visualization(true , false);
	erase_environment(scene);
	visualization(true, false);
	erase_table();
	visualization(true, false);
	getcluster();
	visualization(false, false);
  	fitting();
  	visualization(true, true);
	send_msg();

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
	Filter_env.setFilterLimits (-0.2, 0.7);
	Filter_env.filter (*cloud_filtered);
	// The indices_x array indexes all points of cloud_in that have x between -0.2, 0.7
	
	//y
	Filter_env.setInputCloud (cloud_filtered);
	Filter_env.setFilterFieldName ("y");
	Filter_env.setFilterLimits (-1.0, 1.0);
	// The indices_x array indexes all points of cloud_in that have x between -1.0, 1.0
	Filter_env.filter (*cloud_temp);
		
	//z
	Filter_env.setInputCloud (cloud_temp);
	Filter_env.setFilterFieldName ("z");
	Filter_env.setFilterLimits (0.2, 1.0);
	// The indices_x array indexes all points of cloud_in that have x between 0.2, 1.0
	Filter_env.filter (*cloud);

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
	seg.setDistanceThreshold (0.05);	//distance between two points
    seg.setRadiusLimits (0, 0.2);	//cilynder's radius 20cm
	seg.setInputCloud (cloud_first);
	seg.setInputNormals (cloud_normals);

	//// Obtain the cylinder inliers and coefficients
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

	 	pcl::copyPointCloud<pcl::PointXYZRGBA>( *cloud_cylinder, *cloud);
		 	
	 	// take the coefficients and make a transformation matrix from camera_frame to axis_cylinder's_frame
	 	makeInfoCyl(coefficients_cylinder->values, cloud_cylinder);
	 
	 	
		// remove the first elements in the list
		object_cluster.pop_front();
	
	
	}
}

void phobic_scene::makeInfoCyl(std::vector<float> coeff , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_cyl)
{
	// cylinder's coefficients
	// double x = coeff[0], y = coeff[1], z = coeff[2];	// cordinate di un punto sull asse
	// double p_x = coeff[3], p_y = coeff[4], p_z = coeff[5];	// componenti dell asse realtive alla telecamera
	// double r = coeff[6];

	CYLINDER.radius = coeff[6];
	Eigen::Matrix<double,4,4> Matrix_transform;
	Matrix_transform = Cyl_Transform( coeff);
	CYLINDER.Matrix_transform_inv = Matrix_transform.inverse();

	//create a new frame

	double PI= 3.14159265;

	Eigen::Vector3d Plane_normal, u(0,0,1);
	Plane_normal[0] = Plane_coeff [0];
	Plane_normal[1] = Plane_coeff[1];
	Plane_normal[2] = Plane_coeff[2];

	
	CYLINDER.tetha= acos(Plane_normal.dot(u)) * 180/PI;
	std::cout<<"thetha: "<< CYLINDER.tetha <<std::endl;

	if((CYLINDER.tetha > 90) && (CYLINDER.tetha < 180) )
	{
		CYLINDER.tetha = 180-CYLINDER.tetha;
		std::cout<<"cambiato segno a theta"<<std::endl;
		std::cout<<"thetha: "<<CYLINDER.tetha<<std::endl;
	}
	
	std::cout<<"finito di creare matrice di rotazione"<<std::endl;
	
	//create a new cylinder pointcloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CYl_new_transf (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	pcl::transformPointCloud(*pc_cyl, *CYl_new_transf, CYLINDER.Matrix_transform_inv);
	pcl::copyPointCloud<pcl::PointXYZRGBA>( *CYl_new_transf, *cloud);


	//find height 
	double z_min, z_max, index_up, index_down;
	z_min= CYl_new_transf->points[0].z;
	z_max= CYl_new_transf->points[0].z;
	

	for (int i=1; i< CYl_new_transf->points.size(); i++)
	{
		if(z_min > CYl_new_transf->points[i].z )
		{	
			z_min = CYl_new_transf->points[i].z;
			index_down = i;

		}

		if(z_max < CYl_new_transf->points[i].z)
		{	
			z_max = CYl_new_transf->points[i].z;
			index_up = i; 
		}
	}


	CYLINDER.height= z_max - z_min;
	std::cout<< "altezza: "<< CYLINDER.height <<std::endl;


	CYLINDER.info_disegno_cyl_up.x = CYl_new_transf->points[index_up].x; 
	CYLINDER.info_disegno_cyl_dw.x = CYl_new_transf->points[index_down].x;
	CYLINDER.info_disegno_cyl_up.y = CYl_new_transf->points[index_up].y; 
	CYLINDER.info_disegno_cyl_dw.y = CYl_new_transf->points[index_down].y;
	CYLINDER.info_disegno_cyl_up.z = z_max; 
	CYLINDER.info_disegno_cyl_dw.z = z_min;

	// std::cout<<"finito creare info cerchio"<<std::endl;

	bool ok_transf;
	ok_transf = fromEigenToPose(CYLINDER.Matrix_transform_inv , CYLINDER.Cyl_pose );
	

  		// pcl::ModelCoefficients cylinder_coeff;
		CYLINDER.cylinder_coeff.values.resize (7);    // We need 7 values
		CYLINDER.cylinder_coeff.values[0] = 0;
		CYLINDER.cylinder_coeff.values[1] = 0;
		CYLINDER.cylinder_coeff.values[2] = 0;
		CYLINDER.cylinder_coeff.values[3] = 0;
		CYLINDER.cylinder_coeff.values[4] = 0;
		CYLINDER.cylinder_coeff.values[5] = 1;
		CYLINDER.cylinder_coeff.values[6] = CYLINDER.radius;



		cyl_list.push_back(CYLINDER);

	// //new coefficients with the new cylinder's frame (per me è inutile)

	// Eigen::Matrix<double, 4,1> Point_new_cyl;
	// Eigen::Matrix<double, 4,1> temp_old_cyl;
	// Eigen::Matrix<double, 4,1> Vers_new_cyl;

	// temp_old_cyl.col(0) << coeff[0], coeff[1], coeff[2], 0;
		
	// Point_new_cyl=(CYLINDER.M_rot_inv * temp_old_cyl);
	// //std::cout<<"finito prima molt"<<std::endl;

	// temp_old_cyl.col(0) << coeff[3], coeff[4], coeff[5], 0;

	// Vers_new_cyl=(CYLINDER.M_rot_inv * temp_old_cyl);
	// //std::cout<<"finito sec molt"<<std::endl;

	// std::vector<float> v;

	// v.push_back(Point_new_cyl(0,0));
	// v.push_back(Point_new_cyl(1,0));
	// v.push_back(Point_new_cyl(2,0));
	// v.push_back(Vers_new_cyl(0,0));
	// v.push_back(Vers_new_cyl(1,0));
	// v.push_back(Vers_new_cyl(2,0));
	// v.push_back(r);	//same radius
	// std::cout<<"finito creazione vettore "<<std::endl;

	// for(int i=0; i<=v.size(); i++)
	// {
	// 	std::cout<<"dati vettore coef new: "<< v[i] <<std::endl;

	// }

	// return v;

}


bool phobic_scene::fromEigenToPose(Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &pose)
{
	std::cout<<"dentro from"<<std::endl;
	Eigen::Matrix<double,3,3> Tmatrix;
	Tmatrix = tranfs_matrix.block<3,3>(0,0) ;
	Eigen::Quaterniond cyl_quat_eigen(Tmatrix);
	//std::cout<<"creato quaternione"<<std::endl;
	//cyl_quat_eigen = Quaternion(Tmatrix);
	pose.orientation.x = cyl_quat_eigen.x();
	pose.orientation.y = cyl_quat_eigen.y();
	pose.orientation.z = cyl_quat_eigen.z();
	pose.orientation.w = cyl_quat_eigen.w();
	pose.position.x = tranfs_matrix(0,3);
	pose.position.y = tranfs_matrix(1,3);
	pose.position.z = tranfs_matrix(2,3);


	return true;

}


void  phobic_scene::send_msg()
{
 	std::cout<<"sono in send_msg"<<std::endl;
 	ros::Publisher phobic_talk;
 	//create a msg

 	for (int i=0; i <= cyl_list.size(); i++)
 	{	
 		ROS_INFO("Info cylinder");
 		desperate_housewife::cyl_info msg;
 		msg.id = i;
 		msg.angle = cyl_list[i].tetha;
 		msg.length = cyl_list[i].height;
 		msg.radius = cyl_list[i].radius;
 		msg.transformation.position = cyl_list[i].Cyl_pose.position;
 		msg.transformation.orientation = cyl_list[i].Cyl_pose.orientation;  
 		
		phobic_talk = nodeH.advertise<desperate_housewife::cyl_info>(nodeH.resolveName("phobic_info"), 10);	
		phobic_talk.publish(msg); 
 	}

 	cyl_list.clear();
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
  	ec.setClusterTolerance (0.07); // 5cm
  	ec.setMinClusterSize (100);
  	ec.setMaxClusterSize (cloud->points.size());
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

    ////for visualizer the clusters
	// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_first (new pcl::PointCloud<pcl::PointXYZRGBA>);
	// pcl::copyPointCloud<pcl::PointXYZRGBA>( *object_cluster.begin(), *cloud_first);

	// if(cloud_first->points.size()==0)
	// {
	// 	std::cout<<"pc vuota "<<std::endl;

	// }

 //    visualization(cloud_first);
 //    std::cout<<"sono dentro get cluster e ho finito di creare i cluster"<<std::endl;
 //    object_cluster.pop_front();

	// if (start!=true)
	// {
	// 	check=check_change_pc();

	// }
    //test with supervoxel cluster
		//create a cluster if only the point cloud is change
	// if (check == true)
	// {
	// 	// date
		// float color_importance=0.4f;
		// float spatial_importance=0.2f;
		// float normal_importance=0.4f;
		// float seed_resolution = 0.20f;
		// float voxel_resolution = 0.01f;

		// pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution);
		// std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr>  supervoxel_clusters;
	  
	 //   	super.setInputCloud (cloud);
		// super.setColorImportance (color_importance);
	 //  	super.setSpatialImportance (spatial_importance);
	 //  	super.setNormalImportance (normal_importance);
		// super.extract (supervoxel_clusters);

		
		// std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	 //  	super.getSupervoxelAdjacency (supervoxel_adjacency);
	 //  	//To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
	 //  	std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
	  	
	 //  	for ( ; label_itr != supervoxel_adjacency.end (); )
	 //  	{
	 //  		 //First get the label
	 //   		uint32_t supervoxel_label = label_itr->first;
	 //   		//Now get the supervoxel corresponding to the label
	 //   		pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

	 //    	//Now we need to iterate through the adjacent supervoxels and make a point cloud of them
	 //    	pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
		// 	//std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
	 //    	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> ::iterator label_itr=supervoxel_clusters.begin();
	    
		// for ( ; label_itr!=supervoxel_clusters.end();)
	 //    	{
	 //    		 //first get the label
  //   			uint32_t supervoxel_label= label_itr->first;
  //   			//now get hte supervoxel corresponding to the label
  //   			pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel=supervoxel_clusters.at(supervoxel_label);
  //   			//adds supervoxel in the object class list
  //   			object_cluster.push_back(*supervoxel->voxels_);
  //   			//move iterator forward to next label
  //   			label_itr=supervoxel_clusters.upper_bound(supervoxel_label);
  //   		}
  //   	}

	 // 	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_first (new pcl::PointCloud<pcl::PointXYZRGBA>);
		// pcl::copyPointCloud<pcl::PointXYZRGBA>( *object_cluster.begin(), *cloud_first);
  // 		visualization(cloud_first);
	 //   	std::cout<<"finito di creare cluster"<<std::endl;
	 //   	std::cout<<"numero di cluster"<<object_cluster.size()<<std::endl;

	   	//start=false;
	   
	// }
}


void  phobic_scene::visualization(bool testing, bool circle)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 255, 0);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr room_object  (new pcl::PointCloud<pcl::PointXYZRGBA>);

	double PI= 3.14159265;


	if(circle == false)
	{
		if (testing == true )
		{
			pcl::copyPointCloud<pcl::PointXYZRGBA>( *cloud, *room_object);
		}

		else
		{
			pcl::copyPointCloud<pcl::PointXYZRGBA>( *object_cluster.begin(), *room_object);
		}


  		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(room_object, 0, 0, 0);
  		viewer->addPointCloud<pcl::PointXYZRGBA> (room_object, single_color, "sample cloud");
		//viewer->addPointCloud<pcl::PointXYZRGBA > (room_object, "nome");
		viewer->addCoordinateSystem(0.1);
  	}

  	else
  	{
  		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(cloud, 0, 0, 0);
  		viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, single_color, "sample cloud");
  		viewer->addCoordinateSystem(0.1);

  		viewer-> addCylinder(CYLINDER.cylinder_coeff, "cyl");
  		viewer->addLine(CYLINDER.info_disegno_cyl_up, CYLINDER.info_disegno_cyl_dw, "line" );
  		
  	}



  	while (!viewer->wasStopped ())
	{
   		viewer->spinOnce (100);
   		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}


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
    extract.setNegative (true);

    // // Get the points associated without the planar surface
    extract.filter (*cloud);
    Plane_coeff=coefficients->values;

}


Eigen::Matrix4d Cyl_Transform (const std::vector<float> coeff)
{
 	std::cout<<"creo matrice"<<std::endl;
	  double x = coeff[0], y = coeff[1], z = coeff[2];
	  double p_x = coeff[3], p_y = coeff[4], p_z = coeff[5];
	  double r = coeff[6]; 

	  for(int i=0; i<7; i++ )
	  {

	  	std::cout<<"coefficienti: "<<coeff[i]<<std::endl;
	  }



	Eigen::Vector3d position(x,y,z); //posizione a caso sull'asse del cilindro. 
		  // w is the axis of the cylinder which will be aligned with the z reference frame of the cylinder
	Eigen::Vector3d w(p_x, p_y, p_z);
	Eigen::Vector3d u(1, 0, 0);	//allineo asse x uguale a quella della kinect

	std::cout<<"trovato w e u"<<std::endl;
	Eigen::Vector3d v = w.cross(u).normalized();
	//  u = v.cross(w).normalized();

	std::cout<<"ho normalizzato"<<std::endl;  
	Eigen::Matrix3d rotation;
	rotation.col(0) = u;  // x
	rotation.col(1) = v;  // y
	rotation.col(2) = w;  // z

	std::cout<<"ho creato rotazione"<<std::endl;
	//rotation = rotation.transpose(); //colum

	// for(int i=0; i<=3; i++)
	// {
	// 	std::cout<<"vettore posizione: "<<position[i]<<std::endl;

	// }

	  // Compose the transformation and return it
	Eigen::Matrix<double,4,4> tranfs_matrix;
	tranfs_matrix.row(0) << rotation.row(0), position[0];
	tranfs_matrix.row(1) << rotation.row(1), position[1];
	tranfs_matrix.row(2) << rotation.row(2), position[2]; 
	tranfs_matrix.row(3) << 0,0,0, 1;

	return tranfs_matrix;



	  // The position is directly obtained from the coefficients, and will be corrected later
// 	  tf::Vector3 position(x,y,z); //posizione a caso sull'asse del cilindro. meglio se prendo punto a z max o metà altezza
	  
// 	  // w is the axis of the cylinder which will be aligned with the z reference frame of the cylinder
// 	  tf::Vector3 w(p_x, p_y, p_z);
// 	  tf::Vector3 u(1, 0, 0);	//allineo asse x uguale a quella della kinect
// 	  tf::Vector3 v = w.cross(u).normalized();
// 	  u = v.cross(w).normalized();
// 	  tf::Matrix3x3 rotation;
// 	  rotation[0] = u;  // x
// 	  rotation[1] = v;  // y
// 	  rotation[2] = w;  // z
// 	  rotation = rotation.transpose(); //colum

// 	  // Compose the transformation and return it
// 	  return tf::Transform(rotation, position);





 }