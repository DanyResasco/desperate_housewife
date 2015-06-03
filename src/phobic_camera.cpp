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
	Filter_env.setFilterLimits (-0.2, 0.7);
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
	Filter_env.setFilterLimits (0.2, 1.0);
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
	seg.setDistanceThreshold (0.05);	//distance between two points
    seg.setRadiusLimits (0, 0.2);	//cilynder's radius 20cm
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

	 	CYLINDER.cyl_list.push_back(*cloud_cylinder);
 		
 		std::vector<float> coeff_cyl;
	 	coeff_cyl=makeInfoCyl(coefficients_cylinder->values, cloud_cylinder);
	  	

	 	////hand's info
		// CYLINDER.hand_tr=CylToHand_Transform (coefficients_cylinder->values);

		// std::cout<< "hand transformation rot "<< *CYLINDER.hand_tr.getRotation()<< std::endl;
		// std::cout<< "hand transformation rot "<<*CYLINDER.hand_tr.getOrigin()<< std::endl;

		// remove the first elements in the list
		object_cluster.pop_front();
		//hand_tr=CylToHand_Transform (info_cil);
		std::cout<<"disegno cilindro"<<std::endl;
	  	// visualization(cloud_cylinder); 
		
		//return(CylToHand_tr);
	
	}
}


std::vector<float> phobic_scene::makeInfoCyl(std::vector<float> coeff , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_cyl)
{
	// cylinder's coefficients
	double x = coeff[0], y = coeff[1], z = coeff[2];	// cordinate di un punto sull asse
	double p_x = coeff[3], p_y = coeff[4], p_z = coeff[5];	// componenti dell asse realtive alla telecamera
	double r = coeff[6];

	//create a new frame

	Eigen::Vector3d Ax, Ay, Az;
	Az.x()=p_x;
	Az.y()=p_y;
	Az.z()=p_z;
	
	bool OK=false;
	// std::vector<double> Ax, Ay, Az;
	// Az.push_back(p_x, p_y, p_z);

	/*  make a vectors orthogonal 
		v=(a b c); v*w=0 -> w=(l m n) tc l=-b, m=a, n=0; if a,b!=0 || a,c!=0 !! b,c!=0
	*/

	// cerca di farlo meglio, porta tutto in eigen e usa le sue funzioni in più utilizza case swich al posto di questi for
	//make a vectors orthogonal 
	if((p_x && p_y)!=0) 
	{
		std::cout<<"caso 1"<<std::endl;

		// Ax.push_back(-p_y);
		// Ax.push_back(p_x)
		// Ax.push_back(0)
		OK=true;
		Ax.x()=-p_y;
		Ax.y()=p_x;
		Ax.z()=0;

	}
	
	if (OK==false && (p_x && p_z)!=0)
	 {
	 		std::cout<<"caso 2"<<std::endl;

	// 	Ax.push_back(-p_z);
	// 	Ax.push_back(0);
	// 	Ax.push_back(p_x);
	 	OK=true;
		Ax.x()=-p_z;
		Ax.y()=0;
		Ax.z()=p_x;
	}

	if(OK==false && (p_y && p_z)!=0)
	{
		// Ax.push_back(0);
		// Ax.push_back(-p_z);
		// Ax.push_back(p_y);
		OK=true;
		Ax.x()=0;
		Ax.y()=-p_z;
		Ax.z()=p_y;
		std::cout<<"caso 3"<<std::endl;


	}

	/*y axis orthogonal z,x. 
		vec vector;
	    vector.x = (Ay*Bz)-(By*Az);
	    vector.y = -(Ax*Bz)+(Bx*Az);
	    vector.z = (Ax*By)-(Ay*Bx);
	    // double X,Y,Z;

		// X=Ax[1]*Az[2] - Az[1]* Ax[2];
		// Ay.push_back(X);

		// Y= - Ax[0]* Az[2] + Az[0]* Ax[2];
		// Ay.push_back(Y);

		// Z= Ax[0]* Az[1] - Ax[1]* Az[0];
		// Ay.push_back(Z);

 	*/

	Ay=Ax.cross(Az);
	std::cout<<"prodotto vettore"<<std::endl;

	/* orientation
		a*b=mod(a)*mod(b)*cos(tetha) --> temp1= a*b, temp2=mod(a), temp3=mod(b), temp4=(a*b)/mod(a)*mod(b)
	*/


	double temp1, temp2, temp3, temp4;
	double PI= 3.14159265;

	//take plane's normals (for one of normals plane_normals->points.normal_x)
	double Plane_normal_x = Plane_coeff[0];
	double Plane_normal_y = Plane_coeff[1];
	double Plane_normal_z = Plane_coeff[2];

	temp1= Plane_normal_x * Az.x() + Plane_normal_y * Az.y() + Plane_normal_z * Az.z();
	temp2=std::sqrt(std::pow(Az.x(),2) + std::pow(Az.y(),2) + std::pow(Az.z(),2));
	temp3=std::sqrt(std::pow(Plane_normal_x,2) + std::pow(Plane_normal_y,2) + std::pow(Plane_normal_z,2));	// temp1= plane_normals->normal_x * Az.x() + plane_normals->normal_y * Az.y() + plane_normals->normal_z * Az.z();
	// temp2=std::sqrt(std::pow(Az.x(),2) + std::pow(Az.y(),2) + std::pow(Az.z(),2));
	// temp3=std::sqrt(std::pow(plane_normals->normal_x,2) + std::pow(plane_normals->normal_y,2) + std::pow(plane_normals->normal_z,2));
	temp4= temp1 / (temp2 * temp3);

	//check for obtain  0<thetha<90;
	// if((cos (temp4)* 180/PI)<0)
	// {
	// 	temp4 =temp4*-1;
	// 	std::cout<<"cambiato segno"<<std::endl;
	// }

	CYLINDER.tetha= acos(temp4) * 180/PI;
	std::cout<<"thetha: "<<CYLINDER.tetha<<std::endl;

	if((CYLINDER.tetha > 90) && (CYLINDER.tetha < 180) )
	{
		CYLINDER.tetha = 180-CYLINDER.tetha;
		std::cout<<"cambiato segno a theta"<<std::endl;
		std::cout<<"thetha: "<<CYLINDER.tetha<<std::endl;


	}


	//transformation matrix
	Eigen::Matrix<double, 4,4 > M_rot; 
	std::cout<<"creato matrice rotazione"<<std::endl;
	//Block of size (p,q), starting at (i,j). indices start at 0
	M_rot.row(0) << Ax.x(), Ay.x(), Az.z(), -x;
	M_rot.row(1) << Ax.y(), Ay.y(), Az.y(), -y;
	M_rot.row(2) << Ax.z(), Ay.z(), Az.z(), -z;
	M_rot.row(3) << 0, 0, 0, 1;


	std::cout<<"finito di creare matrice di rotazione"<<std::endl;
	//create a new cylinder pointcloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CYl_new_transf (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//Eigen::Matrix<double, 4,4 > M_rot_inv;
	CYLINDER.M_rot_inv = M_rot.inverse();
	pcl::transformPointCloud(*pc_cyl, *CYl_new_transf, CYLINDER.M_rot_inv);

	//find height 
	double z_min, z_max;
	z_min= CYl_new_transf->points[0].z;
	z_max= CYl_new_transf->points[0].z;


	for (int i=1; i< CYl_new_transf->points.size(); i++)
	{
		if(z_min > CYl_new_transf->points[i].z)
		{	
			z_min = CYl_new_transf->points[i].z;
		}

		if(z_max < CYl_new_transf->points[i].z)
		{	
			z_max = CYl_new_transf->points[i].z;
		}
	}

	// double 	height_cyl;
	CYLINDER.height= z_max - z_min;
	std::cout<<"altezza: "<<CYLINDER.height<<std::endl;

	//new coefficients with the new cylinder's frame 

	Eigen::Matrix<double, 4,1> Point_new_cyl;
	Eigen::Matrix<double, 4,1> temp_old_cyl;
	Eigen::Matrix<double, 4,1> Vers_new_cyl;

	temp_old_cyl.col(0) << coeff[0], coeff[1], coeff[2], 0;
		
	Point_new_cyl=(CYLINDER.M_rot_inv.inverse()*temp_old_cyl);
	//std::cout<<"finito prima molt"<<std::endl;

	temp_old_cyl.col(0) << coeff[3], coeff[4], coeff[5], 0;

	Vers_new_cyl=(CYLINDER.M_rot_inv.inverse()*temp_old_cyl);
	//std::cout<<"finito sec molt"<<std::endl;

	std::vector<float> v;

	v.push_back(Point_new_cyl(0,0));
	v.push_back(Point_new_cyl(1,0));
	v.push_back(Point_new_cyl(2,0));
	v.push_back(Vers_new_cyl(0,0));
	v.push_back(Vers_new_cyl(1,0));
	v.push_back(Vers_new_cyl(2,0));
	v.push_back(r);	//same radius
	std::cout<<"finito creazione vettore "<<std::endl;

	for(int i=0; i<=v.size(); i++)
	{
		std::cout<<"dati vettore coef new: "<< v[i] <<std::endl;

	}

	return v;
}


//tf::Transform CylToHand_Transform (const Eigen::VectorXf coeff)
//dipende dal raggio del cilindro e dall'altezza. da riguardare in seguito con un euristica di presa
// tf::Transform CylToHand_Transform (const std::vector<float> coeff)
// {
//  	std::cout<<"creo matrice"<<std::endl;
// 	  double x = coeff[0], y = coeff[1], z = coeff[2];
// 	  double p_x = coeff[3], p_y = coeff[4], p_z = coeff[5];
// 	  double r = coeff[6]; 

// 	  // The position is directly obtained from the coefficients, and will be corrected later
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
// }


// void  phobic_scene::send_msg()
// {
//  	std::cout<<"sono in send_msg"<<std::endl;
//  	//create a msg
// 	//std_msgs::Float32 date_cyl;
// 	std_msgs::Float32MultiArray array; // pos_ori

//     	for (int i=0; i<=2; i++)
//     	{
//     		//date_cyl=hand_tr.getOrigin()[i];
// 			array.data.push_back(hand_tr.getOrigin()[i]);
//     	}

      	
//     //date_cyl=hand_tr.getRotation().getX();
// 	array.data.push_back(hand_tr.getRotation().getX());
// 	//date_cyl=hand_tr.getRotation().getY();
// 	array.data.push_back(hand_tr.getRotation().getY());
// 	//date_cyl=hand_tr.getRotation().getZ();
// 	array.data.push_back(hand_tr.getRotation().getZ());
//     //date_cyl=hand_tr.getRotation().getW();
// 	array.data.push_back(hand_tr.getRotation().getW());
		
// 	ROS_INFO("Info cylinder");
// 	//send a msg 
// 	Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>(nodeH.resolveName("markers_out"), 10);	
// 	//Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>("desperate_camera", "100");
// 	Scena_info.publish(array); 

// }


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
  	ec.setClusterTolerance (0.05); // 5cm
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

// // bool phobic_scene::check_change_pc()
// // {

// // }

void visualization(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr room_object)
//void  phobic_scene::visualization()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	viewer->setBackgroundColor (0, 255, 0);
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(room_object, 0, 0, 0);
  	viewer->addPointCloud<pcl::PointXYZRGBA> (room_object, single_color, "sample cloud");
	// viewer->addPointCloud<pcl::PointXYZRGBA > (room_object, "nome");


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
    extract.setNegative (true);

    // // Get the points associated with the planar surface
    extract.filter (*cloud);
    Plane_coeff=coefficients->values;

    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    // extract.setNegative (true);
    // extract.filter (*cloud);
   
   	// Estimate point normals
 //    pcl::NormalEstimation< pcl::PointXYZRGBA, pcl::Normal> ne;
 //    pcl::search::KdTree< pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree< pcl::PointXYZRGBA> ());   
	
	// ne.setSearchMethod (tree);
	// ne.setInputCloud (cloud_plane);
	// ne.setKSearch (100);
	// ne.compute (*plane_normals);
    
  	//visualization(cloud);
 	
 // 	std::pair<pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointXYZ> > object_table;
 // 	pcl::copyPointCloud<pcl::PointXYZ>( *PC_w_env, object_table.first);
	// pcl::copyPointCloud<pcl::PointXYZ>( *table, object_table.second);

	// return 	object_table;

}
