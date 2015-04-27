//#include "camera_desperatehousewife.h"
#include "test_classe.h"
using Eigen::VectorXf ;



int main(int argc, char** argv)
{
	std::cout<<"Sono nel main"<<std::endl<<std::flush;
	ros::init(argc, argv, "Phobic_whife_scene");

	// ros::NodeHandle nodeH;
	ros::Publisher Scena_info;
	ros::Subscriber reader;
	std::cout<<"Sono nel main prima di aver creato ogetto classe"<<std::endl;
	//leggo();
	ros::NodeHandle nodeH;
	phobic_scene phobic_scene_local(nodeH); 
	std::cout<<"Sono nel main dopo aver creato ogetto classe"<<std::endl;
	// phobic_scene_local.leggo();
	//ros::NodeHandle nodeH;

	//Subscribe to the camera/deepregistered/poins topic with the master. ROS will call 
	//the pointcloudCallback() function whenever a new message arrives. 
	//The 2nd argument is the queue size
	reader = nodeH.subscribe(nodeH.resolveName("camera/depth_registered/points"), 10, &phobic_scene::pointcloudCallback, &phobic_scene_local);
	//  Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>("desperate_camera", "100");
	std::cout<<"Sono nel main dopo aver letto"<<std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));
	// std::this_thread::sleep_for(2);
	// sleep(2);
	std::cout<<"Sono nel main dopo aver dormito 2s"<<std::endl;
	phobic_scene_local.fitting();
	phobic_scene_local.send_msg();
	phobic_scene_local.fitting(cloud_temp);
	
	//ros::Rate loop_rate(10);   	


	ros::spin();
	return 0;

	
}


void phobic_scene::pointcloudCallback(sensor_msgs::PointCloud2 msg)
{
	
	// create a new pointcloud from msg
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (msg, *scene);
	
	// phobic_scene_local.save_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  scene );
	// temp_point_cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// std::pair< pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ> > pc_temp;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_table (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_object (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc1 (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc2 (new pcl::PointCloud<pcl::PointXYZ>);

	cloud=erase_environment(scene);
	//cloud_temp.push_back(cloud);
	// pc_temp=erase_table(cloud);
	
	// pcl::copyPointCloud<pcl::PointXYZ>( pc_temp.first, *pc_object);
	// pcl::copyPointCloud<pcl::PointXYZ>( pc_temp.second, *pc_table);

	// pcl::copyPointCloud<pcl::PointXYZ>( *pc_object, *cloud_temp);

	//fitting(pc_object, nodeH, Scena_info );

}



pcl::PointCloud<pcl::PointXYZ>::Ptr erase_environment(pcl::PointCloud<pcl::PointXYZ>::Ptr  original_pc )
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
	// copies all inliers of the model computed to another PointCloud
	//pcl::copyPointCloud<pcl::PointXYZ>(*original_pc, inliers, *environment);
	pcl::PassThrough<pcl::PointXYZ> Filter_env;
		
	// DEVI METTERE ANCORA LE MISURE DEL TAVOLO 1.0 è un metro

	// x
	Filter_env.setInputCloud (original_pc);
	Filter_env.setFilterFieldName ("x");
	Filter_env.setFilterLimits (-1.0, 1.0);
	Filter_env.filter (*cloud_filtered);
	// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
	//indices_rem = Filter_env.getRemovedIndices ();
	
	//y
	Filter_env.setInputCloud (cloud_filtered);
	Filter_env.setFilterFieldName ("y");
	Filter_env.setFilterLimits (-1.0, 1.0);
	// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
	Filter_env.filter (*cloud_temp);
	// The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
	//indices_rem = Filter_env.getRemovedIndices ();
	
	//z
	Filter_env.setInputCloud (cloud_temp);
	Filter_env.setFilterFieldName ("z");
	Filter_env.setFilterLimits (0.0, 1.0);
	//Filter_env.filter (*indices_z);
	// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
	//indices_rem = Filter_env.getRemovedIndices ();
	Filter_env.filter (*cloud_filtered);

	return cloud_filtered;


}

/*
std::pair<pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointXYZ> > erase_table (pcl::PointCloud<pcl::PointXYZ>::Ptr PC_w_env)
{
	// fitting

  // created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (PC_w_env));
 
    std::vector<int> inliers;
    
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_plane);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr table (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object tolgo il tavolo dalla point cloud
    // pcl::ExtractIndices<pcl::PointXYZ> F_extract;
    // F_extract.setInputCloud (model_plane);
    // F_extract.setIndices (inliers);
    // F_extract.setNegative (true);	//true. takes data out of table point cloud
    // F_extract.filter (*cloud_p);

    // copies all inliers of the model computed to another PointCloud
 	pcl::copyPointCloud<pcl::PointXYZ>(*PC_w_env, inliers, *table);
 	
 	std::pair<pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointXYZ> > object_table;
 	pcl::copyPointCloud<pcl::PointXYZ>( *PC_w_env, object_table.first);
	pcl::copyPointCloud<pcl::PointXYZ>( *table, object_table.second);

	return 	object_table;

}*/

//pcl::PointCloud<pcl::PointXYZ>::Ptr 
//tf::Transform phobic_scene::fitting (pcl::PointCloud<pcl::PointXYZ>::Ptr  object )
tf::Transform phobic_scene::fitting ()
{
	// fitting pcl::SampleConsensusModelCylinder<Point, pcl::Normal>:
	// pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::copyPointCloud<pcl::PointXYZ>( cloud_temp.begin(), object);
	
	//pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr model_cylinder(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> (*cloud_temp.begin()));
	pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr model_cylinder(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> (cloud));
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    std::vector<int> inliers;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_cylinder);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder (new pcl::PointCloud<pcl::PointXYZ>);
    //::copyPointCloud<pcl::PointXYZ>(**(cloud_temp.begin()), inliers, *cylinder);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cylinder);

	//cylinder's info
    Eigen::VectorXf info_cil;
    ransac.getModelCoefficients(info_cil);

    //hand's info

    tf::Transform CylToHand_tr;    	
	CylToHand_tr=CylToHand_Transform (info_cil);
	
	//hand_tr=CylToHand_Transform (info_cil);
	
 
 //   	//create a msg
	// //std_msgs::Float32 date_cyl;
	// std_msgs::Float32MultiArray array; // pos_ori

 //    	for (int i=0; i<=2; i++)
 //    	{
 //    		//date_cyl=CylToHand_tr.getOrigin()[i];
	// 		array.data.push_back(CylToHand_tr.getOrigin()[i]);
 //    	}

      	
 //    //date_cyl=CylToHand_tr.getRotation().getX();
	// array.data.push_back(CylToHand_tr.getRotation().getX());
	// //date_cyl=CylToHand_tr.getRotation().getY();
	// array.data.push_back(CylToHand_tr.getRotation().getY());
	// //date_cyl=CylToHand_tr.getRotation().getZ();
	// array.data.push_back(CylToHand_tr.getRotation().getZ());
 //    //date_cyl=CylToHand_tr.getRotation().getW();
	// array.data.push_back(CylToHand_tr.getRotation().getW());
		
	// //send a msg 
		
	// //Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>("desperate_camera", "100");
	// //Scena_info.publish(array);

 // 	 ROS_INFO("Info cylinde");
    
 //    //for visualization
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cyl (new pcl::PointCloud<pcl::PointXYZ>);	
  //   pcl::ExtractIndices<pcl::PointXYZ> Cyl_extract;
 	// pcl::PointIndices::Ptr inliers_cyl (new pcl::PointIndices);
  //   Cyl_extract.setInputCloud (cylinder);
  //   Cyl_extract.setIndices (inliers_cyl);
  //   Cyl_extract.setNegative (false);	//false. takes data inside point cloud
  //   Cyl_extract.filter (*cyl);

 	//visualization(cylinder); 
	
	return(CylToHand_tr);

}


tf::Transform CylToHand_Transform (const Eigen::VectorXf coeff)
{
 
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
		
	//send a msg 
	Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>(nodeH.resolveName("markers_out"), 10);	
	//Scena_info = nodeH.advertise<std_msgs::Float32MultiArray>("desperate_camera", "100");
	Scena_info.publish(array);

 	 ROS_INFO("Info cylinder");
    

}






/*


void visualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr room_object)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(room_object, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (room_object, single_color, "sample cloud");
	// viewer.->addPointCloud<pcl::PointXYZ> (room_object, "nome");


	while (!viewer->wasStopped ())
	{
   	viewer->spinOnce (100);
   	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}




}*/