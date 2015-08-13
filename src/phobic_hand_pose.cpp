#include "phobic_hand_pose.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Phobic_whife_MP");
	ros::NodeHandle node_hand;
	phobic_hand phobic_local_hand(node_hand); 
	ros::Subscriber reader;
	reader = node_hand.subscribe(node_hand.resolveName("INFO_CYLINDER"), 1, &phobic_hand::HandPoseCallback, &phobic_local_hand);

	ros::Rate loop_rate( 1 ); // 5Hz
	while (ros::ok())
	{

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}


void phobic_hand::HandPoseCallback(const desperate_housewife::cyl_info cyl_msg)
{
	// To the first we take an informations from cilynders and robot.
	if(cyl_msg.dimension <= 0)
	{
		ROS_INFO("There are not a objects in the scene");
	}
	else
	{
		ROS_INFO("There are a objects in the scene, start the motion planning");
		
		Goal.resize(cyl_msg.dimension);
		cyl_radius.resize(cyl_msg.dimension);
		cyl_height.resize(cyl_msg.dimension);
		//check the vector dimension	
		ROS_INFO("cyl_msg size: %d", cyl_msg.dimension);
		ROS_INFO("goal size: %d", Goal.size());
		ROS_INFO("cyl_radius size: %d", cyl_radius.size());
		ROS_INFO("cyl_height size: %d", cyl_height.size());
	
				
		//read the robot informations in tf::StampedTransform. Vito has 7 link and 6 joint
		if(check_robot == true)
		{
			//Soft Hand information M_k_H 
			listener_info.lookupTransform("/camera_rgb_optical_frame", "right_hand_palm_link" , ros::Time(0), SoftHand_r );
			listener_info.lookupTransform("/camera_rgb_optical_frame", "left_hand_palm_link" , ros::Time(0), SoftHand_l );
	    }
	    else
	    {	//solo per provarlo			
	    	Pos_HAND_r.x = 0.1;
	    	Pos_HAND_r.y = -0.1;
	    	Pos_HAND_r.z = 0.2;
			
			Pos_HAND_l.x = -0.1;
	    	Pos_HAND_l.y = -0.1;
	    	Pos_HAND_l.z = 0.2;
	    }
	
	    	//read the cylinder informations in tf::StampedTransform
		for (int i = 0; i < cyl_msg.dimension; i++)
		{

			listener_info.lookupTransform("/camera_rgb_optical_frame", "cilindro_" + std::to_string(i) , ros::Time(0), Goal[i] );
			cyl_height.push_back(cyl_msg.length[i]);
			cyl_radius.push_back(cyl_msg.radius[i]);
			cyl_info.push_back(cyl_msg.Info[i]);
			cyl_v.push_back(cyl_msg.vol[i]);
			//cyl_transf.push_back(cyl_msg.Transform.poses[i]);
		}	
		
		GetCylPos(*Goal.begin());
		Send();
		Goal.erase(Goal.begin());
		
	}
}



void phobic_hand::GetCylPos( tf::StampedTransform &object)
{	
	ROS_INFO("get goal position");
	//Set robot potential fields 
	Eigen::Matrix4d frame_kinect;
	frame_kinect = FromTFtoEigen(object); //T_k_c
	frame_cylinder = frame_kinect.inverse(); //T_c_k
	
	// test for setting the potential field
	// bool Test_obj  = true;
	// if true the object is a goal, otherwise is a obstacles
	//Test_obj = objectORostacles();
	
	if(Test_obj == true)
	{	
		goal_position.x = frame_cylinder(0,3);
		goal_position.y = frame_cylinder(1,3);
		goal_position.z = frame_cylinder(2,3);

		WhichArm(goal_position);
		// pcl::PointXYZ goal_kinect_frame;
		// goal_kinect_frame.x = frame_kinect(0,3);
		// goal_kinect_frame.y = frame_kinect(1,3);
		// goal_kinect_frame.z = frame_kinect(2,3);

		SetHandPosition();		
	}

	else
	{	//set obstacles repulsion force		
		obstacle_position.x = frame_cylinder(0,3);
		obstacle_position.y = frame_cylinder(1,3);
		obstacle_position.z = frame_cylinder(2,3);
	
	}
}

void phobic_hand::SetHandPosition()
{
	
	Eigen::Matrix4d M_desired_local; // in cyl frame
	Eigen::Vector4d Point_desired,Pos_ori_hand; //in cyl frame
	Eigen::Vector4d translation; //in cyl frame
	Eigen::Vector3d x(1,0,0);
	Eigen::Vector3d y(0,1,0),z_d;
	Eigen::Matrix4d T_K_H;
	
	if (Arm == true) //left arm
	{
		M_desired_local.col(0) << x, 0;
		z_d = (-x.cross(y));
		T_K_H = FromTFtoEigen(SoftHand_l);		
	}

	else
	{
		M_desired_local.col(0) << -x, 0;	//right arm
		z_d = (x.cross(y));
		T_K_H = FromTFtoEigen(SoftHand_r);
	}
	
	if((cyl_info.front() == 0) && (cyl_v.front() == 0)) //se cilindro dritto (o leggermente piegato) e senza tappo
	{
		Point_desired(0) = cyl_radius.front() + 0.05;
		Point_desired(1) = cyl_height.front();
		Point_desired(2) = cyl_radius.front() + 0.05;	//da rivedere!!
		Point_desired(3) = 0; 
		ROS_INFO("cyl dritto e vuoto");
	}
	else if(((cyl_info.front() == 0) && (cyl_v.front() != 0)) && (cyl_radius.front() < max_radius))
	{
		Point_desired(0) = 0;
		Point_desired(1) = cyl_height.front()+ 0.05;
		Point_desired(2) = 0; //da rivedere
		Point_desired(3) = 0; 
		ROS_INFO("cyl dritto e pieno");
	}

	else if ((cyl_info.front() != 0) && (cyl_radius.front() < max_radius))
	{
		Point_desired(0) = cyl_height.front() + 0.05;
		Point_desired(1) = cyl_radius.front() + 0.05;
		Point_desired(2) = 0; //da rivedere
		Point_desired(3) = 0; 
		ROS_INFO("cyl piegato");
	}
	else
	{
		ROS_INFO("caso non contemplato");
	}

	ROS_INFO("Set hand final position");
	Eigen::Matrix4d T_C_H;

	// transformation matrix for the softhand in cyl frame t_c_h = t_c_k * t_k_h
	T_C_H =  frame_cylinder * T_K_H;
	
	Pos_ori_hand = T_K_H.col(3);
	Eigen::Vector4d local;
	local = Pos_ori_hand.transpose()*T_C_H;

	translation = Point_desired - local;
	// translation(0) = Point_desired(0) - local(0);
	// translation(1) = Point_desired(1) - local(1);
	// translation(2) = Point_desired(2) - local(2);
	// translation(3) = Point_desired(3) - local(3);

	M_desired_local.col(1) << -y, 0;
	M_desired_local.col(2) <<  z_d, 0;
	M_desired_local.col(3) << translation;
	M_desired_local.normalize();

	//std::cout<<"translation"<<translation(3)<<std::endl;
	ROS_INFO("translation: %d", translation(3));

	//devo portare il tutto in word frame e metterla in stampedTrasform

	//fromEigenToPose( T_w_c , Hand_pose);


	//cancello primo elemento della lista
	cyl_radius.erase(cyl_radius.begin());
	cyl_height.erase(cyl_height.begin());
	cyl_info.erase(cyl_info.begin());
	cyl_v.erase(cyl_v.begin());

}

void phobic_hand::WhichArm(pcl::PointXYZ Pos)
{
	std::pair<double, pcl::PointXYZ> distance_HtO;
	//Hand position respect kinect frame 
	//Pos_HAND_l = Take_Pos( SoftHand_l);
	//Pos_HAND_r = Take_Pos( SoftHand_r);
	
	double dist_lTo, dist_rTo;	//in kinect frame

	dist_lTo = GetDistance(Pos, Pos_HAND_l).first;
	dist_rTo = GetDistance(Pos, Pos_HAND_r).first;

	if(dist_lTo < dist_rTo)
	{
		Arm = true; 
		distance_HtO = GetDistance(Pos, Pos_HAND_l);
		ROS_INFO("Vito uses a: LEFT ARM");
	}
	else
	{
		Arm = false;
		distance_HtO = GetDistance(Pos ,Pos_HAND_r );
		ROS_INFO("Vito uses a: RIGHT ARM");
	}
}

std::pair<double, pcl::PointXYZ> phobic_hand::GetDistance(pcl::PointXYZ &obj1, pcl::PointXYZ &obj2 )
{
	std::pair<double, pcl::PointXYZ> distance_local;
	pcl::PointXYZ local_point;
	
	distance_local.first = sqrt(pow(obj1.x-obj2.x,2) + pow(obj1.y-obj2.y,2) + pow(obj1.z-obj2.z,2));  
	local_point.x = (obj1.x-obj2.x);
	local_point.y = (obj1.y-obj2.y);
	local_point.z = (obj1.z-obj2.z);
	distance_local.second =  local_point;

	return distance_local;
}


bool phobic_hand::objectORostacles()
{
	bool OrO;

	if((cyl_radius.front() > max_radius) && (cyl_height.front() > max_lenght))
	{
		OrO= false;
		ROS_INFO("object is a obstacles");
	}
	else
	{
		OrO = true;
		ROS_INFO("object is a goal");
	}

	return OrO;
}



Eigen::Matrix4d FromTFtoEigen(tf::StampedTransform &object)
{	
	Eigen::Quaterniond transf_quad(object.getRotation().getW(),object.getRotation().getX(),object.getRotation().getY(),object.getRotation().getZ());
	transf_quad.normalize();
	Eigen::Vector3d translation(object.getOrigin().x(),object.getOrigin().y(),object.getOrigin().z());
	Eigen::Matrix3d rotation(transf_quad.toRotationMatrix());
		
	Eigen::Matrix4d Matrix_transf;

	Matrix_transf.row(0) << rotation.row(0), translation[0];
	Matrix_transf.row(1) << rotation.row(1), translation[1];
	Matrix_transf.row(2) << rotation.row(2), translation[2];
	Matrix_transf.row(3) << 0,0,0,1;

	return Matrix_transf;
}


pcl::PointXYZ phobic_hand::Take_Pos(tf::StampedTransform &M_tf)
{
	pcl::PointXYZ Pos_vito;
	Eigen::Matrix4d Link_eigen;
	
	Link_eigen = FromTFtoEigen(M_tf);

	Pos_vito.x = Link_eigen(0,3);
	Pos_vito.y = Link_eigen(1,3);
	Pos_vito.z = Link_eigen(2,3);

	return Pos_vito;
}

void phobic_hand::Send()
{ 	
	desperate_housewife::hand msg;

	if(Test_obj == true) // if is object
	{
		if(Arm == true)//left
		{
			msg.whichArm = 0;
		}

		else
		{
			msg.whichArm = 1; //right
		}
		
		msg.hand_Pose = Hand_pose;
	}
	else
	{
		msg.whichArm = 2; //obstacles
		msg.hand_Pose.position.x = obstacle_position.x;
		msg.hand_Pose.position.y = obstacle_position.y;
		msg.hand_Pose.position.z = obstacle_position.z;
	}

	hand_info.publish(msg);
}



void phobic_hand::fromEigenToPose(Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &Hand_pose)
{
	Eigen::Matrix<double,3,3> Tmatrix;
	Tmatrix = tranfs_matrix.block<3,3>(0,0) ;
	Eigen::Quaterniond quat_eigen_hand(Tmatrix);
	Hand_pose.orientation.x = quat_eigen_hand.x();
	Hand_pose.orientation.y = quat_eigen_hand.y();
	Hand_pose.orientation.z = quat_eigen_hand.z();
	Hand_pose.orientation.w = quat_eigen_hand.w();
	Hand_pose.position.x = tranfs_matrix(0,3);
	Hand_pose.position.y = tranfs_matrix(1,3);
	Hand_pose.position.z = tranfs_matrix(2,3);

}