#include "phobic_motion_planning.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Phobic_whife_MP");
	ros::NodeHandle node_mp;
	phobic_mp phobic_local_mp(node_mp); 
	ros::Subscriber reader;
	reader = node_mp.subscribe(node_mp.resolveName("INFO_CYLINDER"), 1, &phobic_mp::MotionPlanningCallback, &phobic_local_mp);

	ros::Rate loop_rate( 1 ); // 5Hz
	while (ros::ok())
	{

		loop_rate.sleep();
		ros::spinOnce();

	}
	

	return 0;
	
}


void phobic_mp::MotionPlanningCallback(const desperate_housewife::cyl_info cyl_msg)
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
						
		//read the robot informations in tf::StampedTransform. Vito has 7 link and 6 joint
		if(check_robot == true)
		{
			//SetRobotParam();

			// for(int i = 0; i<=7 ; i++)
			// {	

			// 	listener_info.lookupTransform("/camera_rgb_optical_frame", "left_arm_" + std::to_string(i) + "_link" , ros::Time(0), Vito_desperate.Link_left[i] );
			// 	listener_info.lookupTransform("/camera_rgb_optical_frame", "right_arm_" + std::to_string(i) + "_link" , ros::Time(0), Vito_desperate.Link_right[i] );
			// 	Vito_desperate.robot_position_left.push_back(Take_Pos(Vito_desperate.Link_left[i]));
			// 	Vito_desperate.robot_position_right.push_back(Take_Pos(Vito_desperate.Link_right[i]));
			// }

			// ////Soft Hand information M_k_H 

			// listener_info.lookupTransform("/camera_rgb_optical_frame", "right_hand_palm_link" , ros::Time(0), Vito_desperate.SoftHand_r );
			// listener_info.lookupTransform("/camera_rgb_optical_frame", "left_hand_palm_link" , ros::Time(0), Vito_desperate.SoftHand_l );
	    }
	    else
	    {	for (int i=0;i<10;i++) //numeri a caso per provarlo
	    	{
	    		pcl::PointXYZ r;
	    		r.x = i;
	    		r.y = i+1;
	    		r.z = i*i;

	    		Vito_desperate.robot_position_left.push_back(r);
				Vito_desperate.robot_position_right.push_back(r);
			}

			
	    	Vito_desperate.Pos_HAND_r.x = 0.1;
	    	Vito_desperate.Pos_HAND_r.y = -0.1;
	    	Vito_desperate.Pos_HAND_r.z = 0.2;
			
			Vito_desperate.Pos_HAND_l.x = -0.1;
	    	Vito_desperate.Pos_HAND_l.y = -0.1;
	    	Vito_desperate.Pos_HAND_l.z = 0.2;
	    }
		
		//// with this informations we can make a MP 
		for (int i = 0; i <  cyl_msg.dimension; i++)
		{
			SetPotentialField( *Goal.begin());
		}

	//SetCommandVector();
	

	}

}



void phobic_mp::SetPotentialField( tf::StampedTransform &object)
{	
	ROS_INFO("calculate the potential field");
	//Set robot potential fields 
	Eigen::Matrix4d frame_kinect;
	frame_kinect = FromTFtoEigen(object); //T_k_c
	frame_cylinder = frame_kinect.inverse(); //T_c_k
	
	SetPotentialField_robot(Force_repulsion);

	// test for setting the potential field
	bool Test_obj;
	// if true the object is a goal, otherwise is a obstacles
	Test_obj = objectORostacles();
	
	if(Test_obj == true)
	{	
		goal_position.x = frame_cylinder(0,3);
		goal_position.y = frame_cylinder(1,3);
		goal_position.z = frame_cylinder(2,3);

		pcl::PointXYZ goal_kinect_frame;
		goal_kinect_frame.x = frame_kinect(0,3);
		goal_kinect_frame.y = frame_kinect(1,3);
		goal_kinect_frame.z = frame_kinect(2,3);

		SetAttraciveField( goal_kinect_frame);
		SetRepulsiveFiled( goal_kinect_frame);
		SetHandPosition();		
	}

	else
	{	//set obstacles repulsion force		
		obstacle_position.x = frame_cylinder(0,3);
		obstacle_position.y = frame_cylinder(1,3);
		obstacle_position.z = frame_cylinder(2,3);
		SetRepulsiveFiled( obstacle_position);
	}

	Calculate_force();
	

	Goal.erase(Goal.begin());

}

void phobic_mp::SetHandPosition()
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
		T_K_H = FromTFtoEigen(Vito_desperate.SoftHand_l);		
	}

	else
	{
		M_desired_local.col(0) << -x, 0;	//right arm
		z_d = (x.cross(y));
		T_K_H = FromTFtoEigen(Vito_desperate.SoftHand_r);
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

	//devo portare il tutto in word frame


	//cancello primo elemento della lista
	cyl_radius.erase(cyl_radius.begin());
	cyl_height.erase(cyl_height.begin());
	cyl_info.erase(cyl_info.begin());
	cyl_v.erase(cyl_v.begin());

}

bool phobic_mp::objectORostacles()
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


void phobic_mp::SetAttraciveField( pcl::PointXYZ &Pos) //calculate respect kinect frame
{
	
	std::pair<double, pcl::PointXYZ> distance_HtO;
	//Hand position respect kinect frame
	//Vito_desperate.Pos_HAND_l = Take_Pos( Vito_desperate.SoftHand_l);
	//Vito_desperate.Pos_HAND_r = Take_Pos( Vito_desperate.SoftHand_r);
	
	double dist_lTo, dist_rTo;	//in kinect frame

	dist_lTo = GetDistance(Pos, Vito_desperate.Pos_HAND_l).first;
	dist_rTo = GetDistance(Pos, Vito_desperate.Pos_HAND_r).first;

	if(dist_lTo < dist_rTo)
	{
		Arm = true; 
		distance_HtO = GetDistance(Pos, Vito_desperate.Pos_HAND_l);
		ROS_INFO("Vito uses a: LEFT ARM");
	}
	else
	{
		Arm = false;
		distance_HtO = GetDistance(Pos ,Vito_desperate.Pos_HAND_r );
		ROS_INFO("Vito uses a: RIGHT ARM");
	}	

	// To calculate the attractive force -dissipative(velocity-V_limit*desired_velocity)
	pcl::PointXYZ velocity_d,Vtf;
	velocity_d.x = P_goal/dissipative *(distance_HtO.second).x;
	velocity_d.y = P_goal/dissipative *(distance_HtO.second).y;
	velocity_d.z = P_goal/dissipative *(distance_HtO.second).z;
	
	SetLimitation(velocity_d);

	velocity_d.x = v_lim*velocity_d.x;
	velocity_d.y = v_lim*velocity_d.y;
	velocity_d.z = v_lim*velocity_d.z;

	Vtf.x = -dissipative*(velocity.x-velocity_d.x);
	Vtf.x = -dissipative*(velocity.y-velocity_d.y);
	Vtf.x = -dissipative*(velocity.z-velocity_d.z);


	Force_attractive.push_back(Vtf);
	
}

void phobic_mp::SetLimitation(pcl::PointXYZ &vel_d)
{
	Eigen::Vector3d VEL_eigen;

	VEL_eigen[0] = vel_d.x;
	VEL_eigen[1] = vel_d.y;
	VEL_eigen[2] = vel_d.z;

	double coeff;
	coeff = (velocity_max/sqrt(VEL_eigen.transpose() * VEL_eigen));
	//v_lim = min(1,(v_max/sqrt(V_d.transpose * V_d)));
	if(1 < coeff)
	{
		v_lim = 1;
	}
	else
	{
		v_lim = coeff;
	}
}


void phobic_mp::SetPotentialField_robot(std::vector<pcl::PointXYZ> &Force_repulsion)
{
	std::vector<std::pair<double, pcl::PointXYZ>> distance_link;
	std::vector<pcl::PointXYZ>* robot_link_position;
	//robot_link_position->resize(Vito_desperate.robot_position_left.size());
	
	//Arm=true is left arm, Arm = false is right arm
	if(Arm = true)
	{
		robot_link_position = &Vito_desperate.robot_position_left;

	}
	else
	{
		robot_link_position = &Vito_desperate.robot_position_right;
	}	

	distance_link.push_back(GetDistance((*robot_link_position)[2], (*robot_link_position)[1]));
	distance_link.push_back(GetDistance((*robot_link_position)[3], (*robot_link_position)[1]));
	distance_link.push_back(GetDistance((*robot_link_position)[3], (*robot_link_position)[2]));
	distance_link.push_back(GetDistance((*robot_link_position)[4], (*robot_link_position)[1]));
	distance_link.push_back(GetDistance((*robot_link_position)[4], (*robot_link_position)[2]));
	distance_link.push_back(GetDistance((*robot_link_position)[4], (*robot_link_position)[3]));
	distance_link.push_back(GetDistance((*robot_link_position)[5], (*robot_link_position)[1]));
	distance_link.push_back(GetDistance((*robot_link_position)[5], (*robot_link_position)[2]));
	distance_link.push_back(GetDistance((*robot_link_position)[5], (*robot_link_position)[3]));
	distance_link.push_back(GetDistance((*robot_link_position)[5], (*robot_link_position)[4]));
	distance_link.push_back(GetDistance((*robot_link_position)[6], (*robot_link_position)[1]));
	distance_link.push_back(GetDistance((*robot_link_position)[6], (*robot_link_position)[2]));
	distance_link.push_back(GetDistance((*robot_link_position)[6], (*robot_link_position)[3]));
	distance_link.push_back(GetDistance((*robot_link_position)[6], (*robot_link_position)[4]));
	distance_link.push_back(GetDistance((*robot_link_position)[6], (*robot_link_position)[5]));
	distance_link.push_back(GetDistance((*robot_link_position)[7], (*robot_link_position)[1]));
	distance_link.push_back(GetDistance((*robot_link_position)[7], (*robot_link_position)[2]));
	distance_link.push_back(GetDistance((*robot_link_position)[7], (*robot_link_position)[3]));
	distance_link.push_back(GetDistance((*robot_link_position)[7], (*robot_link_position)[4]));
	distance_link.push_back(GetDistance((*robot_link_position)[7], (*robot_link_position)[5]));
	distance_link.push_back(GetDistance((*robot_link_position)[7], (*robot_link_position)[6]));
	
	// Repulsive fields = K/distance^2 (1/distance -1/influence) partial_derivative_vector

	for (int i=0; i <= distance_link.size(); i++)
	{
		pcl::PointXYZ vec_Temp;
		vec_Temp.x = (P_obj/pow(distance_link[i].first,2)) * (1/distance_link[i].first - 1/influence) *distance_link[i].second.x;
		vec_Temp.y = (P_obj/pow(distance_link[i].first,2)) * (1/distance_link[i].first - 1/influence) *distance_link[i].second.y;
		vec_Temp.z = (P_obj/pow(distance_link[i].first,2)) * (1/distance_link[i].first - 1/influence) *distance_link[i].second.z;

		Force_repulsion.push_back(vec_Temp);
	}
}

void phobic_mp::SetRepulsiveFiled(pcl::PointXYZ &Pos)
{
	std::vector<std::pair<double, pcl::PointXYZ>> distance_local_obj;

	for (int i=2; i <= Vito_desperate.robot_position_left.size();i++)
	{	
		distance_local_obj.push_back(GetDistance(Pos,Vito_desperate.robot_position_left[i])); 

	}

	for (int i=0; i <= distance_local_obj.size(); i++)
	{
		pcl::PointXYZ vec_Temp;
		vec_Temp.x = distance_local_obj[i].second.x;
		vec_Temp.y = distance_local_obj[i].second.y;
		vec_Temp.z = distance_local_obj[i].second.z;

		if(distance_local_obj[i].first <= influence)
		{
			vec_Temp.x = (P_obj/pow(distance_local_obj[i].first,2)) * (1/distance_local_obj[i].first - 1/influence) * vec_Temp.x;
			vec_Temp.y = (P_obj/pow(distance_local_obj[i].first,2)) * (1/distance_local_obj[i].first - 1/influence) * vec_Temp.y;
			vec_Temp.z = (P_obj/pow(distance_local_obj[i].first,2)) * (1/distance_local_obj[i].first - 1/influence) * vec_Temp.z;
			Force_repulsion.push_back(vec_Temp);
		}
		else
		{
			vec_Temp.x = 0;
			vec_Temp.y= 0;
			vec_Temp.z= 0;
			Force_repulsion.push_back(vec_Temp);
		}
	}

}


std::pair<double, pcl::PointXYZ> phobic_mp::GetDistance(pcl::PointXYZ &obj1, pcl::PointXYZ &obj2 )
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



pcl::PointXYZ phobic_mp::Take_Pos(tf::StampedTransform &M_tf)
{
	pcl::PointXYZ Pos_vito;
	Eigen::Matrix4d Link_eigen;
	
	Link_eigen = FromTFtoEigen(M_tf);

	Pos_vito.x = Link_eigen(0,3);
	Pos_vito.y = Link_eigen(1,3);
	Pos_vito.z = Link_eigen(2,3);

	return Pos_vito;
}

void phobic_mp::Calculate_force()
{
	pcl::PointXYZ repulsive_local;
	pcl::PointXYZ attractive_local;
	
	for(int i=0;i < Force_repulsion.size();i++)
	{
		repulsive_local.x = Force_repulsion[i].x +  Force_repulsion[i+1].x;
		repulsive_local.y = Force_repulsion[i].y +  Force_repulsion[i+1].y;
		repulsive_local.z = Force_repulsion[i].z +  Force_repulsion[i+1].z;
	}

	if(Force_attractive.size() > 1)
	{
		for (int i=0; i< Force_attractive.size(); i++)
		{
			attractive_local.x = Force_attractive[i].x + Force_attractive[i+1].x;
			attractive_local.y = Force_attractive[i].y + Force_attractive[i+1].y;
			attractive_local.z = Force_attractive[i].z + Force_attractive[i+1].z;
		}
	}

	else
	{
		attractive_local = *Force_attractive.begin();
	}

	pcl::PointXYZ local_force;
	local_force.x = attractive_local.x + repulsive_local.x;
	local_force.y = attractive_local.y + repulsive_local.y;
	local_force.z = attractive_local.z + repulsive_local.z;
	Force.push_back( local_force);
	ROS_INFO(" FORCE: %d", Force.front());
	
}