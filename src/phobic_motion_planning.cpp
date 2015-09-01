#include "phobic_motion_planning.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Phobic_whife_MP");
	ros::NodeHandle node_mp;
	phobic_mp phobic_local_mp(node_mp);

	
	ros::Subscriber reader; //pose in word frame!!
	reader = node_mp.subscribe(node_mp.resolveName("SoftHand_Pose"), 1, &phobic_mp::MPCallback, &phobic_local_mp);
	//left arm
	listener_info.lookupTransform("/camera_rgb_optical_frame", "left_arm_base_link" , ros::Time(0), Vito_desperate.left_arm_base_link );
	listener_info.lookupTransform("/camera_rgb_optical_frame", "left_arm_softhand" , ros::Time(0), Vito_desperate.left_arm_softhand );
	//joint_listen();//ritonrna msg joint_State pos,vel,acc,coppia
	joint_listen = node_mp.subscribe("left_arm/joint_states", 1, &phobic_mp::Robot_Callback_left, &phobic_local_mp);
	//left arm
	listener_info.lookupTransform("/camera_rgb_optical_frame", "right_arm_base_link" , ros::Time(0),Vito_desperate.right_arm_base_link );
	listener_info.lookupTransform("/camera_rgb_optical_frame", "left_arm_softhand" , ros::Time(0), Vito_desperate.right_arm_softhand );
	//joint_listen();//ritonrna msg joint_State pos,vel,acc,coppia
	joint_listen = node_mp.subscribe("right_arm/joint_states", 1, &phobic_mp::Robot_Callback_right, &phobic_local_mp);


	ros::Rate loop_rate( 1 ); // 1Hz
	while (ros::ok())
	{
		phobic_local_mp.Esegui();
		loop_rate.sleep();
		ros::spinOnce();

	}
	
	return 0;
	
}

void phobic_mp::MPCallback(const desperate_housewife::hand hand_msg)
{	
	//make a robot model
	bool check_urdf;
	check_urdf = GetVitoUrdf();
	if (check_urdf == true)
	{
		//set reference frame
	    //const std::string& robot_namespace;
	    //root_name = robot_namespace + std::string("vito_anchor"); //it's the word frame
	  
		for(int i=0; i < hand_msg.hand_Pose.size(); i++)
		{
			
			switch(hand_msg.whichArm[i]) 
			{
				case 0: //left arm
					ROS_INFO("Vito use left arm");
					tf::poseMsgToKDL(const desperate_housewife::hand hand_msg.*hand_Pose[i], Hand_pose_kdl_l[i]);
					GetEuleroAngle(Hand_pose_kdl_l[i],0);
	    		
				break;

				case 1: //right arm
					ROS_INFO("Vito use right arm");
					tf::poseMsgToKDL(const desperate_housewife::hand hand_msg.*hand_Pose[i], Hand_pose_kdl);
					GetEuleroAngle(Hand_pose_kdl_r[i],1);
			
				break;

				case 2: //obstacles
					ROS_INFO("The object is obstacles");
					obstacle_position.x = hand_Pose[i].position.x;
					obstacle_position.y = hand_Pose[i].position.y;
					obstacle_position.z = hand_Pose[i].position.z;

					//SetRepulsiveFiled(hand_msg.*hand_Pose.begin().position);

				break;
			}

			whichArm.push_back(hand_msg.whichArm[i]);
		}

		hand_msg.hand_Pose.clear();
		hand_msg.whichArm.clear();
	}
	else
	{
		ROS_INFO("Failed to construct Vito urdf");
	}
}



void phobic_mp::Esegui()
{
	for(int i=0; i < whichArm.size();i++)
	{	
		if(whichArm[i] == 2)
		{
			SetRepulsiveFiled();
		}
		else
		{
			GetJacobian(whichArm[i]);
			SetPotentialField_robot(Force_repulsion, whichArm[i]);
			SendetAttractiveField(whichArm[i], i);
		}
	}

	SetCommandVector();
	Send();
}

void phobic_mp::GetEuleroAngle(KDL::Frame &Hand_kdl, int &p)
{
	//KDL::Frame hand_kdl_pose_l;
	double x, y,z,w;
	double roll, pitch, yaw;
	KDL::Rotation::GetRPY(roll, pitch, yaw);
	KDL::Rotation::GetQuaternion(x,y,z,w);

	if (int p == 0 )//left
	{	
		Vito_desperate.Pos_HAND_l.push_back(x,y,z,roll,pitch,yaw);	//x_d in word frame
		
	}
	else //right
	{
		Vito_desperate.Pos_HAND_r.push_back(x,y,z,roll,pitch,yaw);	//x_d in word frame
	}

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

void phobic_mp::SetAttractiveField(int &p, int &i ) //calculate respect word frame
{		
	Eigen::VectorXd pos_Hand;

	if(p == 0)
	{
		pos_Hand = &Vito_desperate.Pos_HAND_l[i];
	}
	else
	{
		pos_Hand = &Vito_desperate.Pos_HAND_r[i];
	}

	// To calculate the attractive force -dissipative(velocity-V_limit*desired_velocity)

	Eigen::VectorXd vel_servo_control; //xd_point
	
	vel_servo_control << P_goal/dissipative * (pos_Hand - Vito_desperate.link_frame_[7]);

	double v_lim;
	v_lim = SetLimitation(vel_servo_control);

	Eigen::VectorXd velocity;
	velocity << (Vito_desperate.link_jac_[7] * Vito_desperate.Vel[7]);  //x_point = jac*q_point

	Force_attractive << (- dissipative *(velocity - v_lim * vel_servo_control)); //in vito frame

	
}

double phobic_mp::SetLimitation(Eigen::VectorXd &vel_d)
{
	Eigen::Vector3d vel_pos;
	vel_pos << vel_d(0), vel_d(1), vel_d(2); //only position

	double coeff, v_lim;
	coeff = (velocity_max/sqrt(vel_pos.transpose() * vel_pos));
	//v_lim = min(1,(v_max/sqrt(V_d.transpose * V_d)));
	if(1 < coeff)
	{
		v_lim = 1;
	}
	else
	{
		v_lim = coeff;
	}
	return v_lim;
}

void phobic_mp::SetCommandVector()
{
	ROS_INFO("dentro SetCommandVector()");
	double local_force_r,local_force_p;

	for (int i=0; i< Force_attractive.size()-1; i++)
	{
		local_force_r = Force_attractive[i]+ Force_attractive[i+1];	
	}
	for (int p=0; p< Force_repulsion.size()-1;p++)
	{
		local_force_p = Force_repulsion[p] + Force_repulsion[p+1];
	}
	
	Force << (local_force_r + local_force_p);
	SetPseudoInvJac();

	//q_point_desired = jac_pseudoInv*Force;
}


// void phobic_mp::SetRepulsiveFiled(pcl::PointXYZ &Pos)
// {
// 	std::vector<std::pair<double, pcl::PointXYZ>> distance_local_obj;

// 	for (int i=2; i <= Vito_desperate.robot_position_left.size();i++)
// 	{	
// 		distance_local_obj.push_back(GetDistance(Pos,Vito_desperate.robot_position_left[i])); 

// 	}

// 	for (int i=0; i <= distance_local_obj.size(); i++)
// 	{
// 		pcl::PointXYZ vec_Temp;
// 		vec_Temp.x = distance_local_obj[i].second.x;
// 		vec_Temp.y = distance_local_obj[i].second.y;
// 		vec_Temp.z = distance_local_obj[i].second.z;

// 		if(distance_local_obj[i].first <= influence)
// 		{
// 			vec_Temp.x = (P_obj/pow(distance_local_obj[i].first,2)) * (1/distance_local_obj[i].first - 1/influence) * vec_Temp.x;
// 			vec_Temp.y = (P_obj/pow(distance_local_obj[i].first,2)) * (1/distance_local_obj[i].first - 1/influence) * vec_Temp.y;
// 			vec_Temp.z = (P_obj/pow(distance_local_obj[i].first,2)) * (1/distance_local_obj[i].first - 1/influence) * vec_Temp.z;
// 			Force_repulsion.push_back(vec_Temp);
// 		}
// 		else
// 		{
// 			vec_Temp.x = 0;
// 			vec_Temp.y= 0;
// 			vec_Temp.z= 0;
// 			Force_repulsion.push_back(vec_Temp);
// 		}
// 	}

// }


// std::pair<double, pcl::PointXYZ> phobic_mp::GetDistance(pcl::PointXYZ &obj1, pcl::PointXYZ &obj2 )
// {
// 	std::pair<double, pcl::PointXYZ> distance_local;
// 	pcl::PointXYZ local_point;
	
// 	distance_local.first = sqrt(pow(obj1.x-obj2.x,2) + pow(obj1.y-obj2.y,2) + pow(obj1.z-obj2.z,2));  
// 	local_point.x = (obj1.x-obj2.x);
// 	local_point.y = (obj1.y-obj2.y);
// 	local_point.z = (obj1.z-obj2.z);
// 	distance_local.second =  local_point;

// 	return distance_local;
// }



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

// void phobic_mp::Calculate_force()
// {
// 	pcl::PointXYZ repulsive_local;
// 	pcl::PointXYZ attractive_local;
	
// 	for(int i=0;i < Force_repulsion.size();i++)
// 	{
// 		repulsive_local.x = Force_repulsion[i].x +  Force_repulsion[i+1].x;
// 		repulsive_local.y = Force_repulsion[i].y +  Force_repulsion[i+1].y;
// 		repulsive_local.z = Force_repulsion[i].z +  Force_repulsion[i+1].z;
// 	}

// 	if(Force_attractive.size() > 1)
// 	{
// 		for (int i=0; i< Force_attractive.size(); i++)
// 		{
// 			attractive_local.x = Force_attractive[i].x + Force_attractive[i+1].x;
// 			attractive_local.y = Force_attractive[i].y + Force_attractive[i+1].y;
// 			attractive_local.z = Force_attractive[i].z + Force_attractive[i+1].z;
// 		}
// 	}

// 	else
// 	{
// 		attractive_local = *Force_attractive.begin();
// 	}

// 	pcl::PointXYZ local_force;
// 	local_force.x = attractive_local.x + repulsive_local.x;
// 	local_force.y = attractive_local.y + repulsive_local.y;
// 	local_force.z = attractive_local.z + repulsive_local.z;
// 	Force.push_back( local_force);
// 	ROS_INFO(" FORCE: %d", Force.front());
	
// }