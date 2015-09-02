#include "phobic_motion_planning.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Phobic_whife_MP");
	ros::NodeHandle node_mp;
	phobic_mp phobic_local_mp(node_mp);

	
	ros::Subscriber reader; //pose in word frame!!
	reader = node_mp.subscribe(node_mp.resolveName("SoftHand_Pose"), 1, &phobic_mp::MPCallback, &phobic_local_mp);
	//left arm
	
	//joint_listen();//ritonrna msg joint_State pos,vel,acc,coppia
	phobic_local_mp.joint_listen = node_mp.subscribe("left_arm/joint_states", 1, &phobic_mp::Robot_Callback_left, &phobic_local_mp);
	//left arm
	
	//joint_listen();//ritonrna msg joint_State pos,vel,acc,coppia
	phobic_local_mp.joint_listen = node_mp.subscribe("right_arm/joint_states", 1, &phobic_mp::Robot_Callback_right, &phobic_local_mp);


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
	// //make a robot model
	// hardware_interface::PositionJointInterface *robot;
	// ros::NodeHandle n;
	// lwr_controllers::KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
	
	bool check_urdf;
	check_urdf = GetVitoUrdf();
	if (check_urdf == true)
	{
		KDL::Frame local_frame_sh;
		KDL::Frame local_frame_base;

		//set reference frame
	    //const std::string& robot_namespace;
	    //root_name = robot_namespace + std::string("vito_anchor"); //it's the word frame
	  
		for(int i=0; i < hand_msg.hand_Pose.size(); i++)
		{
			
			switch(hand_msg.whichArm[i]) 
			{
				case 0: //left arm
					ROS_INFO("Vito use left arm");
					
					tf::poseMsgToKDL(hand_msg.hand_Pose[i], Hand_pose_kdl_l[i]);
					GetEuleroAngle(Hand_pose_kdl_l[i], Vito_desperate.Pos_HAND_l_xd[i]);

					listener_info.lookupTransform("/camera_rgb_optical_frame", "left_arm_base_link" , ros::Time(0), Vito_desperate.left_arm_base_link_st );
					listener_info.lookupTransform("/camera_rgb_optical_frame", "left_hand_palm_link" , ros::Time(0), Vito_desperate.left_arm_softhand_st[i] );

					//from stampedtrasform-->tf::transform-->KDL::frame 
					
					local_frame_sh = FromTFtoKDL(Vito_desperate.left_arm_softhand_st[i]);

					GetEuleroAngle(local_frame_sh, Vito_desperate.Pos_HAND_l_x[i]);

					local_frame_base = FromTFtoKDL(Vito_desperate.left_arm_base_link_st );
					GetEuleroAngle(local_frame_base, Vito_desperate.pos_base_l);
					// tf::Transform hand_tf(left_arm_softhand_st[i].getRotation(), left_arm_softhand_st[i].getOrigin());
					// TransformTFToKDL (hand_tf, hand_frame);

				break;

				case 1: //right arm

					ROS_INFO("Vito use right arm");
					
					tf::poseMsgToKDL( hand_msg.hand_Pose[i], Hand_pose_kdl_r[i]);
					GetEuleroAngle(Hand_pose_kdl_r[i], Vito_desperate.Pos_HAND_r_xd[i]);

					listener_info.lookupTransform("/camera_rgb_optical_frame", "right_arm_base_link" , ros::Time(0), Vito_desperate.right_arm_base_link_st );
					listener_info.lookupTransform("/camera_rgb_optical_frame", "right_hand_palm_link" , ros::Time(0), Vito_desperate.right_arm_softhand_st[i] );
					
					//from stampedtrasform-->tf::transform-->KDL::frame 
					
					local_frame_sh = FromTFtoKDL(Vito_desperate.right_arm_softhand_st[i]);	

					GetEuleroAngle(local_frame_sh, Vito_desperate.Pos_HAND_r_x[i]);
					
					local_frame_base = FromTFtoKDL(Vito_desperate.left_arm_base_link_st );
					GetEuleroAngle(local_frame_base, Vito_desperate.pos_base_l);

				break;

				case 2: //obstacles
					ROS_INFO("The object is obstacles");
					Eigen::Vector3d local_pos(hand_msg.hand_Pose[i].position.x, hand_msg.hand_Pose[i].position.y, hand_msg.hand_Pose[i].position.z);
					obstacle_position.push_back(local_pos);
					

					//SetRepulsiveFiled(hand_msg.*hand_Pose.begin().position);

				break;
			}

			whichArm.push_back(hand_msg.whichArm[i]);
		}

		// hand_msg.hand_Pose.clear();
		// hand_msg.whichArm.clear();
	}
	else
	{
		ROS_INFO("Failed to construct Vito urdf");
	}
}

KDL::Frame FromTFtoKDL(tf::StampedTransform st_transf)
{
	KDL::Frame hand_frame;
	tf::Transform hand_tf(st_transf.getRotation(), st_transf.getOrigin());
	tf::TransformTFToKDL(hand_tf, hand_frame);

	return hand_frame;
}

void phobic_mp::Esegui()
{
	for(int i=0; i < whichArm.size();i++)
	{	
		if(whichArm[i] == 2)
		{
			int arm_c;

			if((i==0) || (i == whichArm.size()-1))
			{
				continue;
			}
			else
			{	switch(whichArm[i+1] == 0)
				{
					case 0:
					
						arm_c = 0; 
					break;
					case 1;

						arm_c =1;
					break;
					case 2:
						continue;
					break;
				}
			}

			SetRepulsiveFiled(obstacle_position[i], arm_c); 
			SetPotentialField_robot(Force_repulsion, arm_c);

		}
		else
		{
			GetJacobian(whichArm[i]);
			SetPotentialField_robot(Force_repulsion, whichArm[i]);
			SetAttractiveField(whichArm[i], i);
		}
	}

	SetCommandVector();
	Send();
}

void phobic_mp::GetEuleroAngle(KDL::Frame &Hand_kdl_xd, Eigen::VectorXd pos_cartisian)
{
	//KDL::Frame hand_kdl_pose_l;
	double x, y,z,w;
	double roll, pitch, yaw;
	Hand_kdl_xd.GetRPY(roll, pitch, yaw);
	Hand_kdl_xd.GetQuaternion(x,y,z,w);

	
	pos_cartisian << x,y,z,roll,pitch,yaw;	//x_d in word frame		
	
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
	Eigen::VectorXd pos_Hand_xd, Vel;
	Eigen::VectorXd Pos_hand_x;

//Eigen::Vector3d phobic_mp::Take_Pos(tf::StampedTransform &M_tf)

	if(p == 0)
	{
		pos_Hand_xd = Vito_desperate.Pos_HAND_l_xd[i]; //6d
		Vel  = Vito_desperate.Vel_l[7];
		Pos_hand_x = Vito_desperate.Pos_HAND_l_x[i];
	}

	else
	{
		pos_Hand_xd = Vito_desperate.Pos_HAND_r_xd[i];
		Vel  = Vito_desperate.Vel_r[7];
		Pos_hand_x = Vito_desperate.Pos_HAND_r_x[i]; //6d
	}

	// To calculate the attractive force -dissipative(velocity-V_limit*desired_velocity)

	Eigen::VectorXd vel_servo_control; //xd_point
	
	vel_servo_control << P_goal/dissipative * (pos_Hand_xd - Pos_hand_x);

	double v_lim;
	v_lim = SetLimitation(vel_servo_control);

	Eigen::VectorXd velocity;
	velocity << (Vito_desperate.link_jac_[7] * Vel);  //x_point = jac*q_point

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


void phobic_mp::SetRepulsiveFiled(Eigen::Vector3d &Pos, int &p)
{
	std::vector<Eigen::Vector3d> distance_local_obj;
	std::vector<Eigen::VectorXd>* local_arm; 

	for(int t = 1; t<7; t++ )
	{
		if(p == 0) //left
		{
			 GetEuleroAngle(Vito_desperate.robot_position_left[t], local_arm );
		}
		else
		{
			 GetEuleroAngle(Vito_desperate.robot_position_righ[t], local_arm );

		}
	
		distance_local_obj.push_back(GetDistance(Pos,*local_arm[i])); 

	}

	std::vector<double>  min_d;

	for (int i=0; i <= distance_local_obj.size(); i++)
	{
		
		if(distance_local_obj <= influence )
		{
			min_d = distance_local_obj[i].norm();
		}

		else
		{
			continue;
		}
	}

	double min_distance;
	Eigen::Vector3d distance_der_partial;
	Eigen::Vector3d vec_Temp;

	for (int i=0; i< min_d.size()-1;i++)
	{
		if(min_d[i] < min_d[i+1])
		{
			min_distance = min_d[i];

        	distance_der_partial[0] =  min_d[i](0) / sqrt(pow(min_d[i](0))+ pow(min_d[i](1)) + pow(min_d[i](2)));
       		distance_der_partial[1] =  min_d[i](1) / sqrt(pow(min_d[i](0))+ pow(min_d[i](1)) + pow(min_d[i](2)));
       		distance_der_partial[2] =  min_d[i](2) / sqrt(pow(min_d[i](0))+ pow(min_d[i](1)) + pow(min_d[i](2)));
			vec_Temp = (P_obj/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial;
			Force_repulsion.push_back(vec_Temp);
		}

		else
		{
			continue;
		}

	}
}


Eigen::Vector3d phobic_mp::GetDistance(Eigen::VectorXd &obj1, Eigen::VectorXd &obj2 )
{
	Eigen::Vector3d distance_local;
	//pcl::PointXYZ local_point;
	
	distance_local = sqrt(pow(obj1[0]-obj2[0],2) + pow(obj1[1]-obj2[1],2) + pow(obj1[2]-obj2[2],2));  
	// local_point.x = (obj1.x-obj2.x);
	// local_point.y = (obj1.y-obj2.y);
	// local_point.z = (obj1.z-obj2.z);
	// distance_local.second =  local_point;

	return distance_local;
}




Eigen::Vector3d phobic_mp::Take_Pos(tf::StampedTransform &M_tf)
{
	Eigen::Vector3d Pos_vito;
	Eigen::Matrix4d Link_eigen;
	
	Link_eigen = FromTFtoEigen(M_tf);

	Pos_vito[0] = Link_eigen(0,3);
	Pos_vito[1] = Link_eigen(1,3);
	Pos_vito[2] = Link_eigen(2,3);

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