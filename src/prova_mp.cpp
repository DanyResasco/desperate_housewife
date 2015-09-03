#include "prova_mp.h"

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
	//phobic_local_mp.joint_listen = node_mp.subscribe("right_arm/joint_states", 1, &phobic_mp::Robot_Callback_right, &phobic_local_mp);


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
	bool check_urdf = true;
	//check_urdf = GetVitoUrdf();
	if (check_urdf == true)
	{	
		tf::StampedTransform left_arm_base_link_st, left_arm_softhand_st;
		
		//base and softhand in word frame
		listener_info.lookupTransform("/vito_anchor", "left_arm_base_link" , ros::Time(0), left_arm_base_link_st );
		InfoSoftHand(left_arm_base_link_st , Vito_desperate.pos_base_l ); //eulero angle base link

		listener_info.lookupTransform("/vito_anchor", "left_hand_palm_link" , ros::Time(0), left_arm_softhand_st );
		InfoSoftHand(left_arm_softhand_st, Vito_desperate.Pos_HAND_l_x); //eulero angle softhand

		int index_dx(0), index_sx(0);
		std::vector<KDL::Frame> Hand_pose_kdl_l;

		for(int i=0; i < hand_msg.hand_Pose.size(); i++) //number of object + obstacle
		{
			
			switch(hand_msg.whichArm[i]) 
			{
				case 0: //left arm
					ROS_INFO("Vito use left arm");
					
					tf::poseMsgToKDL(hand_msg.hand_Pose[i], Hand_pose_kdl_l[index_sx]);
					GetEuleroAngle(Hand_pose_kdl_l[index_sx], Vito_desperate.Pos_HAND_l_xd[index_sx]); //X_d

					index_sx ++;
	
				break;

				case 1: //right arm

					ROS_INFO("Vito use right arm");
					
					// tf::poseMsgToKDL( hand_msg.hand_Pose[i], Hand_pose_kdl_r[index_dx]);
					// GetEuleroAngle(Hand_pose_kdl_r[index_dx], Vito_desperate.Pos_HAND_r_xd[index_dx]);
					// index_dx ++;

				break;

				case 2: //obstacles
					ROS_INFO("The object is obstacles");
					Eigen::Vector3d local_pos(hand_msg.hand_Pose[i].position.x, hand_msg.hand_Pose[i].position.y, hand_msg.hand_Pose[i].position.z);
					obstacle_position.push_back(local_pos);

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


void phobic_mp::Esegui()
{
	for(int i=0; i < whichArm.size();i++) //number of objects+obstacles on the table
	{	
		GetJacobian(0);

		//repulsive field for vito.
		//SetPotentialField_robot(Force_repulsion_left, 0);
		Eigen::VectorXd local_pos_xd(Vito_desperate.Pos_HAND_l_xd[i]); 
		Eigen::VectorXd local_pos_x( Vito_desperate.Pos_HAND_l_x[i]);
		KDL::Jacobian local_jack = Vito_desperate.link_jac_l[7]; 

		SetAttractiveField(local_pos_xd, Vito_desperate.Vel_l, local_pos_x, Force_attractive_left, local_jack);
		//SetPotentialField_robot(Force_repulsion_right, 1);
		
		// switch(whichArm[i])
		// {
		// 	if((i==0) || (i == whichArm.size()-1))
		// 	{
		// 		continue; //NON VA BENE MI PERDO IL PRIMO OGGETTO OD OSTACOLO VEDERE COME FARE
		// 		if(whichArm[i]==0 )
		// 		{

		// 		}
		// 	}
		// 	else
		// 	{
		// 		case 0: //left arm
				
		// 			SetAttractiveField(Vito_desperate.Pos_HAND_l_xd[i], Vito_desperate.Vel_l[7],Vito_desperate.Pos_HAND_l_x[i], Force_attractive_left);

		// 			//repulsive field with other objcets and obstacles
		// 			for(int p =0; p <= i-1; p++)
		// 			{
		// 				for(int k = i+1; k< whichArm.size(); k++)
		// 				{
							
		// 					SetRepulsiveFiled(obstacle_position[i], 0, Force_repulsion_left);
							
		// 					SetRepulsiveFiled(obstacle_position[i], 1, Force_repulsion_left);  
		// 				}
		// 			}

		// 		break;
				
		// 		case 1: //rihgt arm 
		// 			for(int p =0; p <= i-1; p++)
		// 			{
		// 				for(int k = i+1; k< whichArm.size(); k++)
		// 				{
		// 					SetAttractiveField(Vito_desperate.Pos_HAND_r_xd[i], Vito_desperate.Vel_r[7],Vito_desperate.Pos_HAND_r_x[i], Force_attractive_right);
		// 					SetRepulsiveFiled(obstacle_position[i], 0, Force_repulsion_right);
		// 					SetRepulsiveFiled(obstacle_position[i], 1, Force_repulsion_right);  
		// 				}
		// 			}
		// 		break;
					
		// 			case 2:
		// 				SetRepulsiveFiled(obstacle_position[i], 0, Force_repulsion_right);
		// 				SetRepulsiveFiled(obstacle_position[i], 1, Force_repulsion_right);  

		// 			break;
				
		// 	}
		// }
	}


	//SetCommandVector();
	//Send();
}

void phobic_mp::GetEuleroAngle(KDL::Frame &Hand_kdl_xd, Eigen::VectorXd &pos_cartisian)
{
	//KDL::Frame hand_kdl_pose_l;
	double x, y,z,w;
	double roll, pitch, yaw;
	Hand_kdl_xd.M.GetRPY(roll, pitch, yaw); //KDL::FRAME--> kdl::Rotation --> rpy
	Hand_kdl_xd.M.GetQuaternion(x,y,z,w);	//KDL::FRAME--> kdl::Rotation --> x,y,z,w

	
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

// void phobic_mp::SetAttractiveField(int &p, int &i ) //calculate respect word frame
void phobic_mp::SetAttractiveField(Eigen::VectorXd &pos_Hand_xd, Eigen::VectorXd &Vel, Eigen::VectorXd &Pos_hand_x, Eigen::VectorXd &Force_attractive,  KDL::Jacobian &link_jac_)
{		
// 	Eigen::VectorXd pos_Hand_xd, Vel;
// 	Eigen::VectorXd Pos_hand_x;

// //Eigen::Vector3d phobic_mp::Take_Pos(tf::StampedTransform &M_tf)

// 	if(p == 0)
// 	{
// 		pos_Hand_xd = Vito_desperate.Pos_HAND_l_xd[i]; //6d
// 		Vel  = Vito_desperate.Vel_l[7];
// 		Pos_hand_x = Vito_desperate.Pos_HAND_l_x[i];
// 	}

// 	else
// 	{
// 		pos_Hand_xd = Vito_desperate.Pos_HAND_r_xd[i];
// 		Vel  = Vito_desperate.Vel_r[7];
// 		Pos_hand_x = Vito_desperate.Pos_HAND_r_x[i]; //6d
// 	}

	// To calculate the attractive force -dissipative(velocity-V_limit*desired_velocity)

	Eigen::VectorXd vel_servo_control; //xd_point
	
	vel_servo_control = P_goal/dissipative * (pos_Hand_xd - Pos_hand_x); //1*6

	double v_lim;
	v_lim = SetLimitation(vel_servo_control);

	Eigen::VectorXd velocity;
	velocity = (link_jac_.data * Vel.transpose());  //x_point = jac*q_point 6*1

	Force_attractive = (- dissipative *(velocity - v_lim * vel_servo_control.transpose())); //in vito frame

	
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

// void phobic_mp::SetCommandVector()
// {
// 	ROS_INFO("dentro SetCommandVector()");
// 	double local_force_r,local_force_p;

// 	for (int i=0; i< Force_attractive.size()-1; i++)
// 	{
// 		local_force_r = Force_attractive[i]+ Force_attractive[i+1];	
// 	}
// 	for (int p=0; p< Force_repulsion.size()-1;p++)
// 	{
// 		local_force_p = Force_repulsion[p] + Force_repulsion[p+1];
// 	}
	
// 	Force << (local_force_r + local_force_p);
// 	SetPseudoInvJac();

// 	//q_point_desired = jac_pseudoInv*Force;
// }


// void phobic_mp::SetRepulsiveFiled(Eigen::Vector3d &Pos, int p, Eigen::Vector3d Force_repulsion )
// {
// 	std::vector<Eigen::Vector3d> distance_local_obj;
// 	std::vector<Eigen::VectorXd>* local_arm; 

// 	for(int t = 0; t<7; t++ )
// 	{
// 		if(p == 0) //left
// 		{
// 			GetEuleroAngle(Vito_desperate.robot_position_left[t], local_arm );
// 		}
		
// 		else
// 		{
// 			GetEuleroAngle(Vito_desperate.robot_position_righ[t], local_arm );

// 		}
	
// 		distance_local_obj.push_back(GetDistance(Pos,*local_arm[i])); 

// 	}

// 	std::vector<double>  min_d;

// 	for (int i=0; i <= distance_local_obj.size(); i++)
// 	{
// 		double local_distance;
// 		local_distance  = distance_local_obj[i].norm();
		
// 		if( local_distance <= influence )
// 		{
// 			min_d = local_distance;
// 		}

// 		else
// 		{
// 			continue;
// 		}
// 	}

// 	double min_distance;
// 	Eigen::Vector3d distance_der_partial;
// 	Eigen::Vector3d vec_Temp;

// 	for (int i=0; i< min_d.size()-1;i++)
// 	{
// 		if(min_d[i] < min_d[i+1])
// 		{
// 			min_distance = min_d[i];

//         	distance_der_partial[0] =  min_d[i](0) / sqrt(pow(min_d[i](0))+ pow(min_d[i](1)) + pow(min_d[i](2)));
//        		distance_der_partial[1] =  min_d[i](1) / sqrt(pow(min_d[i](0))+ pow(min_d[i](1)) + pow(min_d[i](2)));
//        		distance_der_partial[2] =  min_d[i](2) / sqrt(pow(min_d[i](0))+ pow(min_d[i](1)) + pow(min_d[i](2)));
// 			vec_Temp = (P_obj/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial;
// 			Force_repulsion.push_back(vec_Temp);
// 		}

// 		else
// 		{
// 			continue;
// 		}

// 	}
// }


void phobic_mp::InfoSoftHand(tf::StampedTransform &SoftHand_orBase_tf, Eigen::VectorXd &Eulero_angle  )
{
	//from stampedtrasform-->tf::transform-->KDL::frame -->take eulero angle
	KDL::Frame local_frame_sh;

	local_frame_sh = FromTFtoKDL(SoftHand_orBase_tf);

	GetEuleroAngle(local_frame_sh, Eulero_angle);
}


KDL::Frame FromTFtoKDL(tf::StampedTransform &st_transf)
{
	KDL::Frame hand_frame;
	tf::Transform hand_tf(st_transf.getRotation(), st_transf.getOrigin());
	tf::transformTFToKDL(hand_tf, hand_frame);

	return hand_frame;
}



 Eigen::Vector3d phobic_mp::GetDistance(Eigen::VectorXd &obj1, Eigen::VectorXd &obj2 )
//double phobic_mp::GetDistance(Eigen::VectorXd &obj1, Eigen::VectorXd &obj2 )
{
	Eigen::Vector3d distance_local;
	//double distance_local;
	//distance_local = sqrt(pow(obj1[0]-obj2[0],2) + pow(obj1[1]-obj2[1],2) + pow(obj1[2]-obj2[2],2));  
	  distance_local[0] = obj1[0] - obj2[0];
	  distance_local[1] = obj1[1] - obj2[1];
	  distance_local[2] = obj1[2] - obj2[2];
	
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