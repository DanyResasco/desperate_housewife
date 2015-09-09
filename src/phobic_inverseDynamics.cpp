#include <prova_mp.h>

namespace desperate_inversedynamics
{
	phobic_mp::phobic_mp() {}
	phobic_mp::~phobic_mp() {}

	bool phobic_mp::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
		nh_ = n;

		// get URDF and name of root and tip from the parameter server
		std::string robot_description, root_name, tip_name;

		if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM(" No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
		    return false;
		}

		if (!nh_.getParam("root_name", root_name))
		{
		    ROS_ERROR_STREAM(" No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
		    return false;
		}

		if (!nh_.getParam("tip_name", tip_name))
		{
		    ROS_ERROR_STREAM(" No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
		    return false;
		}
	 
		// Get the gravity vector (direction and magnitude)
		KDL::Vector gravity_ = KDL::Vector::Zero();
		gravity_(2) = -9.81;

		// Construct an URDF model from the xml string
		std::string xml_string;

		if (n.hasParam(robot_description))
			n.getParam(robot_description.c_str(), xml_string);
		else
		{
		    ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
		    n.shutdown();
		    return false;
		}

		if (xml_string.size() == 0)
		{
			ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
		    n.shutdown();
		    return false;
		}

		ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());
		
		// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
		    ROS_ERROR("Failed to parse urdf file");
		    n.shutdown();
		    return false;
		}
		ROS_INFO("Successfully parsed urdf file");
		
		KDL::Tree kdl_tree_;
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
		{
		    ROS_ERROR("Failed to construct kdl tree");
		    n.shutdown();
		    return false;
		}

		check_urdf = true;

		// Parsing joint limits from urdf model
		boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
    	boost::shared_ptr<const urdf::Joint> joint_;
    	joint_limits_.min.resize(kdl_tree_.getNrOfJoints());
		joint_limits_.max.resize(kdl_tree_.getNrOfJoints());
		joint_limits_.center.resize(kdl_tree_.getNrOfJoints());
		int index;

    	for (int i = 0; i < kdl_tree_.getNrOfJoints() && link_; i++)
    	{
    		joint_ = model.getJoint(link_->parent_joint->name);  
    		index = kdl_tree_.getNrOfJoints() - i - 1;

    		joint_limits_.min(index) = joint_->limits->lower;
    		joint_limits_.max(index) = joint_->limits->upper;
    		joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;

    		link_ = model.getLink(link_->getParent()->name);
    	}

		// Populate the KDL chain
		if(!kdl_tree_.getChain(root_name, tip_name, Vito_desperate.kdl_chain_))
		{
		    ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
		    ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
		    ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
		    ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
		    ROS_ERROR_STREAM("  The segments are:");

		    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
		    KDL::SegmentMap::iterator it;

		    for( it=segment_map.begin(); it != segment_map.end(); it++ )
		      ROS_ERROR_STREAM( "    "<<(*it).first);

		  	return false;
		}

		ROS_DEBUG("Number of segments: %d", Vito_desperate.kdl_chain_.getNrOfSegments());
		ROS_DEBUG("Number of joints in chain: %d", Vito_desperate.kdl_chain_.getNrOfJoints());

		// Get joint handles for all of the joints in the chain
		for(std::vector<KDL::Segment>::const_iterator it = Vito_desperate.kdl_chain_.segments.begin()+1; it != Vito_desperate.kdl_chain_.segments.end(); ++it)
		{
		    Vito_desperate.joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
		    ROS_DEBUG("%s", it->getJoint().getName().c_str() );
		}

		ROS_DEBUG(" Number of joints in handle = %lu", Vito_desperate.joint_handles_.size() );

		jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(Vito_desperate.kdl_chain_));
		id_solver_.reset(new KDL::ChainDynParam(Vito_desperate.kdl_chain_,gravity_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(Vito_desperate.kdl_chain_));

		Vito_desperate.joint_msr_states_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		Vito_desperate.joint_des_states_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		Vito_desperate.qdot_last_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		tau_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		Vito_desperate.J_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		Vito_desperate.J_dot_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		Vito_desperate.J_star_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		// Kp_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		// Kd_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		// PIDs_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		
		Vito_desperate.M_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		Vito_desperate.C_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());
		Vito_desperate.G_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());

		Vito_desperate.J_last_.resize(Vito_desperate.kdl_chain_.getNrOfJoints());

		sub_command_ = nh_.subscribe(nh_.resolveName("SoftHand_Pose"), 1, &phobic_mp::MPCallback, this);	//???? 
		//sub_gains_ = nh_.subscribe("set_gains", 1, &OneTaskInverseDynamicsJL::set_gains, this);

		// pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
		// pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
		// pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1000);

		return true;
	}

	void phobic_mp::starting(const ros::Time& time)
	{
		// get joint positions
  		for(int i=0; i < Vito_desperate.joint_handles_.size(); i++) 
  		{
    		Vito_desperate.joint_msr_states_.q(i) = Vito_desperate.joint_handles_[i].getPosition();
    		Vito_desperate.joint_msr_states_.qdot(i) = Vito_desperate.joint_handles_[i].getVelocity();
    		// Vito_desperate.joint_des_states_.q(i) = Vito_desperate.joint_msr_states_.q(i);
    		// Vito_desperate.joint_des_states_.qdot(i) = Vito_desperate.joint_msr_states_.qdot(i);
    		// Kp_(i) = 100;
  			// Kd_(i) = 20;
    	}

    	ROS_INFO("dentro starting");
    	// Kp = 200;
    	// Ki = 1; 
    	// Kd = 5;

    	// for (int i = 0; i < PIDs_.size(); i++)
    	// 	PIDs_[i].initPid(Kp,Ki,Kd,0.1,-0.1);
    	//ROS_INFO("PIDs gains are: Kp = %f, Ki = %f, Kd = %f",Kp,Ki,Kd);

    	I_ = Eigen::Matrix<double,7,7>::Identity(7,7);
    	//e_ref_ = Eigen::Matrix<double,6,1>::Zero();

    	first_step_ = 0;
    	cmd_flag_ = 0;	//for starting the controller
    	step_ = 0;

	}

	void phobic_mp::update(const ros::Time& time, const ros::Duration& period)
	{
		
		tf::StampedTransform left_arm_base_link_st, left_arm_softhand_st;
			
		//base and softhand in word frame
		listener_info.lookupTransform("/vito_anchor", "left_arm_base_link" , ros::Time(0), left_arm_base_link_st );
		Vito_desperate.pos_base_l = FromTFtoKDL(left_arm_base_link_st); 

		// listener_info.lookupTransform("/vito_anchor", "left_hand_palm_link" , ros::Time(0), left_arm_softhand_st );
		// Vito_desperate.Pos_HAND_l_x = FromTFtoKDL(left_arm_softhand_st); 
		// InfoSoftHand(left_arm_softhand_st, Vito_desperate.Pos_HAND_l_x); //eulero angle softhand
		ROS_INFO("dentro update");

		// get joint positions
  		for(int i=0; i < Vito_desperate.joint_handles_.size(); i++) 
  		{
    		Vito_desperate.joint_msr_states_.q(i) = Vito_desperate.joint_handles_[i].getPosition();
    		Vito_desperate.joint_msr_states_.qdot(i) = Vito_desperate.joint_handles_[i].getVelocity();
    	}

    	// // clearing msgs before publishing
    	// msg_err_.data.clear();
    	// msg_pose_.data.clear();
    	
    	if (cmd_flag_)
    	{
    		// resetting N and tau(t=0) for the highest priority task
    		N_trans_ = I_;	
    		SetToZero(tau_);

    		// computing Inertia, Coriolis and Gravity matrices
		    id_solver_->JntToMass(Vito_desperate.joint_msr_states_.q, Vito_desperate.M_);
		    id_solver_->JntToCoriolis(Vito_desperate.joint_msr_states_.q, Vito_desperate.joint_msr_states_.qdot, Vito_desperate.C_);
		    id_solver_->JntToGravity(Vito_desperate.joint_msr_states_.q, Vito_desperate.G_);
		    Vito_desperate.G_.data.setZero();

		    Eigen::MatrixXd M_inv_;
		    // computing the inverse of Vito_desperate.M_ now, since it will be used often
		    // pseudo_inverse(Vito_desperate.M_.data, M_inv_, false); //
		    M_inv_ = Vito_desperate.M_.data.inverse(); 


	    	// computing Jacobian J(q)
	    	jnt_to_jac_solver_->JntToJac(Vito_desperate.joint_msr_states_.q, Vito_desperate.J_);	//6*7

	    	// computing the distance from the mid points of the joint ranges as objective function to be minimized
	    	phi_ = task_objective_function(Vito_desperate.joint_msr_states_.q);

	    	// using the first step to compute jacobian of the tasks
	    	if (first_step_)
	    	{
	    		Vito_desperate.J_last_ =Vito_desperate.J_;
	    		phi_last_ = phi_;
	    		first_step_ = 0;
	    		return;
	    	}

	    	// computing the derivative of Jacobian J_dot(q) through numerical differentiation
	    	Vito_desperate.J_dot_.data = (Vito_desperate.J_.data - Vito_desperate.J_last_.data)/period.toSec();

	    	// computing forward kinematics
	    	for(int i=0; i < Vito_desperate.joint_handles_.size(); i++) 
  			{
  				KDL::Frame x_temp;
	    		fk_pos_solver_->JntToCart(Vito_desperate.joint_msr_states_.q, x_temp, i);	//pos_joint
	    		x_now.push_back(x_temp);
	    		// GetEuleroAngle(x_ [i], x_now[i]);
	    	}
	    	
	    	int index_rep=0;
	    	for(int p =0; p < x_des_.size(); p ++)
	    	{
	    			
			    if (Equal(x_now[7].p, x_des_[p].p,0.05))	//x_now[7] is the position of softhand
			    {
			    	ROS_INFO("On target");
			    	cmd_flag_ = 0;
			    	return;	    		
			    }
			    

				SetAttractiveField(x_des_[p], Vito_desperate.joint_msr_states_.qdot , x_now[7], Force_attractive_left[p],  Vito_desperate.J_);
						
				if (p == 0)	
				{
					// continue;
					for(int k = p+1; k < x_des_.size()-1; k++)
					{
						SetRepulsiveFiled(x_des_[k].p, Force_repulsion_left[index_rep]);
						index_rep ++;  
					}
				}
				
				else
				{
	
					for(int i =0; i <= p-1; i++)
					{
						SetRepulsiveFiled(x_des_[i].p, Force_repulsion_left[index_rep]);
						index_rep ++;
					}
				
				
					for(int k = p+1; k < x_des_.size()-1; k++)
					{
						SetRepulsiveFiled(x_des_[k].p, Force_repulsion_left[index_rep]);
						index_rep++;  
					}
				}
			}			


			for(int k=0; k < obstacle_position.size(); k++)
			{
				SetRepulsiveFiled(obstacle_position[k], Force_repulsion_left[index_rep]);
				index_rep++;
			}


			SetPotentialField_robot(Force_repulsion_left[index_rep], p);


	    	Eigen::Matrix<double,6,1> b_;
			
			Eigen::MatrixXd omega_;
			Eigen::MatrixXd lambda_;
			Eigen::MatrixXd P_;
			// Eigen::MatrixXd J_ext_t;
			Eigen::MatrixXd P;
			KDL::Jacobian J_ext_t;
		
			// computing nullspace
	    	N_trans_ = N_trans_ - Vito_desperate.J_.data.transpose()*lambda_*Vito_desperate.J_.data*M_inv_;  
			
			omega_ = (Vito_desperate.J_.data * M_inv_ * Vito_desperate.J_.data.transpose()).inverse();
			J_ext_t.data = omega_ * Vito_desperate.J_.data * M_inv_;

			lambda_ = (J_ext_t.data * Vito_desperate.C_.data - Vito_desperate.M_.data * Vito_desperate.J_dot_.data * Vito_desperate.joint_msr_states_.qdot.data );
			P_ = J_ext_t.data * Vito_desperate.G_.data;


			Eigen::VectorXd Force_rep_local = Eigen::VectorXd::Zero(6);
			Eigen::VectorXd Force_att_local = Eigen::VectorXd::Zero(6);

			for(int i=0; i< Force_attractive_left.size()-1;i++)
			{
				Force_att_local = Force_att_local +  Force_attractive_left[i];  
			}

			for(int i=0; i< Force_repulsion_left.size()-1;i++)
			{
				Force_rep_local = Force_rep_local +  Force_repulsion_left[i];  
			}


			Force = Force_rep_local + Force_att_local;

			// control law
			tau_.data = Vito_desperate.J_.data.transpose()*(omega_*Force + lambda_ + P_) + N_trans_*(Eigen::Matrix<double,7,1>::Identity(7,1)*(phi_ - phi_last_)/(period.toSec())) ;
	
    	}

    	// set controls for joints
    	for (int i = 0; i < Vito_desperate.joint_handles_.size(); i++)
    	{
    		if(cmd_flag_)
    			Vito_desperate.joint_handles_[i].setCommand(tau_(i));
    		else
      			continue;	
    			// Vito_desperate.joint_handles_[i].setCommand(PIDs_[i].computeCommand(Vito_desperate.joint_des_states_.q(i) - Vito_desperate.joint_msr_states_.q(i),period));
    	}


	    ros::spinOnce();

	}
	//chiamata per leggere il messaggio della posa desiderata della mano
	void phobic_mp::MPCallback(const desperate_housewife::hand hand_msg)
	{	
		if (check_urdf == true)
		{	
			int index_dx(0), index_sx(0);
			std::vector<KDL::Frame> Hand_pose_kdl_l;

			for(int i=0; i < hand_msg.hand_Pose.size(); i++) //number of object + obstacle
			{
				
				switch(hand_msg.whichArm[i]) 
				{
					case 0: //left arm
						ROS_INFO("Vito use left arm");
						
						tf::poseMsgToKDL(hand_msg.hand_Pose[i], x_des_[index_sx]);
						index_sx ++;
		
					break;

					case 1: //right arm

						ROS_INFO("Vito use right arm");
						
						// tf::poseMsgToKDL( hand_msg.hand_Pose[i], Hand_pose_kdl_r[index_dx]);
						// index_dx ++;

					break;

					case 2: //obstacles
						ROS_INFO("The object is obstacles");
						KDL::Vector local_pos(hand_msg.hand_Pose[i].position.x, hand_msg.hand_Pose[i].position.y, hand_msg.hand_Pose[i].position.z);
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

	
	double phobic_mp::task_objective_function(KDL::JntArray q)
	{
		double sum = 0;
		double temp;
		int N = q.data.size();

		for (int i = 0; i < N; i++)
		{
			temp = ((q(i) - joint_limits_.center(i))/(joint_limits_.max(i) - joint_limits_.min(i)));
			sum += temp*temp;
		}

		sum /= 2*N;

		return -sum;
	// }
	}


	void phobic_mp::SetAttractiveField(KDL::Frame &pos_Hand_xd, KDL::JntArray &Vel, KDL::Frame &Pos_hand_x, Eigen::VectorXd &Force_attractive,  KDL::Jacobian &link_jac_)
	{		
		double roll_xd, pitch_xd, yaw_xd;
		double roll_x, pitch_x, yaw_x;
		
		pos_Hand_xd.M.RPY(roll_xd, pitch_xd, yaw_xd);
		Pos_hand_x.M.RPY(roll_x, pitch_x, yaw_x);
		//KDL::Vector vel_servo_control; //xd_point
		Eigen::VectorXd vel_servo_control;
		// KDL::Vector local_dist_v;
		// KDL::Rotation local_dist_r;
		KDL::Frame distance;
		distance.p = pos_Hand_xd.p - Pos_hand_x.p;
		// distance.M = pos_Hand_xd.M. - Pos_hand_x.M;
		
		Eigen::VectorXd temp_dist_eigen;
		temp_dist_eigen << distance.p(0), distance.p(1), distance.p(2) , roll_xd - roll_x , pitch_xd - pitch_x, yaw_xd - yaw_x;

		vel_servo_control = P_goal/dissipative * temp_dist_eigen.transpose(); 
		// (pos_Hand_xd - Pos_hand_x); //1*6

		x_dot_ = Vito_desperate.J_.data* Vel.data; //velocity 6*1

		double v_lim = 1;
		//v_lim = SetLimitation(vel_servo_control);

		// Eigen::VectorXd velocity;
		//KDL::Vector velocity;
		// velocity.data = (link_jac_.data * Vel.data);  //x_point = jac*q_point 6*1

		Force_attractive = (- dissipative *(x_dot_ - v_lim * vel_servo_control)); //in vito frame

		
	}




	void phobic_mp::SetRepulsiveFiled(KDL::Vector &Pos, Eigen::VectorXd &Force_repulsion )
	{
		std::vector<KDL::Vector> distance_local_obj;
		//std::vector<Eigen::VectorXd> local_arm; 

		for(int t = 0; t <= 7; t++ )
		{
			
			distance_local_obj.push_back(Pos - x_now[t].p); 
			
		}

		std::vector<double>  min_d;
		std::vector<int> index_infl;

		for (int i=0; i <= distance_local_obj.size(); i++)
		{
			double local_distance;
			local_distance  = distance_local_obj[i].Norm();
			
			if( local_distance <= influence )
			{
				min_d.push_back(local_distance);
				index_infl.push_back(i); //for tracking wich jacobian used
			}

			else
			{
				continue;
			}
		}

		double min_distance = min_d[0];
		Eigen::Vector3d distance_der_partial;
		Eigen::Vector3d vec_Temp;
		int index_dist;
		
		for (int i=0; i< min_d.size()-1;i++)
		{
			if(min_distance > min_d[i+1])
			{
				min_distance = min_d[i+1];
				index_dist = i+1;
	        }

	        else
			{
				continue;
			}
		}

		int D;
		D = index_infl[index_dist];

	    distance_der_partial[0] =  distance_local_obj[D](0) / sqrt(pow(distance_local_obj[D](0),2)+ pow(distance_local_obj[D](1),2) + pow(distance_local_obj[D](2),2));
	    distance_der_partial[1] =  distance_local_obj[D](1) / sqrt(pow(distance_local_obj[D](0),2)+ pow(distance_local_obj[D](1),2) + pow(distance_local_obj[D](2),2));
	    distance_der_partial[2] =  distance_local_obj[D](2) / sqrt(pow(distance_local_obj[D](0),2)+ pow(distance_local_obj[D](1),2) + pow(distance_local_obj[D](2),2));
				
		vec_Temp = (P_obj/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial;
				
		Force_repulsion << vec_Temp, 0,0,0;
			

	}



void phobic_mp::SetPotentialField_robot(Eigen::VectorXd &Force_repulsion, int p)
{
    std::vector<KDL::Vector> distance_link; //Only position

    std::vector<KDL::Vector> robot_link_position1;
    std::vector<KDL::Vector> robot_link_position2;
    robot_link_position1.resize(x_now.size());

    KDL::Vector HAND;
    KDL::Vector base_link;
    //robot_link_position->resize(Vito_desperate.robot_position_left.size());
    
     //p==0 is left arm, else is right arm
    // if(p ==0)
    // {
      // for(int i=0; i< Vito_desperate.link_frame_l.size();i++)
      // { 
      //   GetEuleroAngle(Vito_desperate.link_frame_l[i], robot_link_position1[i]); //take's the eulero angles
      //   //GetEuleroAngle(Vito_desperate.link_frame_r[i], robot_link_position2[i]);   
  
      // }
    	for (int i=0; i< x_now.size()-1; i++)
    	{
    		robot_link_position1[i] = x_now[i].p;
    	}
   
        HAND =  x_now[7].p;     

        base_link = Vito_desperate.pos_base_l.p ;
        
     //}


     // else //right arm
     // {
     //    for(int i=0; i< Vito_desperate.link_frame_r.size();i++)
     //   { 
     //      GetEuleroAngle(Vito_desperate.link_frame_r[i], robot_link_position1[i]);
     //      GetEuleroAngle(Vito_desperate.link_frame_l[i], robot_link_position2[i]);   

     //    }

     //     HAND = Vito_desperate.Pos_HAND_r_x;
        

     //    base_link = Vito_desperate.pos_base_r; 
     //} 

    //Self collision
    // distance_link.push_back(GetDistance(robot_link_position1[1], robot_link_position2[5]));
    // distance_link.push_back(GetDistance(robot_link_position1[1], robot_link_position2[6]));
    // distance_link.push_back(GetDistance(robot_link_position1[1], robot_link_position2[7]));
    // distance_link.push_back(GetDistance(robot_link_position1[2], robot_link_position2[5]));
    // distance_link.push_back(GetDistance(robot_link_position1[2], robot_link_position2[6]));
    // distance_link.push_back(GetDistance(robot_link_position1[2], robot_link_position2[7])); 
    // distance_link.push_back(GetDistance(robot_link_position1[3], robot_link_position2[4]));
    // distance_link.push_back(GetDistance(robot_link_position1[3], robot_link_position2[5]));
    // distance_link.push_back(GetDistance(robot_link_position1[3], robot_link_position2[6]));
    // distance_link.push_back(GetDistance(robot_link_position1[3], robot_link_position2[7]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[3]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[4]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[5]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[6]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[7]));

    // for(int i=5;i<=7;i++)
    // {
    //   for (int k=1; k<=7; k++)
    //   {
    //     distance_link.push_back(GetDistance(robot_link_position1[i], robot_link_position2[k]));
    //   }
    // }  

    //collision with softhand
    distance_link.push_back(HAND - base_link);

    for(int j = 3; j >= 1; j--) //with himself
    {
      // distance_link.push_back(GetDistance(HAND, robot_link_position1[j]));
    	distance_link.push_back(HAND - robot_link_position1[j]);
    }

    // for(int i=1; i<=7; i++)//with other arm
    // {
    //   distance_link.push_back(GetDistance(HAND,robot_link_position2[i]));
    // }

    // Repulsive fields = K/distance^2 (1/distance -1/influence) partial_derivative_vector

    for (int i=0; i <= distance_link.size(); i++)
    {
       double local_distance;
       local_distance  = distance_link[i].Norm();

      if(local_distance <= influence) //if in minus than influence area
      {
        Eigen::Vector3d distance_der_partial;
        distance_der_partial[0] =  distance_link[i](0) / sqrt(pow(distance_link[i](0),2)+ pow(distance_link[i](1),2) +pow(distance_link[i](2),2));
        distance_der_partial[1] =  distance_link[i](1) / sqrt(pow(distance_link[i](0),2)+ pow(distance_link[i](1),2) +pow(distance_link[i](2),2));
        distance_der_partial[2] =  distance_link[i](2) / sqrt(pow(distance_link[i](0),2)+ pow(distance_link[i](1),2) +pow(distance_link[i](2),2));

        Eigen::Vector3d vec_Temp;
        vec_Temp = (P_obj/pow(distance_link[i].	Norm(),2)) * (1/distance_link[i].Norm() - 1/influence) *distance_der_partial;

        Force_repulsion << vec_Temp, 0,0,0; //Ricorda che se devi metterne più di uno devi mettere le parentesi
      }

      else
      {
        //Eigen::VectorXd fiel_zero(0,0,0);
        Force_repulsion = Eigen::VectorXd::Zero(6);
        //continue;
      }
    }
}

// void OneTaskInverseDynamicsJL::set_marker(KDL::Frame x, int id)
	// {			
	// 			msg_marker_.header.frame_id = "world";
	// 			msg_marker_.header.stamp = ros::Time();
	// 			msg_marker_.ns = "end_effector";
	// 			msg_marker_.id = id;
	// 			msg_marker_.type = visualization_msgs::Marker::SPHERE;
	// 			msg_marker_.action = visualization_msgs::Marker::ADD;
	// 			msg_marker_.pose.position.x = x.p(0);
	// 			msg_marker_.pose.position.y = x.p(1);
	// 			msg_marker_.pose.position.z = x.p(2);
	// 			msg_marker_.pose.orientation.x = 0.0;
	// 			msg_marker_.pose.orientation.y = 0.0;
	// 			msg_marker_.pose.orientation.z = 0.0;
	// 			msg_marker_.pose.orientation.w = 1.0;
	// 			msg_marker_.scale.x = 0.005;
	// 			msg_marker_.scale.y = 0.005;
	// 			msg_marker_.scale.z = 0.005;
	// 			msg_marker_.color.a = 1.0;
	// 			msg_marker_.color.r = 0.0;
	// 			msg_marker_.color.g = 1.0;
	// 			msg_marker_.color.b = 0.0;	
	// }














	// void OneTaskInverseDynamicsJL::command_configuration(const lwr_controllers::PoseRPY::ConstPtr &msg)
	// {	
	// 	KDL::Frame frame_des_;

	// 	switch(msg->id)
	// 	{
	// 		case 0:
	// 		frame_des_ = KDL::Frame(
	// 				KDL::Rotation::RPY(msg->orientation.roll,
	// 					 			  msg->orientation.pitch,
	// 							 	  msg->orientation.yaw),
	// 				KDL::Vector(msg->position.x,
	// 							msg->position.y,
	// 							msg->position.z));
	// 		break;
	
	// 		case 1: // position only
	// 		frame_des_ = KDL::Frame(
	// 			KDL::Vector(msg->position.x,
	// 						msg->position.y,
	// 						msg->position.z));
	// 		break;
		
	// 		case 2: // orientation only
	// 		frame_des_ = KDL::Frame(
	// 			KDL::Rotation::RPY(msg->orientation.roll,
	// 			   	 			   msg->orientation.pitch,
	// 							   msg->orientation.yaw));
	// 		break;

	// 		default:
	// 		ROS_INFO("Wrong message ID");
	// 		return;
	// 	}
		
	// 	x_des_ = frame_des_;
	// 	cmd_flag_ = 1;
	// }

	// void OneTaskInverseDynamicsJL::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
	// {
	// 	if(msg->data.size() == 3)
	// 	{
	// 		for(int i = 0; i < PIDs_.size(); i++)
	// 			PIDs_[i].setGains(msg->data[0],msg->data[1],msg->data[2],0.3,-0.3);
	// 		ROS_INFO("New gains set: Kp = %f, Ki = %f, Kd = %f",msg->data[0],msg->data[1],msg->data[2]);
	// 	}
	// 	else
	// 		ROS_INFO("PIDs gains needed are 3 (Kp, Ki and Kd)");
	// }



















}

PLUGINLIB_EXPORT_CLASS(desperate_inversedynamics::phobic_mp, controller_interface::ControllerBase)