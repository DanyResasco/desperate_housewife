#include <potential_field_control.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <trajectory_msgs/JointTrajectory.h>

#include <math.h>




namespace desperate_housewife 
{
  PotentialFieldControl::PotentialFieldControl() {}
  PotentialFieldControl::~PotentialFieldControl() {}

  bool PotentialFieldControl::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
      PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
      ROS_INFO("Starting controller");
      ROS_WARN("Number of segments: %d", kdl_chain_.getNrOfSegments());
    // for swicht the hand_desired
    n.getParam("desired_reference_topic", desired_reference_topic);
    n.getParam("obstacle_remove_topic", obstacle_remove_topic);
    n.getParam("desired_hand_name", desired_hand_name);
    n.getParam("desired_hand_topic", desired_hand_topic);
    n.getParam("obstacle_avoidance", obstacle_avoidance);
    n.getParam("tip_name", tip_name);
    n.getParam("set_gains_topic", set_gains_);

   
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,gravity_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        
    qdot_last_.resize(kdl_chain_.getNrOfJoints());
    tau_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());
    J_dot_.resize(kdl_chain_.getNrOfJoints());
    J_star_.resize(kdl_chain_.getNrOfJoints());
    Kp_.resize(kdl_chain_.getNrOfJoints());
    Kd_.resize(kdl_chain_.getNrOfJoints());
    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());

    J_last_.resize(kdl_chain_.getNrOfJoints());

    ROS_DEBUG("Subscribed for desired_hand_topic to: %s", desired_reference_topic.c_str());
     //list of obstacles
    ROS_INFO("Subscribed for obstacle_avoidance_topic to : %s", obstacle_avoidance.c_str());
    obstacles_subscribe_ = n.subscribe(obstacle_avoidance.c_str(), 1, &PotentialFieldControl::InfoGeometry, this);

    //Hand_pose for graspable objects
    sub_gains_ = nh_.subscribe(set_gains_.c_str(), 1, &PotentialFieldControl::set_gains, this);

    // hand_publisher_ = n.advertise<trajectory_msgs::JointTrajectory>(desired_hand_topic, 1000);

    ROS_INFO("Subscribed for obstacle_remove_topic to : %s", obstacle_remove_topic.c_str());
    obstacles_remove_sub = n.subscribe(obstacle_remove_topic.c_str(), 1, &PotentialFieldControl::InfoOBj, this);     

    // pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
    pub_error_ = nh_.advertise<desperate_housewife::Error_msg>("error", 1000);
    // pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
    // pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1000);

    sub_command_ = n.subscribe(desired_reference_topic.c_str(), 1, &PotentialFieldControl::command, this); 

    // nh_.param<std::string>("/BasicGeometriesNode/cylinder_names", object_names_, "object");
    // publisher_wrench_command = nh_.advertise<geometry_msgs::WrenchStamped>("left_arm/PotentialFieldControl/wrench_msg", 1000);
    // publisher_wrench_command_rep = nh_.advertise<geometry_msgs::WrenchStamped>("left_arm/PotentialFieldControl/wrench_msg2", 1000);

    return true;
  }

  void PotentialFieldControl::starting(const ros::Time& time)
  {
    // get joint positions
      for(unsigned int i=0; i < joint_handles_.size(); i++) 
      {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
        joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
        Kp_(i) = 1000;  
        Kd_(i) = 200;
       
      }

      I_ = Eigen::Matrix<double,7,7>::Identity(7,7);
      Force_attractive = Eigen::Matrix<double,6,1>::Zero();
      Force_repulsive = Eigen::Matrix<double,7,1>::Zero();
      F_Rep_table = Eigen::Matrix<double,7,1>::Zero();
      Force_total_rep = Eigen::Matrix<double,7,1>::Zero();

      first_step_ = 1;
      cmd_flag_ = 0;
      step_ = 0;
    
      
  }

  void PotentialFieldControl::update(const ros::Time& time, const ros::Duration& period)
  {
      // get joint positions
      for(unsigned int i=0; i < joint_handles_.size(); i++) 
      {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
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
        id_solver_->JntToMass(joint_msr_states_.q, M_);
        id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
        id_solver_->JntToGravity(joint_msr_states_.q, G_);
        G_.data.setZero();

        // computing the inverse of M_ now, since it will be used often
        pseudo_inverse(M_.data,M_inv_,false); //M_inv_ = M_.data.inverse(); 

        // computing Jacobian J(q)
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);
        // std::cout<<"jac_ok: "<<J_.data<<std::endl;

        // computing the distance from the mid points of the joint ranges as objective function to be minimized
        phi_ = task_objective_function(joint_msr_states_.q);

        // using the first step to compute jacobian of the tasks
        if (first_step_)
        {
          J_last_ = J_;
          phi_last_ = phi_;
          first_step_ = 0;
          return;
        }

        // computing the derivative of Jacobian J_dot(q) through numerical differentiation
        J_dot_.data = (J_.data - J_last_.data)/period.toSec();

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q,x_);

        //other method use to calculate the position and jacobian for the repulsive field 
        // int j=0;
        // for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        // {
        for(unsigned int i=0; i<kdl_chain_.getNrOfSegments()+1;i++)  
        {
          // std::cout<<"name segment n: "<<j<<std::endl;
          // std::cout<<"_ " <<it->getName()<<std::endl;
          KDL::Frame x_test;     
          fk_pos_solver_->JntToCart(joint_msr_states_.q,x_test,i);
          x_chain.push_back(x_test);  //x_chain[1-7 + 14];
          KDL::Jacobian jac_repulsive;
          jac_repulsive = KDL::Jacobian(7);
          jnt_to_jac_solver_->JntToJac (joint_msr_states_.q,jac_repulsive , i);
          // std::cout<<"jac_repulsive: "<<jac_repulsive.data<<std::endl;
          JAC_repulsive.push_back(jac_repulsive);
          // j++;
        } 

       // if (Equal(x_,x_des_,0.05))
       // {
           // ROS_INFO("On target");

          // if(hand_step == 0)
          // {
          //   trajectory_msgs::JointTrajectory msg_jointT_hand;
          //   msg_jointT_hand.joint_names.resize(1);
          //   msg_jointT_hand.points.resize(1);
          //   msg_jointT_hand.points[0].positions.resize(1);
          //   msg_jointT_hand.joint_names[0] = desired_hand_name;
          //   msg_jointT_hand.points[0].positions[0] = 1.0;
          //   msg_jointT_hand.points[0].time_from_start = ros::Duration(2); // 2s;
          //   hand_publisher_.publish(msg_jointT_hand);
          //   hand_step =1;
          // }
          // std::cout<<"x_err_: "<<x_err_<<std::endl;
            // error_pose_trajectory.start_controller = 1;
            tf::poseKDLToMsg (x_, error_pose_trajectory.pose_hand);
            tf::poseKDLToMsg (x_des_, error_pose_trajectory.pose_desired);
            error_pose_trajectory.ObjOrObst = ObjOrObst;       
            pub_error_.publish(error_pose_trajectory);

            // hand_step = 1;
          // Force_repulsive = Eigen::Matrix<double,7,1>::Zero();       
        // }


        // computing end-effector position/orientation error w.r.t. desired frame
        x_err_ = diff(x_,x_des_);


        x_dot_ = J_.data*joint_msr_states_.qdot.data; 

        // setting error reference
        for(int i = 0; i < Force_attractive.size(); i++)
        {
          // e = x_des_dotdot + Kd*(x_des_dot - x_dot) + Kp*(x_des - x)
          Force_attractive(i) =  -Kd_(i)*(x_dot_(i)) + V_max_kuka*Kp_(i)*x_err_(i);
        }

        // computing b = J*M^-1*(c+g) - J_dot*q_dot
        b_ = J_.data*M_inv_*(C_.data + G_.data) - J_dot_.data*joint_msr_states_.qdot.data;

        // computing omega = J*M^-1*N^T*J
        omega_ = J_.data*M_inv_*N_trans_*J_.data.transpose();

        // computing lambda = omega^-1
        pseudo_inverse(omega_,lambda_);
       
        if(Object_position.size() > 0)
        {
          Force_repulsive = GetRepulsiveForce(x_chain);    
        }
            

        F_Rep_table = RepulsiveWithTable(x_chain);
        Force_total_rep = Force_repulsive + F_Rep_table;

        // computing nullspace
        N_trans_ = N_trans_ - J_.data.transpose()*lambda_*J_.data*M_inv_;           

        // finally, computing the torque tau
        tau_.data = (J_.data.transpose()*lambda_*(Force_attractive + b_)) + Force_total_rep + N_trans_*(Eigen::Matrix<double,7,1>::Identity(7,1)*(phi_ - phi_last_)/(period.toSec()));
        // std::cout<<" tau_.data: "<< tau_.data<<std::endl;
        // saving J_ and phi of the last iteration
        J_last_ = J_;
        phi_last_ = phi_;

      // set controls for joints
      for (unsigned int i = 0; i < joint_handles_.size(); i++)
      {
        joint_handles_[i].setCommand(tau_(i));
      }
      
      x_chain.clear();
      Object_radius.clear();
      Object_height.clear();
      Object_position.clear();
      JAC_repulsive.clear();


    }

    else
    {
      for (unsigned int i = 0; i < joint_handles_.size(); i++)
      {
           joint_handles_[i].setCommand(PIDs_[i].computeCommand(joint_des_states_.q(i) - joint_msr_states_.q(i),period));
      }
    }
      ros::spinOnce();

  }

  void PotentialFieldControl::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
  { 
    KDL::Frame frame_des_;
    tf::poseMsgToKDL(msg->pose, frame_des_);
    x_des_ = frame_des_;
    cmd_flag_ = 1;
    ObjOrObst = true;
    // std::cout<<"arrivato msg"<<std::endl;
  }

  void PotentialFieldControl::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
  {
      //get info for calculates objects surface
      for(unsigned int i=0; i < msg->geometries.size(); i++)
      {
        KDL::Frame frame_obj;
        Object_radius.push_back(msg->geometries[i].info[0]);  //radius
        Object_height.push_back(msg->geometries[i].info[1]);  //height

        tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
        Object_position.push_back(frame_obj); 
      } 
    cmd_flag_ = 1;
  }

  void PotentialFieldControl::InfoOBj( const desperate_housewife::fittedGeometriesSingle::ConstPtr& obj_rem)
  {
    KDL::Frame frame_des_;
    tf::poseMsgToKDL(obj_rem->pose, frame_des_);
    error_pose_trajectory.WhichArm = obj_rem->info[obj_rem->info.size() - 1]; //last element is whicharm
    x_des_ = frame_des_;
    ObjOrObst = false;
    cmd_flag_=1;

  }



  // void PotentialFieldControl::set_marker(KDL::Frame x, int id)
  // {     
  //       msg_marker_.header.frame_id = "world";
  //       msg_marker_.header.stamp = ros::Time();
  //       msg_marker_.ns = "end_effector";
  //       msg_marker_.id = id;
  //       msg_marker_.type = visualization_msgs::Marker::SPHERE;
  //       msg_marker_.action = visualization_msgs::Marker::ADD;
  //       msg_marker_.pose.position.x = x.p(0);
  //       msg_marker_.pose.position.y = x.p(1);
  //       msg_marker_.pose.position.z = x.p(2);
  //       msg_marker_.pose.orientation.x = 0.0;
  //       msg_marker_.pose.orientation.y = 0.0;
  //       msg_marker_.pose.orientation.z = 0.0;
  //       msg_marker_.pose.orientation.w = 1.0;
  //       msg_marker_.scale.x = 0.005;
  //       msg_marker_.scale.y = 0.005;
  //       msg_marker_.scale.z = 0.005;
  //       msg_marker_.color.a = 1.0;
  //       msg_marker_.color.r = 0.0;
  //       msg_marker_.color.g = 1.0;
  //       msg_marker_.color.b = 0.0;  
  // }

  double PotentialFieldControl::task_objective_function(KDL::JntArray q)
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
  }


  Eigen::Matrix<double,7,1> PotentialFieldControl::GetRepulsiveForce(std::vector<KDL::Frame> &Pos_chain)
  {
      // for the obstacles avoidance we consired only segments 6,8,14 (14 is the softhand)
      std::vector<double> distance_local_obj;
      Eigen::Matrix<double,7,1> F_rep_tot = Eigen::Matrix<double,7,1>::Zero();
      std::vector<Eigen::Matrix<double,7,1> > F_rep; 
      std::vector<double>  min_d;
      std::vector<unsigned int> index_infl;
      int index_dist = 0;
      double influence = 0.30;

      //F_rep_total = SUM(F_rep_each_ostacles)
      // std::cout<<"Object_position.size(): "<<Object_position.size()<<std::endl;
      // for(unsigned int i=0; i < Object_position.size();i++)
      // {
      //   distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[4].p)).Norm() );
      //   distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[5].p)).Norm() );
      //   distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[6].p)).Norm() );
         distance_local_obj.push_back( (diff(Object_position[0].p,Pos_chain[7].p)).Norm() );
        distance_local_obj.push_back( (diff(Object_position[0].p,Pos_chain[14].p)).Norm() );
      
      // double distance = diff(Object_position[0].p,Pos_chain[14].p).Norm();

      // std::cout<<"distance_local_obj.size(): "<<distance_local_obj.size()<<std::endl;
        for(unsigned int i=0; i < distance_local_obj.size(); i++ )
        {
          if( distance_local_obj[i] <= influence )
          // if( distance <= influence )
          {
            min_d.push_back(distance_local_obj[0]);
            index_infl.push_back(i); //for tracking wich object is closet
            // std::cout<<"index_infl: "<<index_infl[i]<<std::endl; 
            // std::cout<<"i: "<<i<<std::endl; 
          }
          else
          {
            continue;
          }
   
        }
               
        //ROS_INFO("finito for e la dimensione è di: %g", min_d.size());
        // std::cout<<"min_d.size(): "<<min_d.size()<<std::endl;
        if(min_d.size() != 0)
        {
          
          double min_distance = min_d[0];
          Eigen::Vector3d distance_der_partial(0,0,0);
          Eigen::Vector3d vec_Temp(0,0,0);
              
          for (unsigned int i=0; i < min_d.size(); i++)
          {
            if(min_distance > min_d[i])
            {
              min_distance = min_d[i];
              index_dist = i;
              
            }

            else
            {
              continue;
            }
          }

          int index_obj = floor(index_infl[index_dist]/2) ;
          // std::cout<<"index_obj: "<<index_obj<<std::endl;
          int index_jac = index_infl[index_dist] % 2;
          // std::cout<<"index_jac: "<<index_jac<<std::endl;


          Eigen::Matrix<double,6,1> Force = Eigen::Matrix<double,6,1>::Zero();
           // distance_der_partial = x^2/radius + y^2 / radius + 2*(z^2n) /l
    
          distance_der_partial[0] = (Object_position[index_obj].p.x()*2 / Object_radius[index_obj] );
          distance_der_partial[1] = (Object_position[index_obj].p.y()*2 / Object_radius[index_obj] );
          // distance_der_partial[2] = (pow(Object_position[index_obj].p.z(),7)*16 / Object_height[index_obj] ); //n=2
           distance_der_partial[2] = (Object_position[index_obj].p.z()*4 / Object_height[index_obj] ); //n=2
          

          double Ni_ = .01;
          
          // vec_Temp = (Ni_/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial;
          // std::cout<<"qui"<<std::endl;       
          Force(0) = (Ni_/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial[0];
          Force(1) = (Ni_/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial[1];
          Force(2) = (Ni_/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial[2];
          Force(3) = 0;
          Force(4) = 0;
          Force(5) = 0;

          switch(index_jac) //T= J_transpose * lambda*repulsive_force
          {

            // case 0: 
            //       F_rep.push_back(JAC_repulsive[4].data.transpose()* lambda_ * Force);
            //       break;
            // case 1:
            //       F_rep.push_back(JAC_repulsive[6].data.transpose()* lambda_  * Force);
            //       break;
            // case 2:
            //       F_rep.push_back(JAC_repulsive[8].data.transpose()* lambda_  * Force);
            //       break;
            case 0:
                  F_rep.push_back(JAC_repulsive[7].data.transpose()* lambda_  * Force);
                  break;
            case 1:
                   F_rep.push_back(JAC_repulsive[14].data.transpose()* lambda_  * Force);
                  break;
          }

           // std::string obstacle_frame;
        //cylinder_frame = 
        // wrench_msg_rep.header.stamp = ros::Time::now();
        // wrench_msg_rep.header.frame_id = object_names_.c_str();
        // wrench_msg_rep.wrench.force.x = Force[0];
        // wrench_msg_rep.wrench.force.y = Force[1];
        // wrench_msg_rep.wrench.force.z = Force[2];
        // publisher_wrench_command_rep.publish(wrench_msg_rep);



        }
        // else
        // {
        //   continue;
        // }
      // }
    else
    {
      F_rep.push_back(Eigen::Matrix<double,7,1>::Zero());
    }
      // std::cout<<"F_rep.size: "<<F_rep.size()<<std::endl;
    // std::cout<<"f_rep: "<<F_rep[0]<<std::endl;
    // std::cout<<"Force_attractive: "<<Force_attractive<<std::endl;
      // for(unsigned int j=0; j < F_rep.size(); j++)
      // {
      //   F_rep_tot = F_rep_tot + F_rep[j];
      // }
      // std::cout<<"F_rep_tot: "<<F_rep_tot<<std::endl;
    

     return F_rep[0];
       

  }


Eigen::Matrix<double,7,1> PotentialFieldControl::RepulsiveWithTable(std::vector<KDL::Frame> &Pos_arm)
{
    // ROS_INFO("dentro_set_repulsive");
    // for the obstacles avoidance we consired only segments 6,8,14 (14 is the softhand)
    KDL::Vector Table_position(0,0,0.15);
    double Rep_inf_table = 0.20;
    Eigen::Matrix<double,6,1> Force = Eigen::Matrix<double,6,1>::Zero();
    std::vector<double> distance_local_obj;
    std::vector<double> distance_influence;
    Eigen::Vector3d vec_Temp(0,0,0);
    std::vector<int> index_infl;
    int index_dist = 0;
    Eigen::Matrix<double,7,1> F_rep = Eigen::Matrix<double,7,1>::Zero();

     distance_local_obj.push_back(- Table_position.z()+ Pos_arm[4].p.z());
    distance_local_obj.push_back(- Table_position.z()+ Pos_arm[5].p.z());
    distance_local_obj.push_back(- Table_position.z()+ Pos_arm[6].p.z());
    distance_local_obj.push_back(- Table_position.z()+ Pos_arm[7].p.z());
    distance_local_obj.push_back(- Table_position.z()+ Pos_arm[14].p.z());

    for(unsigned int i= 0; i< distance_local_obj.size(); i++)
    {
      if(distance_local_obj[i] < Rep_inf_table )
      {
        distance_influence.push_back(distance_local_obj[i] );
        index_infl.push_back(i);

      }
      else
      {
        continue;
      }
    }
    
    //std::cout<<"distance_influence.size(): "<<distance_influence.size()<<std::endl;
    
    if(distance_influence.size() != 0)
    {
      double min_distance = distance_influence[0];
      for (unsigned int i=0; i< distance_influence.size();i++)
      {         
        if(min_distance > distance_influence[i])
        {
          min_distance = distance_influence[i];
          index_dist = i ;
        }
        else
        {
          continue;
        }
      }
            
      double Ni_ = 200;
      Eigen::Vector3d distance_der_partial(0,0,1);
      
      int index_jacobian = index_infl[index_dist] % 5; 
      // std::cout<<"index_jacobian: "<<index_jacobian<<std::endl;
      
      vec_Temp = (Ni_/pow(min_distance,2)) * (1/min_distance - 1/Rep_inf_table) * distance_der_partial;

        Force.row(0) << vec_Temp[0];
        Force.row(1) << vec_Temp[1];
        Force.row(2) << vec_Temp[2];
        Force.row(3) <<  0;
        Force.row(4) <<  0;
        Force.row(5) <<  0;

      // std::cout<<"index_jacobian: "<<index_jacobian<<std::endl;
      switch(index_jacobian) //T= J_transpose * lambda*repulsive_force
        {

          case 0: 
                F_rep = JAC_repulsive[4].data.transpose()* lambda_ * Force;
                // std::cout<<" JAC_repulsive[6].data"<< JAC_repulsive[6].data<<std::endl;
                break;
          case 1:
                F_rep = JAC_repulsive[5].data.transpose()* lambda_  * Force;
                 // std::cout<<" JAC_repulsive[8].data"<< JAC_repulsive[8].data<<std::endl;
                break;
          case 2:
                 F_rep = JAC_repulsive[6].data.transpose()* lambda_  * Force;
                  // std::cout<<" JAC_repulsive[11].data"<< JAC_repulsive[11].data<<std::endl;
                break;
          case 3:
                 F_rep = JAC_repulsive[7].data.transpose()* lambda_  * Force;
                  // std::cout<<" JAC_repulsive[14].data"<< JAC_repulsive[14].data<<std::endl;
                break;
          case 4:
                 F_rep = JAC_repulsive[14].data.transpose()* lambda_  * Force;
                  // std::cout<<" JAC_repulsive[14].data"<< JAC_repulsive[14].data<<std::endl;
                break;
        }

    }

     return F_rep;


}






void PotentialFieldControl::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    if(msg->data.size() == 2*joint_handles_.size())
    {
      for(unsigned int i = 0; i < joint_handles_.size(); i++)
      {
        Kp_(i) = msg->data[i];
        Kd_(i) = msg->data[i + joint_handles_.size()];
      }
    }
    else
      ROS_INFO("Number of Joint handles = %lu", joint_handles_.size());

    ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());


  }



















}

PLUGINLIB_EXPORT_CLASS(desperate_housewife::PotentialFieldControl, controller_interface::ControllerBase)
