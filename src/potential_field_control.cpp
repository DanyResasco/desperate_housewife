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
      KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
      ROS_INFO("Starting controller");
      ROS_WARN("Number of segments: %d", kdl_chain_.getNrOfSegments());
    // for swicht the hand_desired
    n.getParam("desired_reference_topic", desired_reference_topic);
    n.getParam("obstacle_remove_topic", obstacle_remove_topic);
    n.getParam("desired_hand_topic", desired_hand_topic);
    n.getParam("obstacle_avoidance", obstacle_avoidance);
    n.getParam("tip_name", tip_name);
    n.getParam("set_gains_topic", set_gains_);
    n.param<double>("time_interp_desired", T_des, 1);
    n.param<double>("percentage",percentage,0.3);
    
  
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


    ROS_INFO("Subscribed for obstacle_remove_topic to : %s", obstacle_remove_topic.c_str());
    obstacles_remove_sub = n.subscribe(obstacle_remove_topic.c_str(), 1, &PotentialFieldControl::InfoOBj, this);     

   
    pub_error_ = nh_.advertise<desperate_housewife::Error_msg>("error", 1000);
    pub_tau_ = nh_.advertise<std_msgs::Float64MultiArray>("tau_commad", 1000);
  

    sub_command_ = n.subscribe(desired_reference_topic.c_str(), 1, &PotentialFieldControl::command, this); 

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
      }

      I_ = Eigen::Matrix<double,7,7>::Identity(7,7);
      Force_attractive = Eigen::Matrix<double,6,1>::Zero();
      Force_repulsive = Eigen::Matrix<double,7,1>::Zero();
      F_Rep_table = Eigen::Matrix<double,7,1>::Zero();
      Force_total_rep = Eigen::Matrix<double,7,1>::Zero();
      fk_pos_solver_->JntToCart(joint_msr_states_.q,x_des_);

      // Kp_(0) = 50;  Kp_(1) = 50; Kp_(2) = 50;
      // Kp_(3) = 10;  Kp_(4) = 10; Kp_(5) = 10;
      // Kd_(0) = 10; Kd_(1) = 10; Kd_(2) = 10;
      // Kd_(3) = 5; Kd_(4) = 5; Kd_(5) = 5;

      Kp_(0) = 800;  Kp_(1) = 800; Kp_(2) = 800;
      Kp_(3) = 800;  Kp_(4) = 800; Kp_(5) = 800;
      Kd_(0) = 200; Kd_(1) = 200; Kd_(2) = 200;
      Kd_(3) = 200; Kd_(4) = 200; Kd_(5) = 200;

      first_step_ = 1;
      error_pose_trajectory.arrived = 0;
      ObjOrObst = 3;
      // time_inter = 0;   


      
  }

  void PotentialFieldControl::update(const ros::Time& time, const ros::Duration& period)
  {
      // get joint positions
      for(unsigned int i=0; i < joint_handles_.size(); i++) 
      {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
      }

        // resetting N and tau(t=0) for the highest priority task
        N_trans_ = I_;  
        SetToZero(tau_);

        // computing Inertia, Coriolis and Gravity matrices
        id_solver_->JntToMass(joint_msr_states_.q, M_);
        id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
        id_solver_->JntToGravity(joint_msr_states_.q, G_);
        G_.data.setZero();

        // computing the inverse of M_ now, since it will be used often
        pseudo_inverse(M_.data,M_inv_,false); 

        // computing Jacobian J(q)
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);

        // computing the distance from the mid points of the joint ranges as objective function to be minimized
        phi_ = task_objective_function(joint_msr_states_.q);

        // using the first step to compute jacobian of the tasks
        if (first_step_ == 1)
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
     
          
        

        //calculate jacobian and position keeping track of all joints
        for(unsigned int i=0; i<kdl_chain_.getNrOfSegments()+1;i++)  
        {
          KDL::Frame x_test;     
          fk_pos_solver_->JntToCart(joint_msr_states_.q,x_test,i);
          x_chain.push_back(x_test);  //x_chain[1-7 + 14];
          KDL::Jacobian jac_repulsive;
          jac_repulsive = KDL::Jacobian(7);
          jnt_to_jac_solver_->JntToJac (joint_msr_states_.q,jac_repulsive , i);
          JAC_repulsive.push_back(jac_repulsive);
        } 

       
      
        //interpolate the position
        if(error_pose_trajectory.arrived == 1)
        { 
          x_des_.p = x_now_int.p + interpolatormb(time_inter, T_des)* (x_des_int.p - x_now_int.p);

          // tfScalar Time = interpolatormb(time_inter, T_des);

          x_des_int.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);
          x_now_int.M.GetQuaternion(quat_now.v(0),quat_now.v(1),quat_now.v(2),quat_now.a);

          tf::Quaternion quat_tf_des_int(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);
          tf::Quaternion quat_tf_now_int(quat_now.v(0),quat_now.v(1),quat_now.v(2),quat_now.a);

          tf::quaternionTFToKDL((tf::slerp(quat_tf_now_int,quat_tf_des_int,Time)).normalize(),x_des_.M);
          
          KDL::Twist x_err_int;  //error

          x_err_int = diff(x_, x_des_int);
          tf::twistKDLToMsg (x_err_int,  error_pose_trajectory.error_);
         
          // x_des_.M = x_des_int.M;
          time_inter = time_inter + period.toSec();
          // Time = Time + period.toSec();
          Time = interpolatormb(time_inter, T_des);
        }


        x_dot_ = J_.data*joint_msr_states_.qdot.data; 
        
        x_err_ = diff(x_,x_des_);
            

        tf::poseKDLToMsg (x_, error_pose_trajectory.pose_hand);
        

        pub_error_.publish(error_pose_trajectory);
      
        for(int i = 0; i < Force_attractive.size(); i++)
        {
          //Force_attractive(i) =  -Kd_(i)*(x_dot_(i)) + V_max_kuka*Kp_(i)*x_err_(i);
          Force_attractive(i) =  -Kd_(i)*(x_dot_(i)) + Kp_(i)*x_err_(i);
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
        tau_.data = (J_.data.transpose()*lambda_*(Force_attractive + b_));// + Force_total_rep + N_trans_*(Eigen::Matrix<double,7,1>::Identity(7,1)*(phi_ - phi_last_)/(period.toSec()));

        // saving J_ and phi of the last iteration
        J_last_ = J_;
        phi_last_ = phi_;

        // std::cout<<"tau_.data[0]: " <<tau_.data[0]<<std::endl;
        // std::cout<<"tau_(0): " <<tau_(0)<<std::endl;


        tau_(0) = (std::abs(tau_(0)) >= 176 ? std::copysign(176*percentage,tau_(0)) : tau_(0));
        tau_(1) = (std::abs(tau_(1)) >= 176 ? std::copysign(176*percentage,tau_(1)) : tau_(1)); 
        tau_(2) = (std::abs(tau_(2)) >= 100 ? std::copysign(100*percentage,tau_(2)): tau_(2)); 
        tau_(3) = (std::abs(tau_(3)) >= 100 ? std::copysign(100*percentage,tau_(3)): tau_(3)); 
        tau_(4) = (std::abs(tau_(4)) >= 100 ? std::copysign(100*percentage,tau_(4)): tau_(4)); 
        tau_(5) = (std::abs(tau_(5)) >= 38 ? std::copysign(38*percentage,tau_(5)): tau_(5)); 
        tau_(6) = (std::abs(tau_(6)) >= 38 ? std::copysign(38*percentage,tau_(6)): tau_(6));  

        
 
      // set controls for joints
      for (unsigned int i = 0; i < joint_handles_.size(); i++)
      {


        joint_handles_[i].setCommand(tau_(i));
        // if (i==6)
        // {
        //   tau_(i) = 1.0;
        //   joint_handles_[i].setCommand(tau_(i));
        // }

        tau_msg.data.push_back(tau_(i));

      }
        
      pub_tau_.publish(tau_msg);
      x_chain.clear();
      Object_radius.clear();
      Object_height.clear();
      Object_position.clear();
      JAC_repulsive.clear();
      tau_msg.data.clear();

      ros::spinOnce();

  }

  void PotentialFieldControl::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
  { 

    KDL::Frame frame_des_;
    tf::poseMsgToKDL(msg->pose, frame_des_);
    // x_des_ = frame_des_;
    PoseDesiredInterpolation(frame_des_);

    error_pose_trajectory.arrived = 1;
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
       
    // ObjOrObst = 1;
    // erro_arr = 1;
    // err_obj = 1;
    // err_home = 0;

  }

  void PotentialFieldControl::InfoOBj( const desperate_housewife::fittedGeometriesSingle::ConstPtr& obj_rem)
  {
    KDL::Frame frame_des_;
    tf::poseMsgToKDL(obj_rem->pose, frame_des_);
    // error_pose_trajectory.WhichArm = obj_rem->info[obj_rem->info.size() - 1]; //last element is whicharm
    // x_des_ = frame_des_;
    PoseDesiredInterpolation(frame_des_);
     error_pose_trajectory.arrived = 1;
    // ObjOrObst = 2;
    // erro_arr = 1;
    // err_obj = 1;
    // err_home = 0;

  }

  void PotentialFieldControl::PoseDesiredInterpolation(KDL::Frame frame_des_)
  {
    if(Int == 0)
    {
      x_des_int = frame_des_;
      x_des_ = x_des_int;
      fk_pos_solver_->JntToCart(joint_msr_states_.q, x_now_int);
      Int = 1;
      Time = 0;
      time_inter = 0;
    }
    else
    {
      if(!Equal(frame_des_, x_des_int,0.05))
      {
        // Int = 0;
        x_des_int = frame_des_;
        // x_des_ = x_des_int;
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_now_int);
        time_inter = 0;
        Time = 0;
        std::cout<<"aggiorno tempi"<<std::endl;
        std::cout<<"time_inter: "<<time_inter<<std::endl;
        std::cout<<"Time: "<<Time<<std::endl;
        a = 1;
      }
    }

  }

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
          

          double Ni_ = 1;
          
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
