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
      tau_prev_.resize(kdl_chain_.getNrOfJoints());

      J_last_.resize(kdl_chain_.getNrOfJoints());

      ROS_DEBUG("Subscribed for desired_hand_topic to: %s", desired_reference_topic.c_str());
       //list of obstacles
      ROS_INFO("Subscribed for obstacle_avoidance_topic to : %s", obstacle_avoidance.c_str());

      obstacles_subscribe_ = n.subscribe(obstacle_avoidance.c_str(), 1, &PotentialFieldControl::InfoGeometry, this);

      //Hand_pose for graspable objects
      sub_gains_ = nh_.subscribe(set_gains_.c_str(), 1, &PotentialFieldControl::set_gains, this);


      // ROS_INFO("Subscribed for obstacle_remove_topic to : %s", obstacle_remove_topic.c_str());
      // obstacles_remove_sub = n.subscribe(obstacle_remove_topic.c_str(), 1, &PotentialFieldControl::InfoOBj, this);     

     
      pub_error_ = nh_.advertise<desperate_housewife::Error_msg>("error", 1000);
      pub_tau_ = nh_.advertise<std_msgs::Float64MultiArray>("tau_commad", 1000);
      // pub_qdot_ = nh_.advertise<std_msgs::Float64MultiArray>("qdot_commad", 1000);
      pub_Fa_ = nh_.advertise<std_msgs::Float64MultiArray>("Factrative_commad", 1000);
      // pub_sing_val = nh_.advertise<std_msgs::Float64MultiArray>("sing_vals_commad", 1000);
      // pub_Freptavolo_ = nh_.advertise<std_msgs::Float64MultiArray>("Freptavolo_commad", 1000);
      // pub_diff  = nh_.advertise<std_msgs::Float64MultiArray>("Diff_commad", 1000);
      // pub_xdot = nh_.advertise<std_msgs::Float64MultiArray>("xdot_commad", 1000);
      sub_command_ = n.subscribe(desired_reference_topic.c_str(), 1, &PotentialFieldControl::command, this); 
      sub_command_start = n.subscribe("start_control", 1, &PotentialFieldControl::command_start, this); 

      start_flag = false;

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
      Force_repulsive_prev = Eigen::Matrix<double,7,1>::Zero();
      fk_pos_solver_->JntToCart(joint_msr_states_.q,x_des_);
      //gains in cartesian space
      // Kp_(0) = 80;  Kp_(1) = 80; Kp_(2) = 80;
      // Kp_(3) = 15;  Kp_(4) = 15; Kp_(5) = 15;
      // Kd_(0) = 1; Kd_(1) = 1; Kd_(2) = 1;
      // Kd_(3) = 1; Kd_(4) = 1; Kd_(5) = 1;

      Kp_(0) = 1000;  Kp_(1) = 1000; Kp_(2) = 1000;
      Kp_(3) = 1000;  Kp_(4) = 1000; Kp_(5) = 1000;
      Kd_(0) = 200; Kd_(1) = 200; Kd_(2) = 200;
      Kd_(3) = 200; Kd_(4) = 200; Kd_(5) = 200;

      first_step_ = 1;
      error_pose_trajectory.arrived = 0;  
      switch_trajectory = false;
      SetToZero(tau_prev_);
 }

  void PotentialFieldControl::update(const ros::Time& time, const ros::Duration& period)
  {   
      std_msgs::Float64MultiArray tau_msg;
      std_msgs::Float64MultiArray qdot_msg;
      // get joint positions
      for(unsigned int i=0; i < joint_handles_.size(); i++) 
      {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        // qdot_msg.data.push_back(joint_handles_[i].getVelocity());
      }
        // pub_qdot_.publish(qdot_msg);
        // resetting N and tau(t=0) for the highest priority task
        N_trans_ = I_;  
        SetToZero(tau_);

           
        //flag to use this code with real robot
        // if (start_flag)
        // {
        // computing Inertia, Coriolis and Gravity matrices
          id_solver_->JntToMass(joint_msr_states_.q, M_);
          id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
          id_solver_->JntToGravity(joint_msr_states_.q, G_);
          G_.data.setZero();

          JacobiSVD<MatrixXd>::SingularValuesType sing_vals_;
          // computing the inverse of M_ now, since it will be used often
          pseudo_inverse(M_.data,M_inv_,sing_vals_,false);
         

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
          //interpolate the position and rotation
          if(error_pose_trajectory.arrived == 1)
          { 
            x_des_.p = x_now_int.p + interpolatormb(time_inter, T_des)* (x_des_int.p - x_now_int.p);

            x_des_int.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);
            x_now_int.M.GetQuaternion(quat_now.v(0),quat_now.v(1),quat_now.v(2),quat_now.a);

            tf::Quaternion quat_tf_des_int(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);
            tf::Quaternion quat_tf_now_int(quat_now.v(0),quat_now.v(1),quat_now.v(2),quat_now.a);
            quat_tf = (tf::slerp(quat_tf_now_int,quat_tf_des_int,Time)).normalize();
            tf::quaternionTFToKDL(quat_tf,x_des_.M);
            
            KDL::Twist x_err_int;  //error

            x_err_int = diff(x_, x_des_int);
            tf::twistKDLToMsg (x_err_int,  error_pose_trajectory.error_);

            time_inter = time_inter + period.toSec();

            // SO3 Time 
            Time = interpolatormb(time_inter, T_des);
          }


          x_dot_ = J_.data*joint_msr_states_.qdot.data; 
          // std_msgs::Float64MultiArray x_dot_msg;
                   
          // for(int i=0; i<6; i++)
          // {
          //   x_dot_msg.data.push_back(x_dot_(i));
          // }
          // pub_xdot.publish(x_dot_msg);

          // std_msgs::Float64MultiArray xerr_msg;
          x_err_ = diff(x_,x_des_);
          
          // for(int i=0; i<6; i++)
          // {
          //   xerr_msg.data.push_back(x_err_(i));
          // }
          // pub_diff.publish(xerr_msg);

          //to decide the pose of the object to be removed
          tf::poseKDLToMsg (x_, error_pose_trajectory.pose_hand);
          

          pub_error_.publish(error_pose_trajectory);
          std_msgs::Float64MultiArray Fa_msg;
          for(int i = 0; i < Force_attractive.size(); i++)
          {
            // Force_attractive(i) =  -Kd_(i)*(x_dot_(i)) + V_max_kuka*Kp_(i)*x_err_(i);
            Force_attractive(i) =  -Kd_(i)*(x_dot_(i)) + Kp_(i)*x_err_(i);


            
            Fa_msg.data.push_back(Force_attractive(i));
          }
          //jerk trajectory
          if(switch_trajectory == true)
          {
            Time_traj = interpolatormb(time_inter, 2);
            Force_attractive = Force_attractive_last + (Force_attractive - Force_attractive_last) *(10*pow(Time_traj,3) - 15*pow(Time_traj,4) + 6*pow(Time_traj,5));
            // Force_attractive_last = Force_attractive;
            if(Time_traj == 1)
            {
              switch_trajectory = false;
            }
          }
            // std::cout<<"Force_attractive: "<<Force_attractive<<std::endl;

          // std::cout<<"Force_attractive: "<<Force_attractive<<std::endl;
          pub_Fa_.publish(Fa_msg);
          // computing b = J*M^-1*(c+g) - J_dot*q_dot
          b_ = J_.data*M_inv_*(C_.data + G_.data) - J_dot_.data*joint_msr_states_.qdot.data;

          // computing omega = J*M^-1*N^T*J
          omega_ = J_.data*M_inv_*N_trans_*J_.data.transpose();
          JacobiSVD<MatrixXd>::SingularValuesType sing_vals_2;
          // computing lambda = omega^-1
          pseudo_inverse(omega_,lambda_,sing_vals_2);
          //   std_msgs::Float64MultiArray sing_vals_msg;
          // for (int i = 0; i < sing_vals_2.size(); i++)
          //   sing_vals_msg.data.push_back(sing_vals_2(i));
          // pub_sing_val.publish(sing_vals_msg);

         
          if(Object_position.size() > 0)
          {
            Force_repulsive = GetRepulsiveForce(x_chain);
          }
              

          F_Rep_table = RepulsiveWithTable(x_chain);
          Force_total_rep = Force_repulsive + F_Rep_table;

          // computing nullspace
          N_trans_ = N_trans_ - J_.data.transpose()*lambda_*J_.data*M_inv_;           

          // finally, computing the torque tau
          tau_.data = (J_.data.transpose()*lambda_*(Force_attractive )) + Force_total_rep + N_trans_*(Eigen::Matrix<double,7,1>::Identity(7,1)*(phi_ - phi_last_)/(period.toSec()));
          //Fa * b??

          // saving J_ and phi of the last iteration
          J_last_ = J_;
          phi_last_ = phi_;

          // std::cout<<"tau_.data[0]: " <<tau_.data[0]<<std::endl;
          // std::cout<<"tau_(0): " <<tau_(0)<<std::endl;
          for (unsigned int j = 0; j < joint_handles_.size(); j++)
          {
              
              tau_(j) = filters::exponentialSmoothing(tau_(j), tau_prev_(j), 0.2);
              tau_prev_(j) = tau_(j);
              
          }
        
          //CREA PROBLEMI IN SIMULAZIONE --> PROVARE SE È QUESTO CHE MI DA FASTIDIO IN REALE --> percentage 0.3 non va bene 
          tau_(0) = (std::abs(tau_(0)) >= 176*percentage ? std::copysign(176*percentage,tau_(0)) : tau_(0));
          tau_(1) = (std::abs(tau_(1)) >= 176*percentage ? std::copysign(176*percentage,tau_(1)) : tau_(1)); 
          tau_(2) = (std::abs(tau_(2)) >= 100*percentage ? std::copysign(100*percentage,tau_(2)): tau_(2)); 
          tau_(3) = (std::abs(tau_(3)) >= 100*percentage ? std::copysign(100*percentage,tau_(3)): tau_(3)); 
          tau_(4) = (std::abs(tau_(4)) >= 100*percentage ? std::copysign(100*percentage,tau_(4)): tau_(4)); 
          tau_(5) = (std::abs(tau_(5)) >= 38*percentage ? std::copysign(38*percentage,tau_(5)): tau_(5)); 
          tau_(6) = (std::abs(tau_(6)) >= 38*percentage ? std::copysign(38*percentage,tau_(6)): tau_(6));  
      // }
 
      // set controls for joints
      for (unsigned int i = 0; i < joint_handles_.size(); i++)
      {
        joint_handles_[i].setCommand(tau_(i));  
        // std::cout<<"tau_(" << i << "): " << tau_(i)<<std::endl;      
        tau_msg.data.push_back(tau_(i));
      }
        
      pub_tau_.publish(tau_msg);
      x_chain.clear();
      JAC_repulsive.clear();
      ros::spinOnce();

  }


  void PotentialFieldControl::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
  { 

    KDL::Frame frame_des_;
    tf::poseMsgToKDL(msg->pose, frame_des_);
   
    PoseDesiredInterpolation(frame_des_);

    error_pose_trajectory.arrived = 1;
  
  }


  void PotentialFieldControl::command_start(const std_msgs::Bool::ConstPtr& msg)
  { 
     start_flag = true;
  }

  void PotentialFieldControl::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
  {
      Object_radius.clear();
      Object_height.clear();
      Object_position.clear();
      Time_traj_rep = 0;
      
      //get info for calculates objects surface
      for(unsigned int i=0; i < msg->geometries.size(); i++)
      {
        KDL::Frame frame_obj;
        Object_radius.push_back(msg->geometries[i].info[0]);  //radius
        Object_height.push_back(msg->geometries[i].info[1]);  //height

        tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
        Object_position.push_back(frame_obj); 
      }
  }

  // void PotentialFieldControl::InfoOBj( const desperate_housewife::fittedGeometriesSingle::ConstPtr& obj_rem)
  // {
  //   KDL::Frame frame_des_;
  //   tf::poseMsgToKDL(obj_rem->pose, frame_des_);
  //   PoseDesiredInterpolation(frame_des_);
  //    error_pose_trajectory.arrived = 1;


  // }

  void PotentialFieldControl::PoseDesiredInterpolation(KDL::Frame frame_des_)
  {
    if(Int == 0)
    {
      x_des_int = frame_des_;
      // x_des_ = x_des_int;
      fk_pos_solver_->JntToCart(joint_msr_states_.q, x_now_int);
      Int = 1;
      Time = 0;
      time_inter = 0;
    }
    else
    {
      //new pose
      if(!Equal(frame_des_, x_des_int,0.05))
      {
        // update desired frame;
        Force_attractive_last = Force_attractive; 
        x_des_int = frame_des_;
        // update robot position
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_now_int);
        //time update
        time_inter = 0;
        Time = 0;   
        switch_trajectory = true; 
        Time_traj_rep = 0;
      
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
      // obstacles avoidance  ( pos_chain 14 is the softhand)
      std::vector<double> distance_local_obj;
      Eigen::Matrix<double,7,1> F_rep_tot = Eigen::Matrix<double,7,1>::Zero();
      std::vector<Eigen::Matrix<double,7,1> > F_rep;      
      int index_dist;
      
      //F_rep_total = SUM(F_rep_each_ostacles)
      // std::cout<<"Object_position.size(): "<<Object_position.size()<<std::endl;
      for(unsigned int i=0; i < Object_position.size();i++)
      {
        std::vector<double>  min_d;
        std::vector<unsigned int> index_infl;
        double influence = Object_radius[i] +  0.20;
        // std::cout<<"influence: "<<influence<<std::endl;
         distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[4].p)).Norm() );
         distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[5].p)).Norm() );
         distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[6].p)).Norm() );
         distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[7].p)).Norm() );
         distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[14].p)).Norm() );

        // std::cout<<"distance_local_obj.size(): "<<distance_local_obj.size()<<std::endl;
        for(unsigned int j=0; j< distance_local_obj.size(); j++ )
        {
          if( distance_local_obj[j] <= influence )
          {
            min_d.push_back(distance_local_obj[j]);
            index_infl.push_back(j);
          }
        }

        if(min_d.size() > 0)
        {

          std::vector<double>::iterator result = std::min_element(std::begin(min_d), std::end(min_d));
          index_dist = std::distance(std::begin(min_d), result);
          double min_distance;
          min_distance = min_d[index_dist];

            //double min_distance = distance_local_obj[j];
            // std::cout<<"min_distance: "<<min_distance<<std::endl;
             
            Eigen::Vector3d distance_der_partial(0,0,0);
            Eigen::Vector3d vec_Temp(0,0,0);
              
            int index_obj = i; //floor(index_infl[index_dist]/2) ;
            // std::cout<<"index_obj: "<<index_obj<<std::endl;
            int index_jac = index_infl[index_dist] % 5; 
            // index_infl[index_dist] % 5;
            // std::cout<<"index_jac: "<<index_jac<<std::endl;
            Eigen::Matrix<double,6,1> Force = Eigen::Matrix<double,6,1>::Zero();
             // distance_der_partial = x^2/radius + y^2 / radius + 2*(z^2n) /l
      
            distance_der_partial[0] = (Object_position[index_obj].p.x()*2 / Object_radius[index_obj] );
            // std::cout<<"distance_der_partial[0]: "<<distance_der_partial[0]<<std::endl;
            distance_der_partial[1] = (Object_position[index_obj].p.y()*2 / Object_radius[index_obj] );
            // std::cout<<"distance_der_partial[1]: "<<distance_der_partial[1]<<std::endl;
            // distance_der_partial[2] = (pow(Object_position[index_obj].p.z(),7)*16 / Object_height[index_obj] ); //n=2
            distance_der_partial[2] = (Object_position[index_obj].p.z()*4 / Object_height[index_obj] ); //n=1
            // std::cout<<"distance_der_partial[2]: "<<distance_der_partial[2]<<std::endl;

            double Ni_ = 1.0;
            
            // vec_Temp = (Ni_/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial;
            // std::cout<<"qui"<<std::endl;
                  
            Force(0) = (Ni_/pow(min_distance,2)) * (1.0/min_distance - 1.0/influence) * distance_der_partial[0];
            Force(1) = (Ni_/pow(min_distance,2)) * (1.0/min_distance - 1.0/influence) * distance_der_partial[1];
            Force(2) = (Ni_/pow(min_distance,2)) * (1.0/min_distance - 1.0/influence) * distance_der_partial[2];
            Force(3) = 0;
            Force(4) = 0;
            Force(5) = 0;
            // std::cout<<"Force: "<<Force<<std::endl;
           
            switch(index_jac) //T= J_transpose * lambda*repulsive_force
            {

              case 0: 
                    F_rep.push_back(JAC_repulsive[4].data.transpose()* lambda_ * Force);
                    break;
               case 1:
                     F_rep.push_back(JAC_repulsive[5].data.transpose()* lambda_  * Force);
                     break;
              case 2:
                    F_rep.push_back(JAC_repulsive[6].data.transpose()* lambda_  * Force);
                    break;
              case 3:
                    F_rep.push_back(JAC_repulsive[7].data.transpose()* lambda_  * Force);
                    break;
              case 4:
                     F_rep.push_back(JAC_repulsive[14].data.transpose()* lambda_  * Force);
                    break;
            }

            // std::cout<<"F_rep: "<<F_rep[j]<<std::endl;
          

            distance_local_obj.clear();

        }
          // else
          // {
          //   F_rep.push_back(Eigen::Matrix<double,7,1>::Zero());
          // }
        // }
      }
 
    for(unsigned int j=0; j < F_rep.size(); j++)
    {
         F_rep_tot = F_rep_tot + F_rep[j];
    }
    std::cout<<"F_rep_tot: "<<F_rep_tot<<std::endl;

  return F_rep_tot; 
}


  Eigen::Matrix<double,7,1> PotentialFieldControl::RepulsiveWithTable(std::vector<KDL::Frame> &Pos_arm)
  {
      // ROS_INFO("dentro_set_repulsive");
      // for the obstacles avoidance we consired only segments 6,8,14 (14 is the softhand)
      KDL::Vector Table_position(0,0,0.15);
      double Rep_inf_table = 0.10;
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
        //min element in the vector and index
        std::vector<double>::iterator result = std::min_element(std::begin(distance_influence), std::end(distance_influence));
        index_dist = std::distance(std::begin(distance_influence), result);
        double min_distance;
        min_distance = distance_influence[index_dist];
            
        double Ni_ = 0.1; //repulsive gain
        Eigen::Vector3d distance_der_partial(0,0,1);  //direction of the field
        
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

      //test with rqt plot
      // std_msgs::Float64MultiArray Freptavolo_msg;
      // for(int i=0; i<F_rep.size();i++)
      // {
      //   Freptavolo_msg.data.push_back(F_rep(i));
      // }
      // pub_Freptavolo_.publish(Freptavolo_msg);

      // std::cout<<"F rep tavolo: "<< F_rep.col(0)<<std::endl; 
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



// double DiffAndNorm(KDL::Frame Object_position, KDL::Frame &Pos_chain )
// {
//   double x,y,z;
  
//   x = (Object_position.p.x() - Pos_chain.p.x());
//   y = (Object_position.p.y() - Pos_chain.p.y());
//   z = (Object_position.p.z() - Pos_chain.p.z());
//   KDL::Vector temp_diff(x,y,z);

//   return temp_diff.Norm();
// }














}

PLUGINLIB_EXPORT_CLASS(desperate_housewife::PotentialFieldControl, controller_interface::ControllerBase)