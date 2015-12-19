#include <potential_field_control.h>
// #include <grid.hpp>
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

#define  treshold_influence  0.20

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
      n.getParam("point", point_);
      n.param<double>("time_interp_desired", T_des, 1);
      n.param<double>("percentage",percentage,0.5);
      

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

      //Set the number of link that we used

      list_of_link.push_back(4);
      list_of_link.push_back(5);
      list_of_link.push_back(6);
      list_of_link.push_back(7);
      list_of_link.push_back(14);


      ROS_DEBUG("Subscribed for desired_hand_topic to: %s", desired_reference_topic.c_str());
       //list of obstacles
      ROS_INFO("Subscribed for obstacle_avoidance_topic to : %s", obstacle_avoidance.c_str());

      obstacles_subscribe_ = n.subscribe(obstacle_avoidance.c_str(), 1, &PotentialFieldControl::InfoGeometry, this);

      //Hand_pose for graspable objects
      sub_gains_ = nh_.subscribe(set_gains_.c_str(), 1, &PotentialFieldControl::set_gains, this);

      // sub_force_point_ = nh_.subscribe(point_.c_str(), 1, &PotentialFieldControl::GetForce, this);

      pub_error_ = nh_.advertise<desperate_housewife::Error_msg>("error", 1000);
      pub_tau_ = nh_.advertise<std_msgs::Float64MultiArray>("tau_commad", 1000);
      
      //pub_Fa_ = nh_.advertise<std_msgs::Float64MultiArray>("Factrative_commad", 1000);

      sub_command_ = n.subscribe(desired_reference_topic.c_str(), 1, &PotentialFieldControl::command, this); 
      sub_command_start = n.subscribe("start_control", 1, &PotentialFieldControl::command_start, this);
      //vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
      // service = n.advertiseService("grid", GetInfoObject);
      
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
      }

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

          x_err_ = diff(x_,x_des_);

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

            if(Time_traj == 1)
            {
              switch_trajectory = false;
            }
          }

          // pub_Fa_.publish(Fa_msg);
          // computing b = J*M^-1*(c+g) - J_dot*q_dot
          b_ = J_.data*M_inv_*(C_.data + G_.data) - J_dot_.data*joint_msr_states_.qdot.data;

          // computing omega = J*M^-1*N^T*J
          omega_ = J_.data*M_inv_*N_trans_*J_.data.transpose();
          JacobiSVD<MatrixXd>::SingularValuesType sing_vals_2;
          // computing lambda = omega^-1
          pseudo_inverse(omega_,lambda_,sing_vals_2);
          
          if(Object_position.size() > 0)
          {
            std::vector<Eigen::Matrix<double,7,1>> vect_rep;
            
            for(unsigned int i=0; i < Object_position.size();i++)
            {
                std::vector<double> DistanceAndIndex;
                double influence = Object_radius[i] +  treshold_influence;
                std::vector<double> distance_local_obj;

                for(unsigned int j=0; j< list_of_link.size(); j++)
                {
                    distance_local_obj.push_back( (diff(Object_position[i].p, x_chain[list_of_link[j]].p)).Norm() );
                }
                std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex;
                ForceAndIndex = GetRepulsiveForce(distance_local_obj, influence, i);
                vect_rep.push_back (JAC_repulsive[ForceAndIndex.second].data.transpose()* lambda_ * ForceAndIndex.first);
                //Force_repulsive =  JAC_repulsive[ForceAndIndex.second].data.transpose()* lambda_ * ForceAndIndex.first;
            }
            
            Force_repulsive = Eigen::Matrix<double,7,1>::Zero();
            
            for(unsigned int j = 0; j<vect_rep.size(); j++)
            {
              Force_repulsive += vect_rep[j];
            }

          }
          //repulsive with table
          KDL::Vector Table_position(0,0,0.15);           
          std::vector<double> distance_local_obj;
          for(unsigned int j=0; j< list_of_link.size(); j++)
          {
            distance_local_obj.push_back( -Table_position.z() + x_chain[list_of_link[j]].p.z());
          }

          std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex_table;
          ForceAndIndex_table = RepulsiveWithTable(distance_local_obj);   
          F_Rep_table = JAC_repulsive[ForceAndIndex_table.second].data.transpose()* lambda_ * ForceAndIndex_table.first;
          
          Force_total_rep = Force_repulsive + F_Rep_table;

          // computing nullspace
          N_trans_ = N_trans_ - J_.data.transpose()*lambda_*J_.data*M_inv_;           

          // finally, computing the torque tau
          tau_.data = (J_.data.transpose()*lambda_*(Force_attractive )) + Force_total_rep + N_trans_*(Eigen::Matrix<double,7,1>::Identity(7,1)*(phi_ - phi_last_)/(period.toSec()));
          //Fa * b??

          // saving J_ and phi of the last iteration
          J_last_ = J_;
          phi_last_ = phi_;

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
      // std::cout<<"msg->geometries.size(): "<<msg->geometries.size()<<std::endl;
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

  // void PotentialFieldControl::GetForce(const std_msgs::Float64MultiArray::ConstPtr &msg )
  // {
  //     KDL::Vector point_pos(msg->data[0],msg->data[1],msg->data[2]);
  //     std::cout<<"qui"<<std::endl;
  //     std::vector<Eigen::Matrix<double,6,1> > F_rep;
  //     std::vector<double> distance_local_obj;
  //     for(unsigned int i=0; i< Object_position.size(); i++)
  //     {  
  //       std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex;
  //       distance_local_obj.push_back( (diff(Object_position[i].p, point_pos)).Norm() );

  //       double influence = Object_radius[i] + 0.2;

  //       ForceAndIndex = GetRepulsiveForce(distance_local_obj, influence, i);

  //       F_rep.push_back(ForceAndIndex.first);
  //     }
      
  //     Eigen::Matrix<double,6,1> f = Eigen::Matrix<double,6,1>::Zero(); 
      
  //     for(unsigned int k=0; k < F_rep.size();k++)
  //     {
  //       f = f + F_rep[k];
  //     }

  //     //repulsive with table
  //     std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex_table;
  //     KDL::Vector Table_position(0,0,0.15);  
  //     std::vector<double> dist;
  //     dist.push_back(- Table_position.z() + point_pos.z() );
  //     ForceAndIndex_table = RepulsiveWithTable(dist);

  //     Eigen::Matrix<double,6,1> Force_tot_grid;
  //     Force_tot_grid = f + ForceAndIndex_table.first;
  //     KDL::Vector force_vect(Force_tot_grid(0), Force_tot_grid(1),Force_tot_grid(2));
  //     KDL::Vector null(0,0,0);
  //     if(!Equal(force_vect,null,0.05))
  //     {
  //       DrawArrow(force_vect, point_pos); 
  //     }
  //     else
  //       std::cout<<"F ris nulla"<<std::endl;

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


  // std::vector<double> PotentialFieldControl::GetMinDistance(std::vector<KDL::Frame> &Pos_chain, KDL::Vector &Object_position, double influence )
  std::vector<double> PotentialFieldControl::GetMinDistance(std::vector<double> distance_local_obj,  double influence )
  {
      std::vector<double> DistanceAndIndex;
      std::vector<double>::iterator result = std::min_element(std::begin(distance_local_obj), std::end(distance_local_obj));
      int index_dist = std::distance(std::begin(distance_local_obj), result); 
 
      if(distance_local_obj[index_dist] <= influence)
      {
          DistanceAndIndex.push_back(1);
          DistanceAndIndex.push_back( distance_local_obj[index_dist] );
          DistanceAndIndex.push_back( list_of_link[index_dist] );
          std::cout<<"distance_local_obj: "<< distance_local_obj[index_dist]<<std::endl;
          std::cout<<"list_of_link: "<< list_of_link[index_dist]<<std::endl;
      }
      else
        DistanceAndIndex.push_back(0);
      
      return DistanceAndIndex;

  }

  std::pair<Eigen::Matrix<double,6,1>, double> PotentialFieldControl::GetRepulsiveForce(std::vector<double> distance_local_obj, double influence, int inde_obj)
  {
      std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex;
      ForceAndIndex.first =  Eigen::Matrix<double,6,1>::Zero();  
      std::vector<double> DistanceAndIndex;
      DistanceAndIndex = GetMinDistance(distance_local_obj, influence);
       std::cout<<"fuori da GetMinDistance"<<std::endl;
      if(DistanceAndIndex[0] == 1 )
      {
        std::cout<<"dentro"<<std::endl;
          Eigen::Vector3d distance_der_partial = GetPartialDerivate(Object_position[inde_obj].p, Object_radius[inde_obj], Object_height[inde_obj]);
          ForceAndIndex.first = GetFIRAS(DistanceAndIndex[1], distance_der_partial, influence);
          std::cout<<"ForceAndIndex.first: "<<ForceAndIndex.first<<std::endl;
          ForceAndIndex.second = DistanceAndIndex[2];              
      }
   
      return ForceAndIndex; 
  }


  Eigen::Matrix<double,6,1> PotentialFieldControl::GetFIRAS(double &min_distance, Eigen::Vector3d &distance_der_partial , double &influence)
  {
      // std::cout<<"dentro GetFIRAS"<<std::endl;
      double Ni_ = 1.0;
      Eigen::Matrix<double,6,1> Force = Eigen::Matrix<double,6,1>::Zero();
                 
      Force(0) = (Ni_/pow(min_distance,2)) * (1.0/min_distance - 1.0/influence) * distance_der_partial[0];
      Force(1) = (Ni_/pow(min_distance,2)) * (1.0/min_distance - 1.0/influence) * distance_der_partial[1];
      Force(2) = (Ni_/pow(min_distance,2)) * (1.0/min_distance - 1.0/influence) * distance_der_partial[2];
      Force(3) = 0;
      Force(4) = 0;
      Force(5) = 0;

      return Force;

  }





  Eigen::Vector3d PotentialFieldControl::GetPartialDerivate(KDL::Vector &Object_pos, double &radius, double &height)
  {
      
      Eigen::Vector3d distance_der_partial(0,0,0);
      // distance_der_partial = x^2/radius + y^2 / radius + 2*(z^2n) /l 
      distance_der_partial[0] = (Object_pos.x()*2 / radius );
      distance_der_partial[1] = (Object_pos.y()*2 / radius );
      distance_der_partial[2] = (Object_pos.z()*4 / height ); //n=1

      return distance_der_partial;
  }



  
  std::pair<Eigen::Matrix<double,6,1>, double> PotentialFieldControl::RepulsiveWithTable(std::vector<double> distance_local_obj)
  {
    
      double Rep_inf_table = 0.10;
      std::vector<double> DistanceAndIndex;
      DistanceAndIndex = GetMinDistance(distance_local_obj, Rep_inf_table );
      std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex;
      ForceAndIndex.first = Eigen::Matrix<double,6,1>::Zero();
      
      if(DistanceAndIndex[0] == 1 )
      {
          Eigen::Vector3d distance_der_partial(0,0,1);
          ForceAndIndex.first = GetFIRAS(DistanceAndIndex[1], distance_der_partial, Rep_inf_table); 
          ForceAndIndex.second = DistanceAndIndex[2];
      }

      std::cout<<"table: "<<ForceAndIndex.first<<std::endl;

      return ForceAndIndex; 
     
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


  // std::pair<std::vector<KDL::Frame>, std::vector< Eigen::Matrix<double,1,2>>> 
  // bool PotentialFieldControl::GetInfoObject(desperate_housewife::potential_field_control::Request &req, desperate_housewife::potential_field_control::Response &res) 
  // {
  //   std::pair<std::vector<KDL::Frame>, std::vector< Eigen::Matrix<double,1,2>>> Info;
  //   std::cout<<"Object_position.size(): "<<Object_position.size()<<std::endl;

  //   req.pos = Object_position;
  //   req.radius = Object_radius;
  //   req.height = Object_height;

  //   return true;


  //   // for(unsigned int k=0; k < Object_position.size(); k++)
  //   // {
  //   //   Eigen::Matrix<double,1,2> vect_ = Eigen::Matrix<double,1,2>::Zero() ;
  //   //   vect_<< Object_radius[k], Object_radius[k];
  //   //   //vect_.[0,1] << Object_height[k]; 
  //   //   Info.first.push_back(Object_position[k]);
  //   //   Info.second.push_back(vect_);
  //   // }

  //   // return Info;
  // }

  // void PotentialFieldControl::DrawArrow( KDL::Vector &gridspace_Force, KDL::Vector &gridspace_point )
  // {
  //   uint32_t shape = visualization_msgs::Marker::ARROW;
  //   visualization_msgs::Marker marker;
  //   // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  //   marker.header.frame_id = "/my_frame";
  //   marker.header.stamp = ros::Time::now();
  //   marker.ns = "basic_shapes";
  //   marker.id = 0;
  //   marker.type = shape;
  //   marker.action = visualization_msgs::Marker::ADD;
  //   marker.pose.position.x = gridspace_point.x();
  //   marker.pose.position.y = gridspace_point.y();
  //   marker.pose.position.z = gridspace_point.z();

  //   Eigen::Quaterniond quat;
  //   quat =  RotationMarker(gridspace_Force, gridspace_point);
    
  //   marker.pose.orientation.x = quat.x();
  //   marker.pose.orientation.y = quat.y();
  //   marker.pose.orientation.z = quat.z();
  //   marker.pose.orientation.w = quat.w();
    
  //   marker.scale.x = gridspace_Force.x() / 10;
  //   marker.scale.y = gridspace_Force.y() / 10;
  //   marker.scale.z = gridspace_Force.z() / 10;
  //   marker.color.r = 0.0f;
  //   marker.color.g = 1.0f;
  //   marker.color.b = 0.0f;
  //   marker.color.a = 1.0;
  //   marker.lifetime = ros::Duration();
  //   vis_pub.publish( marker ); 
     

  // }

  // Eigen::Quaterniond RotationMarker(KDL::Vector &ris_Force, KDL::Vector &point)
  // {
  //   Eigen::Vector3d  x(1,0,0);
  //   Eigen::Vector3d Force_eigen(ris_Force.x(),ris_Force.y(),ris_Force.z());
  //   double angle = std::acos((x.dot(Force_eigen))/(x.norm()*Force_eigen.norm()) );
  //   Eigen::Vector3d axis(0,0,0);
  //   axis = (x.cross(Force_eigen)) / (x.cross(Force_eigen)).norm();
  //   Eigen::Matrix3d transformation_ = Eigen::Matrix3d::Identity();
  //   Eigen::Vector3d row0(axis[0]*axis[0]*(1-cos(angle))+cos(angle), axis[0]*axis[1]*(1-cos(angle))-axis[2]*sin(angle), axis[0]*axis[2]*(1-cos(angle))+axis[1]*sin(angle));
  //   Eigen::Vector3d row1(axis[0]*axis[1]*(1-cos(angle))-axis[2]*sin(angle), axis[1]*axis[1]*(1-cos(angle))+cos(angle), axis[1]*axis[2]*(1-cos(angle))-axis[0]*sin(angle));
  //   Eigen::Vector3d row2(axis[0]*axis[2]*(1-cos(angle))-axis[1]*sin(angle), axis[1]*axis[2]*(1-cos(angle))+axis[0]*sin(angle),axis[2]*axis[2]*(1-cos(angle))+cos(angle));
  //   transformation_.row(0) << row0.transpose();
  //   transformation_.row(1) << row1.transpose();
  //   transformation_.row(2) << row2.transpose();
  //   Eigen::Quaterniond quat_eigen_hand(transformation_);

  //  return quat_eigen_hand.normalized();

  // }









}

PLUGINLIB_EXPORT_CLASS(desperate_housewife::PotentialFieldControl, controller_interface::ControllerBase)