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

#include <math.h>

namespace desperate_housewife 
{
  PotentialFieldControl::PotentialFieldControl() {}
  PotentialFieldControl::~PotentialFieldControl() {}

  bool PotentialFieldControl::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
        PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

    // for swicht the hand_desired
    if (!n.getParam("desired_reference_topic", desired_reference_topic))
    {
        ROS_ERROR_STREAM(" No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
        return false;
    }
    else
    {
      ROS_INFO("Starting controller");
      ROS_INFO("Number of segments: %d", kdl_chain_.getNrOfSegments());
 
    // ROS_INFO("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
   
    }

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

    ROS_INFO("Subscribed to: %s", desired_reference_topic.c_str());
    sub_command_ = n.subscribe(desired_reference_topic.c_str(), 1, &PotentialFieldControl::command, this);
    
    // std::string obstalces_topic_;
    // n.getParam("/PotentialFieldControl/obstacle_list", obstalces_topic_);
    //n.param<std::string>("/PotentialFieldControl/obstacle_list", obstalces_topic_, "/PotentialFieldControl/obstacle_list");
    obstacles_subscribe_ = n.subscribe("/PotentialFieldControl/obstacle_list", 1, &PotentialFieldControl::InfoGeometry, this);
    
    pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
    pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1000);

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
      Force_repulsive = Eigen::Matrix<double,6,1>::Zero();

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

      // clearing msgs before publishing
      msg_err_.data.clear();
      msg_pose_.data.clear();
      
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
        // std::cout<<"x_: "<<x_<<std::endl;

        for(unsigned int i=0; i<kdl_chain_.getNrOfSegments()+1;i++)  
        {
          KDL::Frame x_test;
          fk_pos_solver_->JntToCart(joint_msr_states_.q,x_test,i);
          x_chain.push_back(x_test);

        }
        // std::cout<<"x_prova: "<<x_chain[14]<<std::endl;
        
        // x_ = x_chain[14]; //14 is softhand
       if (Equal(x_,x_des_,0.05))
        //if (Equal(x_chain[13],x_des_,0.05))
        {
          ROS_INFO("On target");
          for(unsigned int i=0; i < joint_handles_.size(); i++) 
          {
            joint_des_states_.q(i) = joint_msr_states_.q(i);
            joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
          }
          cmd_flag_ = 0;

          
          return;         
        }

        // pushing x to the pose msg
        for (int i = 0; i < 3; i++)
          msg_pose_.data.push_back(x_.p(i));

        // setting marker parameters
        set_marker(x_,msg_id_);

        // computing end-effector position/orientation error w.r.t. desired frame
        x_err_ = diff(x_,x_des_);

        x_dot_ = J_.data*joint_msr_states_.qdot.data; 

        // setting error reference
        for(int i = 0; i < Force_attractive.size(); i++)
        {
          // e = x_des_dotdot + Kd*(x_des_dot - x_dot) + Kp*(x_des - x)
          Force_attractive(i) =  -Kd_(i)*(x_dot_(i)) + V_max_kuka*Kp_(i)*x_err_(i);
          // msg_err_.data.push_back(Force_attractive(i));
          msg_err_.data.push_back(x_err_(i));
        }

        // std::cout<<"qui crash"<<std::endl;
        //std::cout<<"Object_position.size()"<<Object_position.size()<<std::endl;
         // std::cout<<"obstacle size: "<<Object_position.size()<<std::endl;
        if(Object_position.size() > 0)
        {
          //Eigen::Matrix<double,6,1> force_rep_local = Eigen::Matrix<double,6,1>::Zero();
          // for(int i = 1; i< Object_position.size(); i++)
          // {
            Force_repulsive = GetRepulsiveForce(x_chain);
            // std::cout<<"Force_repulsive: "<<Force_repulsive[0].col(0)<<std::endl;
          // }
          // for(int p=0; p<Force_repulsive.size();p++)
          // {
          //  Force_total = Force_total + Force_repulsive[p]; 
          // }
  
        }
        

        // computing b = J*M^-1*(c+g) - J_dot*q_dot
        b_ = J_.data*M_inv_*(C_.data + G_.data) - J_dot_.data*joint_msr_states_.qdot.data;

        // computing omega = J*M^-1*N^T*J
        omega_ = J_.data*M_inv_*N_trans_*J_.data.transpose();

        // computing lambda = omega^-1
        pseudo_inverse(omega_,lambda_);
        //lambda_ = omega_.inverse();

        // computing nullspace
        N_trans_ = N_trans_ - J_.data.transpose()*lambda_*J_.data*M_inv_;           

        // Eigen::Matrix<double,6,1> Force_test;
        // Force_test = Force_attractive + Force_repulsive[0];
        // std::cout<<"Force_test "<<Force_test<<std::endl;
        // finally, computing the torque tau
        tau_.data = J_.data.transpose()*lambda_*(Force_attractive + Force_repulsive + b_);// + N_trans_*(Eigen::Matrix<double,7,1>::Identity(7,1)*(phi_ - phi_last_)/(period.toSec()));

        // saving J_ and phi of the last iteration
        J_last_ = J_;
        phi_last_ = phi_;
  
      }

      // set controls for joints
      for (unsigned int i = 0; i < joint_handles_.size(); i++)
      {
        if(cmd_flag_)
          joint_handles_[i].setCommand(tau_(i));
        else
          joint_handles_[i].setCommand(PIDs_[i].computeCommand(joint_des_states_.q(i) - joint_msr_states_.q(i),period));
      }

      // publishing markers for visualization in rviz
      pub_marker_.publish(msg_marker_);
      msg_id_++;

      // publishing error 
      pub_error_.publish(msg_err_);
      // publishing pose 
      pub_pose_.publish(msg_pose_);

      
      x_chain.clear();
      // Force_repulsive.clear();
        Object_radius.clear();
        Object_height.clear();
        Object_position.clear();
  

      ros::spinOnce();

  }

  void PotentialFieldControl::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
  { 
    KDL::Frame frame_des_;
    tf::poseMsgToKDL(msg->pose, frame_des_);
    
    x_des_ = frame_des_;
    // std::cout<<"pose desired: "<<x_des_ <<std::endl;
    cmd_flag_ = 1;
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
      // std::cout<<"obstacle size: "<<Object_position.size()<<std::endl; 
      
  }

  void PotentialFieldControl::set_marker(KDL::Frame x, int id)
  {     
        msg_marker_.header.frame_id = "world";
        msg_marker_.header.stamp = ros::Time();
        msg_marker_.ns = "end_effector";
        msg_marker_.id = id;
        msg_marker_.type = visualization_msgs::Marker::SPHERE;
        msg_marker_.action = visualization_msgs::Marker::ADD;
        msg_marker_.pose.position.x = x.p(0);
        msg_marker_.pose.position.y = x.p(1);
        msg_marker_.pose.position.z = x.p(2);
        msg_marker_.pose.orientation.x = 0.0;
        msg_marker_.pose.orientation.y = 0.0;
        msg_marker_.pose.orientation.z = 0.0;
        msg_marker_.pose.orientation.w = 1.0;
        msg_marker_.scale.x = 0.005;
        msg_marker_.scale.y = 0.005;
        msg_marker_.scale.z = 0.005;
        msg_marker_.color.a = 1.0;
        msg_marker_.color.r = 0.0;
        msg_marker_.color.g = 1.0;
        msg_marker_.color.b = 0.0;  
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


  Eigen::Matrix<double,6,1> PotentialFieldControl::GetRepulsiveForce(std::vector<KDL::Frame> &Pos_chain)
  {
      // ROS_INFO("dentro_set_repulsive");
      // for the obstacles avoidance we consired only segments 6,8,14 (14 is the softhand)
      std::vector<double> distance_local_obj;
      // double local_distance;
      Eigen::Matrix<double,6,1> Force = Eigen::Matrix<double,6,1>::Zero();
      std::vector<double>  min_d;
      std::vector<int> index_infl;
      int index_dist = 0;
      // std::vector<double> local

      //ROS_INFO("dentro repulsive");

      for(unsigned int i=0; i<Object_position.size();i++)
      {
        
        distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[6].p)).Norm() );
        distance_local_obj.push_back( (diff(Object_position[i].p,Pos_chain[8].p)).Norm() );
      }

      for(unsigned int i=0; i<distance_local_obj.size(); i++ )
      {
        if( distance_local_obj[i] <= influence )
        {
          min_d.push_back(distance_local_obj[i]);
          index_infl.push_back(i); //for tracking wich object is closet
        }
        else
        {
          continue;
        }
 
      }
      
      // distance_local_obj.push_back(diff(Object_position[U].p,Pos_chain[6].p));
      // distance_local_obj.push_back(diff(Object_position[U].p,Pos_chain[8].p));
      // distance_local_obj.push_back(diff(Object_position[U].p,Pos_chain[11].p));
     
      //ROS_INFO("finito for e la dimensione è di: %g", min_d.size());

      if(min_d.size() != 0)
      {
        
        double min_distance = min_d[0];
        //ROS_INFO("fatto init min_distance");
        Eigen::Vector3d distance_der_partial(0,0,0);
        Eigen::Vector3d vec_Temp(0,0,0);
            
        for (unsigned int i=0; i< min_d.size();i++)
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

        int index_d = index_infl[index_dist];
        //object surface (x/radius)^2 + (y/radius)^2 +(z*2/l)^2n -1 = 0   
          
        distance_der_partial[0] = (Object_position[index_d].p.x()*2 / Object_radius[index_d] );
        distance_der_partial[1] = (Object_position[index_d].p.y()*2 / Object_radius[index_d] );
        distance_der_partial[2] = (Object_position[index_d].p.z()*4 / Object_height[index_d] ); //n=1
        

        double Ni_ = 1;
        
        vec_Temp = (Ni_/pow(min_distance,2)) * (1/min_distance - 1/influence) * distance_der_partial;

        Force.row(0) << vec_Temp[0];
        Force.row(1) << vec_Temp[1];
        Force.row(2) << vec_Temp[2];
        Force.row(3) <<  0;
        Force.row(4) <<  0;
        Force.row(5) <<  0;
      }
  

     // distance_local_obj.clear();
     return Force;
       

  }





























}

PLUGINLIB_EXPORT_CLASS(desperate_housewife::PotentialFieldControl, controller_interface::ControllerBase)
