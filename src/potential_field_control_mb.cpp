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

// #define  treshold_influence  1

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
    n.param<double>("percentage",percentage,0.5);
    std::string root_name, tip_name;
    nh_.getParam("root_name", root_name);
    nh_.getParam("tip_name", tip_name);

    bool use_real;
    n.param<bool>("use_real",use_real, false);


//resize the vector that we use for calculates the dynamic      
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
    Ki_.resize(kdl_chain_.getNrOfJoints());
    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());
    tau_prev_.resize(kdl_chain_.getNrOfJoints());
    J_last_.resize(kdl_chain_.getNrOfJoints());

//Set the number of link that we used. 14 is the soft-hand

    XmlRpc::XmlRpcValue my_list;
    n.getParam("links_with_potential_field", my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for ( int i = 0; i < my_list.size(); ++i) 
    {
      ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      list_of_links_pf.push_back(static_cast<std::string>(my_list[i]).c_str());
// sum += static_cast<string>(my_list[i]);
    }


    for (unsigned int i = 0; i < list_of_links_pf.size(); ++i)
    {
      KDL::Chain chain_tmp;

      if(!kdl_tree_.getChain(root_name.c_str(), list_of_links_pf[i].c_str(), chain_tmp))
      {
        ROS_ERROR("Error processing chain from %s to %s ", root_name.c_str(), list_of_links_pf[i].c_str());
      }
      else
      {
        list_of_chains_pf.push_back(chain_tmp);
        list_of_fk_pf.push_back(KDL::ChainFkSolverPos_recursive(chain_tmp));
        ROS_DEBUG("Chain from %s to %s correctly initialized", root_name.c_str(), list_of_links_pf[i].c_str());
      }
    }

    list_of_link.push_back(4);
    list_of_link.push_back(5);
    list_of_link.push_back(6);
    list_of_link.push_back(7);
    list_of_link.push_back(kdl_chain_.getNrOfSegments());

//set the gains
    n.param<double>("Kp_0",Kp_(0),50);
    n.param<double>("Kp_1",Kp_(1),50);
    n.param<double>("Kp_2",Kp_(2),50);
    n.param<double>("Kp_3",Kp_(3),5);
    n.param<double>("Kp_4",Kp_(4),5);
    n.param<double>("Kp_5",Kp_(5),5);

    n.param<double>("Kd_0",Kd_(0),0.5);
    n.param<double>("Kd_1",Kd_(1),0.5);
    n.param<double>("Kd_2",Kd_(2),0.5);
    n.param<double>("Kd_3",Kd_(3),0.5);
    n.param<double>("Kd_4",Kd_(4),0.5);
    n.param<double>("Kd_5",Kd_(5),0.5);

    n.param<double>("Ki_0",Ki_(0),0.01);
    n.param<double>("Ki_1",Ki_(1),0.01);
    n.param<double>("Ki_2",Ki_(2),0.01);
    n.param<double>("Ki_3",Ki_(3),0.01);
    n.param<double>("Ki_4",Ki_(4),0.01);
    n.param<double>("Ki_5",Ki_(5),0.01);

    n.param<double>("repulsive_gains",Ni_ ,1);
    n.param<double>("repulsive_treshold", treshold_influence ,1);
    n.param<double>("Repulsive_table", Repulsive_table ,1);


    ROS_DEBUG("Subscribed for desired_hand_topic to: %s", desired_reference_topic.c_str());
//list of obstacles
    ROS_INFO("Subscribed for obstacle_avoidance_topic to : %s", obstacle_avoidance.c_str());

    obstacles_subscribe_ = n.subscribe(obstacle_avoidance.c_str(), 1, &PotentialFieldControl::InfoGeometry, this);

//callcback for setting the gains at real time
    sub_gains_ = nh_.subscribe(set_gains_.c_str(), 1, &PotentialFieldControl::set_gains, this);

// sub_force_point_ = nh_.subscribe(point_.c_str(), 1, &PotentialFieldControl::GetForce, this);

    pub_error_ = nh_.advertise<desperate_housewife::Error_msg>("error", 1000);
    pub_tau_ = nh_.advertise<std_msgs::Float64MultiArray>("tau_commad", 1000);
    pub_Fa_ = nh_.advertise<std_msgs::Float64MultiArray>("Factrative_commad", 1000);
    pub_Fr_ = nh_.advertise<std_msgs::Float64MultiArray>("Frepulsive_commad", 1000);
    pub_f_total_ = nh_.advertise<std_msgs::Float64MultiArray>("Ftotal_commad", 1000);
    pub_velocity_ = nh_.advertise<std_msgs::Float64MultiArray>("velocity", 1000);
    pub_error_int_ = nh_.advertise<std_msgs::Float64MultiArray>("error_interpolate", 1000);

    sub_command_ = n.subscribe(desired_reference_topic.c_str(), 1, &PotentialFieldControl::command, this); 
    sub_command_start = n.subscribe("start_control", 1, &PotentialFieldControl::command_start, this);

        wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>("wrench_repulsive_table", 512);

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
    tau_repulsive = Eigen::Matrix<double,7,1>::Zero();
    F_Rep_table = Eigen::Matrix<double,7,1>::Zero();
    Force_total_rep = Eigen::Matrix<double,7,1>::Zero();
    Force_repulsive_prev = Eigen::Matrix<double,7,1>::Zero();
    Force_attractive_last = Eigen::Matrix<double,6,1>::Zero();
    Force_total_rep_last = Eigen::Matrix<double,7,1>::Zero();

    Force_repulsive_last = Eigen::Matrix<double,6,1>::Zero();

    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_des_);

    first_step_ = 1;
    error_pose_trajectory.arrived = 0;  
// switch_trajectory = false;
    SetToZero(tau_prev_);
    SetToZero(x_err_integral);
  }

  void PotentialFieldControl::update(const ros::Time& time, const ros::Duration& period)
  {   
    std_msgs::Float64MultiArray tau_msg;
    std_msgs::Float64MultiArray qdot_msg;
    std_msgs::Float64MultiArray vel_msg;
    std_msgs::Float64MultiArray Fr_msg;
    std_msgs::Float64MultiArray Fa_msg;
    std_msgs::Float64MultiArray err_msg;
    std_msgs::Float64MultiArray Ft_msg;
// get joint positions
    for(unsigned int i=0; i < joint_handles_.size(); i++) 
    {
      joint_msr_states_.q(i) = joint_handles_[i].getPosition();
      joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
      vel_msg.data.push_back(joint_handles_[i].getVelocity());     
    }
    pub_velocity_.publish(vel_msg);
    N_trans_ = I_;  
    SetToZero(tau_);

//flag to use this code with real robot
    if (start_flag)
    {
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
//error total
        KDL::Twist x_err_int;  

        x_err_int = diff(x_, x_des_int);
        tf::twistKDLToMsg (x_err_int,  error_pose_trajectory.error_);
        time_inter_jerk = time_inter;
        time_inter = time_inter + period.toSec();

// SO3 Time update
        Time = interpolatormb(time_inter, T_des);
      }

      x_dot_ = J_.data*joint_msr_states_.qdot.data; 
      x_err_ = diff(x_,x_des_);

//just for publish the error interpolate


      for(unsigned int i=0; i<6; i++)
      {
        err_msg.data.push_back(x_err_(i));
      }
      pub_error_int_.publish(err_msg);

//msgs for desperate mind
//to decide the pose of the object to be removed
      tf::poseKDLToMsg (x_, error_pose_trajectory.pose_hand);         
      pub_error_.publish(error_pose_trajectory);

//set the limitation of velocity
      KDL::Vector V_err_(Kp_(0)/Kd_(0)*x_err_.vel.data[0],Kp_(1)/Kd_(1)*x_err_.vel.data[1],Kp_(2)/Kd_(2)*x_err_.vel.data[2]);
      double v_limited = VelocityLimit(V_err_);

//calculate the attractive filed like PID control
      for(int i = 0; i < Force_attractive.size(); i++)
      {
        x_err_integral(i) += x_err_(i)*period.toSec();
        Force_attractive(i) =  -Kd_(i)*(x_dot_(i)) + v_limited*Kp_(i)*x_err_(i) + Ki_(i)*x_err_integral(i);
      }

      Eigen::Matrix<double, 6,1> F_total = Eigen::Matrix<double, 6, 1>::Zero();
      F_total = Force_attractive;

      bool enable_obstacle_avoidance;
      nh_.param<bool>("enable_obstacle_avoidance",enable_obstacle_avoidance ,true);

      Eigen::Matrix<double, 6,1> F_repulsive_end_efector = Eigen::Matrix<double, 6, 1>::Zero();

      if (enable_obstacle_avoidance)
      {
        Eigen::Matrix<double, 6,1> F_obj_base = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6,1> F_table_base = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6,1> F_table = Eigen::Matrix<double, 6, 1>::Zero();


        for (unsigned int i = 0; i < list_of_links_pf.size(); ++i)
        {
// ROS_INFO("number of links of chain %d are: %d", i, list_of_chains_pf[i].getNrOfJoints());
// ROS_INFO("******%d joints in chain******", chain_tmp.getNrOfJoints());
          KDL::JntArray joint_states_chain(list_of_chains_pf[i].getNrOfJoints());
          for (unsigned int j = 0; j < list_of_chains_pf[i].getNrOfJoints(); ++j)
          {
            joint_states_chain(j) = joint_msr_states_.q(j);
          }
          KDL::Frame fk_chain;
          list_of_fk_pf[i].JntToCart(joint_states_chain,fk_chain);
          Eigen::Matrix<double,6,6> Adjoint;
// std::cout << fk_chain << std::endl;
          Adjoint = getAdjointT( fk_chain );
          Eigen::Matrix<double, 6,1> F_obj = Eigen::Matrix<double, 6, 1>::Zero();

          for (unsigned int k = 0; k < Object_position.size(); ++k)
          {

            double influence_local = Object_radius[i] +  treshold_influence;
            F_obj +=  (1.0/(list_of_links_pf.size()*Object_position.size()))*GetRepulsiveForce(fk_chain, influence_local, Object_position[k], Object_radius[k], Object_height[k] );

          }

          F_obj_base += Adjoint*F_obj;

          F_table += GetRepulsiveForceTable(fk_chain, Repulsive_table );
          F_table_base += (1.0/list_of_links_pf.size())*Adjoint*F_table;

          // std::cout <<"Force repulsive from the table in local frame" << std::endl;
          // std::cout << F_table << std::endl << std::endl;

        }
// Eigen::Matrix<double, 6,1> F_repulsive_end_efector = getAdjointT( x_.Inverse() ) * (F_obj_base + F_table_base);

        F_repulsive_end_efector = getAdjointT( x_.Inverse() ) * (F_table_base);
        // std::cout <<"Force repulsive from the table in end_efector frame" << std::endl;
        // std::cout << F_repulsive_end_efector << std::endl << std::endl;
        F_total += F_repulsive_end_efector;

        geometry_msgs::WrenchStamped total_repulsive_wrench_end_efector;
        total_repulsive_wrench_end_efector.header.frame_id = tip_name.c_str();
        total_repulsive_wrench_end_efector.header.stamp = ros::Time::now();
        total_repulsive_wrench_end_efector.wrench.force.x = F_repulsive_end_efector(0);
        total_repulsive_wrench_end_efector.wrench.force.y = F_repulsive_end_efector(1);
        total_repulsive_wrench_end_efector.wrench.force.z = F_repulsive_end_efector(2);
        total_repulsive_wrench_end_efector.wrench.torque.x = F_repulsive_end_efector(3);
        total_repulsive_wrench_end_efector.wrench.torque.y = F_repulsive_end_efector(4);
        total_repulsive_wrench_end_efector.wrench.torque.z = F_repulsive_end_efector(5);
        wrench_pub.publish(total_repulsive_wrench_end_efector);

      }



// Eigen::Matrix<double, 6,1> F_repulsive_end_efector = getAdjointT( x_.Inverse() ) * (0.0*F_obj_base + (1.0/list_of_links_pf.size())*F_table_base);

/*********************************************/
      for(unsigned int t=0; t < F_total.size(); t++ )
      {
        Fr_msg.data.push_back(F_repulsive_end_efector(t));
        Fa_msg.data.push_back(Force_attractive(t));
        Ft_msg.data.push_back(F_total(t));
      }

      pub_Fr_.publish(Fr_msg);
      pub_Fa_.publish(Fa_msg);
      pub_f_total_.publish(Ft_msg);

// computing b = J*M^-1*(c+g) - J_dot*q_dot
      b_ = J_.data*M_inv_*(C_.data + G_.data) - J_dot_.data*joint_msr_states_.qdot.data;

// computing omega = J*M^-1*N^T*J
      omega_ = J_.data*M_inv_*N_trans_*J_.data.transpose();

      JacobiSVD<MatrixXd>::SingularValuesType sing_vals_2;
// computing lambda = omega^-1
      pseudo_inverse(omega_,lambda_,sing_vals_2);


// computing nullspace
      N_trans_ = N_trans_ - J_.data.transpose()*lambda_*J_.data*M_inv_;           

// finally, computing the torque tau
// tau_.data = (J_.data.transpose()*lambda_*(Force_attractive + F_repulsive_end_efector))+ 0.0*Force_total_rep + 0.0*N_trans_*(Eigen::Matrix<double,7,1>::Identity(7,1)*(phi_ - phi_last_)/(period.toSec()));

      tau_.data += (J_.data.transpose()*lambda_*(F_total));
      bool enable_joint_limits_avoidance;
      nh_.param<bool>("enable_joint_limits_avoidance", enable_joint_limits_avoidance ,false);

      if (enable_joint_limits_avoidance)
      {
        tau_.data += JointLimitAvoidance( joint_msr_states_.q ).data;
      }

// saving J_ and phi of the last iteration
      J_last_ = J_;
      phi_last_ = phi_;

//torque saturation
      tau_(0) = (std::abs(tau_(0)) >= 176*percentage ? std::copysign(176*percentage,tau_(0)) : tau_(0));
      tau_(1) = (std::abs(tau_(1)) >= 176*percentage ? std::copysign(176*percentage,tau_(1)) : tau_(1)); 
      tau_(2) = (std::abs(tau_(2)) >= 100*percentage ? std::copysign(100*percentage,tau_(2)): tau_(2)); 
      tau_(3) = (std::abs(tau_(3)) >= 100*percentage ? std::copysign(100*percentage,tau_(3)): tau_(3)); 
      tau_(4) = (std::abs(tau_(4)) >= 100*percentage ? std::copysign(100*percentage,tau_(4)): tau_(4)); 
      tau_(5) = (std::abs(tau_(5)) >= 38*percentage ? std::copysign(38*percentage,tau_(5)): tau_(5)); 
      tau_(6) = (std::abs(tau_(6)) >= 38*percentage ? std::copysign(38*percentage,tau_(6)): tau_(6));  
    }

// set controls for joints
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
      joint_handles_[i].setCommand(tau_(i));  
      tau_msg.data.push_back(tau_(i));
    }

    pub_tau_.publish(tau_msg);
    x_chain.clear();
    JAC_repulsive.clear();
    ros::spinOnce();

  }

  Eigen::Matrix<double,3,3> PotentialFieldControl::getVectorHat(Eigen::Matrix<double,3,1> vector_in)
  {
    Eigen::Matrix<double,3,3> vector_hat = Eigen::Matrix<double,3,3>::Zero();

    vector_hat << 0, -vector_in(2,0), vector_in(1,0),
    vector_in(2,0), 0, -vector_in(0,0),
    -vector_in(1,0), vector_in(0,0), 0;
    return vector_hat; 
  }

  Eigen::Matrix<double,6,6> PotentialFieldControl::getAdjointT( KDL::Frame Frame_in)
  {
    Eigen::Matrix<double,6,6> Adjoint_local = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,3,3> rotation_local(Frame_in.M.data);
    Eigen::Matrix<double,3,1> position_local(Frame_in.p.data);

    Adjoint_local.block<3,3>(0,0) = rotation_local.transpose();
    Adjoint_local.block<3,3>(3,3) = rotation_local.transpose();
    Adjoint_local.block<3,3>(3,0) = -rotation_local.transpose()*getVectorHat(position_local);

    return Adjoint_local;
  }

  Eigen::Matrix<double,7,1>  PotentialFieldControl::GetRepulsiveWithObstacle()
  {
    std::vector<Eigen::Matrix<double,7,1>> vect_rep;
    std::vector<KDL::Vector> point_of_interesting;
    std_msgs::Float64MultiArray Fr_msg;
    std::vector<Eigen::Matrix<double,6,1>> vect_force;

    for(unsigned int i=0; i < Object_position.size();i++)
    {
      double influence = Object_radius[i] +  treshold_influence;        
      Eigen::Matrix<double,6,1> ForceAndIndex;

      for(unsigned int k=0 ; k < list_of_link.size(); k++)
      {
        ForceAndIndex = GetRepulsiveForce(x_chain[list_of_link[k]].p, influence, Object_position[i], Object_radius[i], Object_height[i] );
        vect_rep.push_back (JAC_repulsive[list_of_link[k]].data.transpose()* lambda_ * ForceAndIndex);
        vect_force.push_back(ForceAndIndex);
      }          
    } 

    Eigen::Matrix<double,7,1> tau_repulsive = Eigen::Matrix<double,7,1>::Zero();
    Eigen::Matrix<double,6,1> Force_repulsive_pub = Eigen::Matrix<double,6,1>::Zero();      
// if there are more than one objects
    for(unsigned int j = 0; j<vect_rep.size(); j++)
    {
      tau_repulsive += vect_rep[j];
      Force_repulsive_pub += vect_force[j];
    }

    for(unsigned int t=0; t < Force_repulsive_pub.size(); t++ )
    {
      Fr_msg.data.push_back(Force_repulsive_pub(t));
    }

// pub_Fr_.publish(Fr_msg);

    return tau_repulsive;
  }



  void PotentialFieldControl::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
  { 

    KDL::Frame frame_des_;
    tf::poseMsgToKDL(msg->pose, frame_des_);

    PoseDesiredInterpolation(frame_des_);

    error_pose_trajectory.arrived = 1;

    ROS_INFO("New reference for controller");

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
// Time_traj_rep = 0;
// std::cout<<"msg->geometries.size(): "<<msg->geometries.size()<<std::endl;
//get info for calculates objects surface
    for(unsigned int i=0; i < msg->geometries.size(); i++)
    {
      KDL::Frame frame_obj;
      //radius
      Object_radius.push_back(msg->geometries[i].info[0]);  
//height
      Object_height.push_back(msg->geometries[i].info[1]);  

      tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
      Object_position.push_back(frame_obj); 
    }

    ROS_INFO("New environment: No of Obstacles %lu", msg->geometries.size());

  }


  void PotentialFieldControl::PoseDesiredInterpolation(KDL::Frame frame_des_)
  {
    if(Int == 0)
    {
      x_des_int = frame_des_;
// x_des_ = x_des_int;
      fk_pos_solver_->JntToCart(joint_msr_states_.q, x_now_int);
      Int = 1;
    //time for slerp interpolation
      Time = 0; 
      time_inter = 0;
      SetToZero(x_err_last);
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
// switch_trajectory = true; 
        x_err_last = x_err_;
        SetToZero(x_err_integral);
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

    sum /= N;

    return -sum;
  }

  KDL::JntArray PotentialFieldControl::JointLimitAvoidance(KDL::JntArray q)
  {
/* This function implements joint limit avoidance usung the penalty function V = \sum\limits_{i=1}^n\frac{1}{s^2}
s = q_{l_1}-|q_i| */

    KDL::JntArray tau_limit_avoidance(q.data.size());
    double s, potential, k = .01, treshold, q_abs;

    for (unsigned int i = 0; i < q.data.size(); i++)
    {
      treshold = 5.0*M_PI/180.0;
      q_abs = std::fabs(q.data[i]);
      s = joint_limits_.max(i) - q_abs;
      if (  s < treshold )
      {
        ROS_WARN("Joint limit %d", i);
        potential = 0.5/(s*s);
        tau_limit_avoidance.data[i] = - k * KDL::sign(q.data[i]) * potential;  
      }
      else
      {
        tau_limit_avoidance(i) = 0.0;
      }
    }

    return tau_limit_avoidance;
  }


  Eigen::Matrix<double,6,1> PotentialFieldControl::GetRepulsiveForce(KDL::Frame &T_in, double influence, KDL::Frame &Object_pos, double radius, double height)
  {
    Eigen::Matrix<double,6,1> ForceAndIndex;
    ForceAndIndex =  Eigen::Matrix<double,6,1>::Zero();

    double distance;

    distance = diff(Object_pos.p, T_in.p).Norm();

    if( distance < influence)
    {

      Eigen::Vector3d distance_der_partial = GetPartialDerivate(Object_pos, T_in.p, radius, height);
      ForceAndIndex = GetFIRAS(distance, distance_der_partial, influence);
  
    }

    Eigen::Matrix<double,6,1> force_local_link = Eigen::Matrix<double,6,1>::Zero();
    force_local_link = getAdjointT( T_in.Inverse() * Object_pos) * ForceAndIndex;

    return force_local_link; 
  }


  Eigen::Matrix<double,6,1> PotentialFieldControl::GetRepulsiveForce(KDL::Vector &point_, double influence, KDL::Frame &Object_pos, double radius, double height)
  {
    Eigen::Matrix<double,6,1> ForceAndIndex;
    ForceAndIndex =  Eigen::Matrix<double,6,1>::Zero();

    double distance;

    distance = diff(Object_pos.p, point_).Norm();

    if( distance < influence)
    {

      Eigen::Vector3d distance_der_partial = GetPartialDerivate(Object_pos, point_, radius, height);
      ForceAndIndex = GetFIRAS(distance, distance_der_partial, influence);
  
    }

    return ForceAndIndex; 
  }



  Eigen::Matrix<double,6,1> PotentialFieldControl::GetFIRAS(double &min_distance, Eigen::Vector3d &distance_der_partial , double &influence)
  {

    Eigen::Matrix<double,6,1> Force = Eigen::Matrix<double,6,1>::Zero();

    Force(0) = (Ni_/pow(min_distance,2)) * distance_der_partial[0];
    Force(1) = (Ni_/pow(min_distance,2)) * distance_der_partial[1];
    Force(2) = (Ni_/pow(min_distance,2)) * distance_der_partial[2];
    Force(3) = 0;
    Force(4) = 0;
    Force(5) = 0;


    return Force;
  }


  Eigen::Vector3d PotentialFieldControl::GetPartialDerivate(KDL::Frame &T_v_o, KDL::Vector &Point_v, double &radius, double &height)
  {
    Eigen::Matrix<double,4,4>  Tvo_eigen; 
    Tvo_eigen = FromKdlToEigen(T_v_o);
    Eigen::Vector4d Point_v_eigen(Point_v.x(),Point_v.y(),Point_v.z(),1);

    Eigen::Vector4d Point_o;
    Point_o = Tvo_eigen.inverse() * Point_v_eigen;

    Eigen::Vector4d distance_der_partial(0,0,0,0);
// distance_der_partial = x^2/radius + y^2 / radius + 2*(z^2n) /l 
    distance_der_partial[0] = (Point_o[0]*2) / radius ;
    distance_der_partial[1] = (Point_o[1]*2) / radius ;
// distance_der_partial[2] = (std::pow(Point_o[2],7)*16 / height ); //n=4
  //n=1
    distance_der_partial[2] = (Point_o[2]*4) / height ; 
    distance_der_partial[3] = 0; 

    Eigen::Vector3d Der_v;
    Eigen::Vector4d partial_temp;
    partial_temp = Tvo_eigen*distance_der_partial;
    Der_v[0] = partial_temp[0];
    Der_v[1] = partial_temp[1];
    Der_v[2] = partial_temp[2];

    return Der_v;
  }

  Eigen::Matrix<double,4,4>  FromKdlToEigen(KDL::Frame &T_v_o)
  {
    Eigen::Matrix<double,4,4>  Tvo_eigen;
    Tvo_eigen.row(0) << T_v_o.M.data[0], T_v_o.M.data[1],T_v_o.M.data[2], T_v_o.p.x();
    Tvo_eigen.row(1) << T_v_o.M.data[3], T_v_o.M.data[4],T_v_o.M.data[5], T_v_o.p.y();
    Tvo_eigen.row(2) << T_v_o.M.data[6], T_v_o.M.data[7],T_v_o.M.data[8], T_v_o.p.z();
    Tvo_eigen.row(3) << 0,0,0,1;
    return Tvo_eigen;
  }

  Eigen::Matrix<double,7,1> PotentialFieldControl::RepulsiveWithTable()
  {

// //repulsive with table
  //table position    
    KDL::Vector Table_position(0,0,0.15);       
    std::vector<double> distance_local_obj;
    double distance_;
    Eigen::Matrix<double,6,1> ForceAndIndex_2;
    std::vector<Eigen::Matrix<double,7,1>> vect_rep_table;
    Eigen::Matrix<double,7,1> tau_repulsive_table = Eigen::Matrix<double,7,1>::Zero(); 

    for(unsigned int j=0; j< list_of_link.size(); j++)
    {
   //considered only the z position
      distance_ = ( -Table_position.z() + x_chain[list_of_link[j]].p.z() );

      if(distance_ <= Repulsive_table )
      {
        Eigen::Vector3d distance_der_partial(0,0,1);  
        ForceAndIndex_2 = GetFIRAS(distance_, distance_der_partial, Repulsive_table);
        vect_rep_table.push_back(JAC_repulsive[list_of_link[j]].data.transpose()* lambda_ * ForceAndIndex_2); 
      }
    }

    for(unsigned int j = 0; j<vect_rep_table.size(); j++)
    {
      tau_repulsive_table += vect_rep_table[j];
    }

    return tau_repulsive_table;     
  }


  Eigen::Matrix<double,6,1> PotentialFieldControl::GetRepulsiveForceTable(KDL::Frame &T_in, double influence)
  {
    Eigen::Matrix<double,6,1> force_local_object = Eigen::Matrix<double,6,1>::Zero();
    double min_distance_to_table;

    KDL::Frame T_table_world;

    T_table_world.p = T_in.p;
    T_table_world.p.data[2] = 0;

    nh_.param<double>("repulsive_gains",Ni_ ,1);
    nh_.param<double>("min_distance_to_table",min_distance_to_table ,0.15);

    KDL::Vector Table_position(0,0,0.15);

    double distance_local = ( -Table_position.z() + T_in.p.z());
    Eigen::Vector3d distance_der_partial(0,0,1);

    if (distance_local <= min_distance_to_table )
    {
      force_local_object = GetFIRAS(distance_local, distance_der_partial, min_distance_to_table);
      // ROS_INFO("Repulsing with table");
    }

    Eigen::Matrix<double,6,1> force_local_link = Eigen::Matrix<double,6,1>::Zero();
    force_local_link = getAdjointT( T_in.Inverse() * T_table_world) * force_local_object;

    return force_local_link;
  }


  void PotentialFieldControl::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    if(msg->data.size() == 2*9)
    {
      for(unsigned int i = 0; i < 6; i++)
      {
        Kp_(i) = msg->data[i];
        Kd_(i) = msg->data[i + 6];
        Ki_(i) = msg->data[i + 12];
        std::cout<<"Kp:" << i << " : " << Kp_(i) << std::endl;
        std::cout<<"Kd:" << i << " : " << Kd_(i) << std::endl;
        std::cout<<"Ki:" << i << " : " << Ki_(i) << std::endl;
      }
    }
    else
      ROS_INFO("Number of Joint handles = %lu", joint_handles_.size());

    ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());


  }


//  void PotentialFieldControl::SeeMarker(KDL::Frame &Pos, std::string obst_name)
// {
//    // std::string obst_name= "pose_desired" ;
//     tf::Transform tfGeomTRansform;
//     geometry_msgs::Pose p;
//     p.position.x = Pos.p.x();
//     p.position.y = Pos.p.y();
//     p.position.z = Pos.p.z();
//     Pos.M.GetQuaternion(p.orientation.x, p.orientation.y ,p.orientation.z ,p.orientation.w);
//     // p.orientation.x = 0;
//     // p.orientation.y = 0;
//     // p.orientation.z = 0;
//     // p.orientation.w = 1; 
//     tf::poseMsgToTF( p, tfGeomTRansform );
//     // tf::poseMsgToKDL(p, frames); 
//     tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "vito_anchor", obst_name.c_str()) );
// }




  double PotentialFieldControl::VelocityLimit(KDL::Vector &x_dot_d)
  {
    double v_limited;
    Eigen::Vector3d x_dot_eigen(x_dot_d.data[0],x_dot_d.data[1],x_dot_d.data[2]);
    double temp = V_max_kuka/ std::sqrt(x_dot_eigen.transpose()*x_dot_eigen);
    v_limited = std::min(1.0, temp);

// std::cout<<"v_limited: "<<v_limited<<std::endl;
    return v_limited;
  }


}

PLUGINLIB_EXPORT_CLASS(desperate_housewife::PotentialFieldControl, controller_interface::ControllerBase)