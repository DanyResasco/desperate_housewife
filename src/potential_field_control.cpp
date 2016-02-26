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
    start_controller = false;
    load_parameters(n);

    //resize the vector that we use for calculates the dynamic
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    // qdot_last_.resize(kdl_chain_.getNrOfJoints());
    tau_.resize(kdl_chain_.getNrOfJoints());
    tau_prev_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());
    J_dot_.resize(kdl_chain_.getNrOfJoints());
    J_last_.resize(kdl_chain_.getNrOfJoints());
    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());

    I_ = Eigen::Matrix<double, 7, 7>::Identity(7, 7);
    F_repulsive = Eigen::Matrix<double, 6, 1>::Zero();
    F_attractive = Eigen::Matrix<double, 6, 1>::Zero();
    F_total = Eigen::Matrix<double, 6, 1>::Zero();


    ROS_INFO("Subscribed for desired_reference to: %s", "command");
    sub_command = nh_.subscribe(topic_desired_reference.c_str(), 1, &PotentialFieldControl::command, this);
    //list of obstacles
    ROS_INFO("Subscribed for obstacle_avoidance_topic to : %s", topic_obstacle_avoidance.c_str());
    sub_obstacles = nh_.subscribe(topic_obstacle_avoidance.c_str(), 1, &PotentialFieldControl::InfoGeometry, this);

    sub_start_controller = nh_.subscribe("start_controller", 1, &PotentialFieldControl::startControllerCallBack, this);

    //callcback for setting the gains at real time

    pub_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
    pub_error_id = nh_.advertise<desperate_housewife::Error_msg>("error_id", 1000);
    pub_tau = nh_.advertise<std_msgs::Float64MultiArray>("tau_commad", 1000);
    pub_pf_attractive_force = nh_.advertise<std_msgs::Float64MultiArray>("F_attractive", 1000);
    pub_pf_repulsive_forse = nh_.advertise<std_msgs::Float64MultiArray>("F_repulsive", 1000);
    pub_pf_total_force = nh_.advertise<std_msgs::Float64MultiArray>("F_total", 1000);

    pub_total_wrench = nh_.advertise<geometry_msgs::WrenchStamped>("total_end_effector_wrench", 512);

    srv_start_controller = nh_.advertiseService("load_parameters", &PotentialFieldControl::loadParametersCallback, this);

    error_id.id = 10000;
    // error_id.id_arm = parameters_.id_arm;
    return true;
}

void PotentialFieldControl::starting(const ros::Time& time)
{
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
        joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
    }

    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);
    x_now_int = x_des_;
    x_des_int = x_des_;

    first_step_ = 1;

    SetToZero(x_err_integral);
    SetToZero(x_err_);
    SetToZero(tau_prev_);
    x_err_.vel.data[0] = 10000.0;
    start_controller = false;
}

void PotentialFieldControl::update(const ros::Time& time, const ros::Duration& period)
{

    std_msgs::Float64MultiArray tau_msg;
    std_msgs::Float64MultiArray F_repulsive_msg;
    std_msgs::Float64MultiArray F_attractive_msg;
    std_msgs::Float64MultiArray F_total_msg;
    std_msgs::Float64MultiArray err_msg;

    // get joint positions
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }
    N_trans_ = I_;
    SetToZero(tau_);
    KDL::JntArray tau_repulsive;
    tau_repulsive.resize(7);
    SetToZero(tau_repulsive);

    tf::twistKDLToMsg(x_err_, error_id.error_);

    //flag to use this code with real robot
    Eigen::MatrixXd q_instatntanea  = Eigen::MatrixXd::Zero(7,1);

    KDL::Twist x_err_msg;
    x_err_msg = x_err_;

    if (start_controller)
    {
        // computing Inertia, Coriolis and Gravity matrices
        id_solver_->JntToMass(joint_msr_states_.q, M_);
        id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
        id_solver_->JntToGravity(joint_msr_states_.q, G_);
        G_.data.setZero();

        // JacobiSVD<MatrixXd>::SingularValuesType sing_vals_;
        // computing the inverse of M_ now, since it will be used often
        pseudo_inverse(M_.data, M_inv_);

        // computing Jacobian J(q)
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

        // computing the distance from the mid points of the joint ranges as objective function to be minimized
        // phi_ = task_objective_function(joint_msr_states_.q);
        phi_ = 0;

        // using the first step to compute jacobian of the tasks
        if (first_step_ == 1)
        {
            J_last_ = J_;
            phi_last_ = phi_;
            first_step_ = 0;
            return;
        }

        // computing the derivative of Jacobian J_dot(q) through numerical differentiation
        J_dot_.data = (J_.data - J_last_.data) / period.toSec();

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        //interpolate the position and rotation
        // if(error_pose_trajectory.arrived == 1)
        // {
        if (parameters_.enable_interpolation)
        {
            x_des_.p = x_now_int.p + interpolatormb(time_inter, parameters_.max_time_interpolation) * (x_des_int.p - x_now_int.p);

            x_des_int.M.GetQuaternion(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);
            x_now_int.M.GetQuaternion(quat_now.v(0), quat_now.v(1), quat_now.v(2), quat_now.a);

            tf::Quaternion quat_tf_des_int(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);
            tf::Quaternion quat_tf_now_int(quat_now.v(0), quat_now.v(1), quat_now.v(2), quat_now.a);
            quat_tf = (tf::slerp(quat_tf_now_int, quat_tf_des_int, Time)).normalize();
            tf::quaternionTFToKDL(quat_tf, x_des_.M);
            //error total
            KDL::Twist x_err_int;

            // x_err_int = diff(x_, x_des_int);
            // tf::twistKDLToMsg (x_err_int,  error_pose_trajectory.error_);
            // time_inter_jerk = time_inter;
            time_inter = time_inter + period.toSec();

            // SO3 Time update
            Time = interpolatormb(time_inter, parameters_.max_time_interpolation);
        }
        else
        {
            x_des_ = x_des_int;
        }
        // }

        x_dot_ = J_.data * joint_msr_states_.qdot.data;
        // std::cout << x_des_ << std::endl;
        x_err_ = diff(x_, x_des_);
        // x_err_ = diff(x_,x_now_int);

        KDL::Twist x_dot_d;

        x_dot_d.vel.data[0] = parameters_.k_p(0, 0) / parameters_.k_d(0, 0) * x_err_.vel.data[0];
        x_dot_d.vel.data[1] = parameters_.k_p(1, 1) / parameters_.k_d(1, 1) * x_err_.vel.data[1];
        x_dot_d.vel.data[2] = parameters_.k_p(2, 2) / parameters_.k_d(2, 2) * x_err_.vel.data[2];
        x_dot_d.rot.data[0] = parameters_.k_p(3, 3) / parameters_.k_d(3, 3) * x_err_.rot.data[0];
        x_dot_d.rot.data[1] = parameters_.k_p(4, 4) / parameters_.k_d(4, 4) * x_err_.rot.data[1];
        x_dot_d.rot.data[2] = parameters_.k_p(5, 5) / parameters_.k_d(5, 5) * x_err_.rot.data[2];

        double v_limited = VelocityLimit(x_dot_d);

        //calculate the attractive filed like PID control
        // F_attractive = Eigen::Matrix<double,6,1>::Zero();
        x_err_integral += x_err_ * period.toSec();

        if (parameters_.enable_attractive_field)
        {
            for (int i = 0; i < F_attractive.size(); i++)
            {
                // x_err_integral += x_err_ * period.toSec();
                F_attractive(i) =  -parameters_.k_d(i, i) * ( x_dot_(i) -  v_limited * x_dot_d(i) ) + parameters_.k_i(i,i)*x_err_integral(i);
            }
        }
        // computing b = J*M^-1*(c+g) - J_dot*q_dot
        b_ = J_.data * M_inv_ * (C_.data + G_.data) - J_dot_.data * joint_msr_states_.qdot.data;

        // computing omega = J*M^-1*N^T*J
        omega_ = J_.data * M_inv_ * N_trans_ * J_.data.transpose();

        JacobiSVD<MatrixXd>::SingularValuesType sing_vals_2;
        // computing lambda = omega^-1
        pseudo_inverse(omega_, lambda_);

        N_trans_ = N_trans_ - J_.data.transpose() * lambda_ * J_.data * M_inv_;

        F_total = Eigen::Matrix<double, 6, 1>::Zero();
        F_repulsive  = Eigen::Matrix<double, 6, 1>::Zero();

        if (parameters_.enable_obstacle_avoidance)
        {
            double num_of_links_in_potential = 0.0;
            // ROS_INFO_STREAM("In obstacle ovoidance");
            Eigen::Matrix<double, 6, 1> F_obj_base_link = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::Matrix<double, 6, 1> F_table_base_link = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::Matrix<double, 6, 1> F_obj_base_total = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::Matrix<double, 6, 1> F_table_base_total = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::Matrix<double, 6, 1> F_table = Eigen::Matrix<double, 6, 1>::Zero();

            for (unsigned int i = 0; i < parameters_.pf_list_of_links.size(); ++i)
            {
                KDL::JntArray joint_states_chain(parameters_.pf_list_of_chains[i].getNrOfJoints());

                for (unsigned int j = 0; j < parameters_.pf_list_of_chains[i].getNrOfJoints(); ++j)
                {
                    joint_states_chain(j) = joint_msr_states_.q(j);
                }

                KDL::Frame fk_chain;
                parameters_.pf_list_of_fk[i].JntToCart(joint_states_chain, fk_chain);
                Eigen::Matrix<double, 6, 6> Adjoint;
                Adjoint = getAdjointT( fk_chain );
                Eigen::Matrix<double, 6, 1> F_obj = Eigen::Matrix<double, 6, 1>::Zero();

                for (unsigned int k = 0; k < Object_position.size(); ++k)
                {
                    double influence_local = Object_radius[k] +  parameters_.pf_dist_to_obstacles;
                    // ROS_INFO_STREAM("Checkin collision of link " << i << " with object " << k << "of radius " << Object_radius[k]);
                    Eigen::Matrix<double, 6, 1> F_obj_local = Eigen::Matrix<double, 6, 1>::Zero();
                    F_obj_local = GetRepulsiveForce(fk_chain, influence_local, Object_position[k], Object_radius[k], Object_height[k] );
                    F_obj = F_obj + F_obj_local;
                    if (F_obj_local.norm() != 0.0)
                    {
                        num_of_links_in_potential = num_of_links_in_potential + 1.0;
                    }
                }

                F_obj_base_link = Adjoint * F_obj;
                // F_obj_base_link = F_obj;
                F_obj_base_total += F_obj_base_link;
                // F_obj_base_link += F_obj;

                F_table = GetRepulsiveForceTable(fk_chain, parameters_.pf_dist_to_table );
                if (F_table.norm() != 0.0)
                {
                    num_of_links_in_potential = num_of_links_in_potential + 1.0;
                }
                F_table_base_link = Adjoint * F_table;
                F_table_base_total += F_table_base_link;

                tau_repulsive.data += getTauRepulsive(lambda_, J_, parameters_.pf_list_of_chains[i].getNrOfJoints(), (F_table_base_link + F_obj_base_link) );

            }
            // F_repulsive  = F_table_base_link;

            F_repulsive  = (F_table_base_total + F_obj_base_total);
            // for(unsigned int k=0; k< F_repulsive.size(); k++)
            // {
            //   std::cout<<"F_table_base_total: "<<F_table_base_total[k]<<std::endl;
            //   std::cout<<"F_obj_base_total: "<<F_obj_base_total[k]<<std::endl;
            // }
            if (num_of_links_in_potential > 1.0)
            {
                tau_repulsive.data = (1.0 / num_of_links_in_potential) * tau_repulsive.data;    
            }
            // ROS_INFO_STREAM("Num of liks with potential: " << num_of_links_in_potential);
            tau_.data += tau_repulsive.data;
            // ROS_INFO_STREAM("tau_repulsive: " << tau_.data );
        }

        F_total = (F_attractive + F_repulsive);

        Eigen::Matrix<double, 6, 1> F_to_plot  = getAdjointT( x_.Inverse() ) * (F_total);

        geometry_msgs::WrenchStamped total_repulsive_wrench_end_efector;
        total_repulsive_wrench_end_efector.header.frame_id = parameters_.tip_name.c_str();
        total_repulsive_wrench_end_efector.header.stamp = ros::Time::now();
        total_repulsive_wrench_end_efector.wrench.force.x = F_to_plot(0);
        total_repulsive_wrench_end_efector.wrench.force.y = F_to_plot(1);
        total_repulsive_wrench_end_efector.wrench.force.z = F_to_plot(2);
        total_repulsive_wrench_end_efector.wrench.torque.x = F_to_plot(3);
        total_repulsive_wrench_end_efector.wrench.torque.y = F_to_plot(4);
        total_repulsive_wrench_end_efector.wrench.torque.z = F_to_plot(5);
        pub_total_wrench.publish(total_repulsive_wrench_end_efector);

        tau_.data += (J_.data.transpose() * lambda_ * (F_attractive));

        if (parameters_.enable_joint_limits_avoidance)
        {
            tau_.data += JointLimitAvoidance( joint_msr_states_.q ).data;
        }

        if (parameters_.enable_null_space)
        {
            // tau_.data += N_trans_ * task_objective_function( joint_msr_states_.q );
            // tau_.data += N_trans_ * MaxZYDistance( joint_msr_states_.q );
            tau_.data += N_trans_ * potentialEnergy( joint_msr_states_.q );
        }

        for (unsigned int j = 0; j < joint_handles_.size(); j++)
        {
            tau_(j) = filters::exponentialSmoothing(tau_(j), tau_prev_(j), 0.4);
            tau_prev_(j) = tau_(j);
        }
        // saving J_ and phi of the last iteration
        J_last_ = J_;
        phi_last_ = phi_;

        //torque saturation
        tau_(0) = (std::abs(tau_(0)) >= 176 * parameters_.max_tau_percentage ? std::copysign(176 * parameters_.max_tau_percentage, tau_(0)) : tau_(0));
        tau_(1) = (std::abs(tau_(1)) >= 176 * parameters_.max_tau_percentage ? std::copysign(176 * parameters_.max_tau_percentage, tau_(1)) : tau_(1));
        tau_(2) = (std::abs(tau_(2)) >= 100 * parameters_.max_tau_percentage ? std::copysign(100 * parameters_.max_tau_percentage, tau_(2)) : tau_(2));
        tau_(3) = (std::abs(tau_(3)) >= 100 * parameters_.max_tau_percentage ? std::copysign(100 * parameters_.max_tau_percentage, tau_(3)) : tau_(3));
        tau_(4) = (std::abs(tau_(4)) >= 100 * parameters_.max_tau_percentage ? std::copysign(100 * parameters_.max_tau_percentage, tau_(4)) : tau_(4));
        tau_(5) = (std::abs(tau_(5)) >= 38 * parameters_.max_tau_percentage ? std::copysign(38 * parameters_.max_tau_percentage, tau_(5)) : tau_(5));
        tau_(6) = (std::abs(tau_(6)) >= 38 * parameters_.max_tau_percentage ? std::copysign(38 * parameters_.max_tau_percentage, tau_(6)) : tau_(6));

        // q_instatntanea = M_inv_ * (tau_.data - C_.data) * period.toSec();

        // ROS_INFO_STREAM("Velocity : " << q_instatntanea.transpose() * 180.0 / M_PI);

        x_err_msg = diff(x_, x_des_int);

        tf::twistKDLToMsg(x_err_msg, error_id.error_);

    }

    pub_error_id.publish( error_id );

    for (unsigned int i = 0; i < F_total.size(); i++ )
    {
        F_repulsive_msg.data.push_back(F_repulsive(i));
        F_attractive_msg.data.push_back(F_attractive(i));
        F_total_msg.data.push_back(F_total(i));
        err_msg.data.push_back(x_err_msg(i));
    }

    // set controls for joints
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(tau_(i));
        tau_msg.data.push_back(tau_(i));
    }



    pub_pf_repulsive_forse.publish(F_repulsive_msg);
    pub_pf_attractive_force.publish(F_attractive_msg);
    pub_pf_total_force.publish(F_total_msg);
    pub_error.publish(err_msg);
    pub_tau.publish(tau_msg);

    x_chain.clear();
    ros::spinOnce();
}

Eigen::Matrix<double, 3, 3> PotentialFieldControl::getVectorHat(Eigen::Matrix<double, 3, 1> vector_in)
{
    Eigen::Matrix<double, 3, 3> vector_hat = Eigen::Matrix<double, 3, 3>::Zero();

    vector_hat << 0, -vector_in(2, 0), vector_in(1, 0),
               vector_in(2, 0), 0, -vector_in(0, 0),
               -vector_in(1, 0), vector_in(0, 0), 0;
    return vector_hat;
}

Eigen::Matrix<double, 6, 6> PotentialFieldControl::getAdjointT( KDL::Frame Frame_in)
{
    Eigen::Matrix<double, 6, 6> Adjoint_local = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> rotation_local(Frame_in.M.data);
    Eigen::Matrix<double, 3, 1> position_local(Frame_in.p.data);

    Adjoint_local.block<3, 3>(0, 0) = rotation_local.transpose();
    Adjoint_local.block<3, 3>(3, 3) = rotation_local.transpose();
    Adjoint_local.block<3, 3>(3, 0) = -rotation_local.transpose() * getVectorHat(position_local);

    return Adjoint_local;
}

void PotentialFieldControl::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
{

    KDL::Frame frame_des_;
    tf::poseMsgToKDL(msg->pose, frame_des_);

    PoseDesiredInterpolation(frame_des_);

    error_id.id = msg->id;

    // ROS_INFO("New reference for controller");
}

void PotentialFieldControl::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
    Object_radius.clear();
    Object_height.clear();
    Object_position.clear();
    // Time_traj_rep = 0;
    // std::cout<<"msg->geometries.size(): "<<msg->geometries.size()<<std::endl;
    //get info for calculates objects surface
    for (unsigned int i = 0; i < msg->geometries.size(); i++)
    {
        KDL::Frame frame_obj;
        //radius
        Object_radius.push_back(msg->geometries[i].info[0]);
        // std::cout<<" Object_radius[i]: "<<Object_radius[i]<<std::endl;
        //height
        Object_height.push_back(msg->geometries[i].info[1]);

        tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
        Object_position.push_back(frame_obj);
    }

    ROS_INFO("New environment: No of Obstacles %lu", msg->geometries.size());
}

void PotentialFieldControl::PoseDesiredInterpolation(KDL::Frame frame_des_)
{
    if (Int == 0)
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
        if (!Equal(frame_des_, x_des_int, 0.05))
        {
            // update desired frame;
            // F_attractive_last = F_attractive;
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

KDL::JntArray PotentialFieldControl::JointLimitAvoidance(KDL::JntArray q)
{   // This function implements joint limit avoidance usung the penalty function V = \sum\limits_{i=1}^n\frac{1}{s^2} s = q_{l_1}-|q_i|
    KDL::JntArray tau_limit_avoidance(q.data.size());
    double s, potential, k = .01, treshold, q_abs;

    for (unsigned int i = 0; i < q.data.size(); i++)
    {
        treshold = 5.0 * M_PI / 180.0;
        q_abs = std::fabs(q.data[i]);
        s = joint_limits_.max(i) - q_abs;


        if (  s < treshold )
        {
            ROS_WARN("Joint limit %d", i);
            potential = 0.5 * std::pow((1 / s - 1 / treshold), 2);
            tau_limit_avoidance.data[i] = - k * KDL::sign(q.data[i]) * potential;
        }
        else
        {
            tau_limit_avoidance(i) = 0.0;
        }
    }

    return tau_limit_avoidance;
}


Eigen::Matrix<double, 6, 1> PotentialFieldControl::GetRepulsiveForce(KDL::Frame &T_in, double influence, KDL::Frame &Object_pos, double radius, double height)
{
    Eigen::Matrix<double, 6, 1> ForceAndIndex;
    ForceAndIndex =  Eigen::Matrix<double, 6, 1>::Zero();

    double distance;

    KDL::Vector distance_vector;
    distance_vector = T_in.p - Object_pos.p;
    distance = distance_vector.Norm();
    // std::cout<<"distance: "<<distance<<std::endl;
    // std::cout<<"influence: "<<influence<<std::endl;

    if ( distance <= influence)
    {
        // ROS_INFO_STREAM("There is a collision ");
        // ROS_INFO_STREAM("Distance: " << distance << "Influence: " << influence);
        Eigen::Vector3d distance_der_partial = GetPartialDerivate(Object_pos, T_in.p, radius, height);
        ForceAndIndex = GetFIRAS(distance, distance_der_partial, influence);
        // ROS_INFO_STREAM("Force: " << std::endl << ForceAndIndex);
    }
    // else
    // {
    // ROS_INFO_STREAM("There is not a collision ");
    // ROS_INFO_STREAM("Distance: " << distance << " Influence: " << influence);
    // }

    Eigen::Matrix<double, 6, 1> force_local_link = Eigen::Matrix<double, 6, 1>::Zero();
    force_local_link = getAdjointT( T_in.Inverse() * Object_pos) * ForceAndIndex;
    // force_local_link = getAdjointT( T_in.Inverse() ) * ForceAndIndex;

    return force_local_link;
    // return ForceAndIndex;
}

Eigen::Matrix<double, 6, 1> PotentialFieldControl::GetRepulsiveForceTable(KDL::Frame &T_in, double influence)
{
    Eigen::Matrix<double, 6, 1> force_local_object = Eigen::Matrix<double, 6, 1>::Zero();

    KDL::Frame T_table_world;

    T_table_world.p = T_in.p;
    T_table_world.p.data[2] = 0;

    KDL::Vector Table_position(0, 0, 0.0);

    double distance_local = std::abs( -Table_position.z() + T_in.p.z());

    Eigen::Vector3d distance_der_partial(0, 0, 1);

    if (distance_local <= parameters_.pf_dist_to_table )
    {
        force_local_object = GetFIRAS(distance_local, distance_der_partial, parameters_.pf_dist_to_table);
    }

    Eigen::Matrix<double, 6, 1> force_local_link = Eigen::Matrix<double, 6, 1>::Zero();
    force_local_link = getAdjointT( T_in.Inverse() * T_table_world) * force_local_object;

    return force_local_link;
}

Eigen::Matrix<double, 6, 1> PotentialFieldControl::GetFIRAS(double min_distance, Eigen::Vector3d &distance_der_partial, double influence)
{

    Eigen::Matrix<double, 6, 1> Force = Eigen::Matrix<double, 6, 1>::Zero();
    // double V = parameters_.pf_repulsive_gain/(min_distance * min_distance); // this works
    double V = parameters_.pf_repulsive_gain * ( (1.0 / min_distance) -
               (1.0 / influence) )  * (1.0 / (min_distance * min_distance));
    Force(0) = V * distance_der_partial[0];
    Force(1) = V * distance_der_partial[1];
    Force(2) = V * distance_der_partial[2];
    Force(3) = 0;
    Force(4) = 0;
    Force(5) = 0;


    return Force;
}


Eigen::Vector3d PotentialFieldControl::GetPartialDerivate(KDL::Frame &T_v_o, KDL::Vector &Point_v, double radius, double height)
{
    Eigen::Matrix<double, 4, 4>  Tvo_eigen;
    Tvo_eigen = FromKdlToEigen(T_v_o);
    Eigen::Vector4d Point_v_eigen(Point_v.x(), Point_v.y(), Point_v.z(), 1);

    Eigen::Vector4d Point_o;
    Point_o = Tvo_eigen.inverse() * Point_v_eigen;
    double n = 2; // n as in the paper should be in 4 but considering the shortest distance to obstacles. Here this is not being considered :( TODO

    Eigen::Vector4d distance_der_partial(0, 0, 0, 0);
    // distance_der_partial = x^2/radius + y^2 / radius + 2*(z^2n) /l
    distance_der_partial[0] = (Point_o[0] * 2) / radius ;
    distance_der_partial[1] = (Point_o[1] * 2) / radius ;
    distance_der_partial[2] = (std::pow((Point_o[2] * 2 / height), (2 * n - 1)) * (2 * n) ); //n=4
    //n=1
    // distance_der_partial[2] = (Point_o[2]*4) / height ;
    distance_der_partial[3] = 0;

    Eigen::Vector3d Der_v;
    Eigen::Vector4d partial_temp;
    // partial_temp = Tvo_eigen*distance_der_partial;
    partial_temp = distance_der_partial;
    Der_v[0] = partial_temp[0];
    Der_v[1] = partial_temp[1];
    Der_v[2] = partial_temp[2];

    return Der_v;
}

Eigen::Matrix<double, 4, 4>  FromKdlToEigen(KDL::Frame &T_v_o)
{
    Eigen::Matrix<double, 4, 4>  Tvo_eigen;
    Tvo_eigen.row(0) << T_v_o.M.data[0], T_v_o.M.data[1], T_v_o.M.data[2], T_v_o.p.x();
    Tvo_eigen.row(1) << T_v_o.M.data[3], T_v_o.M.data[4], T_v_o.M.data[5], T_v_o.p.y();
    Tvo_eigen.row(2) << T_v_o.M.data[6], T_v_o.M.data[7], T_v_o.M.data[8], T_v_o.p.z();
    Tvo_eigen.row(3) << 0, 0, 0, 1;
    return Tvo_eigen;
}

double PotentialFieldControl::VelocityLimit(KDL::Twist x_dot_d )
{

    Eigen::Matrix<double, 3, 1> x_dot_d_local = Eigen::Matrix<double, 3, 1>::Zero();
    x_dot_d_local << x_dot_d.vel.data[0], x_dot_d.vel.data[1], x_dot_d.vel.data[2];
    double temp = parameters_.vel_limit_robot / std::sqrt(x_dot_d_local.transpose() * x_dot_d_local);

    return std::min(1.0, temp);
}

void PotentialFieldControl::load_parameters(ros::NodeHandle &n)
{
    nh_.param<std::string>("topic_obstacle", topic_obstacle_avoidance, "obstacles");
    nh_.param<std::string>("tip_name", parameters_.tip_name, "end_effector");
    nh_.param<std::string>("root_name", parameters_.root_name, "world");
    nh_.param<std::string>("topic_desired_reference", topic_desired_reference, "command");
    nh_.param<double>("time_interpolation", parameters_.max_time_interpolation, 1);
    nh_.param<double>("max_tau_percentage", parameters_.max_tau_percentage, 0.5);
    nh_.param<double>("pf_repulsive_gain", parameters_.pf_repulsive_gain , 1);
    nh_.param<double>("pf_dist_to_obstacles", parameters_.pf_dist_to_obstacles , 1);
    nh_.param<double>("pf_dist_to_table", parameters_.pf_dist_to_table , 1);
    nh_.param<double>("vel_limit_robot", parameters_.vel_limit_robot , 0.5);
    nh_.param<double>("gain_null_space", parameters_.gain_null_space , 1.0);

    nh_.param<bool>("enable_obstacle_avoidance", parameters_.enable_obstacle_avoidance , true);
    nh_.param<bool>("enable_joint_limits_avoidance", parameters_.enable_joint_limits_avoidance , true);
    nh_.param<bool>("enable_attractive_field", parameters_.enable_attractive_field , true);
    nh_.param<bool>("enable_null_space", parameters_.enable_null_space , true);
    nh_.param<bool>("enable_interpolation", parameters_.enable_interpolation , false);
    nh_.param<int>("id_arm", parameters_.id_arm , 0);
    // ROS_INFO("topic_desired_reference: %s", desired_reference_topic.c_str());
    ROS_INFO("topic_obstacle: %s", topic_obstacle_avoidance.c_str());
    ROS_INFO("link_tip_name: %s", parameters_.tip_name.c_str());
    ROS_INFO("link_root_name: %s", parameters_.root_name.c_str());
    ROS_INFO("time_interpolation: %f", parameters_.max_time_interpolation);
    ROS_INFO("max_tua_percentage: %f", parameters_.max_tau_percentage);
    ROS_INFO("pf_repulsive_gain: %f", parameters_.pf_repulsive_gain);
    ROS_INFO("pf_dist_to_obstacles: %f", parameters_.pf_dist_to_obstacles);
    ROS_INFO("pf_dist_to_table: %f", parameters_.pf_dist_to_table);
    ROS_INFO("vel_limit_robot: %f", parameters_.vel_limit_robot);
    ROS_INFO("gain_null_space: %f", parameters_.gain_null_space);

    ROS_INFO_STREAM("Obstacle avoidance " << (parameters_.enable_obstacle_avoidance ? "Enabled" : "Disabled") );
    ROS_INFO_STREAM("Joint Limit Avoidance: " << (parameters_.enable_joint_limits_avoidance ? "Enabled" : "Disabled") );
    ROS_INFO_STREAM("Attractive Field: " << (parameters_.enable_attractive_field ? "Enabled" : "Disabled") );
    ROS_INFO_STREAM("Null Space: " << (parameters_.enable_null_space ? "Enabled" : "Disabled") );
    ROS_INFO_STREAM("Interpolation: " << (parameters_.enable_interpolation ? "Enabled" : "Disabled") );

    ROS_INFO_STREAM("start_controller: " << (start_controller ? "Enabled" : "Disabled") );

    if (!start_controller)
    {
        XmlRpc::XmlRpcValue my_list;
        nh_.getParam("links_with_potential_field", my_list);
        ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        parameters_.pf_list_of_links.clear();
        parameters_.pf_list_of_chains.clear();
        parameters_.pf_list_of_fk.clear();

        for ( int i = 0; i < my_list.size(); ++i)
        {
            ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            parameters_.pf_list_of_links.push_back(static_cast<std::string>(my_list[i]).c_str());
            ROS_INFO("Link %s defined as collision point", static_cast<std::string>(my_list[i]).c_str());
        }

        for (unsigned int i = 0; i < parameters_.pf_list_of_links.size(); ++i)
        {
            KDL::Chain chain_tmp;

            if (!kdl_tree_.getChain(parameters_.root_name.c_str(), parameters_.pf_list_of_links[i].c_str(), chain_tmp))
            {
                ROS_ERROR("Error processing chain from %s to %s ", parameters_.root_name.c_str(), parameters_.pf_list_of_links[i].c_str());
            }
            else
            {
                parameters_.pf_list_of_chains.push_back(chain_tmp);
                parameters_.pf_list_of_fk.push_back(KDL::ChainFkSolverPos_recursive(chain_tmp));
                parameters_.pf_list_of_jac.push_back(KDL::ChainJntToJacSolver(chain_tmp));
                ROS_INFO("Chain from %s to %s correctly initialized. Num oj Joints = %d", parameters_.root_name.c_str(), parameters_.pf_list_of_links[i].c_str(),
                         chain_tmp.getNrOfJoints());
            }
        }
    }

    parameters_.k_p = Eigen::Matrix<double, 6, 6>::Zero();
    parameters_.k_d = Eigen::Matrix<double, 6, 6>::Zero();
    parameters_.k_i = Eigen::Matrix<double, 6, 6>::Zero();
    parameters_.k_p = getGainMatrix(std::string("k_p"), n);
    parameters_.k_d = getGainMatrix(std::string("k_d"), n);
    parameters_.k_i = getGainMatrix(std::string("k_i"), n);

    ROS_INFO_STREAM("k_p" << std::endl << parameters_.k_p);
    ROS_INFO_STREAM("k_d" << std::endl << parameters_.k_d);
    ROS_INFO_STREAM("k_i" << std::endl << parameters_.k_i);

    ROS_INFO("Parameters loaded");
    return;
}


Eigen::MatrixXd PotentialFieldControl::getGainMatrix(std::string parameter, ros::NodeHandle n, int dimension)
{
    XmlRpc::XmlRpcValue my_list;
    nh_.getParam(parameter.c_str(), my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    Eigen::MatrixXd K(dimension, dimension);
    K = Eigen::MatrixXd::Zero(dimension, dimension);
    for (int i = 0; i < std::max(my_list.size(), dimension); ++i)
    {
        K(i, i) = static_cast<double>(my_list[i]);
    }
    return K;
}

void PotentialFieldControl::startControllerCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    start_controller = msg->data;
    return;
}


bool PotentialFieldControl::loadParametersCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_parameters(nh_);
    return true;
}

Eigen::Matrix<double, 7, 1> PotentialFieldControl::task_objective_function(KDL::JntArray q)
{
    double sum = 0;
    double temp;
    int N = q.data.size();

    Eigen::Matrix<double, 7, 1> tempret =  Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> weights =  Eigen::Matrix<double, 7, 1>::Zero();
    weights << 1, 1, 1, 1, .1, .1, .1;

    for (int i = 0; i < N; i++)
    {
        temp = weights(i) * ((q(i) - joint_limits_.center(i)) / (joint_limits_.max(i) - joint_limits_.min(i)));
        // sum += temp*temp;
        sum += temp;
    }

    sum /= 2 * N;

    for (int i = 0; i < N; i++)
    {
        tempret(i) = sum;
    }

    // Eigen::Matrix<double, 7, 1> temp_mat = MaxZYDistance( q );

    return -parameters_.gain_null_space *   tempret;

}

Eigen::Matrix<double, 7, 1>  PotentialFieldControl::getTauRepulsive(Eigen::Matrix<double, 6, 6> lambda, KDL::Jacobian &J, unsigned int n_joint, Eigen::Matrix<double, 6, 1> F)
{

    KDL::Jacobian J_local;
    J_local.resize(7);
    J_local.data = J.data;

    for (unsigned int i = 6; i > n_joint - 1; i--)
    {
        J_local.setColumn(i, KDL::Twist::Zero());
    }
    return J_local.data.transpose() * lambda * F;

}

Eigen::Matrix<double, 7, 1> PotentialFieldControl::MaxZYDistance(KDL::JntArray q)
{
    Eigen::Matrix<double, 7, 1> potential = Eigen::Matrix<double, 7, 1>::Zero();
    double cost = 0;

    unsigned int index = 2;
    for (unsigned int i = 0; i < parameters_.pf_list_of_links.size() -1; ++i)
        // for (unsigned int i = 0; i < 1; ++i)
    {
        KDL::Frame T;
        const unsigned int numJoints = parameters_.pf_list_of_chains[i].getNrOfJoints();
        KDL::JntArray q_local( numJoints );

        for (unsigned j = 0; j < numJoints; ++j) {
            q_local(j) = q(j);
        }
        parameters_.pf_list_of_fk[i].JntToCart(q_local, T);

        cost = T.p.data[index] * T.p.data[index];

        KDL::Jacobian jac_local;
        jac_local.resize( numJoints );
        Eigen::Matrix<double, 1, 7> jac_local_eigen = Eigen::Matrix<double, 1, 7>::Zero();
        jac_local_eigen.block(0, 0, 1, numJoints) = jac_local.data.block(index, 0, 1, numJoints);

        potential += jac_local_eigen.transpose() * cost;

        if ( T.p.data[index] > 0)
        {
            potential += jac_local_eigen.transpose() * cost;
        }
        else
        {
            potential -= jac_local_eigen.transpose() * cost;
        }

    }

    index = 1;

    for (unsigned int i = 0; i < parameters_.pf_list_of_links.size() -1; ++i)
    {
        KDL::Frame T;
        const unsigned int numJoints = parameters_.pf_list_of_chains[i].getNrOfJoints();
        KDL::JntArray q_local( numJoints );

        for (unsigned j = 0; j < numJoints; ++j) {
            q_local(j) = q(j);
        }
        parameters_.pf_list_of_fk[i].JntToCart(q_local, T);

        cost = T.p.data[index] * T.p.data[index];

        KDL::Jacobian jac_local;
        jac_local.resize( numJoints );
        Eigen::Matrix<double, 1, 7> jac_local_eigen = Eigen::Matrix<double, 1, 7>::Zero();
        jac_local_eigen.block(0, 0, 1, numJoints) = jac_local.data.block(index, 0, 1, numJoints);

        if ( T.p.data[index] > 0)
        {
            potential += jac_local_eigen.transpose() * cost;
        }
        else
        {
            potential -= jac_local_eigen.transpose() * cost;
        }

    }

    return -parameters_.gain_null_space * potential;

}


Eigen::Matrix<double, 7, 1> PotentialFieldControl::potentialEnergy(KDL::JntArray q)
{

    KDL::JntArray G_local(7);
    id_solver_->JntToGravity(joint_msr_states_.q, G_local);

    return parameters_.gain_null_space * G_local.data ;

}

}



PLUGINLIB_EXPORT_CLASS(desperate_housewife::PotentialFieldControl, controller_interface::ControllerBase)
