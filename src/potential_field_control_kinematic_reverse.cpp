#include <potential_field_control_kinematic_reverse.h>
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
#include <tf_conversions/tf_kdl.h>

#include <math.h>


namespace desperate_housewife
{
PotentialFieldControlKinematicReverse::PotentialFieldControlKinematicReverse() {}
PotentialFieldControlKinematicReverse::~PotentialFieldControlKinematicReverse() {}

bool PotentialFieldControlKinematicReverse::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
{
    KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n);
    ROS_INFO("Starting controller");
    ROS_WARN("Number of segments: %d", kdl_chain_.getNrOfSegments());

    // for switch the hand_desired
    start_controller = false;
    load_parameters(n);

    //resize the vector that we use for calculates the dynamic
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    J_.resize(kdl_chain_.getNrOfJoints());
    joint_des_states_filtered.resize(kdl_chain_.getNrOfJoints());
    joint_des_states_old.resize(kdl_chain_.getNrOfJoints());


    F_repulsive = Eigen::Matrix<double, 6, 1>::Zero();
    F_attractive = Eigen::Matrix<double, 6, 1>::Zero();
    F_total = Eigen::Matrix<double, 6, 1>::Zero();


    ROS_INFO("Subscribed for desired_reference to: %s", "command");
    sub_command = nh_.subscribe(topic_desired_reference.c_str(), 1, &PotentialFieldControlKinematicReverse::command, this);
    //list of obstacles
    ROS_INFO("Subscribed for obstacle_avoidance_topic to : %s", topic_obstacle_avoidance.c_str());
    sub_obstacles = nh_.subscribe(topic_obstacle_avoidance.c_str(), 1, &PotentialFieldControlKinematicReverse::InfoGeometry, this);

    sub_start_controller = nh_.subscribe("start_controller", 1, &PotentialFieldControlKinematicReverse::startControllerCallBack, this);

//callcback for setting the gains at real time

    pub_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
    pub_error_id = nh_.advertise<desperate_housewife::Error_msg>("error_id", 1000);
    pub_q = nh_.advertise<std_msgs::Float64MultiArray>("q_commad", 1000);
    pub_qp = nh_.advertise<std_msgs::Float64MultiArray>("qp_commad", 1000);
    pub_pf_attractive_force = nh_.advertise<std_msgs::Float64MultiArray>("F_attractive", 1000);
    pub_pf_repulsive_forse = nh_.advertise<std_msgs::Float64MultiArray>("F_repulsive", 1000);
    pub_pf_total_force = nh_.advertise<std_msgs::Float64MultiArray>("F_total", 1000);

    pub_total_wrench = nh_.advertise<geometry_msgs::WrenchStamped>("total_end_effector_wrench", 512);

    srv_start_controller = nh_.advertiseService("load_parameters", &PotentialFieldControlKinematicReverse::loadParametersCallback, this);

    collisions_lines_pub = nh_.advertise<visualization_msgs::MarkerArray>("collision_lines", 1);
    arrows_pub = nh_.advertise<visualization_msgs::MarkerArray>("collision_arrows", 1);

    error_id.id = 10000;
    // error_id.id_arm = parameters_.id_arm;
    return true;
}

void PotentialFieldControlKinematicReverse::starting(const ros::Time& time)
{

    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
        joint_des_states_old.qdot(i) = 0.0;
    }

    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);
    x_now_int = x_des_;
    x_des_int = x_des_;

    SetToZero(x_err_integral);
    SetToZero(x_err_);
    x_err_.vel.data[0] = 10000.0;
    start_controller = false;
}

void PotentialFieldControlKinematicReverse::update(const ros::Time& time, const ros::Duration& period)
{

    std_msgs::Float64MultiArray q_msg;
    std_msgs::Float64MultiArray qp_msg;
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

    KDL::JntArray vel_repulsive;
    vel_repulsive.resize(7);
    SetToZero(vel_repulsive);

    tf::twistKDLToMsg(x_err_, error_id.error_);

//flag to use this code with real robot

    KDL::Twist x_err_msg;
    x_err_msg = x_err_;

    if (start_controller)
    {

        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

// computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);


        if (parameters_.enable_interpolation)
        {
            x_des_.p = x_now_int.p + interpolatormb(time_inter, parameters_.max_time_interpolation) * (x_des_int.p - x_now_int.p);

            x_des_int.M.GetQuaternion(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);
            x_now_int.M.GetQuaternion(quat_now.v(0), quat_now.v(1), quat_now.v(2), quat_now.a);

            tf::Quaternion quat_tf_des_int(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);
            tf::Quaternion quat_tf_now_int(quat_now.v(0), quat_now.v(1), quat_now.v(2), quat_now.a);
            quat_tf = (tf::slerp(quat_tf_now_int, quat_tf_des_int, Time)).normalize();
            tf::quaternionTFToKDL(quat_tf, x_des_.M);

            KDL::Twist x_err_int;


            time_inter = time_inter + period.toSec();

// SO3 Time update
            Time = interpolatormb(time_inter, parameters_.max_time_interpolation);
        }
        else
        {
            x_des_ = x_des_int;
        }

        x_dot_ = J_.data * joint_msr_states_.qdot.data;

        x_err_ = diff(x_, x_des_);

        F_total = Eigen::Matrix<double, 6, 1>::Zero();
        F_repulsive  = Eigen::Matrix<double, 6, 1>::Zero();

        if (parameters_.enable_obstacle_avoidance)
        {
            lines_total.markers.clear();
            arrows_total.markers.clear();
            id_global = 0;
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
                    // double influence_local = Object_radius[k] +  parameters_.pf_dist_to_obstacles;
                    double influence_local = parameters_.pf_dist_to_obstacles;
                    Eigen::Matrix<double, 6, 1> F_obj_local = Eigen::Matrix<double, 6, 1>::Zero();
                    F_obj_local = GetRepulsiveForce(fk_chain, influence_local, Object_position[k], Object_radius[k], Object_height[k] );
                    F_obj = F_obj + F_obj_local;
                    if (F_obj_local.norm() != 0.0)
                    {
                        num_of_links_in_potential = num_of_links_in_potential + 1.0;
                    }
                    ++id_global;
                }

                F_obj_base_link = Adjoint * F_obj;

                F_obj_base_total += F_obj_base_link;



                F_table = GetRepulsiveForceTable(fk_chain, parameters_.pf_dist_to_table );
                if (F_table.norm() != 0.0)
                {
                    num_of_links_in_potential = num_of_links_in_potential + 1.0;
                    // ROS_INFO_STREAM(F_table.transpose());
                }
                F_table_base_link = Adjoint * F_table;
                F_table_base_total += F_table_base_link;

                vel_repulsive.data += getVelRepulsive( J_, parameters_.pf_list_of_chains[i].getNrOfJoints(), (F_table_base_link + F_obj_base_link) );
                collisions_lines_pub.publish(lines_total);
                arrows_pub.publish(arrows_total);


            }


            F_repulsive  = (F_table_base_total + F_obj_base_total);

            if (num_of_links_in_potential > 1.0)
            {
                vel_repulsive.data = (1.0 / num_of_links_in_potential) * vel_repulsive.data;
            }

            if ( vel_repulsive.data.norm() != 0.0 )
            {
                // ROS_INFO_STREAM( vel_repulsive.data.transpose() );
                // ROS_INFO_STREAM( "num_of_links_in_potential: " << num_of_links_in_potential);
            }


            // ROS_INFO_STREAM( vel_repulsive.data );

        }

        if (parameters_.enable_attractive_field)
        {
            KDL::Twist x_dot_d;

            x_dot_d.vel.data[0] = parameters_.k_p(0, 0) / parameters_.k_d(0, 0) * x_err_.vel.data[0];
            x_dot_d.vel.data[1] = parameters_.k_p(1, 1) / parameters_.k_d(1, 1) * x_err_.vel.data[1];
            x_dot_d.vel.data[2] = parameters_.k_p(2, 2) / parameters_.k_d(2, 2) * x_err_.vel.data[2];
            x_dot_d.rot.data[0] = parameters_.k_p(3, 3) / parameters_.k_d(3, 3) * x_err_.rot.data[0];
            x_dot_d.rot.data[1] = parameters_.k_p(4, 4) / parameters_.k_d(4, 4) * x_err_.rot.data[1];
            x_dot_d.rot.data[2] = parameters_.k_p(5, 5) / parameters_.k_d(5, 5) * x_err_.rot.data[2];

            double v_limited = VelocityLimit(x_dot_d);

            x_err_integral += x_err_ * period.toSec();
            for (int i = 0; i < F_attractive.size(); i++)
            {
                F_attractive(i) =  -parameters_.k_d(i, i) * ( x_dot_(i) -  v_limited * x_dot_d(i) ) + parameters_.k_i(i, i) * x_err_integral(i);
                // F_attractive(i) =  parameters_.k_p(i, i) * x_err_(i);
            }
        }

        Eigen::MatrixXd x_err_eigen_ = Eigen::MatrixXd::Zero(6, 1);
        // x_err_eigen_ << parameters_.k_p(0, 0) * x_err_(0),
        //              parameters_.k_p(1, 1) * x_err_(1),
        //              parameters_.k_p(2, 2) * x_err_(2),
        //              parameters_.k_p(3, 3) * x_err_(3),
        //              parameters_.k_p(4, 4) * x_err_(4),
        //              parameters_.k_p(5, 5) * x_err_(5);


        x_err_eigen_ = F_attractive;

        int n_task = 2;
        std::vector<Eigen::MatrixXd> qp(n_task + 1, Eigen::MatrixXd::Zero(7, 1));
        qp[n_task] = Eigen::MatrixXd::Zero(7, 1);

        std::vector<Eigen::MatrixXd> xp(n_task + 1);

        Eigen::MatrixXd secondTask = potentialEnergy( joint_msr_states_.q );
        xp[0] = x_err_eigen_;
        xp[1] = secondTask;


        std::vector<Eigen::MatrixXd> J(n_task + 1);
        J[0] = J_.data;
        J[1] = Eigen::MatrixXd::Identity(7, 7);

        qp[1] = secondTask;

        Eigen::MatrixXd Pp1 = Eigen::MatrixXd::Identity(7, 7);
        for (int i = n_task - 1 ; i >= 0; i--)
        {
            Eigen::MatrixXd JkPkp_pseudo;
            pseudo_inverse(J[i] * Pp1,  JkPkp_pseudo);
            qp[i] = qp[i + 1] + JkPkp_pseudo * (xp[i] - J[i] * qp[i + 1]);
        }


        // ROS_INFO_STREAM(qp[1].transpose());


        for (int i = 0; i < 7; i++)
        {
            joint_des_states_.qdot(i) = 0.0;
            joint_des_states_.qdot(i) +=  qp[0](i);

        }

        for (int i = 0; i < 7; i++)
        {

            // joint_des_states_filtered.qdot(i) =  filters::exponentialSmoothing(joint_des_states_.qdot(i), joint_des_states_old.qdot(i), 0.5);
            joint_des_states_filtered.qdot(i) = joint_des_states_.qdot(i);
            joint_des_states_old.qdot(i) = joint_des_states_filtered.qdot(i);
        }

        if (parameters_.enable_obstacle_avoidance)
        {
            for (int i = 0; i < 7; i++)
            {
                joint_des_states_filtered.qdot(i) += vel_repulsive.data[i] ;
            }
            // if ( vel_repulsive.data.norm() != 0.0 )
            // {
            //     ROS_INFO_STREAM(vel_repulsive.data.transpose());
            // }
        }


//// Saturaion of velocities

        const double deg2rad = M_PI  / 180.0;

        joint_des_states_filtered.qdot(0) = (std::abs(joint_des_states_filtered.qdot(0)) >= 110.0 * deg2rad * parameters_.max_vel_percentage ?
                                             std::copysign(110.0 * deg2rad * parameters_.max_vel_percentage, joint_des_states_filtered.qdot(0))
                                             : joint_des_states_filtered.qdot(0));
        joint_des_states_filtered.qdot(1) = (std::abs(joint_des_states_filtered.qdot(1)) >= 110.0 * deg2rad * parameters_.max_vel_percentage ?
                                             std::copysign(110.0 * deg2rad * parameters_.max_vel_percentage, joint_des_states_filtered.qdot(1))
                                             : joint_des_states_filtered.qdot(1));
        joint_des_states_filtered.qdot(2) = (std::abs(joint_des_states_filtered.qdot(2)) >= 128.0 * deg2rad * parameters_.max_vel_percentage ?
                                             std::copysign(128.0 * deg2rad * parameters_.max_vel_percentage, joint_des_states_filtered.qdot(2))
                                             : joint_des_states_filtered.qdot(2));
        joint_des_states_filtered.qdot(3) = (std::abs(joint_des_states_filtered.qdot(3)) >= 128.0 * deg2rad * parameters_.max_vel_percentage ?
                                             std::copysign(128.0 * deg2rad * parameters_.max_vel_percentage, joint_des_states_filtered.qdot(3))
                                             : joint_des_states_filtered.qdot(3));
        joint_des_states_filtered.qdot(4) = (std::abs(joint_des_states_filtered.qdot(4)) >= 204.0 * deg2rad * parameters_.max_vel_percentage ?
                                             std::copysign(204.0 * deg2rad * parameters_.max_vel_percentage, joint_des_states_filtered.qdot(4))
                                             : joint_des_states_filtered.qdot(4));
        joint_des_states_filtered.qdot(5) = (std::abs(joint_des_states_filtered.qdot(5)) >= 184.0 * deg2rad * parameters_.max_vel_percentage ?
                                             std::copysign(184.0 * deg2rad * parameters_.max_vel_percentage, joint_des_states_filtered.qdot(5))
                                             : joint_des_states_filtered.qdot(5));
        joint_des_states_filtered.qdot(6) = (std::abs(joint_des_states_filtered.qdot(6)) >= 184.0 * deg2rad * parameters_.max_vel_percentage ?
                                             std::copysign(184.0 * deg2rad * parameters_.max_vel_percentage, joint_des_states_filtered.qdot(6))
                                             : joint_des_states_filtered.qdot(6));


        // ROS_INFO_STREAM(vel_repulsive.data.transpose());

        x_err_msg = diff(x_, x_des_int);

        tf::twistKDLToMsg(x_err_msg, error_id.error_);

        for (unsigned int i = 0; i < joint_handles_.size(); i++)
        {
            joint_des_states_.q(i) += period.toSec() * joint_des_states_filtered.qdot(i);
        }

    }

    pub_error_id.publish( error_id );

    for (unsigned int i = 0; i < F_total.size(); i++ )
    {
        F_repulsive_msg.data.push_back(F_repulsive(i));
        F_attractive_msg.data.push_back(F_attractive(i));
        F_total_msg.data.push_back(F_total(i));
        err_msg.data.push_back(x_err_msg(i));
    }

    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(joint_des_states_.q(i));
        q_msg.data.push_back(joint_des_states_.q(i));
        qp_msg.data.push_back(joint_des_states_filtered.qdot(i));
    }


    pub_pf_repulsive_forse.publish(F_repulsive_msg);
    pub_pf_attractive_force.publish(F_attractive_msg);
    pub_pf_total_force.publish(F_total_msg);
    pub_error.publish(err_msg);
    pub_q.publish(q_msg);
    pub_qp.publish(qp_msg);

    ros::spinOnce();
}

Eigen::Matrix<double, 3, 3> PotentialFieldControlKinematicReverse::getVectorHat(Eigen::Matrix<double, 3, 1> vector_in)
{
    Eigen::Matrix<double, 3, 3> vector_hat = Eigen::Matrix<double, 3, 3>::Zero();

    vector_hat << 0, -vector_in(2, 0), vector_in(1, 0),
               vector_in(2, 0), 0, -vector_in(0, 0),
               -vector_in(1, 0), vector_in(0, 0), 0;
    return vector_hat;
}

Eigen::Matrix<double, 6, 6> PotentialFieldControlKinematicReverse::getAdjointT( KDL::Frame Frame_in)
{
    Eigen::Matrix<double, 6, 6> Adjoint_local = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> rotation_local(Frame_in.M.data);
    Eigen::Matrix<double, 3, 1> position_local(Frame_in.p.data);

    Adjoint_local.block<3, 3>(0, 0) = rotation_local.transpose();
    Adjoint_local.block<3, 3>(3, 3) = rotation_local.transpose();
    Adjoint_local.block<3, 3>(3, 0) = -rotation_local.transpose() * getVectorHat(position_local);

    return Adjoint_local;
}

void PotentialFieldControlKinematicReverse::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
{

    KDL::Frame frame_des_;
    tf::poseMsgToKDL(msg->pose, frame_des_);

    PoseDesiredInterpolation(frame_des_);

    error_id.id = msg->id;

    // ROS_INFO("New reference for controller");
}

void PotentialFieldControlKinematicReverse::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
    Object_radius.clear();
    Object_height.clear();
    Object_position.clear();
    // Time_traj_rep = 0;
    // std::cout<<"msg->geometries.size(): "<<msg->geometries.size()<<std::endl;
    // get info for calculates objects surface
    for (unsigned int i = 0; i < msg->geometries.size(); i++)
    {
        KDL::Frame frame_obj;
        // radius
        Object_radius.push_back(msg->geometries[i].info[0]);
        // std::cout<<" Object_radius[i]: "<<Object_radius[i]<<std::endl;
        // height
        Object_height.push_back(msg->geometries[i].info[1]);

        tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
        Object_position.push_back(frame_obj);
    }

    ROS_INFO("New environment: No of Obstacles %lu", msg->geometries.size());
}

void PotentialFieldControlKinematicReverse::PoseDesiredInterpolation(KDL::Frame frame_des_)
{
    if (Int == 0)
    {
        x_des_int = frame_des_;
        // x_des_ = x_des_int;
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_now_int);
        Int = 1;
        // time for slerp interpolation
        Time = 0;
        time_inter = 0;
        SetToZero(x_err_last);
    }

    else
    {
        // new pose
        if (!Equal(frame_des_, x_des_int, 0.05))
        {
            // update desired frame;
            // F_attractive_last = F_attractive;
            x_des_int = frame_des_;
            // update robot position
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_now_int);
            // time update
            time_inter = 0;
            Time = 0;
            // switch_trajectory = true;
            x_err_last = x_err_;
            SetToZero(x_err_integral);
        }
    }
}

KDL::JntArray PotentialFieldControlKinematicReverse::JointLimitAvoidance(KDL::JntArray q)
{   // This function implements joint limit avoidance usung the penalty function V = \sum\limits_{i=1}^n\frac{1}{s^2} s = q_{l_1}-|q_i|
    KDL::JntArray tau_limit_avoidance(q.data.size());
    double s, potential, treshold, q_abs;

    for (unsigned int i = 0; i < q.data.size(); i++)
    {
        treshold = 5.0 * M_PI / 180.0;
        q_abs = std::fabs(q.data[i]);
        s = joint_limits_.max(i) - q_abs;


        if (  s < treshold )
        {
            ROS_WARN("Joint limit %d", i);
            potential = 0.5 * std::pow((1 / s - 1 / treshold), 2);
            tau_limit_avoidance.data[i] = - parameters_.gain_null_space * KDL::sign(q.data[i]) * potential;
        }
        else
        {
            tau_limit_avoidance(i) = 0.0;
        }
    }

    return tau_limit_avoidance;
}


Eigen::Matrix<double, 6, 1> PotentialFieldControlKinematicReverse::GetRepulsiveForce(KDL::Frame &T_in, double influence, KDL::Frame &Object_pos, double radius, double height)
{
    Eigen::Matrix<double, 6, 1> ForceAndIndex;
    ForceAndIndex =  Eigen::Matrix<double, 6, 1>::Zero();

    KDL::Frame T_link_object = ( T_in.Inverse() * Object_pos ).Inverse();

    KDL::Frame T_CollisionPoint;

    getClosestPointstoCylinder( T_link_object, T_CollisionPoint, radius, height);

    T_CollisionPoint = Object_pos * T_CollisionPoint;


    double distance = (T_in.p - T_CollisionPoint.p).Norm();

    Eigen::Vector3d Nolmal_to_Cylinder;
    Nolmal_to_Cylinder << T_CollisionPoint.M.UnitZ().data[0], T_CollisionPoint.M.UnitZ().data[1], T_CollisionPoint.M.UnitZ().data[2];

    KDL::Vector Nolmal_to_Cylinder_kdl = T_CollisionPoint.M.UnitZ();
    DrawArrow(  Nolmal_to_Cylinder_kdl, T_CollisionPoint.p , 0, 0, 0 );

    LineCollisions::Point Point1(T_in.p.x(), T_in.p.y(), T_in.p.z());
    LineCollisions::Point Point2(T_CollisionPoint.p.x(), T_CollisionPoint.p.y(), T_CollisionPoint.p.z());
    LineCollisions::Line ClosestPoints(Point1, Point2);

    if ( distance <= influence)
    {
        plotline(ClosestPoints, 1.0, 0.0, 0.0);
        ForceAndIndex = GetFIRAS(distance, Nolmal_to_Cylinder, influence, parameters_.pf_repulsive_gain_obstacles);
    }
    else
    {
        plotline(ClosestPoints);
    }

    // tf::Transform CollisionTransform;
    // tf::TransformKDLToTF( T_CollisionPoint, CollisionTransform);
    // tf_desired_hand_pose.sendTransform( tf::StampedTransform( CollisionTransform, ros::Time::now(), "world", "collision_point") );

    Eigen::Matrix<double, 6, 1> force_local_link = Eigen::Matrix<double, 6, 1>::Zero();
    force_local_link = getAdjointT( T_in.Inverse() * Object_pos) * ForceAndIndex;
    // force_local_link = getAdjointT( T_in.Inverse() ) * ForceAndIndex;

    return force_local_link;
    // return ForceAndIndex;
}

Eigen::Matrix<double, 6, 1> PotentialFieldControlKinematicReverse::GetRepulsiveForceTable(KDL::Frame &T_in, double influence)
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
        force_local_object = GetFIRAS(distance_local, distance_der_partial, parameters_.pf_dist_to_table, parameters_.pf_repulsive_gain_table);
    }

    Eigen::Matrix<double, 6, 1> force_local_link = Eigen::Matrix<double, 6, 1>::Zero();
    force_local_link = getAdjointT( T_in.Inverse() * T_table_world) * force_local_object;

    return force_local_link;
}

Eigen::Matrix<double, 6, 1> PotentialFieldControlKinematicReverse::GetFIRAS(double min_distance, Eigen::Vector3d &distance_der_partial, double influence, double gain)
{

    Eigen::Matrix<double, 6, 1> Force = Eigen::Matrix<double, 6, 1>::Zero();
    double V = gain * ( (1.0 / min_distance) -
                        (1.0 / influence) )  * (1.0 / (min_distance * min_distance));
    Force(0) = V * distance_der_partial[0];
    Force(1) = V * distance_der_partial[1];
    Force(2) = V * distance_der_partial[2];
    Force(3) = 0;
    Force(4) = 0;
    Force(5) = 0;


    return Force;
}


Eigen::Vector3d PotentialFieldControlKinematicReverse::GetPartialDerivate(KDL::Frame &T_v_o, KDL::Vector &Point_v, double radius, double height)
{
    Eigen::Matrix<double, 4, 4>  Tvo_eigen;
    Tvo_eigen = FromKdlToEigen(T_v_o);
    Eigen::Vector4d Point_v_eigen(Point_v.x(), Point_v.y(), Point_v.z(), 1);

    Eigen::Vector4d Point_o;
    Point_o = Tvo_eigen.inverse() * Point_v_eigen;
    double n = 2;   // n as in the paper should be in 4 but considering the shortest distance to obstacles. Here this is not being considered :( TODO

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

    return Der_v.normalized();
}

Eigen::Matrix<double, 4, 4>  PotentialFieldControlKinematicReverse::FromKdlToEigen(KDL::Frame &T_v_o)
{
    Eigen::Matrix<double, 4, 4>  Tvo_eigen;
    Tvo_eigen.row(0) << T_v_o.M.data[0], T_v_o.M.data[1], T_v_o.M.data[2], T_v_o.p.x();
    Tvo_eigen.row(1) << T_v_o.M.data[3], T_v_o.M.data[4], T_v_o.M.data[5], T_v_o.p.y();
    Tvo_eigen.row(2) << T_v_o.M.data[6], T_v_o.M.data[7], T_v_o.M.data[8], T_v_o.p.z();
    Tvo_eigen.row(3) << 0, 0, 0, 1;
    return Tvo_eigen;
}

double PotentialFieldControlKinematicReverse::VelocityLimit(KDL::Twist x_dot_d )
{

    Eigen::Matrix<double, 3, 1> x_dot_d_local = Eigen::Matrix<double, 3, 1>::Zero();
    x_dot_d_local << x_dot_d.vel.data[0], x_dot_d.vel.data[1], x_dot_d.vel.data[2];
    double temp = parameters_.vel_limit_robot / std::sqrt(x_dot_d_local.transpose() * x_dot_d_local);

    return std::min(1.0, temp);
}

void PotentialFieldControlKinematicReverse::load_parameters(ros::NodeHandle &n)
{
    nh_.param<std::string>("topic_obstacle", topic_obstacle_avoidance, "obstacles");
    nh_.param<std::string>("tip_name", parameters_.tip_name, "end_effector");
    nh_.param<std::string>("root_name", parameters_.root_name, "world");
    nh_.param<std::string>("topic_desired_reference", topic_desired_reference, "command");
    nh_.param<double>("time_interpolation", parameters_.max_time_interpolation, 1);
    nh_.param<double>("max_vel_percentage", parameters_.max_vel_percentage, 0.5);
    nh_.param<double>("pf_repulsive_gain_obstacles", parameters_.pf_repulsive_gain_obstacles , 1.0);
    nh_.param<double>("pf_repulsive_gain_table", parameters_.pf_repulsive_gain_table , 1.0);
    nh_.param<double>("pf_dist_to_obstacles", parameters_.pf_dist_to_obstacles , 1.0);
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
    ROS_INFO("max_vel_percentage: %f", parameters_.max_vel_percentage);
    ROS_INFO("pf_repulsive_gain_obstacles: %f", parameters_.pf_repulsive_gain_obstacles);
    ROS_INFO("pf_repulsive_gain_table: %f", parameters_.pf_repulsive_gain_table);
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


Eigen::MatrixXd PotentialFieldControlKinematicReverse::getGainMatrix(std::string parameter, ros::NodeHandle n, int dimension)
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

void PotentialFieldControlKinematicReverse::startControllerCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    start_controller = msg->data;
    return;
}


bool PotentialFieldControlKinematicReverse::loadParametersCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_parameters(nh_);
    return true;
}

Eigen::Matrix<double, 7, 1> PotentialFieldControlKinematicReverse::task_objective_function(KDL::JntArray q)
{
    double sum = 0;
    double temp;
    int N = q.data.size();

    Eigen::Matrix<double, 7, 1> tempret =  Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> weights =  Eigen::Matrix<double, 7, 1>::Zero();
    weights << 1, 1, 1, 10, 1, 1, 1;

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

Eigen::Matrix<double, 7, 1>  PotentialFieldControlKinematicReverse::getVelRepulsive( KDL::Jacobian &J, unsigned int n_joint, Eigen::Matrix<double, 6, 1> F)
{

    KDL::Jacobian J_local;
    J_local.resize(7);
    J_local.data = J.data;

    for (unsigned int i = 6; i > n_joint - 1; i--)
    {
        J_local.setColumn(i, KDL::Twist::Zero());
    }

    Eigen::MatrixXd J_pinv_n;
    pseudo_inverse(J_local.data, J_pinv_n);

    return J_pinv_n * F;
}

Eigen::Matrix<double, 7, 1> PotentialFieldControlKinematicReverse::MaxZYDistance(KDL::JntArray q)
{
    Eigen::Matrix<double, 7, 1> potential = Eigen::Matrix<double, 7, 1>::Zero();
    double cost = 0;

    unsigned int index = 2;
    for (unsigned int i = 0; i < parameters_.pf_list_of_links.size() - 1; ++i)
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

        Eigen::MatrixXd jac_pseudo_local_eigen = Eigen::Matrix<double, 7, 1>::Zero();

        JacobiSVD<MatrixXd>::SingularValuesType sing_vals_2;
        pseudo_inverse(jac_local_eigen, jac_pseudo_local_eigen);

        for (unsigned l = 0; l < 7; ++l) {
            if ( std::isnan( jac_pseudo_local_eigen(l)) )
            {
                jac_pseudo_local_eigen = Eigen::Matrix<double, 7, 1>::Zero();
            }
        }

        if ( T.p.data[index] > 0)
        {
            potential += jac_pseudo_local_eigen * cost;
        }
        else
        {
            potential -= jac_pseudo_local_eigen * cost;
        }

    }

    index = 1;

    for (unsigned int i = 0; i < parameters_.pf_list_of_links.size() - 1; ++i)
    {
        KDL::Frame T;
        const unsigned int numJoints = parameters_.pf_list_of_chains[i].getNrOfJoints();
        KDL::JntArray q_local( numJoints );

        for (unsigned j = 0; j < numJoints; ++j)
        {
            q_local(j) = q(j);
        }
        parameters_.pf_list_of_fk[i].JntToCart(q_local, T);

        cost = T.p.data[index] * T.p.data[index];

        KDL::Jacobian jac_local;
        jac_local.resize( numJoints );
        Eigen::Matrix<double, 1, 7> jac_local_eigen = Eigen::Matrix<double, 1, 7>::Zero();
        jac_local_eigen.block(0, 0, 1, numJoints) = jac_local.data.block(index, 0, 1, numJoints);

        Eigen::MatrixXd jac_pseudo_local_eigen;

        pseudo_inverse(jac_local_eigen, jac_pseudo_local_eigen);

        for (unsigned l = 0; l < 7; ++l)
        {
            if ( std::isnan( jac_pseudo_local_eigen(l)) )
            {
                jac_pseudo_local_eigen = Eigen::Matrix<double, 7, 1>::Zero();
            }
        }

        if ( T.p.data[index] > 0)
        {
            potential += jac_pseudo_local_eigen * cost;
        }
        else
        {
            potential -= jac_pseudo_local_eigen * cost;
        }

    }

    return -parameters_.gain_null_space * potential;
}


Eigen::Matrix<double, 7, 1> PotentialFieldControlKinematicReverse::potentialEnergy(KDL::JntArray q)
{
    KDL::JntArray G_local(7);
    id_solver_->JntToGravity(joint_msr_states_.q, G_local);

    return parameters_.gain_null_space * G_local.data ;
}

void PotentialFieldControlKinematicReverse::plotline(LineCollisions::Line Line_local, float r, float g, float b, int id, int type)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = type;
    line_list.id = id_global;
    line_list.scale.x = 0.01;
    line_list.color.r = r;
    line_list.color.g = g;
    line_list.color.b = b;
    line_list.color.a = 1.0;


    geometry_msgs::Point p1;
    p1.x = Line_local.P1[0];
    p1.y = Line_local.P1[1];
    p1.z = Line_local.P1[2];

    geometry_msgs::Point p2;
    p2.x = Line_local.P2[0];
    p2.y = Line_local.P2[1];
    p2.z = Line_local.P2[2];
    line_list.lifetime = ros::Duration(1);

    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
    lines_total.markers.push_back(line_list);
}



void PotentialFieldControlKinematicReverse::DrawArrow( KDL::Vector &gridspace_Force, KDL::Vector &gridspace_point, int K, double Fmin, double Fmax )
{
    // std::cout<<"disegno"<<std::endl;

    int32_t shape = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = id_global;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = gridspace_point.x();
    marker.pose.position.y = gridspace_point.y();
    marker.pose.position.z = gridspace_point.z();

    Eigen::Quaterniond quat;
    quat =  RotationMarker(gridspace_Force, gridspace_point);

    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    marker.scale.x = 0.1;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    // ROS_INFO("marker %d, r = %f", K, ((gridspace_Force.Norm()-Fmin)/(Fmax-Fmin)) );
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(1);

    arrows_total.markers.push_back(marker);
}

Eigen::Quaterniond PotentialFieldControlKinematicReverse::RotationMarker(KDL::Vector &ris_Force, KDL::Vector &point)
{

    Eigen::Vector3d  x(1, 0, 0);
    Eigen::Vector3d Force_eigen(ris_Force.x(), ris_Force.y(), ris_Force.z());
    // double pi = 3.14159264;
    double angle = std::acos( (x.dot(Force_eigen)) / (x.norm() * Force_eigen.norm()));
    // std::cout<<"angle: "<<angle<<std::endl;
    Eigen::Matrix3d transformation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d axis(0, 0, 0);
    axis = (x.cross(Force_eigen)) / (x.cross(Force_eigen)).norm();
    transformation_ = Eigen::AngleAxisd(angle, axis);

    Eigen::Quaterniond quat_eigen_hand(transformation_);

    return quat_eigen_hand.normalized();
}


void PotentialFieldControlKinematicReverse::getClosestPointstoCylinder( KDL::Frame T_link_object, KDL::Frame &T_CollisionPoint, double radius, double height)
{

    LineCollisions LineCollisionsLocal;

    LineCollisions::Point Point1(T_link_object.p.x(), T_link_object.p.y(), T_link_object.p.z());
    LineCollisions::Point Point2(T_link_object.p.x() + .001, T_link_object.p.y() + .001, T_link_object.p.z() + .001);

    LineCollisions::Point Point3(0.0, 0.0, height / 2.0);
    LineCollisions::Point Point4(0.0, 0.0, -height / 2.0);

    LineCollisions::Line Line1(Point1, Point2);
    LineCollisions::Line Line2(Point3, Point4);
    LineCollisions::Line ClosestPoints;

    ClosestPoints = LineCollisionsLocal.getClosestPoints(Line1, Line2);

    KDL::Rotation Rot_to_collision_point;
    KDL::Vector collision_point;

    KDL::Vector V1(ClosestPoints.P1[0], ClosestPoints.P1[1], ClosestPoints.P1[2]);
    KDL::Vector V2(ClosestPoints.P2[0], ClosestPoints.P2[1], ClosestPoints.P2[2]);
    KDL::Vector V3;
    V3  = (V1 - V2);
    KDL::Frame pos_final;

    if ( (ClosestPoints.P2 == Point4) || (ClosestPoints.P2 == Point3) )
    {

        double pos_on_plane = KDL::Vector( (V2 - V1).data[0], (V2 - V1).data[1], 0.0 ).Norm();
        KDL::Vector V4( V3.data[0], V3.data[1], 0.0 );
        double angle = 0.0;
        KDL::Vector collision_point_on_line(ClosestPoints.P2[0], ClosestPoints.P2[1], ClosestPoints.P2[2] );
        if (pos_on_plane >= radius )
        {

            KDL::Vector Uz(V4 / V4.Norm() );
            KDL::Vector Ux(0.0, 0.0, 1.0);
            Rot_to_collision_point = KDL::Rotation(Ux, Uz * Ux, Uz);
            pos_final = KDL::Frame( Rot_to_collision_point, collision_point_on_line) *
                        KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector( 0.0 , 0.0, radius) );
            KDL::Vector V5;
            V5 = (T_link_object.Inverse() * pos_final).Inverse().p;
            V5 = V5 /  V5.Norm();
            angle = std::atan2(V5.x(), V5.z());
            collision_point = pos_final.p;

            Rot_to_collision_point = Rot_to_collision_point * KDL::Rotation::RotY( angle );
        }
        else
        {
            KDL::Vector Ux;
            KDL::Vector Uz;
            Ux = ( V4 / V4.Norm() );
            Uz = KDL::Vector(0.0, 0.0, 1.0);
            if ( ClosestPoints.P2 == Point4 )
            {
                Uz = KDL::Vector(0.0, 0.0, -1.0);
            }
            Rot_to_collision_point = KDL::Rotation(Ux, Uz * Ux, Uz);
            KDL::Frame pos_final;
            pos_final = KDL::Frame( Rot_to_collision_point, collision_point_on_line) *
                        KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector(pos_on_plane, 0.0, 0.0) );
            collision_point = pos_final.p;

        }

    }
    else
    {
        // ROS_INFO_STREAM("Collision Point on the cylindrical face");

        KDL::Vector Uz(V3 / V3.Norm() );
        KDL::Vector Ux(0.0, 0.0, 1.0);
        Rot_to_collision_point = KDL::Rotation(Ux, Uz * Ux, Uz);
        KDL::Vector collision_point_on_line(ClosestPoints.P2[0], ClosestPoints.P2[1], ClosestPoints.P2[2] );
        pos_final = KDL::Frame( Rot_to_collision_point, collision_point_on_line) *
                    KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector(0.0, 0.0, radius) );
        collision_point = pos_final.p;

    }
    T_CollisionPoint.M = Rot_to_collision_point;
    T_CollisionPoint.p = collision_point;
    LineCollisions::Point Point5( T_CollisionPoint.p.x(), T_CollisionPoint.p.y(), T_CollisionPoint.p.x() );
}

}



PLUGINLIB_EXPORT_CLASS(desperate_housewife::PotentialFieldControlKinematicReverse, controller_interface::ControllerBase)

