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

    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }

    KDL::JntArray vel_repulsive;
    vel_repulsive.resize(7);
    SetToZero(vel_repulsive);

    tf::twistKDLToMsg(x_err_, error_id.error_);

    // flag to use this code with real robot

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
                    F_obj_local = getRepulsiveForceObjects(fk_chain, influence_local, Object_position[k], Object_radius[k], Object_height[k] );
                    F_obj = F_obj + F_obj_local;
                    if (F_obj_local.norm() != 0.0)
                    {
                        num_of_links_in_potential = num_of_links_in_potential + 1.0;
                    }
                    ++id_global;
                }

                F_obj_base_link = Adjoint * F_obj;

                F_obj_base_total += F_obj_base_link;



                F_table = getRepulsiveForceTable(fk_chain, parameters_.pf_dist_to_table );
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

            double v_limited = VelocityLimit(x_dot_d, parameters_.vel_limit_robot);

            x_err_integral += x_err_ * period.toSec();
            for (int i = 0; i < F_attractive.size(); i++)
            {
                F_attractive(i) =  -parameters_.k_d(i, i) * ( x_dot_(i) -  v_limited * x_dot_d(i) ) + parameters_.k_i(i, i) * x_err_integral(i);
                // F_attractive(i) =  parameters_.k_p(i, i) * x_err_(i);
            }
        }

        Eigen::MatrixXd x_err_eigen_ = Eigen::MatrixXd::Zero(6, 1);

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
        }

        saurateJointVelocities( joint_des_states_filtered.qdot, parameters_.max_vel_percentage);

        x_err_msg = diff(x_, x_des_int);

        tf::twistKDLToMsg(x_err_msg, error_id.error_);

        for (unsigned int i = 0; i < joint_handles_.size(); i++)
        {
            joint_des_states_.q(i) += period.toSec() * joint_des_states_filtered.qdot(i);
        }

        saurateJointPositions( joint_des_states_.q, 2.0 * M_PI / 180.0 );


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

void PotentialFieldControlKinematicReverse::command(const desperate_housewife::handPoseSingle::ConstPtr& msg)
{
    KDL::Frame frame_des_;
    tf::poseMsgToKDL(msg->pose, frame_des_);


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

    error_id.id = msg->id;
}

void PotentialFieldControlKinematicReverse::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
    Object_radius.clear();
    Object_height.clear();
    Object_position.clear();

    for (unsigned int i = 0; i < msg->geometries.size(); i++)
    {
        KDL::Frame frame_obj;
        Object_radius.push_back(msg->geometries[i].info[0]);
        Object_height.push_back(msg->geometries[i].info[1]);
        tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
        Object_position.push_back(frame_obj);
    }

    ROS_INFO("New environment: No of Obstacles %lu", msg->geometries.size());
}


Eigen::Matrix<double, 6, 1> PotentialFieldControlKinematicReverse::getRepulsiveForceObjects(KDL::Frame &T_in, double influence, KDL::Frame &Object_pos, double radius, double height)
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
    arrows_total.markers.push_back(Force2MarkerArrow(  Nolmal_to_Cylinder_kdl, T_CollisionPoint.p, id_global));

    LineCollisions::Point Point1(T_in.p.x(), T_in.p.y(), T_in.p.z());
    LineCollisions::Point Point2(T_CollisionPoint.p.x(), T_CollisionPoint.p.y(), T_CollisionPoint.p.z());
    LineCollisions::Line ClosestPoints(Point1, Point2);

    if ( distance <= influence)
    {
        lines_total.markers.push_back(Line2markerLine(ClosestPoints, 1.0, 0.0, 0.0, id_global));
        ForceAndIndex = GetFIRAS(distance, Nolmal_to_Cylinder, influence, parameters_.pf_repulsive_gain_obstacles);
    }
    else
    {
        lines_total.markers.push_back( Line2markerLine(ClosestPoints, 0.0, 1.0, 0.0, id_global) );
    }


    Eigen::Matrix<double, 6

    , 1> force_local_link = Eigen::Matrix<double, 6, 1>::Zero();
    force_local_link = getAdjointT( T_in.Inverse() * Object_pos) * ForceAndIndex;

    tf::Transform CollisionTransform;
    tf::TransformKDLToTF( T_CollisionPoint, CollisionTransform);
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( CollisionTransform, ros::Time::now(), "world", "collision_point") );

    return force_local_link;

}

Eigen::Matrix<double, 6, 1> PotentialFieldControlKinematicReverse::getRepulsiveForceTable(KDL::Frame &T_in, double influence)
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
    parameters_.k_p = getGainMatrix(std::string("k_p"), n, 6);
    parameters_.k_d = getGainMatrix(std::string("k_d"), n, 6);
    parameters_.k_i = getGainMatrix(std::string("k_i"), n, 6);

    ROS_INFO_STREAM("k_p" << std::endl << parameters_.k_p);
    ROS_INFO_STREAM("k_d" << std::endl << parameters_.k_d);
    ROS_INFO_STREAM("k_i" << std::endl << parameters_.k_i);

    ROS_INFO("Parameters loaded");
    return;
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

Eigen::Matrix<double, 7, 1> PotentialFieldControlKinematicReverse::potentialEnergy(KDL::JntArray q)
{
    KDL::JntArray G_local(7);
    id_solver_->JntToGravity(joint_msr_states_.q, G_local);

    return parameters_.gain_null_space * G_local.data ;
}

Eigen::Matrix<double, 7, 1> PotentialFieldControlKinematicReverse::JointLimitAvoidance(KDL::JntArray q, double gain)
{   // This function implements joint limit avoidance usung the penalty function V = \sum\limits_{i=1}^n\frac{1}{s^2} s = q_{l_1}-|q_i|
    Eigen::Matrix<double, 7, 1> tau_limit_avoidance = Eigen::Matrix<double, 7, 1>::Zero();
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
            tau_limit_avoidance(i) = - gain *  KDL::sign(q.data[i]) * potential;
        }
        else
        {
            tau_limit_avoidance(i) = 0.0;
        }
    }

    return tau_limit_avoidance;
}


}



PLUGINLIB_EXPORT_CLASS(desperate_housewife::PotentialFieldControlKinematicReverse, controller_interface::ControllerBase)

