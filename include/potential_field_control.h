
#ifndef POTENTIAL_FIELD_CONTROL_H
#define POTENTIAL_FIELD_CONTROL_H

//ros
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
// #include <desperate_housewife/cyl_info.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/JointState.h>
//#include <vito-robot/kuka-lwr/lwr_controllers/include/lwr_controllers/KinematicChainControllerBase.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <iostream>
#include <utility>
#include <list>
#include <string>
#include <math.h>
#include <vector>
#include <urdf/model.h>
#include <pluginlib/class_list_macros.h>
#include <sstream> 
//msg
// #include <desperate_housewife/cyl_info.h>
// #include <desperate_housewife/hand.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry> 


//Tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>


// KDL
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
// #include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

namespace desperate_housewife
{

	class PotentialFieldControl: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		private:

			ros::NodeHandle nodeH;
			ros::NodeHandle nh_;
			tf::TransformListener listener_tf;
			ros::Subscriber sub_;
	  	
	  		std::vector<int> whichArm;
			bool check_urdf;
			// pcl::PointXYZ goal_position;
			std::vector<KDL::Vector> obstacle_position;
			// Eigen::Matrix4d frame_cylinder; //M_c_k
			double P_hand = 0.5;
			double P_obj = -1;
			double P_goal = 1;	
			double influence = 0.25;	// limit distance of the potential filed influence
			Eigen::VectorXd Force;
			double dissipative;
			std::vector<double> distance;
			std::vector<Eigen::VectorXd> Force_attractive_left,Force_attractive_right;
			std::vector<Eigen::VectorXd> Force_repulsion_left, Force_repulsion_right;
			bool Arm;
			double velocity_max;
			double v_lim = 1;
			bool check_robot;
			double max_radius=0.1, max_lenght=0.20;
			// std::string root_name;
		
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
			boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			std::vector<KDL::Frame> x_,  x_now;
			KDL::Frame x0_ ;	
			std::vector<KDL::Frame> x_des_;	//desired pose
			KDL::Twist x_des_dot_;
			KDL::Twist x_des_dotdot_;
			Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity
			KDL::Twist x_err_;
			KDL::JntArray tau_;

			Eigen::Matrix<double,7,7> I_;
			Eigen::Matrix<double,7,7> N_trans_;
			int step_;
			int first_step_;
			// int msg_id_;
			int cmd_flag_;
			double phi_;	
			double phi_last_;
			// int p;
			tf::TransformListener info;

		

			// struct limits_
			// {
			// 	KDL::JntArray min;
			// 	KDL::JntArray max;
			// 	KDL::JntArray center;
			// } joint_limits_;


			struct Robot
			{	

				// std::vector<double> robot_position_left, robot_position_right; //in joint space is a point
				// Eigen::VectorXd Vel_l, Vel_r;
				// std::vector<KDL::Jacobian> link_jac_l, link_jac_r;
				// std::vector<KDL::Frame> link_frame_l, link_frame_r;
				// std::vector<KDL::ChainJntToJacSolver*> link_jac_solver_;
				// std::vector<KDL::ChainFkSolverPos_recursive*> link_fk_solver_;
				// std::vector<KDL::JntArray> link_joints_;
				// std::vector<KDL::Chain> link_chain_;
				
				std::vector<Eigen::VectorXd> Pos_HAND_r_xd, Pos_HAND_l_xd;
				KDL::Frame Pos_HAND_r_x, Pos_HAND_l_x;
				// Eigen::Matrix4d Pos_final_hand_r, Pos_final_hand_l;
				KDL::Frame pos_base_l, pos_base_r;
				 //make a model
				// KDL::Tree Robot_tree;

				
				std::vector<hardware_interface::JointHandle> joint_handles_;
				KDL::Chain kdl_chain_;
				KDL::JntArrayVel joint_msr_states_, joint_des_states_;	// joint states (measured and desired)
				KDL::JntArray qdot_last_;
				KDL::Jacobian J_;	//Jacobian J(q)
				KDL::Jacobian J_last_;	//Jacobian of the last step
				KDL::Jacobian J_dot_;	//d/dt(J(q))
				KDL::Jacobian J_star_; // it will be J_*P_
				KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
				KDL::JntArray C_;	// coriolis
				KDL::JntArray G_;	// gravity


			} Vito_desperate;

		public:

			void MPCallback(const desperate_housewife::hand::ConstPtr& hand_msg);		
			void SetPotentialField_robot(Eigen::VectorXd &Force_repulsion, int p);
			void SetAttractiveField(KDL::Frame &pos_Hand_xd, KDL::JntArray &Vel, KDL::Frame &Pos_hand_x, Eigen::VectorXd &Force_attractive,  KDL::Jacobian &link_jac_);
			void SetRepulsiveFiled(KDL::Vector &Pos, std::vector<KDL::Frame> &Pos_now, Eigen::VectorXd &Force_repulsion );
			bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
			void starting(const ros::Time& time);
			void update(const ros::Time& time, const ros::Duration& period);


			PotentialFieldControl();
			//{
				
				// ros::param::get("~influence", influence);
				// ROS_INFO("influence %lf", influence);
				// ros::param::get("~velocity_max", velocity_max);
				// ROS_INFO("velocity_max %lf", velocity_max);
				// ros::param::get("~max_radius", max_radius);
				// ROS_INFO("max_radius %lf", max_radius);
				// ros::param::get("~max_lenght", max_lenght);
				// ROS_INFO("max_lenght %lf", max_lenght);
				// check_robot  = false;

			//};


			~PotentialFieldControl();







	};



	// Eigen::Matrix4d FromTFtoEigen(tf::StampedTransform &object);
	KDL::Frame FromTFtoKDL(tf::StampedTransform &st_transf);

}












#endif
