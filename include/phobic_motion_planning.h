#ifndef dany_bicchi_MP_h
#define dany_bicchi_MP_h

//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <desperate_housewife/cyl_info.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/JointState.h>
//#include <vito-robot/kuka-lwr/lwr_controllers/include/lwr_controllers/KinematicChainControllerBase.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <utility>
#include <list>
#include <string>

//msg
#include <desperate_housewife/cyl_info.h>
#include <desperate_housewife/hand.h>

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
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// Gazebo
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>
// #include <urdf/model.h> 

class phobic_mp
{
	private:

		ros::NodeHandle nodeH;

		tf::TransformListener listener_info;
		// Node Handles
  		ros::NodeHandle model_nh_; // namespaces to robot name
  		// Strings
		std::string robot_namespace_;
  		std::string robot_description_;
  		std::vector<std::string> link_names_;
  		//geometry_msgs/Pose[] hand_pose_desired;
  		std::vector<int> whichArm;
		
		pcl::PointXYZ goal_position;
		std::vector<Eigen::Vector3d> obstacle_position;
		Eigen::Matrix4d frame_cylinder; //M_c_k
		double P_hand = 0.5;
		double P_obj = -1;
		double P_goal = 1;	
		double influence ;	// limit distance of the potential filed influence
		Eigen::VectorXd Force;
		double dissipative;
		std::vector<double> distance;
		Eigen::VectorXd Force_attractive;
		Eigen::VectorXd Force_repulsion;
		bool Arm;
		double velocity_max;
		//double v_lim;
		bool check_robot;
		double max_radius=0.1, max_lenght=0.20;
		std::string root_name;
		std::vector<KDL::Frame> Hand_pose_kdl_l, Hand_pose_kdl_r;

		struct Robot
		{	
			//sensor_msgs::JointState joint_state;
			//std::vector<tf::StampedTransform> Link_right;
			//std::vector<tf::StampedTransform> Link_left;
			std::vector<tf::StampedTransform>  left_arm_softhand_st , right_arm_softhand_st ;
			tf::StampedTransform right_arm_base_link_st, left_arm_base_link_st;
			std::vector<double> robot_position_left, robot_position_right; //in joint space is a point
			std::vector<double> Vel_l, Vel_r;
			std::vector<KDL::Jacobian> link_jac_l, link_jac_r;
			std::vector<KDL::Frame> link_frame_l, link_frame_r;
			std::vector<KDL::ChainJntToJacSolver*> link_jac_solver_;
			std::vector<KDL::ChainFkSolverPos_recursive*> link_fk_solver_;
			std::vector<KDL::JntArray> link_joints_;
			std::vector<KDL::Chain> link_chain_;
			//std::vector<pcl::PointXYZ> robot_position_left, robot_position_right;
			std::vector<Eigen::VectorXd> Pos_HAND_r_xd,Pos_HAND_l_xd, Pos_HAND_r_x, Pos_HAND_l_x;
			Eigen::Matrix4d Pos_final_hand_r, Pos_final_hand_l;
			Eigen::VectorXd pos_base_l, pos_base_r;
			 //make a model
			KDL::Tree Robot_tree;
    		//urdf::Model urdf_model;
    		//std::vector< KDL::Jacobian > link_left_jac, link_right_jac; 
			
		} Vito_desperate;


	
	public:

		

		ros::Subscriber  joint_listen;
		void Esegui();
		void MPCallback(const desperate_housewife::hand hand_msg);		
		double SetrepulsiveField( tf::StampedTransform &object, int &p);
		void SetPotentialField_robot(std::vector<Eigen::VectorXd> &Force_repulsion, int &p);
		bool objectORostacles();
		//void Calculate_force();
		void SetAttractiveField( int &p, int &i);
		void SetPotentialField( tf::StampedTransform &object);
		void SetRepulsiveFiled(pcl::PointXYZ &Pos);
		//pcl::PointXYZ  Take_Pos(tf::StampedTransform &M_tf);
		Eigen::Vector3d GetDistance(Eigen::VectorXd &obj1, Eigen::VectorXd &obj2);
		double SetLimitation(Eigen::VectorXd &vel_d);
		void SetCommandVector();
		Eigen::Vector3d Take_Pos(tf::StampedTransform &M_tf);
		bool GetVitoUrdf();
		void GetEuleroAngle(KDL::Frame &Hand_kdl_xd, Eigen::VectorXd pos_cartisian);
		void Robot_Callback_left(sensor_msgs::JointState msg);
		void Robot_Callback_right(sensor_msgs::JointState msg);
		//void Robot_Callback_right(sensor_msgs::JointState msg);
		//void SetRobotPosition(std::vector<double> msg.position);
		void GetJacobian(int &p);
		void SetPseudoInvJac();
		std::string getURDF(std::string param_name);




		phobic_mp(ros::NodeHandle node_mp): nodeH(node_mp)
		{
			
			ros::param::get("~influence", influence);
			ROS_INFO("influence %lf", influence);
			ros::param::get("~velocity_max", velocity_max);
			ROS_INFO("velocity_max %lf", velocity_max);
			ros::param::get("~max_radius", max_radius);
			ROS_INFO("max_radius %lf", max_radius);
			ros::param::get("~max_lenght", max_lenght);
			ROS_INFO("max_lenght %lf", max_lenght);
			check_robot  = false;
			//LINK_jac_= KDL::Jacobian(link_chain_[i].getNrOfJoints());
			//link_jac_ = Link_jack;
			// tf::TransformListener listener_info;
			// ros::Subscriber  joint_lissen;




		}


		~phobic_mp(){};







};



Eigen::Matrix4d FromTFtoEigen(tf::StampedTransform &object);
KDL::Frame FromTFtoKDL(tf::StampedTransform st_transf);













#endif