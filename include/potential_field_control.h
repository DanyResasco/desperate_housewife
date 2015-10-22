#ifndef POTENTIAL_FIELD_CONTROL_H
#define POTENTIAL_FIELD_CONTROL_H

// #include <lwr_controllers/PIDKinematicChainControllerBase.h>
#include <lwr_controllers/MultiPriorityTask.h>
#include <lwr_controllers/PIDKinematicChainControllerBase.h>
#include <desperate_housewife/handPoseSingle.h>
//#include <lwr_controllers/PoseRPY.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <iostream>

//Tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>

#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <desperate_housewife/Error_msg.h>





namespace desperate_housewife
{
	class PotentialFieldControl: public controller_interface::PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		PotentialFieldControl();
		~PotentialFieldControl();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		// void command(const lwr_controllers::PoseRPY::ConstPtr &msg);
		void command(const desperate_housewife::handPoseSingle::ConstPtr& msg);
		void set_marker(KDL::Frame x, int id);
		double task_objective_function(KDL::JntArray q);


		//void SetAttractiveField();
		Eigen::Matrix<double,7,1> GetRepulsiveForce(std::vector<KDL::Frame> &Pos_now);
		void InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
		Eigen::Matrix<double,7,1> RepulsiveWithTable(std::vector<KDL::Frame> &Pos_arm);
		void InfoOBj( const desperate_housewife::fittedGeometriesSingle::ConstPtr& obj_rem);

	private:
		ros::Subscriber sub_command_;
		ros::Publisher pub_error_;
		ros::Publisher pub_pose_;
		ros::Publisher pub_marker_;
		tf::TransformBroadcaster tf_now_hand;

		std_msgs::Float64MultiArray msg_err_;
		std_msgs::Float64MultiArray msg_pose_;
		visualization_msgs::Marker msg_marker_;
		// std::stringstream sstr_;
		std::string desired_reference_topic, desired_hand_name, desired_hand_topic;
        
		KDL::JntArray qdot_last_;

		KDL::Frame x_,x0_;	//current e-e pose
		
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity

		KDL::Frame x_des_;	//desired pose
		KDL::Twist x_des_dot_;
		KDL::Twist x_des_dotdot_;

		KDL::Twist x_err_;

		KDL::JntArray Kp_,Kd_;

		KDL::JntArray tau_;

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))
		KDL::Jacobian J_star_; // it will be J_*P_

		Eigen::MatrixXd J_pinv_;

		// Eigen::Matrix<double,6,1> e_ref_;
		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> N_trans_;
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;
		Eigen::MatrixXd lambda_;
		Eigen::Matrix<double,6,1> b_;

		double phi_;	
		double phi_last_;

		int step_;
		int first_step_;
		int msg_id_;
		int cmd_flag_;
		// int ntasks_;
		// bool on_target_flag_;
		// int links_index_;


		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		
		//dany
		Eigen::Matrix<double,6,1> Force_attractive;
		Eigen::Matrix<double,7,1> Force_total_rep;
		Eigen::Matrix<double,7,1> Force_repulsive;
		Eigen::Matrix<double,7,1> F_Rep_table;
		double V_max_kuka = 1.5;
		std::vector<KDL::Frame> x_chain;	//14 is soft_hand (end_effector)
		
		ros::Subscriber obstacles_subscribe_, obstacles_remove_sub;
		std::vector<double> Object_radius;
		std::vector<double> Object_height;
		std::vector<KDL::Frame> Object_position;
		ros::Publisher hand_publisher_;
		int hand_step;
		std::vector<KDL::Jacobian> JAC_repulsive;
		std::string obstacle_remove_topic, obstacle_avoidance;
		desperate_housewife::Error_msg error_pose_trajectory;
		//int WhichArm_obj_remove;
		bool ObjOrObst = true;
		
		
	};
}

#endif






