#ifndef POTENTIAL_FIELD_CONTROL_H
#define POTENTIAL_FIELD_CONTROL_H

#include <lwr_controllers/MultiPriorityTask.h>
#include <lwr_controllers/PIDKinematicChainControllerBase.h>
#include <desperate_housewife/handPoseSingle.h>
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
#include <geometry_msgs/WrenchStamped.h>
#include <interpolationmb.h>






namespace desperate_housewife
{
	class PotentialFieldControl: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		PotentialFieldControl();
		~PotentialFieldControl();

		/** ros function for control
		*/
		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const desperate_housewife::handPoseSingle::ConstPtr& msg);
		
		/** Function: task_objective_function
		* input: position 
		* output: double
		* Description: with function calculates the position more distance than robot's limits
		*/
		double task_objective_function(KDL::JntArray q);


		/** Function: GetRepulsiveForce
		* input: position of all joint
		* output: force repulsive 
		* Description: this function calculates the repulsive force between robot arm and object
		*/
		Eigen::Matrix<double,7,1> GetRepulsiveForce(std::vector<KDL::Frame> &Pos_now);
		
		/** Caalback: InfoGeometry
		* input: desperate message with obstacle information
		* Description: callback that save the obstacle position
		*/
		void InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
		
		/** Function: RepulsiveWithTable
		* input: position of all joint
		* output: force repulsive 
		* Description: this function calculates the repulsive force between robot arm and table
		*/
		Eigen::Matrix<double,7,1> RepulsiveWithTable(std::vector<KDL::Frame> &Pos_arm);
		
		/** Caalback: InfoOBj
		* input: desperate message with information about obstacle to remove
		* Description: callback that save the obstacle position
		*/
		void InfoOBj( const desperate_housewife::fittedGeometriesSingle::ConstPtr& obj_rem);
		
		/** Caalback: set_gains
		* input: ros message
		* Description: callback that change inline the gains
		*/
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);
		

		/** Function: PoseDesiredInterpolation
		* input: desired pose
		* Description: this function save the position of endeffector and desired pose when msg arrived
		*/
		void PoseDesiredInterpolation(KDL::Frame frame_des_);



	private:
		ros::Subscriber sub_command_;
		ros::Publisher pub_error_,  pub_tau_;
		ros::Publisher pub_pose_;
		tf::TransformBroadcaster tf_now_hand;

		// std::stringstream sstr_;
		std::string desired_reference_topic, desired_hand_name, desired_hand_topic;
        
		KDL::JntArray qdot_last_;

		KDL::Frame x_;	//current e-e pose

		
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity

		KDL::Frame x_des_;	//desired pose interpolate
		KDL::Frame x_des_int;	//desired pose
		KDL::Frame x_now_int; //position robot for interpolate

		KDL::Twist x_err_;	//error

		KDL::JntArray Kp_,Kd_;	//gains

		KDL::JntArray tau_;	//tau

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))
		KDL::Jacobian J_star_; // it will be J_*P_

		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> N_trans_;
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;	//M in operation space
		Eigen::MatrixXd lambda_;	//omega_inv
		Eigen::Matrix<double,6,1> b_;

		double phi_;	
		double phi_last_;
		int first_step_;

		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		

		Eigen::Matrix<double,6,1> Force_attractive;
		Eigen::Matrix<double,7,1> Force_total_rep;
		Eigen::Matrix<double,7,1> Force_repulsive;
		Eigen::Matrix<double,7,1> F_Rep_table;
		double V_max_kuka = 1.5;
		std::vector<KDL::Frame> x_chain;	//14 is soft_hand (end_effector)
		
		ros::Subscriber obstacles_subscribe_, obstacles_remove_sub, sub_gains_;
		std::vector<double> Object_radius;
		std::vector<double> Object_height;
		std::vector<KDL::Frame> Object_position;
		ros::Publisher hand_publisher_;
		std::vector<KDL::Jacobian> JAC_repulsive;	//vector with all jabobian
		std::string obstacle_remove_topic, obstacle_avoidance;
		desperate_housewife::Error_msg error_pose_trajectory;
		int ObjOrObst;
		// geometry_msgs::WrenchStamped wrench_msg;
		// geometry_msgs::WrenchStamped wrench_msg_rep;
		// ros::Publisher publisher_wrench_command,publisher_wrench_command_rep ;
		std::string tip_name, object_names_,set_gains_;
		std::string error_topic;
		ros::Publisher pub_error_right, pub_error_left;
		std::string tau_commad;
		std_msgs::Float64MultiArray tau_msg;
		//information for message
		int erro_arr, err_obj, err_home ;
		double time_inter; //time to interpolate
		double T_des; //time desired to interpolate
		int Int = 0; 

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_now, quat_des_;

		tf::Quaternion quat_tf;
		double percentage;
		tfScalar Time;
		int a ;
		
		
	};
}

#endif






