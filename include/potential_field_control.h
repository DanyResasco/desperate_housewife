#ifndef POTENTIAL_FIELD_CONTROL_H
#define POTENTIAL_FIELD_CONTROL_H

#include <lwr_controllers/MultiPriorityTask.h>
#include <lwr_controllers/PIDKinematicChainControllerBase.h>
#include <desperate_housewife/handPoseSingle.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <iostream>

/*!Tf*/
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>

#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <desperate_housewife/Error_msg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <interpolationmb.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry> 


#include <control_toolbox/filters.h>

namespace desperate_housewife
{
	
	static int Int(0);

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
		void setTreshold(double tresholdin){treshold_influence=tresholdin;}
		void setNi(double Niin){Ni_=Niin;}
		
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
		// Eigen::Matrix<double,7,1> GetRepulsiveForce(std::vector<KDL::Frame> &Pos_now);
		
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
		 // std::pair<Eigen::Matrix<double,6,1>, double> RepulsiveWithTable(std::vector<double> distance_local_obj);
	  Eigen::Matrix<double,7,1> RepulsiveWithTable();

	   Eigen::Matrix<double,7,1>  GetRepulsiveWithObstacle();
		
		
		/** Caalback: InfoOBj
		* input: desperate message with information about obstacle to remove
		* Description: callback that save the obstacle position
		*/
		// void InfoOBj( const desperate_housewife::fittedGeometriesSingle::ConstPtr& obj_rem);
		
		/** Caalback: set_gains
		* input: ros message
		* Description: callback that change inline the gains
		*/
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);

		/** Caalback: command_start
		* input: ros message
		* Description: callback for start the code with real robot
		*/
		void command_start(const std_msgs::Bool::ConstPtr& msg);
		

		/** Function: PoseDesiredInterpolation
		* input: desired pose
		* Description: this function save the position of endeffector, desired pose, and updaes interpolate times when msg arrived
		*/
		void PoseDesiredInterpolation(KDL::Frame frame_des_);

		/** Function: GetPartialDerivate
		* input: object frame, link position, object radius and height 
		* output: object derivate
		* Description: calculates the object derivate in cylinder frame 
		*/
		Eigen::Vector3d GetPartialDerivate(KDL::Frame &T_v_o, KDL::Vector &Point_v, double &radius, double &height);
		
		/** Function: GetFIRAS
		* input: min distance, object derivate, influence of repulsive field
		* output: repulsive forces
		* Description: calculates the repulsive forces  like article O.Khatib 
		*/
		Eigen::Matrix<double,6,1> GetFIRAS(double &min_distance, Eigen::Vector3d &distance_der_partial , double &influence);

		/** Function: GetMinDistance
		* input: vecotr of distance, influence of repulsive field
		* output: vector with data[0] information if there is a link with distance minor tha influnce, data[1] the minor distance, data[2] index of vector
		* Description: return the minor distance from a input vector
		*/
		std::vector<double> GetMinDistance(std::vector<double> distance_local_obj,  double influence );
		
		/** Function: GetRepulsiveForce
		* input: vector with the interested point, object position, influence of repulsive field, object radius and height
		* output: repulsive force and the index for the jacobian
		* Description: functions tha call the funciont for calculates the repulsive forces 
		*/
		// std::pair<Eigen::Matrix<double,6,1>, double> GetRepulsiveForce(std::vector<KDL::Vector> &point_, double influence, KDL::Frame &Object_pos, double radius, double height);
		
		Eigen::Matrix<double,6,1> GetRepulsiveForce(KDL::Vector &point_, double influence, KDL::Frame &Object_pos, double radius, double height);


		// void SeeMarker(KDL::Frame &Pos, std::string obst_name);

		/** Function: VelocityLimit
		* input: vector with the interested point, object position, influence of repulsive field, object radius and height
		* output: repulsive force and the index for the jacobian
		* Description: functions tha call the funciont for calculates the repulsive forces 
		*/
		double VelocityLimit(KDL::Vector &x_dot_d);

		Eigen::Matrix<double,6,6> getAdjointT( KDL::Frame Frame_in);
		Eigen::Matrix<double,3,3> getVectorHat(Eigen::Matrix<double,3,1> vector_in);
		Eigen::Matrix<double,6,1> GetRepulsiveForceTable(KDL::Vector &point_, double influence);
		KDL::JntArray test_joint_limits_mb(KDL::JntArray q);
		
	private:
		ros::Subscriber sub_command_, sub_command_start;
		ros::Publisher pub_error_,  pub_tau_;
		ros::Publisher pub_pose_;
		tf::TransformBroadcaster tf_now_hand;

		// std::stringstream sstr_;
		std::string desired_reference_topic, desired_hand_name, desired_hand_topic;
        
		KDL::JntArray qdot_last_;

		KDL::Frame x_;	/*!current e-e pose*/

		
		Eigen::Matrix<double,6,1> x_dot_;	/*!current e-e velocity*/

		KDL::Frame x_des_;	/*!desired pose interpolate*/
		KDL::Frame x_des_int;	/*!desired pose*/
		KDL::Frame x_now_int; /*!position robot for interpolate*/

		KDL::Twist x_err_,x_err_last;	/*!error*/

		KDL::JntArray Kp_,Kd_,Ki_;	/*!gains*/

		KDL::JntArray tau_;	/*!tau*/

		KDL::JntSpaceInertiaMatrix M_;	/*! intertia matrix*/
		KDL::JntArray C_;	/*! coriolis*/
		KDL::JntArray G_;	/*! gravity*/

		KDL::Jacobian J_;	/*!Jacobian J(q)*/
		KDL::Jacobian J_last_;	/*!Jacobian of the last step*/
		KDL::Jacobian J_dot_;	/*!d/dt(J(q))*/
		KDL::Jacobian J_star_; /*! it will be J_*P_*/

		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> N_trans_;
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;	/*!M in operation space*/
		Eigen::MatrixXd lambda_;	/*!omega_inv*/
		Eigen::Matrix<double,6,1> b_;

		double phi_;	
		double phi_last_;
		int first_step_;

		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		

		Eigen::Matrix<double,6,1> Force_attractive;

		Eigen::Matrix<double,7,1> Force_total_rep;
		Eigen::Matrix<double,7,1> Force_repulsive_prev;
		Eigen::Matrix<double,7,1> tau_repulsive;
		Eigen::Matrix<double,7,1> F_Rep_table;
		Eigen::Matrix<double,7,1> Force_repulsive;
		double V_max_kuka = 1.5;
		std::vector<KDL::Frame> x_chain;	/*!14 is soft_hand (end_effector)*/
		
		ros::Subscriber obstacles_subscribe_, obstacles_remove_sub, sub_gains_;
		std::vector<double> Object_radius;
		std::vector<double> Object_height;
		std::vector<KDL::Frame> Object_position;
		ros::Publisher hand_publisher_ ,pub_xdes_;
		std::vector<KDL::Jacobian> JAC_repulsive;	/*!vector with all jabobian*/
		std::string obstacle_remove_topic, obstacle_avoidance;
		desperate_housewife::Error_msg error_pose_trajectory;
		int ObjOrObst;
		std::string tip_name, object_names_,set_gains_;
		std::string error_topic;
		ros::Publisher pub_error_right, pub_error_left,pub_qdot_;
		std::string tau_commad;

		/*!information for message*/
		int erro_arr, err_obj, err_home ;
		double time_inter; /*!time to interpolate*/
		double T_des; /*!time desired to interpolate*/
		/*! int Int = 0;*/
		double time_inter_jerk; 

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_now, quat_des_;

		tf::Quaternion quat_tf;
		double percentage;
		tfScalar Time;
		int a ;
		bool start_flag;
		ros::Publisher pub_Freptavolo_, pub_Fa_, pub_f_total_, pub_diff,pub_xdot,pub_sing_val;
		std::vector<KDL::Frame> test_pos_jerk;
		Eigen::Matrix<double,6,1> Force_attractive_last,  Force_repulsive_last;
		Eigen::Matrix<double,7,1> Force_total_rep_last;
		bool switch_trajectory;
		double Time_traj, Time_traj_rep;
		KDL::JntArray tau_prev_;
		ros::Subscriber sub_grid_;
		std::vector<int> list_of_link;

		std::string point_;
		ros::Subscriber sub_force_point_;
		ros::Publisher vis_pub,pub_Fr_,pub_velocity_,pub_error_int_;
		tf::TransformBroadcaster tf_geometriesTransformations_;
		KDL::Twist x_err_integral;

		double Time_log;
		double Ni_, treshold_influence, Repulsive_table;

		std::vector<std::string> list_of_links_pf;
		std::vector<KDL::Chain> list_of_chains_pf;
		std::vector<KDL::ChainFkSolverPos_recursive> list_of_fk_pf;

	};

	/** Function: FromKdlToEigen
		* input: kdl frame
		* output: eigen matrix
		* Description: functions tha transform kdl frame into eigen matrix
	*/
	Eigen::Matrix<double,4,4>  FromKdlToEigen(KDL::Frame &T_v_o);
	  


}




#endif


