#ifndef POTENTIAL_FIELD_CONTROL_KINEMATIC_H
#define POTENTIAL_FIELD_CONTROL_KINEMATIC_H

#include <lwr_controllers/MultiPriorityTask.h>
#include <lwr_controllers/PIDKinematicChainControllerBase.h>
#include <desperate_housewife/handPoseSingle.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/WrenchStamped.h>

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
#include <std_srvs/Empty.h>

namespace desperate_housewife
{

static int Int(0);

class PotentialFieldControlKinematic: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
{
public:
	PotentialFieldControlKinematic();
	~PotentialFieldControlKinematic();

	/** ros function for control
	*/
	bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
	void starting(const ros::Time& time);
	void update(const ros::Time& time, const ros::Duration& period);
	void command(const desperate_housewife::handPoseSingle::ConstPtr& msg);
	void load_parameters(ros::NodeHandle &n);
	Eigen::Matrix<double, 6, 1> GetFIRAS(double min_distance, Eigen::Vector3d &distance_der_partial , double influence);
	Eigen::Matrix<double, 6, 1> GetRepulsiveForce(KDL::Frame &T_in, double influence, KDL::Frame &Object_pos, double radius, double height);


private:

	KDL::JntArray qdot_last_;

	KDL::Frame x_;	/*!current e-e pose*/


	Eigen::Matrix<double, 6, 1> x_dot_;	/*!current e-e velocity*/

	KDL::Frame x_des_;	/*!desired pose interpolate*/
	KDL::Frame x_des_int;	/*!desired pose*/
	KDL::Frame x_now_int; /*!position robot for interpolate*/

	KDL::Twist x_err_, x_err_last;	/*!error*/

	// KDL::JntArray Kp_,Kd_,Ki_;	/*!gains*/

	KDL::Jacobian J_;	/*!Jacobian J(q)*/


	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
	boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
	boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;


	// ros::Subscriber obstacles_subscribe_, obstacles_remove_sub, sub_gains_;
	std::vector<double> Object_radius;
	std::vector<double> Object_height;
	std::vector<KDL::Frame> Object_position;

	double time_inter; /*!time to interpolate*/
	// double T_des; /*!time desired to interpolate*/
	/*! int Int = 0;*/
	tfScalar Time;

	struct quaternion_
	{
		KDL::Vector v;
		double a;
	} quat_curr_, quat_des_, quat_now, q_e_c_des, q_e_now;

	Eigen::Matrix<double,3,3> skew_;
		KDL::Vector v_temp_;

	tf::Quaternion quat_tf;
	// double percentage;

	double Time_traj, Time_traj_rep;

	KDL::Twist x_err_integral;


	struct parameters
	{
		Eigen::Matrix<double, 6, 6> k_p;
		Eigen::Matrix<double, 6, 6> k_d;
		Eigen::Matrix<double, 6, 6> k_i;
		std::string root_name, tip_name;
		double pf_dist_to_obstacles, pf_dist_to_table, pf_repulsive_gain;
		double max_time_interpolation, max_tau_percentage, vel_limit_robot, gain_null_space;
		std::vector<std::string> pf_list_of_links;
		std::vector<KDL::Chain> pf_list_of_chains;
		std::vector<KDL::ChainFkSolverPos_recursive> pf_list_of_fk;
		std::vector<KDL::ChainJntToJacSolver> pf_list_of_jac;
		bool enable_obstacle_avoidance, enable_joint_limits_avoidance, enable_attractive_field, enable_null_space, enable_interpolation;
		int id_arm;
	} parameters_;

	std::string  topic_obstacle_avoidance, topic_desired_reference;
	ros::Publisher pub_error, pub_tau, pub_pf_repulsive_forse, pub_pf_attractive_force, pub_pf_total_force, pub_total_wrench;
	ros::Subscriber sub_command, sub_obstacles, sub_start_controller;
	ros::ServiceServer srv_start_controller;

	ros::Publisher pub_error_id;
	desperate_housewife::Error_msg error_id;

	bool start_controller;

	Eigen::MatrixXd F_repulsive, F_attractive, F_total;

	/** Function: task_objective_function
	* input: position
	* output: double
	* Description: with function calculates the position more distance than robot's limits
	*/
	// double task_objective_function(KDL::JntArray q);


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
	Eigen::Matrix<double, 7, 1> RepulsiveWithTable();

	Eigen::Matrix<double, 7, 1>  GetRepulsiveWithObstacle();
	Eigen::Matrix<double, 7, 1>  getVelRepulsive(KDL::Jacobian &J, unsigned int n_joint, Eigen::Matrix<double, 6, 1> F);


	/** Caalback: InfoOBj
	* input: desperate message with information about obstacle to remove
	* Description: callback that save the obstacle position
	*/
	// void InfoOBj( const desperate_housewife::fittedGeometriesSingle::ConstPtr& obj_rem);

	/** Caalback: set_gains
	* input: ros message
	* Description: callback that change inline the gains
	*/
	// void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);

	/** Caalback: command_start
	* input: ros message
	* Description: callback for start the code with real robot
	*/
	// void command_start(const std_msgs::Bool::ConstPtr& msg);


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
	Eigen::Vector3d GetPartialDerivate(KDL::Frame &T_v_o, KDL::Vector &Point_v, double radius, double height);

	/** Function: GetFIRAS
	* input: min distance, object derivate, influence of repulsive field
	* output: repulsive forces
	* Description: calculates the repulsive forces  like article O.Khatib
	*/
	// Eigen::Matrix<double,6,1> GetFIRAS(double min_distance, Eigen::Vector3d &distance_der_partial , double influence);

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

	// Eigen::Matrix<double,6,1> GetRepulsiveForce(KDL::Vector &point_, double influence, KDL::Frame &Object_pos, double radius, double height);



	// void SeeMarker(KDL::Frame &Pos, std::string obst_name);

	/** Function: VelocityLimit
	* input: vector with the interested point, object position, influence of repulsive field, object radius and height
	* output: repulsive force and the index for the jacobian
	* Description: functions tha call the funciont for calculates the repulsive forces
	*/

	Eigen::Matrix<double, 6, 6> getAdjointT( KDL::Frame Frame_in);
	Eigen::Matrix<double, 3, 3> getVectorHat(Eigen::Matrix<double, 3, 1> vector_in);
	Eigen::Matrix<double, 6, 1> GetRepulsiveForceTable(KDL::Frame &T_in, double influence);
	KDL::JntArray JointLimitAvoidance(KDL::JntArray q);
	Eigen::MatrixXd getGainMatrix(std::string parameter, ros::NodeHandle n, int dimension = 6);
	void startControllerCallBack(const std_msgs::Bool::ConstPtr& msg);

	bool loadParametersCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	Eigen::Matrix<double, 7, 1> task_objective_function(KDL::JntArray q);
	Eigen::Matrix<double, 7, 1> MaxZYDistance(KDL::JntArray q);
	double VelocityLimit(KDL::Twist x_dot_d );
	Eigen::Matrix<double, 7, 1> potentialEnergy(KDL::JntArray q);
	Eigen::Matrix<double, 4, 4>  FromKdlToEigen(KDL::Frame &T_v_o);
	void getClosestPointstoCylinder( KDL::Frame T_link_object, KDL::Frame &T_CollisionPoint, double &distance_to_T_CollisionPoint, double radius, double height);

};

/** Function: FromKdlToEigen
	* input: kdl frame
	* output: eigen matrix
	* Description: functions tha transform kdl frame into eigen matrix
*/
// Eigen::Matrix<double, 4, 4>  FromKdlToEigen(KDL::Frame &T_v_o);







}




#endif


