#ifndef TRASH_POS_H
#define TRASH_POS_H

#include "state.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <desperate_housewife/Error_msg.h>
// #include <check_error.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames_io.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <desperate_housewife/handPoseSingle.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
# include <sensor_msgs/JointState.h>


class Pos_trash : public state<transition>
{
public:
    Pos_trash(const shared& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
     virtual void reset();

   	bool IsEqual(KDL::Twist E_pf);

	void Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg);
	void Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg);
    void HandInforRight(const sensor_msgs::JointState::ConstPtr &msg);
    void SendTrashPosRight();
    void SendTrashPosLeft();

private:
	std::string type;
	desperate_housewife::handPoseSingle trash_robot_pose;
	std::string desired_hand_right_pose_topic_, desired_hand_left_pose_topic_;
	ros::Publisher desired_hand_publisher_right, desired_hand_publisher_left;
	// int msg_arrived;
	KDL::Twist e_;
	KDL::Twist E_t;
	int step;
	std::vector<KDL::Twist> vect_error;
	// check_error Error_th;
	bool finish, failed;
	ros::NodeHandle nh;
	ros::Subscriber error_sub_right, error_sub_left;
	std::string error_topic_right;
	ros::Subscriber hand_info_right;
	std::string hand_joint_position_r, error_topic_left;
	int info_hand;
	ros::Publisher hand_publisher_right;
	std::string hand_close_right;
	double yaw,pitch,roll;
	double Info_open_hand;
	int index_sh;
	int id_error_msgs;
	int id_class;
	const shared& data;
	// int id_msgs;
	int id_arm_msg;

	struct quaternion_
    {
      KDL::Vector v;
      double a;
    } quat_now, quat_des_;

    tf::TransformBroadcaster  tf_desired_hand_pose;
    std::string base_frame_;

};

#endif // TRASH_POS_H