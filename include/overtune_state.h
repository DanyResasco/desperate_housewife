#ifndef OVERTUNE_STATE_H
#define OVERTUNE_STATE_H

#include "state.h"
// #include <check_error.hpp>
#include <desperate_housewife/Error_msg.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames_io.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <desperate_housewife/handPoseSingle.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <ros/console.h>

class Overtune_state : public state<transition>
{
public:
    Overtune_state(const shared& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();

    void Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg);
    void Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg);
    bool IsEqual(KDL::Twist E_pf);
    void InfoArm();
    // void SendArmReplaceTable();

private:

	KDL::Twist e_;
   	desperate_housewife::handPoseSingle New_Hand_Position_right, New_Hand_Position_left;
   	std::string base_frame_;
   	ros::Publisher desired_hand_publisher_right, desired_hand_publisher_left;
   	ros::Subscriber error_sub_right, error_sub_left;
    int step;
    // check_error Error_th;
    ros::NodeHandle nh;
    std::string error_topic_right, desired_hand_right_pose_topic_;
    std::string error_topic_left, desired_hand_left_pose_topic_;
    bool finish;
    tf::TransformBroadcaster tf_desired_hand_pose;
    int overturn_check;
    tf::TransformListener listener_info;
    std::string right_hand_frame_, left_hand_frame_;
    int id_class;
    int id_error_msgs;
    KDL::Twist E_t;
    const shared& data;
    std::vector<KDL::Twist> vect_error;
};

#endif // OVERTUNE_STATE_H