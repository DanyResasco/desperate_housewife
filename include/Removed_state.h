#ifndef REMOVED_STATE_H
#define REMOVED_STATE_H

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


class Removed_moves : public state<transition>
{
public:
    Removed_moves(const shared& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
    // virtual void reset();
    void Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg);
    void Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg);
    bool IsEqual(KDL::Twist E_pf);
    void RemObjLeft();
    void RemObjRight();

   private:
   	KDL::Twist e_;
   	desperate_housewife::handPoseSingle New_Hand_Position;
   	std::string base_frame_;
   	ros::Publisher desired_hand_publisher_right, desired_hand_publisher_left;
   	ros::Subscriber error_sub_right;
    ros::NodeHandle nh;
    std::string error_topic_right, desired_hand_right_pose_topic_;
    std::string desired_hand_left_pose_topic_;
    bool finish;
    bool failed;
    tf::TransformBroadcaster tf_desired_hand_pose;
    tf::TransformListener listener_info;
    std::string right_hand_frame_, left_hand_frame_;
    int id_error_msgs;
    int id_class;
    KDL::Twist E_t;
    std::vector<KDL::Twist> vect_error;
    const shared& data;
};

#endif // REMOVED_STATE_H