#ifndef HOME_STATE_H
#define HOME_STATE_H

#include "state.h"
#include <desperate_housewife/Error_msg.h>
#include <desperate_housewife/handPoseSingle.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames_io.hpp>
// #include <desperate_housewife/Start.h>
// #include <check_error.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>


class Home_state : public state<transition>
{
public:
    Home_state();
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();

   void Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg);
    void SendHomeRobot_right();
      void command_start(const std_msgs::Bool::ConstPtr& msg);
    bool IsEqual(KDL::Twist E_pf);

    private:

    ros::NodeHandle nh;
    std::string base_frame_;
    ros::Publisher desired_hand_right_pose_publisher_;
    std::string desired_hand_right_pose_topic_;
    // check_error Error_th;
    ros::Subscriber error_sub_right;
    ros::Publisher right_home_publisher_;
    std::string home_right_topic_;
    std::string error_topic_right;
	KDL::Twist e_;
    int msg_arrived;
    bool finish, failed;
    // desperate_housewife::Start start_controller;
    bool use_both_arm = false;
    ros::Publisher right_start_controller_pub;
    std::string start_topic_right;
    std::string type;
    int step;
    tf::TransformBroadcaster  tf_desired_hand_pose;
    ros::Subscriber sub_command_start;
    int id_class;
    int id_error_msgs;

    struct quaternion_
    {
      KDL::Vector v;
      double a;
    } quat_now, quat_des_;
    bool start_flag;
    KDL::Twist E_t;
    // ros::NodeHandle nh;
    double x,y,z,rotx,roty,rotz;




};

#endif // HOME_STATE_H