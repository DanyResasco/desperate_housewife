#ifndef GRASP_OBJ_H
#define GRASP_OBJ_H

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
#include <std_msgs/Float64MultiArray.h>

#include <ros/ros.h>
#include <ros/console.h>

class Grasp_move : public state<transition>
{
public:
    Grasp_move(const shared& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
   	void Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg);

	bool IsEqual(KDL::Twist E_pf);

private:
	std::string type;
	// check_error Error_th;
	ros::NodeHandle nh;
	ros::Subscriber error_sub_right;
	std::string error_topic_right;
	KDL::Twist e_;
	// int msg_arrived;
	bool finish;
	bool failed;
	KDL::Twist E_t;
	int id_class;
	int id_msgs;
  const shared& data;
};

#endif // GRASP_OBJ_H