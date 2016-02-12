#ifndef MSGS_WAIT_H
#define MSGS_WAIT_H

#include "state.h"
#include <std_msgs/UInt16.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>

class Wait_msgs : public state<transition>
{
public:
    Wait_msgs();
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
    virtual void reset();

    void ObjOrObst_right(const std_msgs::UInt16::ConstPtr& obj_msg);


private:
  	ros::Subscriber objects_info_right_sub ;
  	int ObjOrObst;
  	std::string type;
    ros::NodeHandle nh;
    std::string obj_info_topic_r;
    int arrived_r;
    bool finish;
};

#endif // MSGS_WAIT_H