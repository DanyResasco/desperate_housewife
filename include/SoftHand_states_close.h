#ifndef SOFTHAND_STATE_H
#define SOFTHAND_STATE_H

#include "state.h"
#include <std_msgs/UInt16.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h> 
#include <trajectory_msgs/JointTrajectory.h>

class SoftHand_close : public state<transition>
{
public:
    SoftHand_close();
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();

    void HandInforRight(const sensor_msgs::JointState::ConstPtr &msg);

private:
	std::string type;
	ros::NodeHandle nh;
	ros::Subscriber hand_info_right;
	std::string hand_joint_position_r;
	double info_hand;
	int index_sh;
	ros::Publisher hand_publisher_right;
	std::string hand_close_right;
	double Info_closed_hand;
	bool finish;
};

#endif // SOFTHAND_STATE_H