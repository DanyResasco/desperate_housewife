#ifndef SOFTHAND_O_H
#define SOFTHAND_O_H

#include "state.h"
#include <std_msgs/UInt16.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h> 
#include <trajectory_msgs/JointTrajectory.h>

class SoftHand_open : public state<transition>
{
public:
    SoftHand_open(const shared& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();

    void HandInforRight(const sensor_msgs::JointState::ConstPtr &msg);
    void HandInforLeft(const sensor_msgs::JointState::ConstPtr &msg);
    /*! 
      * \fn  HandInforLeft(const sensor_msgs::JointState::ConstPtr &msg);
      * \brief callback that store the softhand information i.e the joint position
      * \param  ros message
      * \return void
    */ 

private:
	std::string type;
	ros::NodeHandle nh;
	ros::Subscriber hand_info_right, hand_info_left;
	std::string hand_joint_position_r, hand_joint_position_l;
	double info_hand_right, info_hand_left;
	int index_sh;
	ros::Publisher hand_publisher_right, hand_publisher_left;
	std::string hand_open_right, hand_open_left;
	bool finish;
	double Info_open_hand;
  const  shared& data;




};

#endif // SOFTHAND_O_H