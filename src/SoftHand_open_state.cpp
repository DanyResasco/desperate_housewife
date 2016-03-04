#include <SoftHand_states_open.h>


SoftHand_open::SoftHand_open(const shared& m):data(m)
{

  /*reads the softhand information*/
  nh.param<std::string>("/right_hand/joint_states", hand_joint_position_r, "/right_hand/joint_states");
  hand_info_right = nh.subscribe(hand_joint_position_r.c_str(),1, &SoftHand_open::HandInforRight,this);

  nh.param<std::string>("/left_hand/joint_states", hand_joint_position_l, "/left_hand/joint_states");
  hand_info_left = nh.subscribe(hand_joint_position_l.c_str(),1, &SoftHand_open::HandInforLeft,this);

  bool use_sh_sim ;
  nh.param<bool>("/use_sh_sim",use_sh_sim,false);
  nh.param<double>("/open_hand", Info_open_hand, 0.6);

  index_sh = use_sh_sim == false ? 0 : 28;

  /*send a command for closing the softhand*/
  nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_open_right, "/right_hand/joint_trajectory_controller/command");
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_open_right.c_str(), 1000);

  nh.param<std::string>("/left_hand/joint_trajectory_controller/command", hand_open_left, "/left_hand/joint_trajectory_controller/command");
  hand_publisher_left = nh.advertise<trajectory_msgs::JointTrajectory>(hand_open_left.c_str(), 1000);

  finish = false;
  home_reset = false;
}



void SoftHand_open::HandInforRight(const sensor_msgs::JointState::ConstPtr &msg)
{

  info_hand_right = msg->position[index_sh];
}

void SoftHand_open::HandInforLeft(const sensor_msgs::JointState::ConstPtr &msg)
{
  info_hand_left = msg->position[index_sh];
}



std::map< transition, bool > SoftHand_open::getResults()
{
  std::map< transition, bool > results;

    if(home_reset == true)
    {
       results[transition::home_reset] = finish;
    }
    else
    {
      results[transition::Wait_Open_Softhand] = finish;
      finish = false;
    }


  // if(finish == true)
  //   results[transition::Wait_Open_Softhand] = finish;

  return results;
}

void SoftHand_open::run()
{

  switch(data.arm_to_use)
    {
    case 0: //right
      {
        /*close the softhand*/
        trajectory_msgs::JointTrajectory msg_jointT_hand;
        msg_jointT_hand.header.stamp = ros::Time::now();
        msg_jointT_hand.points.resize(1);
        msg_jointT_hand.joint_names.resize(1);
        msg_jointT_hand.points[0].positions.resize(1);
        msg_jointT_hand.points[0].positions[0] = 0.0;
        msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
        msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
        hand_publisher_right.publish(msg_jointT_hand);

        if(info_hand_right >= Info_open_hand )
          {
            finish = false;
            ROS_DEBUG("Waiting closing softhand");
          }
        else
          {
            finish = true;
            ROS_DEBUG("Closing softhand ");
          }

        break;
      }

    case 1: //left
      {
        /*close the softhand*/
        trajectory_msgs::JointTrajectory msg_jointT_hand;
        msg_jointT_hand.points.resize(1);
        msg_jointT_hand.joint_names.resize(1);
        msg_jointT_hand.points[0].positions.resize(1);
        msg_jointT_hand.points[0].positions[0] = 0.0;
        msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
        msg_jointT_hand.joint_names[0] = "left_hand_synergy_joint";
        hand_publisher_left.publish(msg_jointT_hand);

        if(info_hand_left >= Info_open_hand )
          {
            finish = false;
            ROS_DEBUG("Waiting closing softhand");
          }
        else
          {
            finish = true;
            ROS_DEBUG("Closing softhand ");
          }

        break;
      }
    case 2:
      {
        trajectory_msgs::JointTrajectory msg_jointT_hand;
        msg_jointT_hand.points.resize(1);
        msg_jointT_hand.joint_names.resize(1);
        msg_jointT_hand.points[0].positions.resize(1);
        msg_jointT_hand.points[0].positions[0] = 0.0;
        msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
        msg_jointT_hand.joint_names[0] = "left_hand_synergy_joint";
        hand_publisher_left.publish(msg_jointT_hand);
        msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
        hand_publisher_right.publish(msg_jointT_hand);

        if((info_hand_left >= Info_open_hand ) && (info_hand_right >= Info_open_hand ))
          {
            finish = false;
            ROS_DEBUG("Waiting closing softhand");
          }
        else
          {
            finish = true;
            ROS_DEBUG("Open softhand ");
          }

        break;

      }
    }

}

bool SoftHand_open::isComplete()
{
  return finish;
}

std::string SoftHand_open::get_type()
{
  return "SoftHand_open";
}

void SoftHand_open::resetCallBack(const std_msgs::Bool::ConstPtr msg)
{
  ROS_INFO("SoftHand_open::Reset called");
  home_reset = msg->data;
  finish = true;
  // failed = false;
  // return true;
}


void SoftHand_open::reset()
{
  home_reset = false;
  finish = false;
  // failed = false;
}


