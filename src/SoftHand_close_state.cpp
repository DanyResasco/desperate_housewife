#include <SoftHand_states_close.h>

SoftHand_close::SoftHand_close(const shared& m): data(m)
{
  // this->type=type;

  nh.param<std::string>("/right_hand/joint_states", hand_joint_position_r, "/right_hand/joint_states");
  hand_info_right = nh.subscribe(hand_joint_position_r.c_str(), 1, &SoftHand_close::HandInforRight, this);

  nh.param<std::string>("/left_hand/joint_states", hand_joint_position_l, "/left_hand/joint_states");
  hand_info_left = nh.subscribe(hand_joint_position_l.c_str(), 1, &SoftHand_close::HandInforLeft, this);

  bool use_sh_sim ;
  nh.param<bool>("/use_sh_sim", use_sh_sim, false);
  nh.param<double>("/closed_hand", Info_closed_hand, 0.6);

  index_sh = use_sh_sim == false ? 0 : 28;

  nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_close_right, "/right_hand/joint_trajectory_controller/command");
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);

  nh.param<std::string>("/left_hand/joint_trajectory_controller/command", hand_close_left, "/left_hand/joint_trajectory_controller/command");
  hand_publisher_left = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_left.c_str(), 1000);

  finish = false;
  overtune_state = 0;
  home_reset = false;

    srv_reset = nh.subscribe("/reset",1, &SoftHand_close::resetCallBack, this);
  // srv_reset = nh.advertiseService("/reset", &SoftHand_close::resetCallBack, this);
}



void SoftHand_close::HandInforRight(const sensor_msgs::JointState::ConstPtr &msg)
{
  info_hand_right = msg->position[index_sh];
}

void SoftHand_close::HandInforLeft(const sensor_msgs::JointState::ConstPtr &msg)
{
  info_hand_left = msg->position[index_sh];
}



std::map< transition, bool > SoftHand_close::getResults()
{
  std::map< transition, bool > results;

  if ( (overtune_state == 0))
  {
    if (home_reset == true)
    {
      results[transition::home_reset] = finish;
      home_reset = false;
    }
    else
    {
      results[transition::Wait_Closed_Softhand] = finish;
    }
  }
  else
  {
    results[transition::Overtune_table] = finish;
  }



  finish = false;


  // if((finish == true) && (overtune_state == 0))
  //   {
  //     results[transition::Wait_Closed_Softhand] = finish;
  //   }
  // else if((finish == true) && (overtune_state == 1))
  //   results[transition::Overtune_table] = finish;
  return results;
}

void SoftHand_close::run()
{
  switch (data.arm_to_use)
  {
  case 0: //right
  {
    /*close the softhand*/
    trajectory_msgs::JointTrajectory msg_jointT_hand;
    msg_jointT_hand.header.stamp = ros::Time::now();
    msg_jointT_hand.points.resize(1);
    msg_jointT_hand.joint_names.resize(1);
    msg_jointT_hand.points[0].positions.resize(1);
    msg_jointT_hand.points[0].positions[0] = 1.0;
    msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
    msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
    hand_publisher_right.publish(msg_jointT_hand);

    if (info_hand_right <= Info_closed_hand )
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
    msg_jointT_hand.points[0].positions[0] = 1.0;
    msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
    msg_jointT_hand.joint_names[0] = "left_hand_synergy_joint";
    hand_publisher_left.publish(msg_jointT_hand);

    if (info_hand_left <= Info_closed_hand )
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
  case 2: //overtune state
  {
    trajectory_msgs::JointTrajectory msg_jointT_hand;
    msg_jointT_hand.points.resize(1);
    msg_jointT_hand.joint_names.resize(1);
    msg_jointT_hand.points[0].positions.resize(1);
    msg_jointT_hand.points[0].positions[0] = 1.0;
    msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
    msg_jointT_hand.joint_names[0] = "left_hand_synergy_joint";
    hand_publisher_left.publish(msg_jointT_hand);
    msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
    hand_publisher_right.publish(msg_jointT_hand);

    if ((info_hand_left <= Info_closed_hand ) && (info_hand_right <= Info_closed_hand ))
    {
      finish = false;
      ROS_DEBUG("Waiting closing softhand");
    }
    else
    {
      finish = true;
      overtune_state = 1;
      ROS_DEBUG("Closing softhand ");
    }


  }
  }




}

bool SoftHand_close::isComplete()
{
  return finish;
}

std::string SoftHand_close::get_type()
{
  return "SoftHand_close";
}


void SoftHand_close::resetCallBack(const std_msgs::Bool::ConstPtr msg)
{
  // ROS_INFO("SoftHand_close::Reset called");
  home_reset = msg->data;
  finish = true;
  // failed = false;
  // return true;
}


void SoftHand_close::reset()
{
  home_reset = false;
  finish = false;
  // failed = false;
}


















// bool SoftHand_close::resetCallBack(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
// {
//   ROS_INFO("SoftHand_close::Reset called");
//   home_reset = true;
//   finish = true;
//   return true;
// }