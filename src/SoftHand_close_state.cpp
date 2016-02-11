#include <SoftHand_states_close.h>

SoftHand_close::SoftHand_close()
{
	  this->type=type;
	
	  nh.param<std::string>("/right_hand/joint_states", hand_joint_position_r, "/right_hand/joint_states");
  	hand_info_right = nh.subscribe(hand_joint_position_r.c_str(),1, &SoftHand_close::HandInforRight,this);

  	bool use_sh_sim ;
  	nh.param<bool>("/use_sh_sim",use_sh_sim,false);
  	nh.param<double>("/closed_hand", Info_closed_hand, 0.6);
   
    index_sh = use_sh_sim == false ? 0 : 28;

    nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_close_right, "/right_hand/joint_trajectory_controller/command");
  	hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);

  	finish = false;
}



void SoftHand_close::HandInforRight(const sensor_msgs::JointState::ConstPtr &msg)
{
  // info_hand = msg->position[28];  //28 in simulazione 0 in reale
  info_hand = msg->position[index_sh];
  // ROS_INFO("index hand pose %d", index_sh);
  // std::cout<<"info_hand: "<<info_hand<<std::endl;
}


std::map< transition, bool > SoftHand_close::getResults()
{
	std::map< transition, bool > results;
	if(finish == true)
		results[transition::Wait_Closed_Softhand] = finish;

  return results;
}

void SoftHand_close::run()
{
    /*close the softhand*/
    trajectory_msgs::JointTrajectory msg_jointT_hand;
    msg_jointT_hand.points.resize(1);
    msg_jointT_hand.joint_names.resize(1);
    msg_jointT_hand.points[0].positions.resize(1);
    msg_jointT_hand.points[0].positions[0] = 1.0;
    msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
    msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
    hand_publisher_right.publish(msg_jointT_hand);

    if(info_hand < Info_closed_hand )
    {
         finish = false;
         ROS_DEBUG("Waiting closing softhand");
    }
    else
    {
      finish = true;
      ROS_DEBUG("Closing softhand ");
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
