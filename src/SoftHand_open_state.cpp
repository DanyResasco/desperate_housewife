#include <SoftHand_states_open.h>


SoftHand_open::SoftHand_open(const shared& data)
{
  	this->type=type;
  	
  	nh.param<std::string>("/right_hand/joint_states", hand_joint_position_r, "/right_hand/joint_states");
  	hand_info_right = nh.subscribe(hand_joint_position_r.c_str(),1, &SoftHand_open::HandInforRight,this);

  	bool use_sh_sim ;
  	nh.param<bool>("/use_sh_sim",use_sh_sim,false);
  	nh.param<double>("/open_hand", Info_open_hand, 0.6);

    // std::cout<<"use_sh_sim: "<<std::boolalpha<<use_sh_sim<<std::endl;
   
    index_sh = use_sh_sim == false ? 0 : 28;

    nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_open_right, "/right_hand/joint_trajectory_controller/command");
  	hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_open_right.c_str(), 1000);

  	finish = false;
}



void SoftHand_open::HandInforRight(const sensor_msgs::JointState::ConstPtr &msg)
{
  // info_hand = msg->position[28];  //28 in simulazione 0 in reale
  info_hand = msg->position[index_sh];
  // std::cout<<"index_sh: "<<index_sh<<std::endl;
}



std::map< transition, bool > SoftHand_open::getResults()
{
	std::map< transition, bool > results;
  // std::cout<<"finish: "<<std::boolalpha<<finish<<std::endl;
	if(finish == true)
			results[transition::Wait_Open_Softhand] = finish;

  return results;
}

void SoftHand_open::run()
{
	/*close the softhand*/
    trajectory_msgs::JointTrajectory msg_jointT_hand;
    msg_jointT_hand.points.resize(1);
    msg_jointT_hand.joint_names.resize(1);
    msg_jointT_hand.points[0].positions.resize(1);
    msg_jointT_hand.points[0].positions[0] = 0.0;
    msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
    msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
    hand_publisher_right.publish(msg_jointT_hand);
   
    if(info_hand >= Info_open_hand )
    {
      finish = false;
      // std::cout<<"aspetto apertura mano"<<std::endl;
    }
    else
    {
      finish = true;
       // std::cout<<"apro mano"<<std::endl;

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