#include <trash_position.h>


Pos_trash::Pos_trash()
{
    std::string string_temp;
    
    nh.param<std::string>("/right_arm/PotentialFieldControl/topic_desired_reference", string_temp, "command");
    desired_hand_right_pose_topic_ = std::string("/right_arm/PotentialFieldControl/") + string_temp;

    desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

    //  nh.param<std::string>("/right_arm/PotentialFieldControl/topic_desired_reference", desired_hand_right_pose_topic_, "/right_arm/PotentialFieldControl/command");
  	// desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  	nh.param<std::string>("/right_arm/PotentialFieldControl/error_id", error_topic_right, "/right_arm/PotentialFieldControl/error_id");
  	error_sub_right = nh.subscribe(error_topic_right, 1, &Pos_trash::Error_info_right, this);

  	nh.param<std::string>("/right_arm/PotentialFieldControl/root_name", base_frame_, "world");
	  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);

	  id_class = static_cast<int>(transition_id::Vito_trash);
  	
    step = 1;
  	msg_arrived = 0;
  	finish = false;
  	failed = false;


  	double x,y,z,rotx,roty,rotz;
    /*treshold error*/
    nh.param<double>("/error/pos/x",x,0.01);
    nh.param<double>("/error/pos/y",y,0.01);
    nh.param<double>("/error/pos/z",z,0.01);
    nh.param<double>("/error/rot/x",rotx,0.01);
    nh.param<double>("/error/rot/y",roty,0.01);
    nh.param<double>("/error/rot/z",rotz,0.01);


      nh.param<double>("/trash/right_arm_position_x", trash_robot_pose.pose.position.x, -0.75022);
      nh.param<double>("/trash/right_arm_position_y",  trash_robot_pose.pose.position.y,  -0.47078);
      nh.param<double>("/trash/right_arm_position_z", trash_robot_pose.pose.position.z, 0.74494);
      nh.param<double>("/trash/right_arm_A_yaw", yaw,  -0.12690);
      nh.param<double>("/trash/right_arm_B_pitch", pitch, -0.06571);
      nh.param<double>("/trash/right_arm_C_roll", roll, -0.11774);

      KDL::Vector vel;
      KDL::Vector rot;
      vel.data[0] = x;
      vel.data[1] = y;
      vel.data[2] = z;
      rot.data[0] = rotx;
      rot.data[1] = roty;
      rot.data[2] = rotz;
      E_t.vel = vel;
      E_t.rot = rot;

      finish = false;
}


void Pos_trash::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
    tf::twistMsgToKDL (error_msg->error_, e_);

    id_error_msgs = error_msg->id;
}


std::map< transition, bool > Pos_trash::getResults()
{
	std::map< transition, bool > results;
	
  if(finish == true)
	{
		results[transition::Error_arrived] = true;
	}

	return results;
}

void Pos_trash::run()
{
    
	// if(step == 1)
	if((id_class != id_error_msgs) && (IsEqual(e_)))
	{	//send robot at trash position

	    KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll, pitch ,yaw);

	    Rot_matrix_r.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

	    trash_robot_pose.pose.orientation.x = quat_des_.v(0);
	    trash_robot_pose.pose.orientation.y = quat_des_.v(1);
	    trash_robot_pose.pose.orientation.z = quat_des_.v(2);
	    trash_robot_pose.pose.orientation.w = quat_des_.a;

      trash_robot_pose.id = id_class;
	    desired_hand_publisher_right.publish( trash_robot_pose );

      // ROS_INFO("trash published to %s", desired_hand_right_pose_topic_.c_str());
	    // step = 0; 
      finish = false;
	    tf::Transform tfHandTrasform1;    
      tf::poseMsgToTF( trash_robot_pose.pose, tfHandTrasform1);    
      tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"trash_robot_pos") );  
	}

	else if((id_class == id_error_msgs) && (IsEqual(e_)))
  { 	
      // std::cout<<"same id and error is equal"<<std::endl;
  	  finish = true;
  }

}	


   

bool Pos_trash::isComplete()
{
    return finish;
}

std::string Pos_trash::get_type()
{
    return "Pos_trash";
}

bool Pos_trash::IsEqual(KDL::Twist E_pf)
{
    KDL::Twist E_pf_abs;
    // std::cout<<"IsEqual"<<std::endl;
    E_pf_abs.vel.data[0] = std::abs(E_pf.vel.data[0] );
    E_pf_abs.vel.data[1] = std::abs(E_pf.vel.data[1] );
    E_pf_abs.vel.data[2] = std::abs(E_pf.vel.data[2] );
    E_pf_abs.rot.data[0] = std::abs(E_pf.rot.data[0] );
    E_pf_abs.rot.data[1] = std::abs(E_pf.rot.data[1] );
    E_pf_abs.rot.data[2] = std::abs(E_pf.rot.data[2] );


    if (  (E_pf_abs.vel.data[0] < E_t.vel.data[0]) &&
          (E_pf_abs.vel.data[1] < E_t.vel.data[1]) &&
          (E_pf_abs.vel.data[2] < E_t.vel.data[2]) &&
          (E_pf_abs.rot.data[0] < E_t.rot.data[0]) &&
          (E_pf_abs.rot.data[1] < E_t.rot.data[1]) &&
          (E_pf_abs.rot.data[2] < E_t.rot.data[2]) )
    {
       ROS_DEBUG("is equal");
      return true;
    }
    else
    {
      ROS_DEBUG("is not equal");
      ROS_DEBUG("error linear: E_pf_abs.vel.data[0] %g E_pf_abs.vel.data[1] %g  E_pf_abs.vel.data[2]: %g",  E_pf_abs.vel.data[0],E_pf_abs.vel.data[1], E_pf_abs.vel.data[2]);
      ROS_DEBUG("error agular: E_pf_abs.rot.data[0] %g E_pf_abs.rot.data[1] %g E_pf_abs.rot.data[2] %g", E_pf_abs.rot.data[0], E_pf_abs.rot.data[1], E_pf_abs.rot.data[2]);
      return false;
    }
}


