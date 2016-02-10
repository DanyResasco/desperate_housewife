#include <home_state.h>
// #include <check_error.hpp>

Home_state::Home_state()
{
  	nh.param<std::string>("/right_arm/PotentialFieldControl/error_id", error_topic_right, "/right_arm/PotentialFieldControl/error_id");
  	error_sub_right = nh.subscribe(error_topic_right, 1, &Home_state::Error_info_right, this);

  	nh.param<std::string>("/right_arm//PotentialFieldControl/command", desired_hand_right_pose_topic_, "/right_arm/PotentialFieldControl/command");
  	desired_hand_right_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);
    
    nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "world");

    sub_command_start = nh.subscribe("/right_arm/PotentialFieldControl/start_controller", 1, &Home_state::command_start, this);

    id_class = static_cast<int>(transition_id::Vito_home);
    /*treshold error*/
    nh.param<double>("/x_treshold",x,0.01);
    nh.param<double>("/y_treshold",y,0.01);
    nh.param<double>("/z_treshold",z,0.01);
    nh.param<double>("/rot_x_treshold",rotx,0.01);
    nh.param<double>("/rot_y_treshold",roty,0.01);
    nh.param<double>("/rot_z_treshold",rotz,0.01);

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

  	// msg_arrived = 0;
  	finish = false;
  	// step = 1;
    // this->type = type;
    start_flag = false;
}

bool Home_state::IsEqual(KDL::Twist E_pf)
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


void Home_state::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
    tf::twistMsgToKDL (error_msg->error_, e_);

    id_error_msgs = error_msg->id;
    // std::cout<<"error: "<<error_msg->data[0]<<error_msg->data[1]<<error_msg->data[2]<<std::endl;
}

  void Home_state::command_start(const std_msgs::Bool::ConstPtr& msg)
  { 
      start_flag = true;
      std::cout<<"ricevuto sms"<<std::endl;
  }


void Home_state::run()
{
  if(start_flag == true)
  {
    if((id_class != id_error_msgs) && (!IsEqual(e_)))
    {
      std::cout<<"send vito at home"<<std::endl;
  		SendHomeRobot_right(); 
  	}
    
    else  if((id_class == id_error_msgs) && (IsEqual(e_)))
    {      
        finish = true;
	  }
  }
  else
    finish = false;

}

bool Home_state::isComplete()
{
	return finish;
}


std::map< transition, bool > Home_state::getResults()
{
	std::map< transition, bool > results;
    if(finish == true)
    {
      results[transition::Error_arrived] = finish;
    }
    return results;
}


std::string Home_state::get_type()
{
 return "Home_state";
}

void Home_state::SendHomeRobot_right()
{
    desperate_housewife::handPoseSingle home_robot_right;  
    
    home_robot_right.home = 1;
    home_robot_right.obj = 0; 
    
    double roll_r,pitch_r,yaw_r;
    nh.param<double>("/home_right_arm_position_x", home_robot_right.pose.position.x, -0.75022);
    nh.param<double>("/home_right_arm_position_y",  home_robot_right.pose.position.y,  0.47078);
    nh.param<double>("/home_right_arm_position_z", home_robot_right.pose.position.z, 0.74494);
    nh.param<double>("/home_right_arm_A_yaw", yaw_r,  0.334);
    nh.param<double>("/home_right_arm_B_pitch", pitch_r, -0.08650);
    nh.param<double>("/home_right_arm_C_roll", roll_r, -0.5108);

  
    KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll_r,pitch_r,yaw_r);

    Rot_matrix_r.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

    home_robot_right.pose.orientation.x = quat_des_.v(0);
    home_robot_right.pose.orientation.y = quat_des_.v(1);
    home_robot_right.pose.orientation.z = quat_des_.v(2);
    home_robot_right.pose.orientation.w = quat_des_.a;

    home_robot_right.id = id_class;

    desired_hand_right_pose_publisher_.publish( home_robot_right );

    tf::Transform tfHandTrasform1;    
    tf::poseMsgToTF( home_robot_right.pose, tfHandTrasform1);    
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"home_robot_right") );  
}