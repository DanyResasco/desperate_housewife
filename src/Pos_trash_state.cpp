#include <Trash_position.h>


Pos_trash::Pos_trash(const shared& m):data(m)
{
  std::string string_temp;

  nh.param<std::string>("/right_arm/PotentialFieldControl/topic_desired_reference", string_temp, "command");
  desired_hand_right_pose_topic_ = std::string("/right_arm/PotentialFieldControl/") + string_temp;

  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);


  std::string string_temp_l;

  nh.param<std::string>("/left_arm/PotentialFieldControl/topic_desired_reference", string_temp_l, "command");
  desired_hand_left_pose_topic_ = std::string("/left_arm/PotentialFieldControl/") + string_temp_l;

  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);



  //  nh.param<std::string>("/right_arm/PotentialFieldControl/topic_desired_reference", desired_hand_right_pose_topic_, "/right_arm/PotentialFieldControl/command");
  // desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  nh.param<std::string>("/right_arm/PotentialFieldControl/error_id", error_topic_right, "/right_arm/PotentialFieldControl/error_id");
  error_sub_right = nh.subscribe(error_topic_right, 1, &Pos_trash::Error_info_right, this);

  nh.param<std::string>("/left_arm/PotentialFieldControl/error_id", error_topic_left, "/left_arm/PotentialFieldControl/error_id");
  error_sub_left = nh.subscribe(error_topic_left, 1, &Pos_trash::Error_info_left, this);

  nh.param<std::string>("/right_arm/PotentialFieldControl/root_name", base_frame_, "world");
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);

  id_class = static_cast<int>(transition_id::Vito_trash);

  vect_error.resize(2);
  KDL::Twist temp;
  SetToZero(temp);
  vect_error[0] = temp;
  vect_error[1] = temp;

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
  KDL::Twist temp;
  tf::twistMsgToKDL (error_msg->error_, temp);

  id_error_msgs = error_msg->id;
  id_arm_msg = error_msg->id_arm;
  vect_error[0] = temp;
}


void Pos_trash::Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Twist temp;
  tf::twistMsgToKDL (error_msg->error_, temp);

  id_error_msgs = error_msg->id;
  id_arm_msg = error_msg->id_arm;
  vect_error[1] = temp;
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
  e_ = vect_error[0] + vect_error[1];

  // int temp = data.arm_to_use;
  if((id_class != id_error_msgs) && (IsEqual(e_)))
    {
      switch(data.arm_to_use)
        {
        case 0: //right
          {
            SendTrashPosRight();
            break;
          }
        case 1:
          {
            SendTrashPosLeft();
            break;
          }
        }
    }

  else if((id_class == id_error_msgs) && (IsEqual(e_)))
    {
      // std::cout<<"same id and error is equal"<<std::endl;
      finish = true;
    }

}	

// void Pos_trash::reset()
// {
//   finish = false;
// }



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


void Pos_trash::SendTrashPosRight()
{
  desperate_housewife::handPoseSingle home_robot_right;

  double roll_r,pitch_r,yaw_r;
  nh.param<double>("/trash/right_arm_position_x", home_robot_right.pose.position.x, -0.75022);
  nh.param<double>("/trash/right_arm_position_y",  home_robot_right.pose.position.y,  0.47078);
  nh.param<double>("/trash/right_arm_position_z", home_robot_right.pose.position.z, 0.74494);
  nh.param<double>("/trash/right_arm_A_yaw", yaw_r,  0.334);
  nh.param<double>("/trash/right_arm_B_pitch", pitch_r, -0.08650);
  nh.param<double>("/trash/right_arm_C_roll", roll_r, -0.5108);

  KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll_r,pitch_r,yaw_r);

  Rot_matrix_r.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

  home_robot_right.pose.orientation.x = quat_des_.v(0);
  home_robot_right.pose.orientation.y = quat_des_.v(1);
  home_robot_right.pose.orientation.z = quat_des_.v(2);
  home_robot_right.pose.orientation.w = quat_des_.a;

  home_robot_right.id = id_class;

  desired_hand_publisher_right.publish( home_robot_right );
  // ROS_INFO("Sending robot to home in topic: %s", desired_hand_right_pose_topic_.c_str() );

  tf::Transform tfHandTrasform1;
  tf::poseMsgToTF( home_robot_right.pose, tfHandTrasform1);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"trash_robot_right") );


  finish = false;

}


void Pos_trash::SendTrashPosLeft()
{
  desperate_housewife::handPoseSingle home_robot_left;

  double roll_r,pitch_r,yaw_r;
  nh.param<double>("/trash/left_arm_position_x", home_robot_left.pose.position.x, -0.75022);
  nh.param<double>("/trash/left_arm_position_y",  home_robot_left.pose.position.y,  0.47078);
  nh.param<double>("/trash/left_arm_position_z", home_robot_left.pose.position.z, 0.74494);
  nh.param<double>("/trash/left_arm_A_yaw", yaw_r,  0.334);
  nh.param<double>("/trash/left_arm_B_pitch", pitch_r, -0.08650);
  nh.param<double>("/trash/left_arm_C_roll", roll_r, -0.5108);

  KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll_r,pitch_r,yaw_r);

  Rot_matrix_r.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

  home_robot_left.pose.orientation.x = quat_des_.v(0);
  home_robot_left.pose.orientation.y = quat_des_.v(1);
  home_robot_left.pose.orientation.z = quat_des_.v(2);
  home_robot_left.pose.orientation.w = quat_des_.a;

  home_robot_left.id = id_class;

  desired_hand_publisher_left.publish( home_robot_left );
  // ROS_INFO("Sending robot to home in topic: %s", desired_hand_left_pose_topic_.c_str() );

  tf::Transform tfHandTrasform1;
  tf::poseMsgToTF( home_robot_left.pose, tfHandTrasform1);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"trash_robot_left") );


  finish = false;

}
