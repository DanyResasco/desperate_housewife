#include "Removed_state.h"

Removed_moves::Removed_moves(const shared& m):data(m)
{
  nh.param<std::string>("/right_arm/PotentialFieldControl/error_id", error_topic_right, "/right_arm/PotentialFieldControl/error_id");
  error_sub_right = nh.subscribe(error_topic_right, 1, &Removed_moves::Error_info_right, this);

  nh.param<std::string>("/left_arm/PotentialFieldControl/error_id", error_topic_left, "/left_arm/PotentialFieldControl/error_id");
  error_sub_left = nh.subscribe(error_topic_left, 1, &Removed_moves::Error_info_left, this);

  // nh.param<std::string>("/right_arm/PotentialFieldControl/topic_desired_reference", desired_hand_right_pose_topic_, "/right_arm/PotentialFieldControl/command");
  // desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  std::string string_temp;

  nh.param<std::string>("/right_arm/PotentialFieldControl/topic_desired_reference", string_temp, "command");
  desired_hand_right_pose_topic_ = std::string("/right_arm/PotentialFieldControl/") + string_temp;
  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);


  std::string string_temp_l;

  nh.param<std::string>("/left_arm/PotentialFieldControl/topic_desired_reference", string_temp_l, "command");
  desired_hand_left_pose_topic_ = std::string("/left_arm/PotentialFieldControl/") + string_temp_l;
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);


  nh.param<std::string>("/right_arm/PotentialFieldControl/root_name", base_frame_, "world");

  nh.param<std::string>("/right_arm/PotentialFieldControl/tip_name", right_hand_frame_, "right_hand_palm_ref_link");
  nh.param<std::string>("/left_arm/PotentialFieldControl/tip_name", left_hand_frame_, "left_hand_palm_ref_link");


    srv_reset = nh.subscribe("/reset",1, &Removed_moves::resetCallBack, this);
  id_class = static_cast<int>(transition_id::Vito_Removed);


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

  vect_error.resize(2);
  KDL::Twist temp;
  SetToZero(temp);
  vect_error[0] = temp;
  vect_error[1] = temp;

}

std::map< transition, bool > Removed_moves::getResults()
{

  std::map< transition, bool > results;

  if(home_reset == true)
  {
    results[transition::home_reset] = finish;    
  }
  else
      results[transition::Error_arrived] = finish;
  return results;
}


void Removed_moves::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Twist e_r;
  tf::twistMsgToKDL (error_msg->error_, e_r);
  id_error_msgs_r = error_msg->id;
  vect_error[0] = e_r;
}


void Removed_moves::Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Twist e_l;
  tf::twistMsgToKDL (error_msg->error_, e_l);
  id_error_msgs_l = error_msg->id;
  vect_error[1] = e_l;
}



void Removed_moves::run()
{
  e_ = vect_error[0] + vect_error[1];

  switch(data.arm_to_use)
    {
    case 0: //right
      {
        if((id_class != id_error_msgs_r) && IsEqual(e_))
          {
            RemObjRight();

          }
        else if((id_class = id_error_msgs_r) && IsEqual(e_))
          {
            finish = true;
          }
        else if( (id_class != id_error_msgs_r) && (!IsEqual(e_)))
          {
            finish = false;
          }
        break;
      }
    case 1:
      {
        if((id_class != id_error_msgs_l) && IsEqual(e_))
          {
            RemObjLeft();

          }
        else if((id_class = id_error_msgs_l) && IsEqual(e_))
          {
            finish = true;
          }
        else if( (id_class != id_error_msgs_l) && (!IsEqual(e_)))
          {
            finish = false;
          }
        break;
      }
    }
}

bool Removed_moves::isComplete()
{
  return finish;
}

// void Removed_moves::reset()
// {
//   finish = false;
// }

std::string Removed_moves::get_type()
{
  return "Removed_moves";
}


bool Removed_moves::IsEqual(KDL::Twist E_pf)
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


void Removed_moves::RemObjRight()
{
  
  tf::StampedTransform hand_rigth;
  listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
  listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

  New_Hand_Position.pose.position.x = hand_rigth.getOrigin().x() - 0.20; /*move along the x axis for push down from the table*/
  New_Hand_Position.pose.position.y = hand_rigth.getOrigin().y();
  New_Hand_Position.pose.position.z = hand_rigth.getOrigin().z();
  KDL::Rotation matrix_temp;
  tf::quaternionTFToKDL(hand_rigth.getRotation(),matrix_temp);

  matrix_temp.GetQuaternion(New_Hand_Position.pose.orientation.x,New_Hand_Position.pose.orientation.y,
                            New_Hand_Position.pose.orientation.z,New_Hand_Position.pose.orientation.w);


  New_Hand_Position.id = id_class;

  desired_hand_publisher_right.publish( New_Hand_Position );


  tf::Transform tfHandTrasform;
  tf::poseMsgToTF( New_Hand_Position.pose, tfHandTrasform);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"ObstacleReject_new_pose") );
  finish = false;
}

void Removed_moves::RemObjLeft()
{
  tf::StampedTransform hand_left;

  listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
  listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

  New_Hand_Position.pose.position.x = hand_left.getOrigin().x() - 0.20; /*move along the x axis for push down from the table*/
  New_Hand_Position.pose.position.y = hand_left.getOrigin().y();
  New_Hand_Position.pose.position.z = hand_left.getOrigin().z();
  KDL::Rotation matrix_temp;
  tf::quaternionTFToKDL(hand_left.getRotation(),matrix_temp);

  matrix_temp.GetQuaternion(New_Hand_Position.pose.orientation.x,New_Hand_Position.pose.orientation.y,
                            New_Hand_Position.pose.orientation.z,New_Hand_Position.pose.orientation.w);

  New_Hand_Position.id = id_class;

  desired_hand_publisher_left.publish( New_Hand_Position );

  tf::Transform tfHandTrasform;
  tf::poseMsgToTF( New_Hand_Position.pose, tfHandTrasform);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"ObstacleReject_new_pose") );
  finish = false;
}



void Removed_moves::resetCallBack(const std_msgs::Bool::ConstPtr msg)
{
  ROS_INFO("Removed_moves::Reset called");
  home_reset = msg->data;
  finish = true;
  // failed = false;
  // return true;
}


void Removed_moves::reset()
{
  home_reset = false;
  finish = false;
  // failed = false;
}

