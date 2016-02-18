#include "overtune_state.h"

Overtune_state::Overtune_state(const shared& m):data(m)
{

  nh.param<std::string>("/right_arm/PotentialFieldControl/error_id", error_topic_right, "/right_arm/PotentialFieldControl/error_id");
  error_sub_right = nh.subscribe(error_topic_right, 1, &Overtune_state::Error_info_right, this);

  nh.param<std::string>("/left_arm/PotentialFieldControl/error_id", error_topic_left, "/left_arm/PotentialFieldControl/error_id");
  error_sub_left = nh.subscribe(error_topic_left, 1, &Overtune_state::Error_info_left, this);

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

  id_class = static_cast<int>(transition_id::Vito_Overtune);
  vect_error.resize(2);
  KDL::Twist TEMP;
  SetToZero(TEMP);
  vect_error[0] = TEMP;
  vect_error[1] = TEMP;

  double x,y,z,rotx,roty,rotz;
  
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
}

std::map< transition, bool > Overtune_state::getResults()
{
  std::map< transition, bool > results;

  if(finish == true)
    {
      results[transition::Error_arrived] = finish;
    }

  return results;
}


void Overtune_state::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Twist e_right;
  tf::twistMsgToKDL (error_msg->error_, e_right);
  id_error_msgs = error_msg->id;
  vect_error[0] = e_right;
}

void Overtune_state::Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Twist e_left;
  tf::twistMsgToKDL (error_msg->error_, e_left);
  id_error_msgs = error_msg->id;
  vect_error[1] = e_left;
}


void Overtune_state::run()
{
  e_ = vect_error[0] + vect_error[1];

  if((id_class != id_error_msgs) && IsEqual(e_))
    {
      InfoArm();
      New_Hand_Position_right.pose.position.z = New_Hand_Position_right.pose.position.z + 0.20;
      New_Hand_Position_right.pose.position.x = New_Hand_Position_right.pose.position.x ; /*move along the x axis for push down from the table*/
      New_Hand_Position_right.id = id_class;
      desired_hand_publisher_right.publish( New_Hand_Position_right );

      New_Hand_Position_left.pose.position.z = New_Hand_Position_left.pose.position.z + 0.20;
      New_Hand_Position_left.pose.position.x = New_Hand_Position_left.pose.position.x - 0.10; /*move along the x axis for push down from the table*/
      New_Hand_Position_left.id = id_class;
      desired_hand_publisher_left.publish( New_Hand_Position_left );
      
      overturn_check = 0;
      finish = false;
      tf::Transform tfHandTrasform;
      tf::poseMsgToTF( New_Hand_Position_right.pose, tfHandTrasform);
      tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str()," right_ribalto2") );
      tf::Transform tfHandTrasform2;
      tf::poseMsgToTF( New_Hand_Position_left.pose, tfHandTrasform2);
      tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str()," left_ribalto2") );

    }
  else if((id_class = id_error_msgs) && IsEqual(e_))
    {

      switch(overturn_check)
        {
        InfoArm();
        case 0: //replace table
          {

            New_Hand_Position_right.pose.position.z = New_Hand_Position_right.pose.position.z - 0.20;
            New_Hand_Position_right.pose.position.x = New_Hand_Position_right.pose.position.x + 0.20; /*move along the x axis for push down from the table*/
            New_Hand_Position_right.id = id_class;
            desired_hand_publisher_right.publish( New_Hand_Position_right );

            New_Hand_Position_left.pose.position.z = New_Hand_Position_left.pose.position.z - 0.20;
            New_Hand_Position_left.pose.position.x = New_Hand_Position_left.pose.position.x + 0.20; /*move along the x axis for push down from the table*/
            New_Hand_Position_left.id = id_class;
            desired_hand_publisher_left.publish( New_Hand_Position_left );

            overturn_check = 1;
            finish = false;

            tf::Transform tfHandTrasform;
            tf::poseMsgToTF( New_Hand_Position_right.pose, tfHandTrasform);
            tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str()," right_ribalto3") );
            tf::Transform tfHandTrasform2;
            tf::poseMsgToTF( New_Hand_Position_left.pose, tfHandTrasform2);
            tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str()," left_ribalto3") );

            break;
          }

        case 1: //open hand and go to home
          {
            finish = true;
            break;
          }
        }
    }
}

bool Overtune_state::isComplete()
{
  return finish;
}

std::string Overtune_state::get_type()
{
  return "Overtune_state";
}


bool Overtune_state::IsEqual(KDL::Twist E_pf)
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


void Overtune_state::InfoArm()
{

  tf::StampedTransform hand_rigth;
  listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
  listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

  New_Hand_Position_right.pose.position.x = hand_rigth.getOrigin().x() ; /*move along the x axis for push down from the table*/
  New_Hand_Position_right.pose.position.y = hand_rigth.getOrigin().y();
  New_Hand_Position_right.pose.position.z = hand_rigth.getOrigin().z();
  KDL::Rotation matrix_temp;
  tf::quaternionTFToKDL(hand_rigth.getRotation(),matrix_temp);

  matrix_temp.GetQuaternion(New_Hand_Position_right.pose.orientation.x,New_Hand_Position_right.pose.orientation.y,
                            New_Hand_Position_right.pose.orientation.z,New_Hand_Position_right.pose.orientation.w);



  tf::StampedTransform hand_left;
  listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
  listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

  New_Hand_Position_left.pose.position.x = hand_left.getOrigin().x() ; /*move along the x axis for push down from the table*/
  New_Hand_Position_left.pose.position.y = hand_left.getOrigin().y();
  New_Hand_Position_left.pose.position.z = hand_left.getOrigin().z();
  KDL::Rotation matrix_temp_l;
  tf::quaternionTFToKDL(hand_left.getRotation(),matrix_temp_l);

  matrix_temp_l.GetQuaternion(New_Hand_Position_left.pose.orientation.x,New_Hand_Position_left.pose.orientation.y,
                              New_Hand_Position_left.pose.orientation.z,New_Hand_Position_left.pose.orientation.w);

}
