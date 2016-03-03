#include <Home_state.h>
// #include <check_error.hpp>

Home_state::Home_state(const shared& m):data(m)
{

  nh.param<std::string>("/right_arm/controller", control_topic_right, "PotentialFieldControl");
  nh.param<std::string>("/left_arm/controller", control_topic_left, "PotentialFieldControl");

  ROS_INFO_STREAM("**** control_topic_right: "<< control_topic_right.c_str() );
  
  /*reads the error*/
  error_topic_right = "/right_arm/" + control_topic_right + "/error_id";
  error_sub_right = nh.subscribe(error_topic_right.c_str(), 1, &Home_state::Error_info_right, this);

  error_topic_left = "/left_arm/" + control_topic_left + "/error_id";
  error_sub_left = nh.subscribe(error_topic_left.c_str(), 1, &Home_state::Error_info_left, this);


  /*sends a hand pose*/
  desired_hand_right_pose_topic_ = "/right_arm/" + control_topic_right + "/command";
  desired_hand_right_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);


  desired_hand_left_pose_topic_ = "/left_arm/" + control_topic_left + "/command";
  desired_hand_left_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);


  nh.param<std::string>("/right_arm/" + control_topic_right + "/root_name", base_frame_, "world");

  /*subscribe to start commad for both arm*/
  sub_command_start = nh.subscribe("/desperate_housewife/start_controller", 1, &Home_state::command_start, this);

  /*subcribe to start command for single arm*/
  sub_start_r = nh.subscribe(("/right_arm/" + control_topic_right + "/start_controller").c_str(), 1, &Home_state::state_right, this);
  sub_start_l = nh.subscribe(("/left_arm/" + control_topic_left + "/start_controller").c_str(), 1, &Home_state::state_left, this);

  /*publish the start to PotentialFieldContol*/
  ros_pub_start_right = nh.advertise<std_msgs::Bool > ("/right_arm/" + control_topic_right + "/start_controller",1);
  ros_pub_start_left = nh.advertise<std_msgs::Bool > ("/left_arm/" + control_topic_left + "/start_controller",1);

  /*id class for msg*/
  id_class = static_cast<int>(transition_id::Vito_home);

  vect_error.resize(2);
  KDL::Twist TEMP;
  SetToZero(TEMP);
  vect_error[0] = TEMP;
  vect_error[1] = TEMP;

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
  KDL::Twist e_right;
  tf::twistMsgToKDL (error_msg->error_, e_right);
  id_error_msgs = error_msg->id;
  // Arm_used = 0;
  vect_error[0] = e_right;
  // std::cout<<"error: "<<error_msg->data[0]<<error_msg->data[1]<<error_msg->data[2]<<std::endl;
}

void Home_state::Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Twist e_left;
  tf::twistMsgToKDL (error_msg->error_, e_left);

  id_error_msgs = error_msg->id;
  // Arm_used = 1;
  vect_error[1] = e_left;
  // std::cout<<"error: "<<error_msg->data[0]<<error_msg->data[1]<<error_msg->data[2]<<std::endl;
}


void Home_state::command_start(const std_msgs::Bool::ConstPtr& msg)
{ 
  std_msgs::Bool msg_send;
  msg_send.data = true;
  ros_pub_start_left.publish(msg_send);
  ros_pub_start_right.publish(msg_send);
  start_flag = true;
  Arm_used = 2;
}

void Home_state::state_right(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("Message to go Home on right arm received");
  start_flag = true;
  Arm_used = 0;
}

void Home_state::state_left(const std_msgs::Bool::ConstPtr& msg)
{
  start_flag = true;
  Arm_used = 1;
}

void Home_state::reset()
{
  id_error_msgs = 100;
  finish = false;
  // Arm_used =  data.arm_to_use;
}


void Home_state::run()
{
  if(start_flag == true)
    {
      // Arm = data.arm_to_use;
      e_ = vect_error[0] + vect_error [1];

      if( ((id_class != id_error_msgs) && (!IsEqual(e_))) )
        {
          switch(Arm_used)
            {
            case 0: /*use only the righ arm*/
              {
                SendHomeRobot_right();
                finish = false;

                break;
              }
            case 1: /*use only the left arm*/
              {
                SendHomeRobot_left();
                finish = false;
                break;
              }
            case 2: /*use both arm*/
              {
                SendHomeRobot_right();
                SendHomeRobot_left();
                finish = false;

                break;
              }
            }
        }

      else if( (id_class == id_error_msgs) && (IsEqual(e_)) )
        finish = true;

      else if ((id_class != id_error_msgs) && (IsEqual(e_)))
      {
          switch(Arm_used)
          {
            case 0: /*use only the righ arm*/
              {
                SendHomeRobot_right();
                finish = false;
                break;
              }
            case 1: /*use only the left arm*/
              {
                SendHomeRobot_left();
                finish = false;
                break;
              }
            case 2: /*use both arm*/
              {
                SendHomeRobot_right();
                SendHomeRobot_left();
                finish = false;

                break;
              }
          }
      }
      Arm_used = data.arm_to_use;


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
  // std::cout<<"send vito right"<<std::endl;
  double roll_r,pitch_r,yaw_r;
  nh.param<double>("/home/right_arm_position_x", home_robot_right.pose.position.x, -0.75022);
  nh.param<double>("/home/right_arm_position_y",  home_robot_right.pose.position.y,  0.47078);
  nh.param<double>("/home/right_arm_position_z", home_robot_right.pose.position.z, 0.74494);
  nh.param<double>("/home/right_arm_A_yaw", yaw_r,  0.334);
  nh.param<double>("/home/right_arm_B_pitch", pitch_r, -0.08650);
  nh.param<double>("/home/right_arm_C_roll", roll_r, -0.5108);

  KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll_r,pitch_r,yaw_r);

  Rot_matrix_r.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

  home_robot_right.pose.orientation.x = quat_des_.v(0);
  home_robot_right.pose.orientation.y = quat_des_.v(1);
  home_robot_right.pose.orientation.z = quat_des_.v(2);
  home_robot_right.pose.orientation.w = quat_des_.a;

  home_robot_right.id = id_class;

  desired_hand_right_pose_publisher_.publish( home_robot_right );
  // ROS_INFO("Sending robot to home in topic: %s", desired_hand_right_pose_topic_.c_str() );

  tf::Transform tfHandTrasform1;
  tf::poseMsgToTF( home_robot_right.pose, tfHandTrasform1);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"home_robot_right") );
}

void Home_state::SendHomeRobot_left()
{
  desperate_housewife::handPoseSingle home_robot_left;
  // std::cout<<"send vito left"<<std::endl;
  double roll_r,pitch_r,yaw_r;
  nh.param<double>("/home/left_arm_position_x", home_robot_left.pose.position.x, -0.75022);
  nh.param<double>("/home/left_arm_position_y",  home_robot_left.pose.position.y,  0.47078);
  nh.param<double>("/home/left_arm_position_z", home_robot_left.pose.position.z, 0.74494);
  nh.param<double>("/home/left_arm_A_yaw", yaw_r,  0.334);
  nh.param<double>("/home/left_arm_B_pitch", pitch_r, -0.08650);
  nh.param<double>("/home/left_arm_C_roll", roll_r, -0.5108);

  KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll_r,pitch_r,yaw_r);

  Rot_matrix_r.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

  home_robot_left.pose.orientation.x = quat_des_.v(0);
  home_robot_left.pose.orientation.y = quat_des_.v(1);
  home_robot_left.pose.orientation.z = quat_des_.v(2);
  home_robot_left.pose.orientation.w = quat_des_.a;

  home_robot_left.id = id_class;

  desired_hand_left_pose_publisher_.publish( home_robot_left );
  // ROS_INFO("Sending robot to home in topic: %s", desired_hand_right_pose_topic_.c_str() );

  tf::Transform tfHandTrasform1;
  tf::poseMsgToTF( home_robot_left.pose, tfHandTrasform1);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"home_robot_left") );
}

// switch(Arm_used)
//         {
//             case 0: /*use only the righ arm*/
//             {   
//                 std::cout<<"case 0"<<std::endl;
//                 if( ((id_class != id_error_msgs) && (!IsEqual(e_right))) || ((id_class != id_error_msgs) && (IsEqual(e_right))) )
//                 {
//                     SendHomeRobot_right(); 
//                     finish = false;
//                 }
//                 if( (id_class == id_error_msgs) && (IsEqual(e_right)) )
//                     finish = true;
//                 break;
//             }
//             case 1: /*use only the left arm*/
//             {
//                 if( ((id_class != id_error_msgs) && (!IsEqual(e_left))) || ((id_class != id_error_msgs) && (IsEqual(e_left))) )
//                 {
//                     SendHomeRobot_left(); 
//                     finish = false;
//                 }
//                 if( (id_class == id_error_msgs) && (IsEqual(e_left)) )
//                     finish = true;
//                 break;
//             }
//             case 2: /*use both arm*/
//             {
//                 // std::cout<<"**********+send both arm at home position"<<std::endl;
//                 if( ((id_class != id_error_msgs) && (!IsEqual(e_right))) || ((id_class != id_error_msgs) && (!IsEqual(e_left))) )
//                 {
//                     SendHomeRobot_left();
//                     SendHomeRobot_right(); 
//                     finish = false;
//                 }
//                  if( (id_class == id_error_msgs) && (IsEqual(e_right)) &&  (IsEqual(e_left)) )
//                     finish = true;
//                 break;
//             }
//         }       
//     }
//     else
//         finish = false;
