#include <ros/ros.h>
#include <ros/console.h>
#include <desperate_mind.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "desperate mind node");
  DesperateDecisionMaker node;

  ROS_INFO("[DesperateDecisionMaker] Node is ready");

  double spin_rate = 10;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);

  ros::Rate rate(spin_rate); 

  while (node.nh.ok())
  {
    ros::spinOnce(); 
    rate.sleep();
  }
  return 0;
}

DesperateDecisionMaker::DesperateDecisionMaker()
{

  nh.param<std::string>("PotentialFieldControl/home_desperate_left" , home_left_topic_, "/left_arm/PotentialFieldControl/home_left");
  left_home_publisher_ = nh.advertise<std_msgs::Bool> (home_left_topic_.c_str(),1);

  nh.param<std::string>("PotentialFieldControl/home_desperate_right", home_right_topic_, "/right_arm/PotentialFieldControl/home_right");
  right_home_publisher_ = nh.advertise<std_msgs::Bool> (home_right_topic_.c_str(),1);

  nh.param<std::string>("/left_arm/PotentialFieldControl/error", error_topic_left, "/left_arm/PotentialFieldControl/error");
  error_sub_left = nh.subscribe(error_topic_left, 1, &DesperateDecisionMaker::Error_info_left, this);

  nh.param<std::string>("/right_arm/PotentialFieldControl/error", error_topic_right, "/right_arm/PotentialFieldControl/error");
  error_sub_right = nh.subscribe(error_topic_right, 1, &DesperateDecisionMaker::Error_info_right, this);

  
  nh.param<std::string>("/left_hand/joint_trajectory_controller/command", hand_close_left, "/left_hand/joint_trajectory_controller/command");
  nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_close_right, "/right_hand/joint_trajectory_controller/command");
  hand_publisher_left = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_left.c_str(), 1000);
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);


   nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_left, "/left_arm/PotentialFieldControl/start_controller");
  left_start_controller_pub = nh.advertise<desperate_housewife::Start > (start_topic_left.c_str(),1);
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_right, "/right_arm/PotentialFieldControl/start_controller");
  right_start_controller_pub = nh.advertise<desperate_housewife::Start > (start_topic_right.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right", desired_hand_right_pose_topic_, "/PotentialFieldControl/desired_hand_right_pose");
  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left", desired_hand_left_pose_topic_, "/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "vito_anchor");
 
}




void DesperateDecisionMaker::Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Vector vel;
  KDL::Vector rot;
  vel.data[0] = x;
  vel.data[1] = y;
  vel.data[2] = z;
  rot.data[0] = rot_x;
  rot.data[1] = -rot_y;
  rot.data[2] = -rot_z;
  std_msgs::Bool home_vito;
  
  //if potential filed has riceved one msg 
  if(error_msg->arrived == 1)
  { 
    // std::cout<<"dentro"<<std::endl;
   
    KDL::Twist e_;
    KDL::Twist error_treshold(vel,rot);
    tf::twistMsgToKDL (error_msg->error_, e_);
    // std::cout<<"e_: "<<error_msg->error_<<std::endl;

    if(Equal(e_, error_treshold, 0.05))
    {
      if(error_msg->home == 1)
      {
        start_controller_left = 1;
        home_vito.data = false;
        left_home_publisher_.publish(home_vito);
        start_controller.start_left = 1;
        left_start_controller_pub.publish(start_controller);
      }
      else
      { 
        // std::cout<<"error_msg->obj"<<error_msg->obj<<std::endl;
        if(error_msg->obj == 1)
        {
          ControllerStartAndNewPOse(error_msg);
        }
      }

    }
    // else
      // ROS_INFO("NON CI SONO ARRIVATO");
  }

  else
  {
    std::cout<<"invio home left"<<std::endl;
    home_vito.data = true;
    left_home_publisher_.publish(home_vito);
  }
}

void DesperateDecisionMaker::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Vector vel;
  KDL::Vector rot;
  vel.data[0] = x;
  vel.data[1] = -y;
  vel.data[2] = z;
  rot.data[0] = -rot_x;
  rot.data[1] = -rot_y;
  rot.data[2] = -rot_z;
  std_msgs::Bool home_vito;

   // std::cout<<"ricevuto errore right"<<std::endl;
  if(error_msg->arrived == 1)
  {
    KDL::Twist e_;
    KDL::Twist error_treshold(vel,rot);
    tf::twistMsgToKDL (error_msg->error_, e_);
    // std::cout<<"E: "<<e_<<std::endl;
  
   if(Equal(e_, error_treshold, 0.05))
    {
      if(error_msg->home == 1)
      {     
        start_controller_right = 1;
        home_vito.data = false;
        right_home_publisher_.publish(home_vito);
        start_controller.start_right = 1;
        right_start_controller_pub.publish(start_controller);
      }
      else
      {
        // std::cout<<"error_msg->obj"<<error_msg->obj<<std::endl;
        if(error_msg->obj == 1)
        {
          ControllerStartAndNewPOse(error_msg);
        }
      }
    }

    // else
    //   ROS_INFO("NON CI SONO ARRIVATO");

  }

  else
  {
    std::cout<<"invio home right"<<std::endl;
    home_vito.data = true;
    right_home_publisher_.publish(home_vito);
  } 
 
}


void DesperateDecisionMaker::ControllerStartAndNewPOse(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  //start decison hand maker
  std::cout<<"ControllerStartAndNewPOse"<<std::endl;
  if((start_controller_right !=0) && (start_controller_left !=0))
  {
    switch(error_msg->ObjOrObst)
    {
      
      case 0: //object to grasp
      {     
            std::cout<<"0"<<std::endl;   
            trajectory_msgs::JointTrajectory msg_jointT_hand;
            msg_jointT_hand.joint_names.resize(1);
            desperate_housewife::handPoseSingle new_obj_pos_remove; //new hand position
            //hand msg
            new_obj_pos_remove.pose = error_msg->pose_hand;
            msg_jointT_hand.points.resize(1);
            msg_jointT_hand.points[0].positions.resize(1);
            msg_jointT_hand.points[0].positions[0] = 1.0;
            msg_jointT_hand.points[0].time_from_start = ros::Duration(2); // 2s;
            std::cout<<"inviato mano"<<std::endl;
            std::cout<<"error_msg->WhichArm: "<<error_msg->WhichArm<<std::endl;
            // if(error_msg->WhichArm == 1)
            // {
            //     new_obj_pos_remove.pose.position.y = new_obj_pos_remove.pose.position.y - 0.3;
            //     msg_jointT_hand.joint_names[0] = left_hand_synergy_joint.c_str();
            //     hand_publisher_left.publish(msg_jointT_hand);
            //     desired_hand_publisher_left.publish( new_obj_pos_remove );
            // }
            // else
            // {
            //     new_obj_pos_remove.pose.position.y = new_obj_pos_remove.pose.position.y + 0.3;
            //     msg_jointT_hand.joint_names[0] = right_hand_synergy_joint.c_str();
            //     hand_publisher_right.publish(msg_jointT_hand);
            //     desired_hand_publisher_right.publish( new_obj_pos_remove );
            // }

            // tf::Transform tfHandTrasform2;
            // tf::poseMsgToTF( new_obj_pos_remove.pose, tfHandTrasform2);  
            // tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"Moveobj_") );
          break;
      }

      case 1: //obstacle
          std::cout<<"1"<<std::endl; 
            break;

      case 2: //obstacle to remove
      {      std::cout<<"2"<<std::endl;     
            desperate_housewife::handPoseSingle New_Hand_Position;
              // geometry_msgs::Pose pos_new, pose_local;
            New_Hand_Position.pose = error_msg->pose_hand;
            New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x - 0.25;
               // std::cout<<"error_msg->WhichArm: "<<error_msg->WhichArm<<std::endl;

            trajectory_msgs::JointTrajectory msg_jointT_hand;
            msg_jointT_hand.joint_names.resize(1);
            //hand msg
            msg_jointT_hand.points.resize(1);
            msg_jointT_hand.points[0].positions.resize(1);
            msg_jointT_hand.points[0].positions[0] = 0.0;
            msg_jointT_hand.points[0].time_from_start = ros::Duration(2); // 2s;

            if(error_msg->WhichArm ==1)
            {
                msg_jointT_hand.joint_names[0] = left_hand_synergy_joint.c_str();
                hand_publisher_left.publish(msg_jointT_hand);
                desired_hand_publisher_left.publish( New_Hand_Position );
            }
            else
            {
                msg_jointT_hand.joint_names[0] = right_hand_synergy_joint.c_str();
                hand_publisher_right.publish(msg_jointT_hand);
                desired_hand_publisher_right.publish( New_Hand_Position );
            }

              tf::Transform tfHandTrasform;
              tf::poseMsgToTF( New_Hand_Position.pose, tfHandTrasform);  
              tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"ObstacleReject_new_pose") ); 
         break;
      }
      case 3:
        break;
    }
  }

}