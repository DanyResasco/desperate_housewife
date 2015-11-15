#include <ros/ros.h>
#include <ros/console.h>
#include <desperate_mind.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "desperate_mind_node");
  DesperateDecisionMaker node;
  // node.SendVitoHome();

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
  //to send vito at home
  nh.param<std::string>("PotentialFieldControl/home_desperate_left" , home_left_topic_, "/left_arm/PotentialFieldControl/home_left");
  left_home_publisher_ = nh.advertise<std_msgs::Bool> (home_left_topic_.c_str(),1);

  nh.param<std::string>("PotentialFieldControl/home_desperate_right", home_right_topic_, "/right_arm/PotentialFieldControl/home_right");
  right_home_publisher_ = nh.advertise<std_msgs::Bool> (home_right_topic_.c_str(),1);

  //to monitoring error
  nh.param<std::string>("/left_arm/PotentialFieldControl/error", error_topic_left, "/left_arm/PotentialFieldControl/error");
  error_sub_left = nh.subscribe(error_topic_left, 1, &DesperateDecisionMaker::Error_info_left, this);

  nh.param<std::string>("/right_arm/PotentialFieldControl/error", error_topic_right, "/right_arm/PotentialFieldControl/error");
  error_sub_right = nh.subscribe(error_topic_right, 1, &DesperateDecisionMaker::Error_info_right, this);

  //to close and open hand
  nh.param<std::string>("/left_hand/joint_trajectory_controller/command", hand_close_left, "/left_hand/joint_trajectory_controller/command");
  nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_close_right, "/right_hand/joint_trajectory_controller/command");
  hand_publisher_left = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_left.c_str(), 1000);
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);

  //to start the controller
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_left, "/left_arm/PotentialFieldControl/start_controller");
  left_start_controller_pub = nh.advertise<desperate_housewife::Start > (start_topic_left.c_str(),1);
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_right, "/right_arm/PotentialFieldControl/start_controller");
  right_start_controller_pub = nh.advertise<desperate_housewife::Start > (start_topic_right.c_str(),1);

  //send pose to remove objects
  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right", desired_hand_right_pose_topic_, "/PotentialFieldControl/desired_hand_right_pose");
  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left", desired_hand_left_pose_topic_, "/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "vito_anchor");

  //read if object is to grasp or to remove
  nh.param<std::string>("/PotentialFieldControl/objects_info_r", obj_info_topic_r, "/PotentialFieldControl/objects_info_right");
  objects_info_right_sub = nh.subscribe(obj_info_topic_r.c_str(),1, &DesperateDecisionMaker::ObjOrObst_right,this);
  nh.param<std::string>("/PotentialFieldControl/objects_info_l", obj_info_topic_l, "/PotentialFieldControl/objects_info_left");
  objects_info_left_sub = nh.subscribe(obj_info_topic_l.c_str(),1, &DesperateDecisionMaker::ObjOrObst_left,this);

  //stop the filter publisher
  // nh.param<std::string>("/PotentialFieldControl/stop_pub_right", stop_pub_filter_topic_r, "/PotentialFieldControl/stop_pub_filter_right");
  // stop_publisher_r = nh.advertise<std_msgs::UInt16 > (stop_pub_filter_topic_r.c_str(),1);

  // nh.param<std::string>("/PotentialFieldControl/stop_pub_left", stop_pub_filter_topic_l, "/PotentialFieldControl/stop_pub_filter_right");
  // stop_publisher_l = nh.advertise<std_msgs::UInt16 > (stop_pub_filter_topic_l.c_str(),1);

  // nh.param<std::string>("PotentialFieldControl/desperate_msg_demo2", desperate_msg_demo2, "/PotentialFieldControl/desperate_msg_demo2");
  // desperate_msg_demo2_publisher_ = nh.advertise<std_msgs::Bool> (desperate_msg_demo2.c_str(),1);

  // nh.param<std::string>("/left_hand/PotentialFieldControl/Reject_obstacle_left", Reject_obstalces_topic_left, "/PotentialFieldControl/Reject_obstacle_list_left");
  // Reject_obstacles_publisher_left = nh.advertise<desperate_housewife::fittedGeometriesSingle > (Reject_obstalces_topic_left.c_str(),1);

  // nh.param<std::string>("/right_hand/PotentialFieldControl/Reject_obstacle_right", Reject_obstalces_topic_right, "/PotentialFieldControl/Reject_obstacle_list_right");
  // Reject_obstacles_publisher_right = nh.advertise<desperate_housewife::fittedGeometriesSingle > (Reject_obstalces_topic_right.c_str(),1);
 
}

void DesperateDecisionMaker::SendVitoHome()
{
    if(home == 1)
    {
      ROS_INFO("*****Send vito at home");
      std_msgs::Bool home_vito;
      home_vito.data = true;
      left_home_publisher_.publish(home_vito);
      right_home_publisher_.publish(home_vito);
      home = 0;
      stop_home = 1;
      ObjOrObst = 0;
    }
}

void DesperateDecisionMaker::ObjOrObst_right(const std_msgs::UInt16::ConstPtr& obj_msg)
{
  whichArm = 0;
  ObjOrObst = obj_msg->data;
  arrived = 1;
  //stop the generator pose
  // std_msgs::UInt16 stop;
  // stop.data = 1;
  // stop_publisher_r.publish(stop); 
}

void DesperateDecisionMaker::ObjOrObst_left(const std_msgs::UInt16::ConstPtr& obj_msg)
{
  whichArm = 1;
  ObjOrObst = obj_msg->data;
  arrived = 1; 
  //stop the generator pose
  // std_msgs::UInt16 stop;
  // stop.data = 1;
  // stop_publisher_l.publish(stop) ;
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
  KDL::Twist error_treshold;
  //if potential filed has riceved one msg 
  SendVitoHome();
  KDL::Twist e_;
  tf::twistMsgToKDL (error_msg->error_, e_);

  if(ObjOrObst == 1) // to remove
  {
      vel.data[0] = -x;
      vel.data[1] = -y;
      vel.data[2] = -z;
      rot.data[0] = -rot_x;
      error_treshold.vel = vel;
      error_treshold.rot = rot;
  }
  //to grasp
  else
  {
      error_treshold.vel = vel;
      error_treshold.rot = rot;
  }

  if(error_msg->arrived == 1)
  {
    if(Equal(e_, error_treshold, 0.05))
    { 

      if(stop_home == 1)
      {
        start_controller.start_left = 1;
        start_controller.stop = 0;
        left_start_controller_pub.publish(start_controller);
        stop_home = 0;
      }
      else
      {

        if(restart != 1)
        {
          ControllerStartAndNewPOse(error_msg);
        }
        else
        {
          start_controller.stop = 0;
          left_start_controller_pub.publish(start_controller);
          restart = 0;
        }
      }

    }
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
  KDL::Twist error_treshold;
  SendVitoHome();
  
   
  if(error_msg->arrived == 1)
  {
    // std::cout<<"ricevuto errore right"<<std::endl;
    KDL::Twist e_;

    if(ObjOrObst == 1)
    {
      
     vel.data[2] = -z;

     error_treshold.vel = vel;
     error_treshold.rot = rot;
    }
    else
    {
      
      rot.data[1] = rot_y;
    
      error_treshold.vel = vel;
      error_treshold.rot = rot;
    }

    // KDL::Twist error_treshold(vel,rot);
    tf::twistMsgToKDL (error_msg->error_, e_);

   if(Equal(e_, error_treshold, 0.05))
    {
      
      if(stop_home == 1)
      {
        start_controller.start_right = 1;
        start_controller.stop = 0;
        right_start_controller_pub.publish(start_controller);
        stop_home = 0;
      }
      else
      {

       if(restart != 1)
        {
          ControllerStartAndNewPOse(error_msg);
        }
        else
        {
          start_controller.stop = 1;
          right_start_controller_pub.publish(start_controller);
          restart = 0;
        }
      }

    }
  }
 
}


void DesperateDecisionMaker::ControllerStartAndNewPOse(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  //start decison hand maker
  std::cout<<"ControllerStartAndNewPOse"<<std::endl;
  //wait both arms
  if((start_controller_right !=0) && (start_controller_left !=0))
  {
    // std::cout<<"error_msg->ObjOrObst: "<<error_msg->ObjOrObst<<std::endl;
    switch(ObjOrObst)
    {
      
      case 0: //object to grasp
      {     
            std::cout<<"case 0"<<std::endl;   
            trajectory_msgs::JointTrajectory msg_jointT_hand;
            msg_jointT_hand.joint_names.resize(1);
            desperate_housewife::handPoseSingle new_obj_pos_remove; //new hand position
            //hand msg
            new_obj_pos_remove.pose = error_msg->pose_hand;
            msg_jointT_hand.points.resize(1);
            msg_jointT_hand.points[0].positions.resize(1);
            msg_jointT_hand.points[0].positions[0] = 1.0;
            msg_jointT_hand.points[0].time_from_start = ros::Duration(2); // 2s;

            if(whichArm == 1)
            {
                new_obj_pos_remove.pose.position.y = new_obj_pos_remove.pose.position.y - 0.3;
                msg_jointT_hand.joint_names[0] = left_hand_synergy_joint.c_str();
                hand_publisher_left.publish(msg_jointT_hand);
                desired_hand_publisher_left.publish( new_obj_pos_remove );
                
            }
            else
            {
                new_obj_pos_remove.pose.position.y = new_obj_pos_remove.pose.position.y + 0.3;
                msg_jointT_hand.joint_names[0] = right_hand_synergy_joint.c_str();
                hand_publisher_right.publish(msg_jointT_hand);
                desired_hand_publisher_right.publish( new_obj_pos_remove );
               
            }


            restart = 1;

            tf::Transform tfHandTrasform2;
            tf::poseMsgToTF( new_obj_pos_remove.pose, tfHandTrasform2);  
            tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"Moveobj_") );
          break;
      }

      // case 1: //obstacle
      //     std::cout<<"case 1"<<std::endl; 
      //       break;

      case 1: //obstacle to remove
      {     std::cout<<"case 1"<<std::endl;     
            desperate_housewife::handPoseSingle New_Hand_Position;
            New_Hand_Position.pose = error_msg->pose_hand;

            trajectory_msgs::JointTrajectory msg_jointT_hand;
            msg_jointT_hand.joint_names.resize(1);
            //hand msg
            msg_jointT_hand.points.resize(1);
            msg_jointT_hand.points[0].positions.resize(1);
            msg_jointT_hand.points[0].positions[0] = 0.0;
            msg_jointT_hand.points[0].time_from_start = ros::Duration(2); // 2s;

            if(whichArm == 1)
            {
                New_Hand_Position.pose.position.y = New_Hand_Position.pose.position.y - 0.10;
                msg_jointT_hand.joint_names[0] = left_hand_synergy_joint.c_str();
                hand_publisher_left.publish(msg_jointT_hand);
                desired_hand_publisher_left.publish( New_Hand_Position );
              
            }
            else
            {
                New_Hand_Position.pose.position.y = New_Hand_Position.pose.position.y + 0.15;
                msg_jointT_hand.joint_names[0] = right_hand_synergy_joint.c_str();
                hand_publisher_right.publish(msg_jointT_hand);
                desired_hand_publisher_right.publish( New_Hand_Position );
               
            }
            restart = 1;

            tf::Transform tfHandTrasform;
            tf::poseMsgToTF( New_Hand_Position.pose, tfHandTrasform);  
            tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"ObstacleReject_new_pose") ); 
         break;
      }
      
    }
  }

}