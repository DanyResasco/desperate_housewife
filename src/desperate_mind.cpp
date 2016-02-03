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
  /*to send vito at home*/
  nh.param<std::string>("PotentialFieldControl/home_desperate_left" , home_left_topic_, "/left_arm/PotentialFieldControl/home_left");
  left_home_publisher_ = nh.advertise<std_msgs::Bool> (home_left_topic_.c_str(),1);

  nh.param<std::string>("PotentialFieldControl/home_desperate_right", home_right_topic_, "/right_arm/PotentialFieldControl/home_right");
  right_home_publisher_ = nh.advertise<std_msgs::Bool> (home_right_topic_.c_str(),1);

  /*to monitoring error*/
  nh.param<std::string>("/left_arm/PotentialFieldControl/error", error_topic_left, "/left_arm/PotentialFieldControl/error");
  error_sub_left = nh.subscribe(error_topic_left, 1, &DesperateDecisionMaker::Error_info_left, this);

  nh.param<std::string>("/right_arm/PotentialFieldControl/error", error_topic_right, "/right_arm/PotentialFieldControl/error");
  error_sub_right = nh.subscribe(error_topic_right, 1, &DesperateDecisionMaker::Error_info_right, this);

  /*to close and open hand*/
  nh.param<std::string>("/left_hand/joint_trajectory_controller/command", hand_close_left, "/left_hand/joint_trajectory_controller/command");
  nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_close_right, "/right_hand/joint_trajectory_controller/command");
  hand_publisher_left = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_left.c_str(), 1000);
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);

  /*to start the controller*/
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_left, "/left_arm/PotentialFieldControl/start_controller");
  left_start_controller_pub = nh.advertise<desperate_housewife::Start > (start_topic_left.c_str(),1);
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_right, "/right_arm/PotentialFieldControl/start_controller");
  right_start_controller_pub = nh.advertise<desperate_housewife::Start > (start_topic_right.c_str(),1);

  /*send pose to remove objects*/
  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right", desired_hand_right_pose_topic_, "/PotentialFieldControl/desired_hand_right_pose");
  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left", desired_hand_left_pose_topic_, "/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "vito_anchor");

  /*read if object is to grasp or to remove*/
  nh.param<std::string>("/PotentialFieldControl/objects_info_r", obj_info_topic_r, "/PotentialFieldControl/objects_info_right");
  objects_info_right_sub = nh.subscribe(obj_info_topic_r.c_str(),1, &DesperateDecisionMaker::ObjOrObst_right,this);
  nh.param<std::string>("/PotentialFieldControl/objects_info_l", obj_info_topic_l, "/PotentialFieldControl/objects_info_left");
  objects_info_left_sub = nh.subscribe(obj_info_topic_l.c_str(),1, &DesperateDecisionMaker::ObjOrObst_left,this);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right", desired_hand_right_pose_topic_, "/right_arm/PotentialFieldControl/desired_hand_right_pose");
  desired_hand_right_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left", desired_hand_left_pose_topic_, "/left_arm/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_left_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);

  /*treshold error*/
  nh.param<double>("/desperate_mind_node/x_treshold",x,0.01);
  nh.param<double>("/desperate_mind_node/y_treshold",y,0.01);
  nh.param<double>("/desperate_mind_node/z_treshold",z,0.01);
  nh.param<double>("/desperate_mind_node/rot_x_treshold",rot_x,0.01);
  nh.param<double>("/desperate_mind_node/rot_y_treshold",rot_y,0.01);
  nh.param<double>("/desperate_mind_node/rot_z_treshold",rot_z,0.01);

  ROS_INFO("Treshold error parameter x: %f, y: %f, z: %f, rot_x: %f, rot_y: %f, rot_z: %f, ", x, y, x, rot_x, rot_y, rot_z);

  nh.param<bool>("use_both_arm",use_both_arm,true);
  
  nh.param<double>("/desperate_mind_node/Info_closed_hand", Info_closed_hand, 0.6);
  // std::cout<<"***** Info_closed_hand: ****"<<Info_closed_hand<<std::endl;

  nh.param<std::string>("/PotentialFieldControl/obstacle_list_left", obstacles_topic_left, "/PotentialFieldControl/obstacle_pose_left");
  obstacles_publisher_left = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstacles_topic_left.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/obstacle_list_right", obstacles_topic_right, "/PotentialFieldControl/obstacle_pose_right");
  obstacles_publisher_right = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstacles_topic_right.c_str(),1);

  nh.param<std::string>("/right_hand/joint_states", hand_joint_position_r, "/right_hand/joint_states");
  hand_info_right = nh.subscribe(hand_joint_position_r.c_str(),1, &DesperateDecisionMaker::HandInforRight,this);

  block_info_hand = 0;
  reject_ = 0;
  overturn_check = 1;
  overturn1_ = 0;
}

void DesperateDecisionMaker::HandInforRight(const sensor_msgs::JointState::ConstPtr &msg)
{
  info_hand = msg->position[28];
  // std::cout<<"info_hand: "<<info_hand<<std::endl;
}


void DesperateDecisionMaker::ObjOrObst_right(const std_msgs::UInt16::ConstPtr& obj_msg)
{
  whichArm = 0;
  ObjOrObst = obj_msg->data;
  arrived_r = 1;
  restart = 0;
}

void DesperateDecisionMaker::ObjOrObst_left(const std_msgs::UInt16::ConstPtr& obj_msg)
{
  whichArm = 1;
  ObjOrObst = obj_msg->data;
  arrived_l = 1; 
  restart = 0;

}


void DesperateDecisionMaker::Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  KDL::Vector vel;
  KDL::Vector rot;
  vel.data[0] = x;
  vel.data[1] = y;
  vel.data[2] = z;
  rot.data[0] = rot_x;
  rot.data[1] = rot_y;
  rot.data[2] = rot_z;
  std_msgs::Bool home_vito;
  KDL::Twist error_treshold;

  if(home_l == 1)
  {
    SendHomeRobot_left(); /*send left arm at home position */
    ObjOrObst = 0;
    home_l = 0;
  }

  KDL::Twist e_;
  tf::twistMsgToKDL (error_msg->error_, e_);

  /*if potential filed has riceved one msg */
  if(error_msg->arrived == 1)
  {

    error_treshold.vel = vel;
    error_treshold.rot = rot;
    
    if(IsEqual(e_,error_treshold))
    {
      ROS_INFO("Arrived in position"); 
      if(stop_home == 1)
      { 
        /*send flag to start hand_pose_generator*/
        start_controller.start_left = 1;

        if(use_both_arm == false)
        {
          /*to test only the left arm*/
          start_controller.start_right = 1;
        }

        start_controller.stop = 0;
        left_start_controller_pub.publish(start_controller);
        stop_home = 0;
      }
      else if(arrived_l == 1)
      {
        if(restart != 1)
        {
          ControllerStartAndNewPOse(error_msg);
        }
        else
        {
          /*unlock hand_pose_generator*/
          start_controller.stop = 0;
          left_start_controller_pub.publish(start_controller);
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
  vel.data[1] = y;
  vel.data[2] = z;
  rot.data[0] = rot_x;
  rot.data[1] = rot_y;
  rot.data[2] = rot_z;
  std_msgs::Bool home_vito;
 
 
 if(home_r == 1)
  {
    SendHomeRobot_right();  /*send rigtt arm at home position */
    ObjOrObst = 0;
    home_r = 0;
  }
  
  /*if potential filed has riceved one msg */
  if(error_msg->arrived == 1)
  {
    
    KDL::Twist e_;
    KDL::Twist error_treshold;
    error_treshold.vel = vel;
    error_treshold.rot = rot;

    tf::twistMsgToKDL (error_msg->error_, e_);
    if(IsEqual(e_,error_treshold))  /*if true*/
    {
      // ROS_DEBUG("Arrived in position");
      /* vito at home.. start the controller*/
      if(stop_home_r == 1)
      {
        /*send flag to start hand_pose_generator*/
        start_controller.start_right = 1;

        if(use_both_arm == false)
        {
          /*to test only the right arm*/
          start_controller.start_left = 1;
        }

        start_controller.stop = 0;
        right_start_controller_pub.publish(start_controller);
        stop_home_r = 0;
      }
      else if(arrived_r == 1)
      {
          if(restart != 1)
          {
            ControllerStartAndNewPOse(error_msg);
          }
          else
          {
            if(block_info_hand == 1)  /*after waiting closing hand sends the next move*/
            {
              if(reject_ == 1)  /*after removing objects, robot came back to home*/
              {
                SendHomeRobot_right();  /*send rigtt arm at home position */
                ObjOrObst = 0;
                reject_ = 0;
              }
              else
              {
                /*unlock hand_pose_generator*/
                start_controller.stop = 0;
                right_start_controller_pub.publish(start_controller);
                
                 /*open the softhand*/
                trajectory_msgs::JointTrajectory msg_jointT_hand;
                msg_jointT_hand.points.resize(1);
                msg_jointT_hand.joint_names.resize(1);
                msg_jointT_hand.points[0].positions.resize(1);
                msg_jointT_hand.points[0].positions[0] = 0.0;
                msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;
                msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
                hand_publisher_right.publish(msg_jointT_hand);
                block_info_hand = 0;
              }

            }
          }
      }
    }
    
  }
 
}

bool  DesperateDecisionMaker::IsEqual(KDL::Twist E_pf, KDL::Twist E_t)
{
  KDL::Twist E_pf_abs;

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

void DesperateDecisionMaker::ControllerStartAndNewPOse(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  /*start decison hand maker*/
  ROS_DEBUG("ControllerStartAndNewPOse");
  /*wait both arms*/
  if((start_controller.start_right !=0) && (start_controller.start_left !=0))
  {
    switch(ObjOrObst)
    {
      
        case 0: /*object to grasp*/
        {     
              ROS_DEBUG("Case 0: graspable object");
              // std::cout<<"case 0"<<std::endl;   
              trajectory_msgs::JointTrajectory msg_jointT_hand;
              msg_jointT_hand.joint_names.resize(1);
              desperate_housewife::handPoseSingle new_obj_pos_remove; /*new hand position*/
              /*hand msg*/
              geometry_msgs::Pose pose_temp;
              pose_temp = error_msg->pose_hand;
              msg_jointT_hand.points.resize(1);
              msg_jointT_hand.points[0].positions.resize(1);
              msg_jointT_hand.points[0].positions[0] = 1.0;
              msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;

              if(whichArm == 1)
              {
                  new_obj_pos_remove.pose = TrashObjectPOsition(1, pose_temp.orientation);
                  msg_jointT_hand.joint_names[0] = "left_hand_synergy_joint";
                  hand_publisher_left.publish(msg_jointT_hand);
                  
                    while(info_hand <= Info_closed_hand )
                  {
                    // ROS_INFO("waiting");
                    // std::cout<<"info_hand: "<<info<<std::endl;
                    ros::spinOnce(); 

                    block_info_hand = 1;
                  }
                  desired_hand_publisher_left.publish( new_obj_pos_remove );
                  
              }
              else
              {
                  new_obj_pos_remove.pose = TrashObjectPOsition(0, pose_temp.orientation);
                  msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
                  hand_publisher_right.publish(msg_jointT_hand);
                   // ros::Duration(1.0).sleep();
                  while(info_hand <= Info_closed_hand )
                  {
                    ROS_DEBUG("waiting");
                    ros::spinOnce(); /*To updating info_hand*/
                    block_info_hand = 1;
                  }

                  desired_hand_publisher_right.publish( new_obj_pos_remove );
                 
              }
              restart = 1;  /*for unlock the code */

              tf::Transform tfHandTrasform2;
              tf::poseMsgToTF( new_obj_pos_remove.pose, tfHandTrasform2);  
              tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"trash_robot_pos") );
            break;
        }
        case 1: /*obstacle to remove*/
        {     
              ROS_DEBUG("Case 1: remove object");
      
              desperate_housewife::handPoseSingle New_Hand_Position;
              New_Hand_Position.pose = error_msg->pose_hand;

              // trajectory_msgs::JointTrajectory msg_jointT_hand;
              // msg_jointT_hand.joint_names.resize(1);
              // /*hand msg*/
              // msg_jointT_hand.points.resize(1);
              // msg_jointT_hand.points[0].positions.resize(1);
              // msg_jointT_hand.points[0].positions[0] = 0.0;
              // msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;

              if(whichArm == 1)
              {
                  // obstacles_publisher_left.publish(obstaclesMsg);
                  New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x - 0.20; /*move along the x axis for push down from the table*/
                  // msg_jointT_hand.joint_names[0] = "left_hand_synergy_joint";
                  // hand_publisher_left.publish(msg_jointT_hand);
                  //   while(info_hand <= Info_closed_hand )
                  // {
                  //   // ROS_INFO("waiting");
                  //   // std::cout<<"info_hand: "<<info<<std::endl;
                  //   ros::spinOnce(); 

                    block_info_hand = 1;
                  // }

                  desired_hand_publisher_left.publish( New_Hand_Position );              
              }
              else
              {
                  New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x - 0.20; /*move along the x axis for push down from the table*/
                  // msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
                  // hand_publisher_right.publish(msg_jointT_hand);

                  // while(info_hand <= Info_closed_hand )
                  // {
                  //   // ROS_INFO("waiting");
                  //   // std::cout<<"info_hand: "<<info<<std::endl;
                  //   ros::spinOnce(); 

                    block_info_hand = 1;
                  // }

                  desired_hand_publisher_right.publish( New_Hand_Position );
                  // obstacles_publisher_right.publish(obstaclesMsg);
                 
              }

              restart = 1;  /*for unlock the code */
              reject_ = 1;  /*for sends vito at home position */

              tf::Transform tfHandTrasform;
              tf::poseMsgToTF( New_Hand_Position.pose, tfHandTrasform);  
              tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"ObstacleReject_new_pose") ); 
           break;
        } 
        // case 2:
        // {
        //     ROS_DEBUG("Case 2: Overtune");
        //     desperate_housewife::handPoseSingle New_Hand_Position;
        //     New_Hand_Position.pose = error_msg->pose_hand;
        //     trajectory_msgs::JointTrajectory msg_jointT_hand;
        //     msg_jointT_hand.joint_names.resize(1);
        //     desperate_housewife::handPoseSingle new_obj_pos_remove; /*new hand position*/
        //       /*hand msg*/
        //     geometry_msgs::Pose pose_temp;
        //     pose_temp = error_msg->pose_hand;
        //     msg_jointT_hand.points.resize(1);
        //     msg_jointT_hand.points[0].positions.resize(1);
        //     msg_jointT_hand.points[0].positions[0] = 0.0;
        //     msg_jointT_hand.points[0].time_from_start = ros::Duration(1.0); // 1s;

        //     if(overturn_check == 0) /*overtune*/
        //     {  
        //       if(whichArm == 1)
        //       {
        //           New_Hand_Position.pose.position.z = New_Hand_Position.pose.position.z + 0.20;
        //           New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x - 0.20; /*move along the x axis for push down from the table*/
        //           desired_hand_publisher_left.publish( New_Hand_Position );              
        //       }
        //       else
        //       {
        //           New_Hand_Position.pose.position.z = New_Hand_Position.pose.position.z + 0.20;
        //           New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x - 0.20; /*move along the x axis for push down from the table*/
        //       }

        //        desired_hand_publisher_right.publish( New_Hand_Position );
        //     }

        //     else /*put down the table*/
        //     {
        //       if(whichArm == 1)
        //       {
        //           New_Hand_Position.pose.position.z = New_Hand_Position.pose.position.z - 0.20;
        //           New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x + 0.20; /*move along the x axis for push down from the table*/
        //           desired_hand_publisher_left.publish( New_Hand_Position );              
        //       }
        //       else
        //       {
        //           New_Hand_Position.pose.position.z = New_Hand_Position.pose.position.z - 0.20;
        //           New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x + 0.20; /*move along the x axis for push down from the table*/
        //       }
        //       overtun_2 = 1;           
        //     }

        //     if(overturn1_ == 1)
        //     {
        //        if(whichArm == 1)
        //       {
        //           msg_jointT_hand.joint_names[0] = "left_hand_synergy_joint";
        //           hand_publisher_left.publish(msg_jointT_hand);
                  
        //             while(info_hand <= 0.3 )
        //           {
        //             // ROS_INFO("waiting");
        //             // std::cout<<"info_hand: "<<info<<std::endl;
        //             ros::spinOnce(); 

        //             block_info_hand = 1;
        //           }
        //       }
        //       else
        //       {
        //           msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
        //           hand_publisher_right.publish(msg_jointT_hand);
        //            // ros::Duration(1.0).sleep();
        //           while(info_hand <= 0.3 )
        //           {
        //             ROS_DEBUG("waiting");
        //             ros::spinOnce(); /*To updating info_hand*/
        //             block_info_hand = 1;
        //           }
        //       }
        //       overturn1_ = 0;
        //       restart = 1; 
        //       reject_ = 1;  /*for sends vito at home position */ 
        //     }
          
        //   overturn_check = 0; 
        //   if(onvertune_2 == 1)
        //   {
        //     overturn1_ = 1;
        //   }                     
        // }
         
    }
  }

}
double DesperateDecisionMaker::GetInfoHand()
{
  return info_hand;
}