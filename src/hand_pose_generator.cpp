#include <hand_pose_generator.h>

#include <place_hand_dany.hpp>

HandPoseGenerator::HandPoseGenerator()
{
  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right", desired_hand_right_pose_topic_, "/PotentialFieldControl/desired_hand_right_pose");
  desired_hand_right_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left", desired_hand_left_pose_topic_, "/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_left_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);
  
  //SendHomeRobot();
  //nh.param<std::string>("/PotentialFieldControl/desired_hand_name", left_hand_synergy_joint, "/PotentialFieldControl/desired_hand_name" );
  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  stream_subscriber_ = nh.subscribe(geometries_topic_, 1, &HandPoseGenerator::HandPoseGeneratorCallback, this);

  nh.param<std::string>("/PotentialFieldControl/obstacle_list_left", obstacles_topic_left, "/PotentialFieldControl/obstacle_pose_left");
  obstacles_publisher_left = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstacles_topic_left.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/obstacle_list_right", obstacles_topic_right, "/PotentialFieldControl/obstacle_pose_right");
  obstacles_publisher_right = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstacles_topic_right.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/Reject_obstacle_left", Reject_obstalces_topic_left, "/PotentialFieldControl/Reject_obstacle_list_left");
  Reject_obstacles_publisher_left = nh.advertise<desperate_housewife::fittedGeometriesSingle > (Reject_obstalces_topic_left.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/Reject_obstacle_right", Reject_obstalces_topic_right, "/PotentialFieldControl/Reject_obstacle_list_right");
  Reject_obstacles_publisher_right = nh.advertise<desperate_housewife::fittedGeometriesSingle > (Reject_obstalces_topic_right.c_str(),1);


  nh.param<std::string>("/PotentialFieldControl/desired_hand_frame", desired_hand_frame_, "desired_hand_pose");
  nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "vito_anchor");
  nh.param<std::string>("/PotentialFieldControl/left_hand_frame", left_hand_frame_, "left_hand_palm_ref_link");
  nh.param<std::string>("/PotentialFieldControl/right_hand_frame", right_hand_frame_, "right_hand_palm_ref_link");
   nh.param<std::string>("/left_hand/joint_trajectory_controller/command", hand_close_left, "/left_hand/joint_trajectory_controller/command");
   nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_close_right, "/right_hand/joint_trajectory_controller/command");

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left_filter", desired_hand_pose_left_topic_, "/PotentialFieldControl/desired_hand_left_filter");
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_left_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right_filter", desired_hand_pose_right_topic_, "/PotentialFieldControl/desired_hand_right_filter");
  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_right_topic_.c_str(),1);
  
  nh.param<std::string>("/left_arm/PotentialFieldControl/error", error_topic_left, "/left_arm/PotentialFieldControl/error");
  error_sub_left = nh.subscribe(error_topic_left, 1, &HandPoseGenerator::Error_info_left, this);

  nh.param<std::string>("/right_arm/PotentialFieldControl/error", error_topic_right, "/right_arm/PotentialFieldControl/error");
  error_sub_right = nh.subscribe(error_topic_right, 1, &HandPoseGenerator::Error_info_right, this);

  hand_publisher_left = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_left.c_str(), 1000);
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);



}
//move vito in desired location befor control start
void HandPoseGenerator::SendHomeRobot()
{
  desperate_housewife::handPoseSingle home_robot_left, home_robot_right;
  double roll,pitch,yaw;
  nh.param<double>("/home_left_arm_position_x", home_robot_left.pose.position.x, -0.75022);
  nh.param<double>("/home_left_arm_position_y",  home_robot_left.pose.position.y,  -0.47078);
  nh.param<double>("/home_left_arm_position_z", home_robot_left.pose.position.z, 0.74494);
  nh.param<double>("/home_left_arm_A", yaw,  -0.12690);
  nh.param<double>("/home_left_arm_B", pitch, -0.06571);
  nh.param<double>("/home_left_arm_C", roll, -0.11774);

  Eigen::Matrix4d ROT_y;
  ROT_y.row(0)<< -1,0,0,0;
  ROT_y.row(1)<< 0,1,0,0;
  ROT_y.row(2)<< 0,0,-1,0;
  ROT_y.row(3)<< 0,0,0,1;
  Eigen::Matrix4d Rot_z;
  Rot_z.row(0)<< -1,0,0,0;
  Rot_z.row(1)<< 0,-1,0,0;
  Rot_z.row(2)<< 0,0,1,0;
  Rot_z.row(3)<< 0,0,0,1;


  Eigen::Matrix3d Tmatrix;
  Tmatrix = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Matrix4d Vito_home_base = Eigen::Matrix4d::Identity(4,4);
  Vito_home_base.block<3,3>(0,0) = Tmatrix;
  Vito_home_base(0,3) = home_robot_left.pose.position.x;
  Vito_home_base(1,3) = home_robot_left.pose.position.y;
  Vito_home_base(2,3) = home_robot_left.pose.position.z;
  
   Eigen::Matrix4d Vito_home_base_rot = Vito_home_base*ROT_y;

  Eigen::Quaterniond quat_eigen_hand(Vito_home_base_rot.block<3,3>(0,0));
  home_robot_left.pose.orientation.x = quat_eigen_hand.x();
  home_robot_left.pose.orientation.y = quat_eigen_hand.y();
  home_robot_left.pose.orientation.z = quat_eigen_hand.z();
  home_robot_left.pose.orientation.w = quat_eigen_hand.w();

  desired_hand_left_pose_publisher_.publish( home_robot_left );

  nh.param<double>("/home_right_arm_position_x", home_robot_right.pose.position.x, -0.75022);
  nh.param<double>("/home_right_arm_position_y",  home_robot_right.pose.position.y,  0.47078);
  nh.param<double>("/home_right_arm_position_z", home_robot_right.pose.position.z, 0.74494);
  nh.param<double>("/home_right_arm_A", yaw,  0.12690);
  nh.param<double>("/home_right_arm_B", pitch, 0.06571);
  nh.param<double>("/home_right_arm_C", roll, 0.11774);


  Eigen::Matrix3d Tmatrix_right;
  Tmatrix_right = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Matrix4d Vito_home_base_right = Eigen::Matrix4d::Identity(4,4);
  Vito_home_base_right.block<3,3>(0,0) = Tmatrix_right;
  Vito_home_base_right(0,3) = home_robot_right.pose.position.x;
  Vito_home_base_right(1,3) = home_robot_right.pose.position.y;
  Vito_home_base_right(2,3) = home_robot_right.pose.position.z;

  Eigen::Matrix4d Vito_home_base_right_rot = Vito_home_base_right*ROT_y*Rot_z;

  //std::cout<<"Tmatrix: "<<Tmatrix<<std::endl;
  Eigen::Quaterniond quat_eigen_hand_right(Vito_home_base_right_rot.block<3,3>(0,0));
  home_robot_right.pose.orientation.x = quat_eigen_hand_right.x();
  home_robot_right.pose.orientation.y = quat_eigen_hand_right.y();
  home_robot_right.pose.orientation.z = quat_eigen_hand_right.z();
  home_robot_right.pose.orientation.w = quat_eigen_hand_right.w();


  desired_hand_right_pose_publisher_.publish( home_robot_right );
    tf::Transform tfHandTrasform2, tfHandTrasform1;
    tf::poseMsgToTF( home_robot_left.pose, tfHandTrasform2);
    tf::poseMsgToTF( home_robot_right.pose, tfHandTrasform1);    
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"home_robot_left") ); 
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"home_robot_right") ); 
  
}

void HandPoseGenerator::Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
    KDL::Frame frame_hand, frame_des;
    tf::poseMsgToKDL(error_msg->pose_hand, frame_hand);
     tf::poseMsgToKDL(error_msg->pose_desired, frame_des);

   if (Equal(frame_hand, frame_des, 0.05))
    {
      start_controller_left = 1;
      ControllerStartAndNewPOse(error_msg);
    }
}

void HandPoseGenerator::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
    KDL::Frame frame_hand, frame_des;
    tf::poseMsgToKDL(error_msg->pose_hand, frame_hand);
     tf::poseMsgToKDL(error_msg->pose_desired, frame_des);
   if (Equal(frame_hand,frame_des,0.05))
    {
      start_controller_right = 1;
      ControllerStartAndNewPOse(error_msg);
    }
}



void HandPoseGenerator::ControllerStartAndNewPOse(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
     if(error_msg->ObjOrObst == true) //object to grasp
      {
        //SendmsgToCloseHand
        trajectory_msgs::JointTrajectory msg_jointT_hand;
            msg_jointT_hand.joint_names.resize(1);
            msg_jointT_hand.points.resize(1);
            msg_jointT_hand.points[0].positions.resize(1);
            // msg_jointT_hand.joint_names[0] = desired_hand_name;
            msg_jointT_hand.points[0].positions[0] = 1.0;
            msg_jointT_hand.points[0].time_from_start = ros::Duration(2); // 2s;
            if(error_msg->WhichArm ==1)
            {
               msg_jointT_hand.joint_names[0] = left_hand_synergy_joint.c_str();
              hand_publisher_left.publish(msg_jointT_hand);
            }
            else
            {
               msg_jointT_hand.joint_names[0] = right_hand_synergy_joint.c_str();
              hand_publisher_right.publish(msg_jointT_hand);
            }
      } 
      else //object to move
      {
      
        desperate_housewife::handPoseSingle New_Hand_Position;
        // geometry_msgs::Pose pos_new, pose_local;
        New_Hand_Position.pose = error_msg->pose_hand;
        New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x - 0.25;
         // std::cout<<"error_msg->WhichArm: "<<error_msg->WhichArm<<std::endl;
         if(error_msg->WhichArm ==1)
            {
              desired_hand_left_pose_publisher_.publish( New_Hand_Position );
            }
            else
            {
               desired_hand_right_pose_publisher_.publish( New_Hand_Position );
            }

        tf::Transform tfHandTrasform2;
        tf::poseMsgToTF( New_Hand_Position.pose, tfHandTrasform2);  
        tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"ObstacleReject_new_pose") ); 
       }
    
    return;


}







void HandPoseGenerator::SendObjRejectMsg(desperate_housewife::fittedGeometriesSingle obj_msg, int arm_)
{
  desperate_housewife::fittedGeometriesSingle obstacle_rej;
  obstacle_rej.pose = ObstacleReject(obj_msg);
  
  for (unsigned j=0; j < obj_msg.info.size(); j++)
  {
      obstacle_rej.info.push_back(obj_msg.info[j]);
          //obstacle.info.push_back(obj_msg.geometries.info[j]);
  }
  obstacle_rej.info.push_back(arm_);
  
  if(arm_ == 1) //left
  {
    Reject_obstacles_publisher_left.publish(obstacle_rej);
          // obstacles_publisher_left.publish(obstaclesMsg);
  }
  else
  {
     Reject_obstacles_publisher_right.publish(obstacle_rej);
          // obstacles_publisher_right.publish(obstaclesMsg);
          // std::cout<<"right"<<std::endl
  } 
  tf::Transform tfHandTrasform2;
  tf::poseMsgToTF( obstacle_rej.pose, tfHandTrasform2); 
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"ObstacleReject") );
        
}





void HandPoseGenerator::HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  if((start_controller_left != 0) && (start_controller_right != 0))
  {
  
    desperate_housewife::fittedGeometriesSingle obstacle;
    desperate_housewife::fittedGeometriesArray obstaclesMsg;
    desperate_housewife::handPoseSingle DesiredHandPose;

    if ( msg->geometries.size() == 1)
    {
         DesiredHandPose = generateHandPose( msg->geometries[0] );
          //check if is graspable (send hand desired pose) or not (remove object)
        if(DesiredHandPose.isGraspable != true)
        {
          SendObjRejectMsg(msg->geometries[0] , DesiredHandPose.whichArm);
        }
        else
         {
            if (DesiredHandPose.whichArm == 1) 
            {
              desired_hand_left_pose_publisher_.publish( DesiredHandPose );
              
            }
            else
            {
              desired_hand_right_pose_publisher_.publish( DesiredHandPose );
              
            }
          }
      
      tf::Transform tfHandTrasform;
      tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
      tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );

    }

    else
    {
        ROS_DEBUG("More than one geometry identified");
        //sort the cylinder by the shortes distance from softhand 
        for (unsigned int i=0; i<msg->geometries.size(); i++)
        {
          objects_vec.push_back(msg->geometries[i]);
        }

        std::sort(objects_vec.begin(), objects_vec.end(), [](desperate_housewife::fittedGeometriesSingle first, desperate_housewife::fittedGeometriesSingle second) {
          double distfirst = std::sqrt( first.pose.position.x*first.pose.position.x + first.pose.position.y*first.pose.position.y + first.pose.position.z*first.pose.position.z);
          double distsecond = std::sqrt( second.pose.position.x*second.pose.position.x + second.pose.position.y*second.pose.position.y + second.pose.position.z*second.pose.position.z);
          return (distfirst < distsecond); });

        int obj_grasp = 0;
        int index_obj;
         
      //find the first graspagle geometry      
        for(unsigned int k=0; k < objects_vec.size(); k++)
        {
            DesiredHandPose = generateHandPose( objects_vec[k] );
            if (DesiredHandPose.isGraspable )
            {
              obj_grasp=1;
              index_obj = k;
              break;
            }
        }
        //if none is graspable, send all geometry like remove object
        if (obj_grasp == 0)
        {
          for(unsigned int k=0; k < objects_vec.size(); k++)
          {
           int arm_;
           arm_ =  whichArm( objects_vec[k].pose );

            SendObjRejectMsg(objects_vec[k], arm_);
          }

          ROS_ERROR("NO graspable objects found, exiting... :( , all geometries are obstacles to remove");
          
        }
        //locate some geometry graspable 
        else
        {
          DesiredHandPose = generateHandPose( objects_vec[index_obj] );
         
         //sending other geometries as obstacles
          for (int i_ = 0; i_< index_obj; i_++)
          {
            obstacle.pose = objects_vec[i_].pose;
        
            for (unsigned j=0; j < objects_vec[i_].info.size(); j++)
            {
              obstacle.info.push_back(objects_vec[i_].info[j]); 
            }
            
            obstaclesMsg.geometries.push_back( obstacle );
          }

          for (unsigned int i_ = index_obj + 1; i_ < objects_vec.size(); i_++ )
          {
             
            obstacle.pose = objects_vec[i_].pose;
        
            for (unsigned j=0; j < objects_vec[i_].info.size(); j++)
            {
              obstacle.info.push_back(objects_vec[i_].info[j]); 
            }
            
            obstaclesMsg.geometries.push_back( obstacle );
          }
          //send a desired pose and obstacles location
          if(DesiredHandPose.isGraspable == true)
          {
            // desired_hand_publisher_.publish( DesiredHandPose );
            if (DesiredHandPose.whichArm == 1) 
            {
              obstacles_publisher_left.publish(obstaclesMsg);
              // desired_hand_left_pose_publisher_.publish( DesiredHandPose );
                desired_hand_publisher_left.publish(DesiredHandPose);
   
            }
            else
            {
              obstacles_publisher_right.publish(obstaclesMsg);
              // desired_hand_right_pose_publisher_.publish( DesiredHandPose );
              desired_hand_publisher_right.publish(DesiredHandPose);
              
            }
          }
         
          tf::Transform tfHandTrasform;
          tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
          tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );
          // desired_hand_publisher_.publish( DesiredHandPose );  //to filtered the hand_pose

        }
      }
   objects_vec.clear();
  }
  else
  SendHomeRobot();
 
  
}


desperate_housewife::handPoseSingle HandPoseGenerator::generateHandPose( desperate_housewife::fittedGeometriesSingle geometry )
{
  desperate_housewife::handPoseSingle hand_pose_local;

  if ( isGeometryGraspable ( geometry ))
  {
      hand_pose_local.whichArm = whichArm( geometry.pose );
      hand_pose_local.pose = placeHand( geometry, hand_pose_local.whichArm );
      hand_pose_local.isGraspable = true;
  }
  else
  {
    hand_pose_local.pose = geometry.pose;
    hand_pose_local.isGraspable = false;
    hand_pose_local.whichArm = whichArm( geometry.pose );
  }

  
  return hand_pose_local;
}

bool HandPoseGenerator::isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry )
{
  if (geometry.info[geometry.info.size() - 1] >=55 && geometry.info[0] < 0.10)
  {

      return true;
  }
  
  return false;


}
