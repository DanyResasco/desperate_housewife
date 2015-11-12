#include <test_hand_pose_generator.h>

#include <place_hand_dany.hpp>

HandPoseGenerator::HandPoseGenerator()
{
  nh.param<int>("/demo", demo, 0);
  nh.param<int>("/Number_obj", Number_obj, 5);

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

  nh.param<std::string>("/PotentialFieldControl/objects_info_r", obj_info_topic_r, "/PotentialFieldControl/objects_info_right");
  objects_info_right_pub = nh.advertise<std_msgs::UInt16 > (obj_info_topic_r.c_str(),1, this);
  nh.param<std::string>("/PotentialFieldControl/objects_info_l", obj_info_topic_l, "/PotentialFieldControl/objects_info_left");
  objects_info_left_pub = nh.advertise<std_msgs::UInt16 > (obj_info_topic_l.c_str(),1, this);



  nh.param<std::string>("/PotentialFieldControl/desired_hand_frame", desired_hand_frame_, "desired_hand_pose");
  nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "vito_anchor");
  nh.param<std::string>("/PotentialFieldControl/left_hand_frame", left_hand_frame_, "left_hand_palm_ref_link");
  nh.param<std::string>("/PotentialFieldControl/right_hand_frame", right_hand_frame_, "right_hand_palm_ref_link");
   

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left_filter", desired_hand_pose_left_topic_, "/PotentialFieldControl/desired_hand_left_filter");
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_left_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right_filter", desired_hand_pose_right_topic_, "/PotentialFieldControl/desired_hand_right_filter");
  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_right_topic_.c_str(),1);
  
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_left, "/left_arm/PotentialFieldControl/start_controller");
  left_start_controller_sub = nh.subscribe(start_topic_left, 1, &HandPoseGenerator::Start_left, this);
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_right, "/right_arm/PotentialFieldControl/start_controller");
  right_start_controller_sub =  nh.subscribe(start_topic_right, 1, &HandPoseGenerator::Start_right, this);
}

void HandPoseGenerator::Start_left(const desperate_housewife::Start::ConstPtr& msg)
{
  std::cout<<"start left"<<std::endl;
  start_controller_left = msg->start_left ;
  stop = msg->stop;

}
void HandPoseGenerator::Start_right(const desperate_housewife::Start::ConstPtr& msg)
{
   std::cout<<"start right"<<std::endl;
  
  start_controller_right = msg->start_right;
  stop = msg->stop;
}


void HandPoseGenerator::SendObjRejectMsg(desperate_housewife::fittedGeometriesSingle obj_msg, int arm_)
{
  desperate_housewife::fittedGeometriesSingle obstacle_rej;
  obstacle_rej.pose = ObstacleReject(obj_msg, arm_);
  std_msgs::UInt16 Obj_info;
  
  for (unsigned j=0; j < obj_msg.info.size(); j++)
  {
    obstacle_rej.info.push_back(obj_msg.info[j]);
  }
  obstacle_rej.info.push_back(arm_);
  
  if(arm_ == 1) //left
  {
    Reject_obstacles_publisher_left.publish(obstacle_rej);
    
    Obj_info.data = 1;
    objects_info_left_pub.publish(Obj_info);
    stop = 1;
  }
  else
  { 
    // std::cout<<"pubblico right"<<std::endl;
    Reject_obstacles_publisher_right.publish(obstacle_rej);
    Obj_info.data = 1;
    objects_info_right_pub.publish(Obj_info);
    stop = 1;
  } 
  tf::Transform tfHandTrasform2;
  tf::poseMsgToTF( obstacle_rej.pose, tfHandTrasform2); 
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"ObstacleReject") );
        
}





void HandPoseGenerator::HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  
  if((start_controller_left != 0) && (start_controller_right != 0))
  {
    if( stop == 0)
    {
      std::cout<<"start controller"<<std::endl;
       
      desperate_housewife::handPoseSingle DesiredHandPose;
      
      DesiredHandPose.home = 0;
      std_msgs::UInt16 Obj_info;

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
                desired_hand_publisher_left.publish( DesiredHandPose );
                Obj_info.data = 0;
                objects_info_left_pub.publish(Obj_info);
                stop = 1;
              }
              
              else
              {
                desired_hand_publisher_right.publish( DesiredHandPose );
                Obj_info.data = 0;
                objects_info_right_pub.publish(Obj_info);
                  stop = 1;       
              }
          }
        
        tf::Transform tfHandTrasform;
        tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
        tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );

      }

      else if (msg->geometries.size() >=  Number_obj )
      {
        Overturn();
      }
      else
      {
        std::cout<<"demo: "<<demo<<std::endl;
        switch(demo)
        {
          case 0: 
           DesperateDemo1(msg);
           break;
          case 1:
           DesperateDemo2(msg);
           break;
          case 2:
            ROS_ERROR("IMPOSSIBLE DEMO");
            break;
        }
      }
    }
    else
      return;
       
  }
}
  



desperate_housewife::handPoseSingle HandPoseGenerator::generateHandPose( desperate_housewife::fittedGeometriesSingle geometry )
{
  desperate_housewife::handPoseSingle hand_pose_local;
  hand_pose_local.obj = 1;

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

void HandPoseGenerator::DesperateDemo1(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  ROS_INFO("DEMO1, take first graspable with obstacles avoidance");
  std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;
  desperate_housewife::handPoseSingle DesiredHandPose;
  desperate_housewife::fittedGeometriesSingle obstacle;
  desperate_housewife::fittedGeometriesArray obstaclesMsg;
  //sort the cylinder by the shortes distance from softhand 
  for (unsigned int i=0; i< msg->geometries.size(); i++)
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
            obj_grasp = 1;
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
            if (DesiredHandPose.whichArm == 1) 
            {
              obstacles_publisher_left.publish(obstaclesMsg);
              desired_hand_publisher_left.publish(DesiredHandPose);
              stop = 1;
            }
            else
            {
              obstacles_publisher_right.publish(obstaclesMsg);
              desired_hand_publisher_right.publish(DesiredHandPose);
              stop = 1;
            }
        }
         
        tf::Transform tfHandTrasform;
        tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
        tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );
    }

}

void  HandPoseGenerator::DesperateDemo2(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  ROS_INFO("DEMO2, take first graspable without obstacles avoidance");
  std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;
  desperate_housewife::handPoseSingle DesiredHandPose;
  desperate_housewife::fittedGeometriesSingle obstacle;
  desperate_housewife::fittedGeometriesArray obstaclesMsg;
  //sort the cylinder by the shortes distance from softhand 
  for (unsigned int i=0; i<msg->geometries.size(); i++)
  {
      objects_vec.push_back(msg->geometries[i]);
  }

  std::sort(objects_vec.begin(), objects_vec.end(), [](desperate_housewife::fittedGeometriesSingle first, desperate_housewife::fittedGeometriesSingle second) {
      double distfirst = std::sqrt( first.pose.position.x*first.pose.position.x + first.pose.position.y*first.pose.position.y + first.pose.position.z*first.pose.position.z);
      double distsecond = std::sqrt( second.pose.position.x*second.pose.position.x + second.pose.position.y*second.pose.position.y + second.pose.position.z*second.pose.position.z);
      return (distfirst < distsecond); });

  DesiredHandPose = generateHandPose( objects_vec[0] );

  if (DesiredHandPose.isGraspable )
  {
      
      if (DesiredHandPose.whichArm == 1) 
      {
        desired_hand_publisher_left.publish(DesiredHandPose);
        stop = 1;
      }
      else
      {
          desired_hand_publisher_right.publish(DesiredHandPose);
          stop = 1;
      }
 
    tf::Transform tfHandTrasform;
    tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );  
    // flag_obj = 0;       
  }
  else
  {
      std::cout<<"remove"<<std::endl;
      SendObjRejectMsg(objects_vec[0], DesiredHandPose.whichArm);
      // flag_obj = 0;
  }
}




















// void  HandPoseGenerator::DesperateDemo2(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
// {
//   ROS_INFO("DEMO2, take first graspable without obstacles avoidance");
//   std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;
//   desperate_housewife::handPoseSingle DesiredHandPose;
//   desperate_housewife::fittedGeometriesSingle obstacle;
//   desperate_housewife::fittedGeometriesArray obstaclesMsg;
//   //sort the cylinder by the shortes distance from softhand 
//   for (unsigned int i=0; i<msg->geometries.size(); i++)
//   {
//       objects_vec.push_back(msg->geometries[i]);
//   }

//   std::sort(objects_vec.begin(), objects_vec.end(), [](desperate_housewife::fittedGeometriesSingle first, desperate_housewife::fittedGeometriesSingle second) {
//       double distfirst = std::sqrt( first.pose.position.x*first.pose.position.x + first.pose.position.y*first.pose.position.y + first.pose.position.z*first.pose.position.z);
//       double distsecond = std::sqrt( second.pose.position.x*second.pose.position.x + second.pose.position.y*second.pose.position.y + second.pose.position.z*second.pose.position.z);
//       return (distfirst < distsecond); });

//     //find the first graspable geometry      
//     for(unsigned int k=0; k < objects_vec.size(); k++)
//     {
//         DesiredHandPose = generateHandPose( objects_vec[k] );
//         // std::cout<<"DesiredHandPose.whichArm: "<<DesiredHandPose.whichArm<<std::endl;

//         if (DesiredHandPose.isGraspable )
//         {
//           if(step_grasp == 0)
//           {
//              if (DesiredHandPose.whichArm == 1) 
//               {
//                desired_hand_publisher_left.publish(DesiredHandPose);
//               }
//               else
//               {
//                 desired_hand_publisher_right.publish(DesiredHandPose);
//               }
//             step_grasp = 1;
//           }

//           if(flag_obj == 1)
//           {
//               std::cout<<"grasp"<<std::endl;           
//              if (DesiredHandPose.whichArm == 1) 
//               {
//                desired_hand_publisher_left.publish(DesiredHandPose);
//               }
//               else
//               {
//                 desired_hand_publisher_right.publish(DesiredHandPose);
//               }
            
//             tf::Transform tfHandTrasform;
//             tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
//             tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );  
//             flag_obj = 0;
//           }
//         }
//         else
//         {
//           if(step_grasp == 0)
//           {
//             SendObjRejectMsg(objects_vec[k], DesiredHandPose.whichArm);
//             step_grasp = 1;
//           }

//           if(flag_remove == 1)
//           {
//             std::cout<<"remove"<<std::endl;
//             SendObjRejectMsg(objects_vec[k], DesiredHandPose.whichArm);
//             flag_obj = 0;
//           }
//         }
//     }
// }

