#include <hand_pose_generator.h>

#include <place_hand_dany.hpp>


HandPoseGenerator::HandPoseGenerator()
{
  nh.param<int>("/demo", demo, 0);
  nh.param<int>("/Number_obj", Number_obj, 5);

  /*reads geometry information*/
  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  stream_subscriber_ = nh.subscribe(geometries_topic_, 1, &HandPoseGenerator::HandPoseGeneratorCallback, this);
  /*sends obstacle informations*/
  nh.param<std::string>("/PotentialFieldControl/obstacle_list_left", obstacles_topic_left, "/PotentialFieldControl/obstacle_pose_left");
  obstacles_publisher_left = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstacles_topic_left.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/obstacle_list_right", obstacles_topic_right, "/PotentialFieldControl/obstacle_pose_right");
  obstacles_publisher_right = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstacles_topic_right.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/Reject_obstacle_left", Reject_obstalces_topic_left, "/PotentialFieldControl/Reject_obstacle_list_left");
  Reject_obstacles_publisher_left = nh.advertise<desperate_housewife::fittedGeometriesSingle > (Reject_obstalces_topic_left.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/Reject_obstacle_right", Reject_obstalces_topic_right, "/PotentialFieldControl/Reject_obstacle_list_right");
  Reject_obstacles_publisher_right = nh.advertise<desperate_housewife::fittedGeometriesSingle > (Reject_obstalces_topic_right.c_str(),1);
  /*sends information about the remove or grasp objects */
  nh.param<std::string>("/PotentialFieldControl/objects_info_r", obj_info_topic_r, "/PotentialFieldControl/objects_info_right");
  objects_info_right_pub = nh.advertise<std_msgs::UInt16 > (obj_info_topic_r.c_str(),1, this);
  nh.param<std::string>("/PotentialFieldControl/objects_info_l", obj_info_topic_l, "/PotentialFieldControl/objects_info_left");
  objects_info_left_pub = nh.advertise<std_msgs::UInt16 > (obj_info_topic_l.c_str(),1, this);

  /*config parameteres*/
  nh.param<std::string>("/PotentialFieldControl/desired_hand_frame", desired_hand_frame_, "desired_hand_pose");
  nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "world");
  nh.param<std::string>("/PotentialFieldControl/left_hand_frame", left_hand_frame_, "left_hand_palm_ref_link");
  nh.param<std::string>("/PotentialFieldControl/right_hand_frame", right_hand_frame_, "right_hand_palm_ref_link");
   
  /*sends hand pose*/
  nh.param<std::string>("/right_arm/PotentialFieldControl/desired_hand_right_pose", desired_hand_pose_right_topic_, "/right_arm/PotentialFieldControl/desired_hand_right_pose");
  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_right_topic_.c_str(),1);

  nh.param<std::string>("/left_arm/PotentialFieldControl/desired_hand_left_pose", desired_hand_pose_left_topic_, "/left_arm/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_left_topic_.c_str(),1);

  nh.param<std::string>("/left_arm/PotentialFieldControl/desired_hand_left_pose", desired_hand_pose_left_topic_, "/left_arm/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_left_topic_.c_str(),1);

  /*reads the flag to start the control*/
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_left, "/left_arm/PotentialFieldControl/start_controller");
  left_start_controller_sub = nh.subscribe(start_topic_left, 1, &HandPoseGenerator::Start_left, this);
  nh.param<std::string>("PotentialFieldControl/start_controller" , start_topic_right, "/right_arm/PotentialFieldControl/start_controller");
  right_start_controller_sub =  nh.subscribe(start_topic_right, 1, &HandPoseGenerator::Start_right, this);

  nh.param<bool>("use_both_arm", use_both_arm, true);


  /*to close and open hand*/
  nh.param<std::string>("/left_hand/joint_trajectory_controller/command", hand_close_left, "/left_hand/joint_trajectory_controller/command");
  nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_close_right, "/right_hand/joint_trajectory_controller/command");
  hand_publisher_left = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_left.c_str(), 1);
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1);

}

void HandPoseGenerator::Start_left(const desperate_housewife::Start::ConstPtr& msg)
{
    std::cout<<"start left"<<std::endl;
    start_controller_left = msg->start_left ;

    if(use_both_arm == false)
    {
      start_controller_right = msg->start_right;
    }

    stop = msg->stop;

}
void HandPoseGenerator::Start_right(const desperate_housewife::Start::ConstPtr& msg)
{
    std::cout<<"start right"<<std::endl;
    
    start_controller_right = msg->start_right;
    
    if(use_both_arm == false)
    {
      start_controller_left = msg->start_left;
    } 
    
    stop = msg->stop;
}


void HandPoseGenerator::HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  
  if((start_controller_left != 0) && (start_controller_right != 0))
  {
    // CheckRealTimeObstacleMovements(msg);
    if(stop == 0)
    {
      std::cout<<"start controller"<<std::endl;
      ROS_DEBUG("start controller");
      desperate_housewife::handPoseSingle DesiredHandPose;
      
      DesiredHandPose.home = 0;
      std_msgs::UInt16 Obj_info;  /*msg for desperate_mind with object's information*/

      desperate_housewife::fittedGeometriesArray obstaclesMsg;

      if ( msg->geometries.size() == 1)
      {
          DesiredHandPose = generateHandPose( msg->geometries[0], 0 );

          /*check if is graspable (send hand desired pose) or not (remove object)*/
          if(DesiredHandPose.isGraspable != true)
          {
            ROS_DEBUG("Object to Reject");
            DesiredHandPose.pose = ObstacleReject(msg->geometries[0] , DesiredHandPose.whichArm);
            Obj_info.data = 1;  /*flag for desperate_mind code.*/ 
            tf::Transform tfHandTrasform2;
            tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform2); 
            tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"ObstacleReject") );

            /*send the geometry like obstacle for not touch the obstacle   */
            obstaclesMsg.geometries.push_back( msg->geometries[0]);  
          }

          else
          {
            Obj_info.data = 0;  /*flag for desperate_mind code.*/
            ROS_DEBUG("Graspable objects");
          }  
          
          if (DesiredHandPose.whichArm == 1) /*left arm*/
          {
              desired_hand_publisher_left.publish( DesiredHandPose );
              objects_info_left_pub.publish(Obj_info);
              obstacles_publisher_left.publish(obstaclesMsg);
              stop = 1;
          } 

          else /*right arm*/
          {
              desired_hand_publisher_right.publish( DesiredHandPose );
              objects_info_right_pub.publish(Obj_info);
              obstacles_publisher_right.publish(obstaclesMsg);
              stop = 1;      
          }

          /*to show with rviz   */ 
          tf::Transform tfHandTrasform;
          tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
          tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );
      }

      /*if there are more than a user defined number of object */
      else if (msg->geometries.size() >= (uint) Number_obj )
      {
        Overturn(); //da finire
        // Obj_info.data = 2;
        // objects_info_left_pub.publish(Obj_info);
        // objects_info_right_pub.publish(Obj_info);
        // stop = 1;
      }
      else
      {      
        switch(demo)
        {
          case 0: 
           DesperateDemo1(msg); /*take graspable object with obstacle avoidance*/
           break;
          case 1:
           DesperateDemo2(msg); /*take graspable object with removing the obstalce */
           break;
          case 2:
            ROS_ERROR("IMPOSSIBLE DEMO.. Demo does not exist");
            break;
        }
      }
    }
    /*until the robot doesn't arrived at home stay still.*/
    else
      return;
       
  }
}
  

desperate_housewife::handPoseSingle HandPoseGenerator::generateHandPose( desperate_housewife::fittedGeometriesSingle geometry, int cyl_nbr )
{
  desperate_housewife::handPoseSingle hand_pose_local;
  hand_pose_local.obj = 1;

  if ( isGeometryGraspable ( geometry ))
  {
    hand_pose_local.whichArm = whichArm( geometry.pose , cyl_nbr);
    std::cout<<"^^^^^^^hand_pose_local.whichArm^^^^ : "<<hand_pose_local.whichArm <<std::endl;
    hand_pose_local.pose = placeHand( geometry, hand_pose_local.whichArm );
    hand_pose_local.isGraspable = true;
  }
  else
  {
    hand_pose_local.pose = geometry.pose;
    hand_pose_local.isGraspable = false;
    hand_pose_local.whichArm = whichArm( geometry.pose, cyl_nbr );
  }

  return hand_pose_local;
}

bool HandPoseGenerator::isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry )
{
  /*comparision between ration and treshold and cylinder radius with another threshold*/
  if (geometry.info[geometry.info.size() - 1] >=55 && geometry.info[0] < 0.10)
  {
    return true;
  }
  
  return false;
}



void HandPoseGenerator::DesperateDemo1(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
    ROS_INFO("***DEMO1, take first graspable object with obstacle avoidance***");
    std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;
        
    desperate_housewife::fittedGeometriesArray obstaclesMsg;
    std_msgs::UInt16 Obj_info;
    
    for (unsigned int i=0; i< msg->geometries.size(); i++)
    {
        objects_vec.push_back(msg->geometries[i]);
    }
    
    /* sort the cylinder by the shortes distance from softhand */
    std::sort(objects_vec.begin(), objects_vec.end(), [](desperate_housewife::fittedGeometriesSingle first, desperate_housewife::fittedGeometriesSingle second) {
        double distfirst = std::sqrt( first.pose.position.x*first.pose.position.x + first.pose.position.y*first.pose.position.y + first.pose.position.z*first.pose.position.z);
        double distsecond = std::sqrt( second.pose.position.x*second.pose.position.x + second.pose.position.y*second.pose.position.y + second.pose.position.z*second.pose.position.z);
        return (distfirst < distsecond); });

    int obj_grasp = 0;
         
    /*find the first graspagle geometry */   
    for(unsigned int k=0; k < objects_vec.size(); k++)
    {
        desperate_housewife::handPoseSingle DesiredHandPose;
        desperate_housewife::fittedGeometriesSingle obstacle;

        DesiredHandPose = generateHandPose( objects_vec[k], k );

        if(DesiredHandPose.isGraspable != true)
        {
          // obstaclesMsg.geometries.push_back( SetObstacleMsg(msg->geometries[0]) );  
          obstaclesMsg.geometries.push_back( msg->geometries[0]);         
        }

        else
        {
            Obj_info.data = 0;
            obj_grasp = obj_grasp + 1;
            ROS_DEBUG("Graspable objects");

            /*send all other cylinders like obstalcle*/
            for (unsigned int i_ = k + 1; i_ < objects_vec.size(); i_++ )
            {                   
                // obstaclesMsg.geometries.push_back( SetObstacleMsg(objects_vec[i_]) );
              obstaclesMsg.geometries.push_back( objects_vec[i_]);
            }
               
            if (DesiredHandPose.whichArm == 1) /*left arm*/
            {
                desired_hand_publisher_left.publish( DesiredHandPose );
                obstacles_publisher_left.publish(obstaclesMsg);
                objects_info_left_pub.publish(Obj_info);
                stop = 1;
            } 

            else /*right arm*/
            {
                desired_hand_publisher_right.publish( DesiredHandPose );
                obstacles_publisher_right.publish(obstaclesMsg);
                objects_info_right_pub.publish(Obj_info);
                stop = 1;    
            }

            tf::Transform tfHandTrasform;
            tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
            tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );
          
            break;
        }
    }

    /*if there aren't graspable object call the funciont overtun */
    if(obj_grasp == 0)
      Overturn(); 
}


void  HandPoseGenerator::DesperateDemo2(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
    ROS_INFO("***DEMO2, take first graspable without obstacles avoidance***");
    std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;
    desperate_housewife::handPoseSingle DesiredHandPose;
    desperate_housewife::fittedGeometriesSingle obstacle;
    desperate_housewife::fittedGeometriesArray obstaclesMsg;
    std_msgs::UInt16 Obj_info;
    
    for (unsigned int i=0; i< msg->geometries.size(); i++)
    {
        objects_vec.push_back(msg->geometries[i]);
    }
    /*sort the cylinder by the shortes distance from softhand */
    std::sort(objects_vec.begin(), objects_vec.end(), [](desperate_housewife::fittedGeometriesSingle first, desperate_housewife::fittedGeometriesSingle second) {
        double distfirst = std::sqrt( first.pose.position.x*first.pose.position.x + first.pose.position.y*first.pose.position.y + first.pose.position.z*first.pose.position.z);
        double distsecond = std::sqrt( second.pose.position.x*second.pose.position.x + second.pose.position.y*second.pose.position.y + second.pose.position.z*second.pose.position.z);
        return (distfirst < distsecond); });

    for(unsigned int k=0; k < objects_vec.size(); k++)
    {
      DesiredHandPose = generateHandPose( objects_vec[k], k );

      if (!DesiredHandPose.isGraspable )
      {
          ROS_DEBUG("Object to Reject");
          DesiredHandPose.pose = ObstacleReject(objects_vec[k], DesiredHandPose.whichArm);
          Obj_info.data = 1; 
          tf::Transform tfHandTrasform2;
          tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform2); 
          tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"ObstacleReject") );
      }

      else
      {
          ROS_DEBUG("Graspable objects");
          Obj_info.data = 0; /*flag to grasp object in the desperate_mind code*/
          tf::Transform tfHandTrasform;
          tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
          tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) ); 
      }
        
      if (DesiredHandPose.whichArm == 1) 
      {
          desired_hand_publisher_left.publish(DesiredHandPose);
          stop = 1; /* flag to stop this procedure */
          objects_info_left_pub.publish(Obj_info);
      }
      else
      {
          desired_hand_publisher_right.publish(DesiredHandPose);
          stop = 1;
          objects_info_right_pub.publish(Obj_info);
      }
    }
}

// void HandPoseGenerator::CheckRealTimeObstacleMovements(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
// {
//   desperate_housewife::handPoseSingle InfoPose;
//   desperate_housewife::fittedGeometriesSingle New_obstacle;
//    desperate_housewife::fittedGeometriesArray obstaclesMsg;

//   for(unsigned int i=0; i < msg->geometries.size();i++ )
//   {

//     if( !isGeometryGraspable ( msg->geometries[i] ))
//     {
//       New_obstacle.pose = msg->geometries[i].pose;
        
//       for (unsigned j=0; j < msg->geometries[i].info.size(); j++)
//       {
//             New_obstacle.info.push_back(msg->geometries[i].info[j]);

//       }
            
//       obstaclesMsg.geometries.push_back( New_obstacle );
//     }
//   }
//   obstacles_publisher_left.publish(obstaclesMsg);
//   obstacles_publisher_right.publish(obstaclesMsg);
// }


// desperate_housewife::fittedGeometriesSingle SetObstacleMsg(desperate_housewife::fittedGeometriesSingle geo_obst)
// {
// //send the geometry like obstacle for not touch the obstacle
//     desperate_housewife::fittedGeometriesSingle obstacle;
//     obstacle.pose = geo_obst.pose;
        
//     for (unsigned j=0; j < geo_obst.info.size(); j++)
//     {
//         obstacle.info.push_back(geo_obst.info[j]);
//     }
              
//     // obstaclesMsg.geometries.push_back( obstacle );
//     return obstacle;

// }

// }
// void HandPoseGenerator::SendObjRejectMsg(desperate_housewife::fittedGeometriesSingle obj_msg, int arm_)
// {
//   desperate_housewife::fittedGeometriesSingle obstacle_rej;
//   // desperate_housewife::handPoseSingle obstacle_rej;
//   obstacle_rej.pose = ObstacleReject(obj_msg, arm_);
//   std_msgs::UInt16 Obj_info;
  
//   for (unsigned j=0; j < obj_msg.info.size(); j++)
//   {
//     obstacle_rej.info.push_back(obj_msg.info[j]);
//   }
//   // obstacle_rej.info.push_back(arm_);
  
//   if(arm_ == 1) //left
//   {
//     Reject_obstacles_publisher_left.publish(obstacle_rej);
    
//     Obj_info.data = 1;
//     objects_info_left_pub.publish(Obj_info);
//     stop = 1;
//   }
//   else
//   { 
//     // std::cout<<"pubblico right"<<std::endl;
//     Reject_obstacles_publisher_right.publish(obstacle_rej);
//     Obj_info.data = 1;
//     objects_info_right_pub.publish(Obj_info);
//     stop = 1;
//   } 
//   tf::Transform tfHandTrasform2;
//   tf::poseMsgToTF( obstacle_rej.pose, tfHandTrasform2); 
//   tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"ObstacleReject") );
        
// }
