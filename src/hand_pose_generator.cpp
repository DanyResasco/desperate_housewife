#include <hand_pose_generator.h>

#include <place_hand_dany.hpp>

HandPoseGenerator::HandPoseGenerator()
{

  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  stream_subscriber_ = nh.subscribe(geometries_topic_, 1, &HandPoseGenerator::HandPoseGeneratorCallback, this);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right", desired_hand_right_pose_topic_, "/PotentialFieldControl/desired_hand_right_pose");
  desired_hand_right_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left", desired_hand_left_pose_topic_, "/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_left_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/obstacle_list", obstalces_topic_, "/PotentialFieldControl/obstacle_list");
  obstacles_publisher_ = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstalces_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_frame", desired_hand_frame_, "desired_hand_pose");

  nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "vito_anchor");
  nh.param<std::string>("/PotentialFieldControl/left_hand_frame", left_hand_frame_, "left_hand_palm_link");
  nh.param<std::string>("/PotentialFieldControl/right_hand_frame", right_hand_frame_, "right_hand_palm_link");

  // nh.param<std::string>("/PotentialFieldControl/desired_hand_pose", desired_hand_pose_topic_, "/PotentialFieldControl/desired_hand_pose");
  // desired_hand_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > ("/PotentialFieldControl/desired_hand_pose",1);

}

void HandPoseGenerator::HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{

  desperate_housewife::fittedGeometriesSingle obstacle;
  desperate_housewife::fittedGeometriesArray obstaclesMsg;
  desperate_housewife::handPoseSingle DesiredHandPose;

  if ( msg->geometries.size() == 1)
    {
      // DesiredHandPose.pose.position.x = -0.879097;
      // DesiredHandPose.pose.position.y = -0.11868;
      // DesiredHandPose.pose.position.z = -0.598046;
      // DesiredHandPose.pose.orientation.x = 0.0584961;
      // DesiredHandPose.pose.orientation.y = 0.0608033;
      // DesiredHandPose.pose.orientation.z = 0.718077;
      // DesiredHandPose.pose.orientation.w = 0.69083;
      // DesiredHandPose.whichArm = 1;
      // DesiredHandPose.isGraspable = true;


      DesiredHandPose = generateHandPose( msg->geometries[0] );
      // if(DesiredHandPose.isGraspable != true)
      // {
      //   obstacle.pose = msg->geometries[0].pose;
      //   for (unsigned j=0; j < msg->geometries[0].info.size(); j++)
      //   {
        
      //     obstacle.info.push_back(msg->geometries[0].info[j]);
      //   }
      //   obstaclesMsg.geometries.push_back( obstacle );
      // }
    }
    else
    {
      ROS_DEBUG("More than one geometry identified");
      for (unsigned int i=0; i<msg->geometries.size(); i++)
      {
        objects_vec.push_back(msg->geometries[i]);
      }

      std::sort(objects_vec.begin(), objects_vec.end(), [](desperate_housewife::fittedGeometriesSingle first, desperate_housewife::fittedGeometriesSingle second) {
        double distfirst = std::sqrt( first.pose.position.x*first.pose.position.x + first.pose.position.y*first.pose.position.y + first.pose.position.z*first.pose.position.z);
        double distsecond = std::sqrt( second.pose.position.x*second.pose.position.x + second.pose.position.y*second.pose.position.y + second.pose.position.z*second.pose.position.z);
        return (distfirst < distsecond); });

      DesiredHandPose = generateHandPose( objects_vec[0] );

      unsigned int i = (DesiredHandPose.isGraspable != true ? 0 : 1 );


      for (unsigned int i_ = i; i_<objects_vec.size(); i_++)
      {
        obstacle.pose = objects_vec[i_].pose;
    
        for (unsigned j=0; j < objects_vec[i_].info.size(); j++)
        {
          obstacle.info.push_back(objects_vec[i_].info[j]); 
        }
        
        obstaclesMsg.geometries.push_back( obstacle );
      }

    }
   
    if(DesiredHandPose.isGraspable == true)
    {
      if (DesiredHandPose.whichArm == 1) 
      {
        desired_hand_left_pose_publisher_.publish( DesiredHandPose );
      }
      else
      {
        desired_hand_right_pose_publisher_.publish( DesiredHandPose );
      }
      // desired_hand_publisher_.publish( DesiredHandPose );  //to filtered the hand_pose
    }

   
    
    objects_vec.clear();
    tf::Transform tfHandTrasform;
    tf::poseMsgToTF( DesiredHandPose.pose, tfHandTrasform);
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), desired_hand_frame_.c_str()) );
  

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
  }

  
  return hand_pose_local;
}

bool HandPoseGenerator::isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry )
{
  // std::cout<<"geometry.info[0]: "<<geometry.info[0]<<std::endl;
  // if (( geometry.type == 3 && geometry.info[0] < .11 && geometry.info[4] > 65) ) //0.15
  // {
  //   return true;
  // }
  // return false;
    if (geometry.info[geometry.info.size() - 1] >=50 && geometry.info[0] < 0.15)
      {

        return true;
      }
      return false;



}
