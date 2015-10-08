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
  obstacles_publisher_ = nh.advertise<desperate_housewife::obstacleArray > (obstalces_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_frame", desired_hand_frame_, "desired_hand_pose");

  nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "vito_anchor");
  nh.param<std::string>("/PotentialFieldControl/left_hand_frame", left_hand_frame_, "left_hand_palm_link");
  nh.param<std::string>("/PotentialFieldControl/right_hand_frame", right_hand_frame_, "right_hand_palm_link");

}

void HandPoseGenerator::HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{

  desperate_housewife::obstacleArray obstaclesMsg;
  desperate_housewife::obstacleSingle obstacle;
  desperate_housewife::handPoseSingle DesiredHandPose;

  if ( msg->geometries.size() == 1)
  {
    DesiredHandPose = generateHandPose( msg->geometries[0] );
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

    for (unsigned int i=1; i<objects_vec.size(); i++)
    {
      obstacle.point = objects_vec[i].pose.position;
      obstacle.radius = 0.05; // to set for the radius o length or whatever
      obstaclesMsg.obstacles.push_back( obstacle );
    }
  }
 
  if (DesiredHandPose.whichArm == 1) 
  {
    desired_hand_left_pose_publisher_.publish( DesiredHandPose );
  }
  else
  {
    desired_hand_right_pose_publisher_.publish( DesiredHandPose );
  }

 
  obstacles_publisher_.publish( obstaclesMsg );

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
  if ( geometry.type == 3 && geometry.info[0] < .15 )
  {
    return true;
  }
  return false;
}
