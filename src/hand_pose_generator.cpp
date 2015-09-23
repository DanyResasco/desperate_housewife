#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>



#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <desperate_housewife/obstacleSingle.h>
#include <desperate_housewife/obstacleArray.h>
#include <desperate_housewife/handPoseSingle.h>

#include <algorithm>

class HandPoseGenerator{

private:

  std::string geometries_topic_, desired_hand_pose_topic_, obstalces_topic_;
  std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;

public:

  ros::Subscriber stream_subscriber_;
  ros::NodeHandle nh;
  ros::Publisher desired_hand_pose_publisher_, obstacles_publisher_;

  HandPoseGenerator();
  ~HandPoseGenerator(){};

  void HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
  desperate_housewife::handPoseSingle generateHandPose( desperate_housewife::fittedGeometriesSingle geometry );
  bool isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry );
  geometry_msgs::Pose placeHand ( desperate_housewife::fittedGeometriesSingle geometry );

};

HandPoseGenerator::HandPoseGenerator()
{

  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  stream_subscriber_ = nh.subscribe(geometries_topic_, 1, &HandPoseGenerator::HandPoseGeneratorCallback, this);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose", desired_hand_pose_topic_, "/PotentialFieldControl/desired_hand_pose");
  desired_hand_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/obstacle_list", obstalces_topic_, "/PotentialFieldControl/obstacle_list");
  obstacles_publisher_ = nh.advertise<desperate_housewife::obstacleArray > (obstalces_topic_.c_str(),1);

}

void HandPoseGenerator::HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{

  if( msg->geometries.size() <= 0 )
  {
    ROS_INFO("There are not a objects in the scene");
  }
  else
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

    desired_hand_pose_publisher_.publish( DesiredHandPose );
    obstacles_publisher_.publish( obstaclesMsg );
  }

}




desperate_housewife::handPoseSingle HandPoseGenerator::generateHandPose( desperate_housewife::fittedGeometriesSingle geometry )
{
  desperate_housewife::handPoseSingle hand_pose_local;

  if ( isGeometryGraspable ( geometry ))
  {
      hand_pose_local.pose = placeHand( geometry );
      hand_pose_local.isGraspable = true;
  }
  else
  {
    hand_pose_local.pose = geometry.pose;
    hand_pose_local.isGraspable = false;
  }

  hand_pose_local.whichArm = 0;
  return hand_pose_local;
}

bool HandPoseGenerator::isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry )
{
  return true;
}


geometry_msgs::Pose HandPoseGenerator::placeHand ( desperate_housewife::fittedGeometriesSingle geometry )
{

  geometry_msgs::Pose pose_local = geometry.pose;
  return pose_local;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "HandPoseGenerator Node");
  HandPoseGenerator node;
  ROS_INFO("[HandPoseGenerator] Node is ready");

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

