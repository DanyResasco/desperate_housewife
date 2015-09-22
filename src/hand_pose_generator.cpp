#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>


#include <desperate_housewife/fittedGeometries.h>
#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/PointArray.h>

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

  void HandPoseGeneratorCallback(const desperate_housewife::fittedGeometries::ConstPtr& msg);
  // bool dist_compare ( const desperate_housewife::fittedGeometriesSingle& first, const desperate_housewife::fittedGeometriesSingle& second);
  // bool dist_compare ( desperate_housewife::fittedGeometriesSingle& first, desperate_housewife::fittedGeometriesSingle second);

};

HandPoseGenerator::HandPoseGenerator()
{

  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  stream_subscriber_ = nh.subscribe(geometries_topic_, 1, &HandPoseGenerator::HandPoseGeneratorCallback, this);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose", desired_hand_pose_topic_, "/PotentialFieldControl/desired_hand_pose");
  desired_hand_pose_publisher_ = nh.advertise<geometry_msgs::Pose > (desired_hand_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/obstacle_list", obstalces_topic_, "/PotentialFieldControl/obstacle_list");
  obstacles_publisher_ = nh.advertise<desperate_housewife::PointArray > (obstalces_topic_.c_str(),1);

}

void HandPoseGenerator::HandPoseGeneratorCallback(const desperate_housewife::fittedGeometries::ConstPtr& msg)
{



  if( msg->geometries.size() <= 0 )
  {
    ROS_INFO("There are not a objects in the scene");
  }
  else
  {
    if ( msg->geometries.size() == 1)
    {
      // generate_hand_pose( msg->geometries[0].pose );
      desired_hand_pose_publisher_.publish( msg->geometries[0].pose );
    }
    else
    {
      for (unsigned int i=0; i<msg->geometries.size(); i++)
      {
        objects_vec.push_back(msg->geometries[i]);
      }
      // std::sort( objects_vec.begin(), objects_vec.end(), dist_compare );

      std::sort(objects_vec.begin(), objects_vec.end(), [](desperate_housewife::fittedGeometriesSingle first, desperate_housewife::fittedGeometriesSingle second) {
          double distfirst = std::sqrt( first.pose.position.x*first.pose.position.x + first.pose.position.y*first.pose.position.y + first.pose.position.z*first.pose.position.z);
          double distsecond = std::sqrt( second.pose.position.x*second.pose.position.x + second.pose.position.y*second.pose.position.y + second.pose.position.z*second.pose.position.z);
  return (distfirst < distsecond); });

        // generate_hand_pose( objects_vec[0] );
        desired_hand_pose_publisher_.publish( objects_vec[0].pose );
        desperate_housewife::PointArray obstaclesMsg;
        for (unsigned int i=1; i<objects_vec.size(); i++)
        {
          obstaclesMsg.point.push_back( objects_vec[i].pose.position );
        }
        obstacles_publisher_.publish( obstaclesMsg );

    }
  }

}

// bool HandPoseGenerator::dist_compare ( desperate_housewife::fittedGeometriesSingle& first, desperate_housewife::fittedGeometriesSingle second)
// {
//   double distfirst = std::sqrt( first.pose.position.x*first.pose.position.x + first.pose.position.y*first.pose.position.y + first.pose.position.z*first.pose.position.z);
//   double distsecond = std::sqrt( second.pose.position.x*second.pose.position.x + second.pose.position.y*second.pose.position.y + second.pose.position.z*second.pose.position.z);

//   return (distfirst < distsecond);
// }

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

