#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>


#include <desperate_housewife/fittedGeometries.h>
#include <desperate_housewife/fittedGeometriesSingle.h>

class HandPoseGenerator{

private:

  std::string geometries_topic_, desired_hand_pose_topic_, obstalces_topic_;
  std::list<desperate_housewife::fittedGeometriesSingle> objects_list;

public:

  ros::Subscriber stream_subscriber_;
  ros::NodeHandle nh;
  ros::Publiser desired_hand_pose_publisher_;
  ros::Publiser obstacles_publisher_;

  HandPoseGenerator();
  ~HandPoseGenerator(){};

  void HandPoseGeneratorCallback(const desperate_housewife::fittedGeometries::ConstPtr& msg);

};

HandPoseGenerator::HandPoseGenerator()
{

  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  stream_subscriber_ = nh.subscribe(geometries_topic_, 1, &HandPoseGenerator::HandPoseGeneratorCallback, this);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose", desired_hand_pose_topic_, "/PotentialFieldControl/desired_hand_pose");
  nh.param<geometry_msgs::Pose>("")

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

      desired_hand_pose_publisher.publish(  );
    }
    for (unsigned int i=0; i<msg->geometries.size(); i++)
    {
      objects_list.push_back(msg->geometries[i]);
    }
    
  }

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

