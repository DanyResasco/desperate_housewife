#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

#include <string>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

#include <desperate_housewife/handPoseSingle.h>

#include "desperate_housewife/goPose.h"

class SimulatedReferences
{
public:
  SimulatedReferences( ros::NodeHandle n_in){
    n = n_in;
    n.param<std::string>("/reference_simulated_topic", reference_simulated_topic, "/right_arm/PotentialFieldControlKinematic/command");
    pose_publisher = n.advertise< desperate_housewife::handPoseSingle >(reference_simulated_topic.c_str(), 100);
    ROS_INFO("Publishing references in: %s", reference_simulated_topic.c_str());
  };
  SimulatedReferences();
  // ~SimulatedReferences();
  void goHome();
  bool goPose(desperate_housewife::goPose::Request &request, desperate_housewife::goPose::Response &response);
  void goNewPose(geometry_msgs::Pose pose);
  desperate_housewife::handPoseSingle reference;
  ros::NodeHandle n;
  ros::Publisher pose_publisher;
  std::string reference_simulated_topic;
  tf::TransformBroadcaster tf_desired_hand_pose;

};

void SimulatedReferences::goNewPose(geometry_msgs::Pose pose)
{
  reference.pose = pose;
  pose_publisher.publish(reference);
  tf::Transform tfHandTrasform1;
  tf::poseMsgToTF( pose, tfHandTrasform1);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(),  "world", "new_desired_pose"));
  ROS_INFO("Going To Pose: [Position: x: %f, y: %f, z: %f] [Orientation x: %f, y: %f, z: %f w: %f]", pose.position.x,
           pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  return;
}
void SimulatedReferences::goHome()
{

  // reference.home = 1;
  // reference.obj = 0;

  double roll,pitch,yaw;
  n.param<double>("/home/right_arm_position_x", reference.pose.position.x, -0.75022);
  n.param<double>("/home/right_arm_position_y",  reference.pose.position.y,  0.47078);
  n.param<double>("/home/right_arm_position_z", reference.pose.position.z, 0.74494);
  n.param<double>("/home/right_arm_A_yaw", yaw,  0.334);
  n.param<double>("/home/right_arm_B_pitch", pitch, -0.08650);
  n.param<double>("/home/right_arm_C_roll", roll, -0.5108);
  KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll,pitch,yaw);

  Rot_matrix_r.GetQuaternion(reference.pose.orientation.x, reference.pose.orientation.y,
                             reference.pose.orientation.z, reference.pose.orientation.w);
  pose_publisher.publish(reference);

  tf::Transform tfHandTrasform1;
  tf::poseMsgToTF( reference.pose, tfHandTrasform1);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(),  "world", "new_desired_pose"));

  ROS_INFO("Going Home: [Position: x: %f, y: %f, z: %f] [Orientation x: %f, y: %f, z: %f w: %f]", reference.pose.position.x,
           reference.pose.position.y, reference.pose.position.z, reference.pose.orientation.x, reference.pose.orientation.y, reference.pose.orientation.z,
           reference.pose.orientation.w);


  return;
}

bool SimulatedReferences::goPose(desperate_housewife::goPose::Request &request, desperate_housewife::goPose::Response &response)
{

  ROS_INFO("Task is go to %s", request.where.c_str());

  if (request.where.compare("home") == 0)
    {
      goHome();
      return true;
    }
  if (request.where.compare("newPose") == 0)
    {
      goNewPose(request.pose);
      return true;
    }

  ROS_INFO("Note defined behavior");

  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SimulatedDesiredPose");

  ros::NodeHandle n;
  SimulatedReferences simulated_references(n);

  ros::ServiceServer service = simulated_references.n.advertiseService("goPose", &SimulatedReferences::goPose, &simulated_references);


  ros::Rate loop_rate(1);


  while (ros::ok())
    {

      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}



