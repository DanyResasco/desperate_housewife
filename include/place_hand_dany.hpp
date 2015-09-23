#include <hand_pose_generator.h>


geometry_msgs::Pose HandPoseGenerator::placeHand ( desperate_housewife::fittedGeometriesSingle geometry )
{


  
  geometry_msgs::Pose pose_local = geometry.pose;
  return pose_local;

}


int HandPoseGenerator::whichArm( geometry_msgs::Pose object_pose ){

	tf::StampedTransform hand_left, hand_rigth;

	listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

	listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

	Eigen::Vector3d object_position(object_pose.position.x, object_pose.position.y, object_pose.position.z);
	Eigen::Vector3d hand_left_position(hand_left.getOrigin().x(),hand_left.getOrigin().y(),hand_left.getOrigin().z());
	Eigen::Vector3d hand_right_position(hand_rigth.getOrigin().x(),hand_rigth.getOrigin().y(),hand_rigth.getOrigin().z());

	double dist_to_left_hand = std::sqrt((object_position[0] - hand_left_position[0]) * (object_position[0] - hand_left_position[0]) +
				   (object_position[1] - hand_left_position[1]) * (object_position[1] - hand_left_position[1]) +
				   (object_position[2] - hand_left_position[2]) * (object_position[2] - hand_left_position[2]) );

	double dist_to_right_hand = std::sqrt((object_position[0] - hand_right_position[0]) * (object_position[0] - hand_right_position[0]) +
				   (object_position[1] - hand_right_position[1]) * (object_position[1] - hand_right_position[1]) +
				   (object_position[2] - hand_right_position[2]) * (object_position[2] - hand_right_position[2]) );

	if(dist_to_left_hand < dist_to_right_hand)
	{
	  return 0;
	  ROS_DEBUG("Vito uses a: left arm");
	}
	
	ROS_DEBUG("Vito uses a: Right arm");
	return 1;
}
