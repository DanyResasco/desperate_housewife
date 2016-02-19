#include <HandPoseGenerator_state.h>
#include <visualization_msgs/Marker.h>



std::vector< desperate_housewife::fittedGeometriesSingle > HandPoseGenerator::getClosestObject(std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec)
{
	int Arm_to_use_ = CheckWhichArmIsActive();

	std::vector< desperate_housewife::fittedGeometriesSingle > obj_vect_sorted = objects_vec;

	tf::StampedTransform hand_right, hand_left;
	listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_right);
	listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

	switch (Arm_to_use_)
	{

	case 0: /*only right arm*/
	{
		obj_vect_sorted = transform_to_world( SortbyHand(objects_vec, hand_right), hand_right );
		break;
	}

	case 1: /*left arm*/
	{
		obj_vect_sorted = transform_to_world( SortbyHand(objects_vec, hand_left), hand_left);
		break;
	}
	case 2: /*both arm*/
	{
		obj_vect_sorted = SortBoth(objects_vec, hand_right, hand_left);
		break;
	}
	}

	return obj_vect_sorted;
}


std::vector<desperate_housewife::fittedGeometriesSingle> HandPoseGenerator::SortbyHand(std::vector<desperate_housewife::fittedGeometriesSingle> objects_vec, tf::StampedTransform hand_position)
{

	geometry_msgs::Pose hand_position_local;
	hand_position_local.position.x = hand_position.getOrigin().x();
	hand_position_local.position.y = hand_position.getOrigin().y();
	hand_position_local.position.z = hand_position.getOrigin().z();
	hand_position_local.orientation.x = hand_position.getRotation().x();
	hand_position_local.orientation.y = hand_position.getRotation().y();
	hand_position_local.orientation.z = hand_position.getRotation().z();
	hand_position_local.orientation.w = hand_position.getRotation().w();

	Eigen::Matrix4d T_h_w = FromMsgtoEigen(hand_position_local);

	std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec_roted_by_hand = objects_vec;

	for (unsigned int i = 0; i < objects_vec.size(); i++)
	{
		Eigen::Matrix4d T_o_w = FromMsgtoEigen(objects_vec_roted_by_hand[i].pose);
		geometry_msgs::Pose pose_temp;
		Eigen::Matrix4d T_o_h = T_h_w.inverse() * T_o_w;
		fromEigenToPose(T_o_h, pose_temp);
		// desperate_housewife::fittedGeometriesSingle object_temp = objects_vec[i];
		objects_vec_roted_by_hand[i].pose =  pose_temp;
		// objects_vec_roted_by_hand.push_back(object_temp);
	}

	std::sort(objects_vec_roted_by_hand.begin(), objects_vec_roted_by_hand.end(), [](desperate_housewife::fittedGeometriesSingle first, desperate_housewife::fittedGeometriesSingle second) {
		double distfirst = std::sqrt( first.pose.position.x * first.pose.position.x + first.pose.position.y * first.pose.position.y + first.pose.position.z * first.pose.position.z);
		double distsecond = std::sqrt( second.pose.position.x * second.pose.position.x + second.pose.position.y * second.pose.position.y + second.pose.position.z * second.pose.position.z);
		return (distfirst < distsecond);
	});

	return objects_vec_roted_by_hand;
}


std::vector<desperate_housewife::fittedGeometriesSingle> HandPoseGenerator::SortBoth(std::vector<desperate_housewife::fittedGeometriesSingle> objects_vec, tf::StampedTransform hand_right, tf::StampedTransform hand_left)
{
	// ROS_INFO("both arm is active");
	std::vector<desperate_housewife::fittedGeometriesSingle> sorted_right = SortbyHand(objects_vec, hand_right);
	std::vector<desperate_housewife::fittedGeometriesSingle> sorted_left = SortbyHand(objects_vec, hand_left);
	std::vector<desperate_housewife::fittedGeometriesSingle> sorted_vect = objects_vec;

	Eigen::Vector3f right(sorted_right[0].pose.position.x, sorted_right[0].pose.position.y, sorted_right[0].pose.position.z);
	Eigen::Vector3f left(sorted_left[0].pose.position.x, sorted_left[0].pose.position.y, sorted_left[0].pose.position.z);

	sorted_vect = transform_to_world( sorted_right, hand_right);

	if (right.norm() > left.norm())
	{
		sorted_vect = transform_to_world( sorted_left, hand_left);
		// ROS_INFO("Left arm is closer");
	}

	return sorted_vect;

}


std::vector<desperate_housewife::fittedGeometriesSingle> HandPoseGenerator::transform_to_world(std::vector<desperate_housewife::fittedGeometriesSingle> objects_vec, tf::StampedTransform hand_position)
{

	geometry_msgs::Pose hand_position_local;
	hand_position_local.position.x = hand_position.getOrigin().x();
	hand_position_local.position.y = hand_position.getOrigin().y();
	hand_position_local.position.z = hand_position.getOrigin().z();
	hand_position_local.orientation.x = hand_position.getRotation().x();
	hand_position_local.orientation.y = hand_position.getRotation().y();
	hand_position_local.orientation.z = hand_position.getRotation().z();
	hand_position_local.orientation.w = hand_position.getRotation().w();

	Eigen::Matrix4d T_h_w = FromMsgtoEigen(hand_position_local);

	std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec_roted_by_hand = objects_vec;

	for (unsigned int p = 0; p < objects_vec_roted_by_hand.size(); p ++)
	{
		geometry_msgs::Pose pose_temp;
		Eigen::Matrix4d T_o_h = FromMsgtoEigen(objects_vec_roted_by_hand[p].pose);
		Eigen::Matrix4d Temp = T_h_w * T_o_h;
		fromEigenToPose(Temp, pose_temp);
		objects_vec_roted_by_hand[p].pose = pose_temp;
	}

	return objects_vec_roted_by_hand;
}