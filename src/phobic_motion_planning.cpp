#include "phobic_motion_planning.h"

int main(int argc, char** argv)
{
	
	ros::NodeHandle node_mp;
	phobic_mp phobic_local_mp(node_mp); 
	ros::Subscriber reader;
	reader = node_mp.subscribe(node_mp.resolveName("cyl_N"), 1, &phobic_mp::MotionPlanningCallback, &phobic_local_mp);

	return 0;
	
}


void phobic_mp::MotionPlanningCallback(std_msgs::UInt32 Num_cyl_msg)
{
	
	if(Num_cyl_msg.data > 0)
	{
		//read the cylinder informations in tf::StampedTransform
		for (int i = 0; i < Num_cyl_msg.data; i++)
		{

			listener_info.lookupTransform("/camera_rgb_optical_frame", "cilindro_" + std::to_string(i) , ros::Time(0), Goal[i] );
		}
		
		//read the robot informations in tf::StampedTransform. Vito has 7 link and 6 joint
		for(int i = 0; i<=7; i++)
		{
			listener_info.lookupTransform("/camera_rgb_optical_frame", "left_arm_" + std::to_string(i) + "_joint" , ros::Time(0), Vito_desperate.Link_left[i] );
			listener_info.lookupTransform("/camera_rgb_optical_frame", "right_arm_" + std::to_string(i) + "_joint" , ros::Time(0), Vito_desperate.Link_right[i] );

		}

		//Soft Hand information

		listener_info.lookupTransform("/camera_rgb_optical_frame", "right_hand_palm_link" , ros::Time(0), Vito_desperate.SoftHand_r );
		listener_info.lookupTransform("/camera_rgb_optical_frame", "left_hand_palm_link" , ros::Time(0), Vito_desperate.SoftHand_l );
                              
	}
	else
	{
		ROS_INFO("There are not a objects in the scene");
	}



}