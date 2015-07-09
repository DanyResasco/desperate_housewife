#include "phobic_motion_planning.h"

int main(int argc, char** argv)
{
	
	ros::NodeHandle node_mp;
	phobic_mp phobic_local_mp(node_mp); 
	ros::Subscriber reader;
	reader = node_mp.subscribe(node_mp.resolveName("INFO_CYLINDER"), 1, &phobic_mp::MotionPlanningCallback, &phobic_local_mp);

	return 0;
	
}


void phobic_mp::MotionPlanningCallback(phobic_camera::cyl_info cyl_msg)
{
	
	// To the first we take an informations from cilynders and robot.
	if(cyl_msg.dimension <= 0)
	{
		ROS_INFO("There are not a objects in the scene");
	}
	else
	{
		
		//read the cylinder informations in tf::StampedTransform
		for (int i = 0; i < cyl_msg.dimension; i++)
		{

			listener_info.lookupTransform("/camera_rgb_optical_frame", "cilindro_" + std::to_string(i) , ros::Time(0), Goal[i] );
			cyl_height[i] = cyl_msg[i].height;
			cyl_radius[i] = cyl_msg[i].radius;
		}
		
		//read the robot informations in tf::StampedTransform. Vito has 7 link and 6 joint
		for(int i = 0; i<=7 ; i++)
		{
			listener_info.lookupTransform("/camera_rgb_optical_frame", "left_arm_" + std::to_string(i) + "_joint" , ros::Time(0), Vito_desperate.Link_left[i] );
			listener_info.lookupTransform("/camera_rgb_optical_frame", "right_arm_" + std::to_string(i) + "_joint" , ros::Time(0), Vito_desperate.Link_right[i] );

		}

		//Soft Hand information

		listener_info.lookupTransform("/camera_rgb_optical_frame", "right_hand_palm_link" , ros::Time(0), Vito_desperate.SoftHand_r );
		listener_info.lookupTransform("/camera_rgb_optical_frame", "left_hand_palm_link" , ros::Time(0), Vito_desperate.SoftHand_l );
                              
		// with this informations we can make a MP 

		for (int i = 0; i <  cyl_msg.dimension; i++)
		{
			SetPotentialField( *Goal[i].begin());
		}


	}

		


}

double phobic_mp::SetPotentialField( tf::StampedTransform object)
{
	//Set robot potential fields only first time
	if(first == true )
	{
		PF_ROBOT = SetPotentialField_robot();
		first= false;
	}

	// test for setting the potential field
	bool Test_obj;
	// if true the object is a goal, otherwise is a obstacles

	Test_obj = objectORostacles(object)
	
	Eigen::Matrix4d frame_eigen, frame_kinect;
	frame_eigen = FromTFtoEigen(tf::StampedTransform object);
	frame_kinect = frame_eigen.inverse();


	if(Test_obj == true)
	{	
		
		goal_position.x = frame_kinect(0,3);
		goal_position.y = frame_kinect(1,3);
		goal_position.z = frame_kinect(2,3);

	}

	else
	{	//set obstacles repulsion force
		
		obstacle_position.x = frame_kinect(0,3);
		obstacle_position.y = frame_kinect(1,3);
		obstacle_position.z = frame_kinect(2,3);

	}





	Goal.erase(Goal.begin());



}


bool phobic_mp::objectORostacles(tf::StampedTransform frame)
{

	
	cyl_height.front() ;
	cyl_radius[i] =cyl_msg.radius;






	return true;
}





Eigen::Matrix4d FromTFtoEigen(tf::StampedTransform object)
{
	Eigen::Quaterniond transf_quad(object);
	std::vector<double> translation;
	Eigen::Matrix3d rotation;

	translation = object.getAxis();
	rotation = toRotationMatrix	(transf_quad);

	Eigen::Matrix4d Matrix_transf;

	Matrix_transf.row(0) << rotation.row(0), translation[0];
	Matrix_transf.row(1) << rotation.row(1), translation[1];
	Matrix_transf.row(2) << rotation.row(2), translation[2];
	Matrix_transf.row(3) << 0,0,01;

	return Matrix_transf;

}