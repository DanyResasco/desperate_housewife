#include "phobic_motion_planning.h"

int main(int argc, char** argv)
{
	
	ros::NodeHandle node_mp;
	phobic_mp phobic_local_mp(node_mp); 
	ros::Subscriber reader;
	reader = node_mp.subscribe(node_mp.resolveName("INFO_CYLINDER"), 1, &phobic_mp::MotionPlanningCallback, &phobic_local_mp);

	return 0;
	
}


void phobic_mp::MotionPlanningCallback(const desperate_housewife::cyl_info cyl_msg)
{
	
	// To the first we take an informations from cilynders and robot.
	if(cyl_msg.dimension <= 0)
	{
		ROS_INFO("There are not a objects in the scene");
	}
	else
	{
		Goal.resize(cyl_msg.dimension);
		//read the cylinder informations in tf::StampedTransform
		for (int i = 0; i < cyl_msg.dimension; i++)
		{

			listener_info.lookupTransform("/camera_rgb_optical_frame", "cilindro_" + std::to_string(i) , ros::Time(0), Goal[i] );
			cyl_height[i] = cyl_msg.length[i];
			cyl_radius[i] = cyl_msg.radius[i];
		}
		
		//read the robot informations in tf::StampedTransform. Vito has 7 link and 6 joint
		for(int i = 0; i<=7 ; i++)
		{
			listener_info.lookupTransform("/camera_rgb_optical_frame", "left_arm_" + std::to_string(i) + "_joint" , ros::Time(0), Vito_desperate.Link_left[i] );
			listener_info.lookupTransform("/camera_rgb_optical_frame", "right_arm_" + std::to_string(i) + "_joint" , ros::Time(0), Vito_desperate.Link_right[i] );
			Vito_desperate.robot_position_left.push_back(Take_Pos(Vito_desperate.Link_left[i]));
			Vito_desperate.robot_position_right.push_back(Take_Pos(Vito_desperate.Link_right[i]));
		}

		// //Soft Hand information

		listener_info.lookupTransform("/camera_rgb_optical_frame", "right_hand_palm_link" , ros::Time(0), Vito_desperate.SoftHand_r );
		listener_info.lookupTransform("/camera_rgb_optical_frame", "left_hand_palm_link" , ros::Time(0), Vito_desperate.SoftHand_l );
                              
		// // with this informations we can make a MP 

		for (int i = 0; i <  cyl_msg.dimension; i++)
		{
			SetPotentialField( *Goal.begin());
		}


	}

		


}

double phobic_mp::SetPotentialField( tf::StampedTransform object)
{
	//Set robot potential fields only first time
	// if(first == true )
	// {
	// 	SetPotentialField_robot();
	// 	first= false;
	// }

	// test for setting the potential field
	bool Test_obj = true;
	// if true the object is a goal, otherwise is a obstacles

	//Test_obj = objectORostacles(object)
	
	Eigen::Matrix4d frame_eigen, frame_kinect;
	frame_eigen = FromTFtoEigen(object);
	frame_kinect = frame_eigen.inverse();


	if(Test_obj == true)
	{	
		goal_position.x = frame_kinect(0,3);
		goal_position.y = frame_kinect(1,3);
		goal_position.z = frame_kinect(2,3);

	//	Calculate_force( goal_position);
	

	}

	else
	{	//set obstacles repulsion force
		
		obstacle_position.x = frame_kinect(0,3);
		obstacle_position.y = frame_kinect(1,3);
		obstacle_position.z = frame_kinect(2,3);

	}





	Goal.erase(Goal.begin());



}


// bool phobic_mp::objectORostacles(tf::StampedTransform frame)
// {

	
// 	cyl_height.front() ;
// 	cyl_radius[i] =cyl_msg.radius;






// 	return true;
// }





Eigen::Matrix4d FromTFtoEigen(tf::StampedTransform object)
{
	Eigen::Quaterniond transf_quad(object.getRotation().getW(),object.getRotation().getX(),object.getRotation().getY(),object.getRotation().getZ());
	transf_quad.normalize();
	Eigen::Vector3d translation(object.getOrigin().x(),object.getOrigin().y(),object.getOrigin().z());
	Eigen::Matrix3d rotation(transf_quad.toRotationMatrix());
	
	Eigen::Matrix4d Matrix_transf;

	Matrix_transf.row(0) << rotation.row(0), translation[0];
	Matrix_transf.row(1) << rotation.row(1), translation[1];
	Matrix_transf.row(2) << rotation.row(2), translation[2];
	Matrix_transf.row(3) << 0,0,01;

	return Matrix_transf;

}


// // // void phobic_mp::Calculate_force( pcl::PointXYZ Pos);
// // // {
	








// // }


void phobic_mp::SetPotentialField_robot(std::vector<double> Force_repulsion)
{
	std::vector<std::pair<double, pcl::PointXYZ>> distance_link_left;
		
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[2], Vito_desperate.robot_position_left[1]).first,GetDistance(Vito_desperate.robot_position_left[2], Vito_desperate.robot_position_left[1]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[3], Vito_desperate.robot_position_left[1]).first,GetDistance(Vito_desperate.robot_position_left[3], Vito_desperate.robot_position_left[1]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[3], Vito_desperate.robot_position_left[2]).first,GetDistance(Vito_desperate.robot_position_left[3], Vito_desperate.robot_position_left[2]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[4], Vito_desperate.robot_position_left[1]).first,GetDistance(Vito_desperate.robot_position_left[4], Vito_desperate.robot_position_left[1]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[4], Vito_desperate.robot_position_left[2]).first,GetDistance(Vito_desperate.robot_position_left[4], Vito_desperate.robot_position_left[2]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[4], Vito_desperate.robot_position_left[3]).first,GetDistance(Vito_desperate.robot_position_left[4], Vito_desperate.robot_position_left[3]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[5], Vito_desperate.robot_position_left[1]).first,GetDistance(Vito_desperate.robot_position_left[5], Vito_desperate.robot_position_left[1]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[5], Vito_desperate.robot_position_left[2]).first,GetDistance(Vito_desperate.robot_position_left[5], Vito_desperate.robot_position_left[2]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[5], Vito_desperate.robot_position_left[3]).first,GetDistance(Vito_desperate.robot_position_left[5], Vito_desperate.robot_position_left[3]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[5], Vito_desperate.robot_position_left[4]).first,GetDistance(Vito_desperate.robot_position_left[5], Vito_desperate.robot_position_left[4]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[1]).first,GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[1]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[2]).first,GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[2]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[3]).first,GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[3]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[4]).first,GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[4]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[5]).first,GetDistance(Vito_desperate.robot_position_left[6], Vito_desperate.robot_position_left[5]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[1]).first,GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[1]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[2]).first,GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[2]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[3]).first,GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[3]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[4]).first,GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[4]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[5]).first,GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[5]).second));
	distance_link_left.push_back(std::make_pair(GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[6]).first,GetDistance(Vito_desperate.robot_position_left[7], Vito_desperate.robot_position_left[6]).second));
	
	// Repulsive fields = K/distance^2 (1/distance -1/influence) partial_derivative_vector

	for (int i=0; i <= distance_link_left.size(); i++)
	{
		std::vector<double> vec_Temp;
		vec_Temp.push_back(distance_link_left[i].second.x);
		vec_Temp.push_back(distance_link_left[i].second.y);
		vec_Temp.push_back(distance_link_left[i].second.z);

		Force_repulsion.push_back( (P_obj/pow(distance_link_left[i].first,2)) * (1/distance_link_left[i].first - 1/influence) * vec_Temp[i] );
	}

	//

}

std::pair<double, pcl::PointXYZ> phobic_mp::GetDistance(pcl::PointXYZ obj1, pcl::PointXYZ obj2 )
{
	std::pair<double, pcl::PointXYZ> distance_local;
	pcl::PointXYZ local_point;
	
	distance_local.first = sqrt(pow(obj1.x-obj2.x,2) + pow(obj1.y-obj2.y,2) + pow(obj1.z-obj2.z,2));  
	local_point.x = (obj1.x-obj2.x);
	local_point.y = (obj1.y-obj2.y);
	local_point.z = (obj1.z-obj2.z);
	distance_local.second =  local_point;

	return distance_local;
}






pcl::PointXYZ phobic_mp::Take_Pos(tf::StampedTransform M_tf)
{
	pcl::PointXYZ Pos_vito;
	Eigen::Matrix4d Link_eigen;
	
	Link_eigen = FromTFtoEigen(M_tf);

	Pos_vito.x = Link_eigen(0,3);
	Pos_vito.y = Link_eigen(1,3);
	Pos_vito.z = Link_eigen(2,3);

	return Pos_vito;
	
}


