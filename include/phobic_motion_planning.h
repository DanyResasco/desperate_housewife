#ifndef dany_bicchi_MP_h
#define dany_bicchi_MP_h

//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <desperate_housewife/cyl_info.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt32.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <utility>
#include <list>
#include <string>

//Eigen
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry> 


//Tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <desperate_housewife/cyl_info.h>

class phobic_mp
{


	private:

		tf::TransformListener listener_info;
		std::vector<tf::StampedTransform> Goal;
		ros::NodeHandle nodeH;
		bool first, insert;
		std::vector<double> cyl_height;
		std::vector<double> cyl_radius;
		pcl::PointXYZ goal_position, obstacle_position;
		Eigen::Matrix4d frame_kinect;
		double P_hand = 0.5;
		double P_obj = -1;
		double P_goal = 1;	
		double influence ;	// limit distance of the potential filed influence
		double Force;
		std::vector<double> distance;
		std::vector<double> Force_attractive;
		std::vector<double> Force_repulsion;
		bool Arm;

		
		
		struct Robot
		{
			std::vector<tf::StampedTransform> Link_right;
			std::vector<tf::StampedTransform> Link_left;
			tf::StampedTransform SoftHand_r;
			tf::StampedTransform SoftHand_l;
			std::vector<pcl::PointXYZ> robot_position_left, robot_position_right;
			pcl::PointXYZ Pos_HAND_r,Pos_HAND_l;
			
		} Vito_desperate;


	
	public:

		void MotionPlanningCallback(const desperate_housewife::cyl_info cyl_msg);
		double SetrepulsiveField( tf::StampedTransform object);
		void SetPotentialField_robot(std::vector<double> &Force_repulsion);
		bool objectORostacles();
		void Calculate_force();
		void SetAttraciveField( pcl::PointXYZ Pos);
		void SetPotentialField( tf::StampedTransform object);
		void SetRepulsiveFiled(pcl::PointXYZ Pos);
		pcl::PointXYZ  Take_Pos(tf::StampedTransform M_tf);
		std::pair<double, pcl::PointXYZ> GetDistance(pcl::PointXYZ obj1,pcl::PointXYZ obj2 );




		phobic_mp(ros::NodeHandle node_mp): nodeH(node_mp)
		{
			
			ros::param::get("~influence", influence);
			ROS_INFO("influence %lf", influence);


		}


		~phobic_mp(){};







};



Eigen::Matrix4d FromTFtoEigen(tf::StampedTransform object);













#endif