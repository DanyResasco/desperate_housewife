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

//Tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


class phobic_mp
{


	private:

		tf::TransformListener listener_info;
		std::vector<tf::StampedTransform> Goal;
		ros::NodeHandle nodeH;

		struct Robot
		{
			std::vector<tf::StampedTransform> Link_right;
			std::vector<tf::StampedTransform> Link_left;
			tf::StampedTransform SoftHand_r;
			tf::StampedTransform SoftHand_l;
			
		} Vito_desperate;


	
	public:

		void MotionPlanningCallback(std_msgs::UInt32 Num_cyl_msg);


		phobic_mp(ros::NodeHandle node_mp): nodeH(node_mp)
		{


		}


		~phobic_mp(){};







};

















#endif