#ifndef dany_bicchi_hand_h
#define dany_bicchi_hand_h

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
#include <desperate_housewife/hand.h>
#include <geometry_msgs/PoseArray.h>


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

// KDL
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

class phobic_hand
{


	private:

		tf::TransformListener listener_info;
		std::vector<tf::StampedTransform> Goal;
		//tf::TransformBroadcaster tf_br;
		ros::NodeHandle nodeH;
		// std::vector<double> cyl_height;
		// std::vector<double> cyl_radius;
		// std::vector<double> cyl_info;
		// std::vector<double> cyl_v;
		double cyl_height;
		double cyl_radius;
		double cyl_info;
		double cyl_v;
		pcl::PointXYZ goal_position, obstacle_position;
		Eigen::Matrix4d frame_cylinder; //M_c_k
		std::vector<double> distance;
		bool Arm;
		double velocity_max;
		bool check_robot;
		double max_radius=0.1, max_lenght=0.20;
		pcl::PointXYZ Pos_HAND_r,Pos_HAND_l;
		Eigen::Matrix4d Pos_final_hand_r, Pos_final_hand_l;
		std::vector<geometry_msgs::Pose> Hand_pose;
		//tf::StampedTransform hand_tf_pose;
		tf::StampedTransform SoftHand_r;
		tf::StampedTransform SoftHand_l;
		ros::Publisher hand_info;
		bool Test_obj;

	
	public:

		void HandPoseCallback(const desperate_housewife::cyl_info cyl_msg);
		void GetCylPos(tf::StampedTransform &object);
		void WhichArm(pcl::PointXYZ Pos);
		bool objectORostacles();
		pcl::PointXYZ  Take_Pos(tf::StampedTransform &M_tf);
		std::pair<double, pcl::PointXYZ> GetDistance(pcl::PointXYZ &obj1,pcl::PointXYZ &obj2 );
		void SetHandPosition();
		void Send();
		void fromEigenToPose(Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &pose);
		
	
		phobic_hand(ros::NodeHandle node_hand): nodeH(node_hand)
		{
			hand_info = node_hand.advertise<desperate_housewife::hand>( "SoftHand_Pose", 1 );
			check_robot  = false;
			Test_obj  = true;
		}


		~phobic_hand(){};

};



Eigen::Matrix4d FromTFtoEigen(tf::StampedTransform &object);













#endif