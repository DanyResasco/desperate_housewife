#ifndef test_H
#define test_H

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry> 

#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <visualization_msgs/Marker.h>

class sendObj
{

public:
  sendObj();
  ~sendObj(){};
  ros::Publisher  geometries_publisher_, marker_publisher_;
  ros::NodeHandle nh;
  std::string obstacles_topic_right;
  ros::Subscriber sub_grid_;
  tf::TransformBroadcaster tf_geometriesTransformations_;
private:

  void obst(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void generateMarkerMessages( std::vector<KDL::Frame> &Obj_pose, std::vector<double> radius, std::vector<double> height );
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "sendObj_obst_test");
  sendObj node;
  // node.SendVitoHome();

  ROS_INFO("[sendObj] Node is ready");

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




sendObj::sendObj()
{
  std::string publish_on_topic;
  nh.param<std::string>("obstacles_to_pub", publish_on_topic, "obstacles");
  
  sub_grid_ = nh.subscribe("send_obst", 1, &sendObj::obst, this);

  ROS_INFO_STREAM("publish_on_topic: "<<publish_on_topic.c_str());
  geometries_publisher_ = nh.advertise<desperate_housewife::fittedGeometriesArray > (publish_on_topic.c_str(), 1);

  marker_publisher_ = nh.advertise<visualization_msgs::Marker >( "frame_obst", 1 );
}


void sendObj::obst(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  desperate_housewife::fittedGeometriesArray fittedGeometriesArrayMsg;
  int p = 0;
  std::cout<<"sms"<<std::endl;
  int loop =  msg->data.size()/5;
  std::vector<KDL::Frame> vect_obst;
  std::vector<double> geom_radius;
  std::vector<double> geom_height;

  for( int i=0; i< loop;i++)
    {
      desperate_housewife::fittedGeometriesSingle fittedGeometriesSingleMsg;

      fittedGeometriesSingleMsg.pose.position.x = msg->data[0+p];
      fittedGeometriesSingleMsg.pose.position.y =  msg->data[1+p];
      fittedGeometriesSingleMsg.pose.position.z = msg->data[2+p];
      double angle = msg->data[3+p];
      KDL::Rotation transformation_ = KDL::Rotation::RotX(angle*M_PI/180.0);
      transformation_.GetQuaternion(fittedGeometriesSingleMsg.pose.orientation.x, fittedGeometriesSingleMsg.pose.orientation.y,
                                 fittedGeometriesSingleMsg.pose.orientation.z, fittedGeometriesSingleMsg.pose.orientation.w );
      std::vector<double> geom_info;
      geom_info.push_back(msg->data[4+p]);
      geom_info.push_back(msg->data[5+p]);
      geom_radius.push_back(msg->data[4+p]);
      geom_height.push_back(msg->data[5+p]);

      for(unsigned int j=0; j< geom_info.size();j++)
      {
          fittedGeometriesSingleMsg.info.push_back(geom_info[j]);
      }

      fittedGeometriesArrayMsg.geometries.push_back( fittedGeometriesSingleMsg );
      p += 6;
      KDL::Frame frame_obj;
      tf::poseMsgToKDL(fittedGeometriesSingleMsg.pose, frame_obj);
      vect_obst.push_back(frame_obj);
    }


  generateMarkerMessages( vect_obst, geom_radius, geom_height );

  geometries_publisher_.publish(fittedGeometriesArrayMsg);
}

void sendObj::generateMarkerMessages( std::vector<KDL::Frame> &Obj_pose, std::vector<double> radius, std::vector<double> height )
{
  for (unsigned int i = 0; i < Obj_pose.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = Obj_pose[i].p.x();
      marker.pose.position.y = Obj_pose[i].p.y();
      marker.pose.position.z = Obj_pose[i].p.z();
      double x,y,z,w;
      Obj_pose[i].M.GetQuaternion(x,y,z,w);

      marker.pose.orientation.x = x;
      marker.pose.orientation.y = y;
      marker.pose.orientation.z = z;
      marker.pose.orientation.w = w;

      marker.scale.x = radius[i]*2;
      marker.scale.y = radius[i]*2;
      marker.scale.z = height[i];

      marker.color.a = 1.0; // for the clearness
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.lifetime = ros::Duration(100);
      marker_publisher_.publish(marker);

      std::string obst_name= "obj_" + std::to_string(i);

      tf::Transform tfGeomTRansform;
      geometry_msgs::Pose p;
      tf::poseKDLToMsg (Obj_pose[i], p );
      tf::poseMsgToTF( p, tfGeomTRansform );
      tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "world", obst_name.c_str()) );
    }
}








#endif
