#ifndef GRID_H
#define GRID_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <desperate_housewife/fittedGeometriesSingle.h>
#include <desperate_housewife/fittedGeometriesArray.h>
#include <potential_field_control.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>


namespace desperate_housewife
{
  class ball
  {
  public:
    ros::NodeHandle nh;
   
    void DrawSphere( Eigen::Matrix<double,6,1> &pos_ball );
   
    void InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
    ball();
    ~ball();
    Eigen::Matrix<double,6,1> GetFieldRep(KDL::Vector &point_pos);
    //to visualize in rviz
    void SeeMarker(KDL::Vector &Pos, std::string obst_name, KDL::Frame &frames);
    //integration
    void GetVelocityAndPosition(Eigen::Matrix<double,6,1> &Force,  double delta);
    bool init(ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void ballInfo(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void DrawCylinder();

  private:
    ros::Subscriber sub_grid_,obstacles_subscribe_;
    std::vector<double> Object_radius;
    std::vector<double> Object_height;
    ros::Publisher marker_pub,marker_publisher_ ;
    std::string obstacle_avoidance;
    desperate_housewife::PotentialFieldControl pfc;
    std::vector<KDL::Frame> Object_position;
    ros::Publisher vis_pub;
    KDL::Vector POS_init;
    KDL::Vector pos_des;
    KDL::Vector ball_pos;
    double radius,mass;
    Eigen::Matrix<double,6,1> Force_attractive;
    Eigen::Matrix<double,6,1> x_dot;
    Eigen::Matrix<double,6,1> F_repulsive;
    KDL::Frame frame_des,frame_init;
    KDL::Twist x_err_;
    Eigen::Matrix<double,6,1> velocity,pos; 
    tf::TransformBroadcaster tf_geometriesTransformations_;

  };

  KDL::Frame FromEigenToKdl(Eigen::Matrix<double,6,1> Force);
}
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "desperate_ball_test_node");
//   ball node;
//   // node.SendVitoHome();

//   ROS_INFO("[ball test] Node is ready");

//   double spin_rate = 10;
//   ros::param::get("~spin_rate",spin_rate);
//   ROS_DEBUG( "Spin Rate %lf", spin_rate);

//   ros::Rate rate(spin_rate); 

//   while (node.nh.ok())
//   {
//     ros::spinOnce(); 
//     rate.sleep();
//   }
//   return 0;
// }

// ball::ball()
// {
//     //ball   
//     sub_grid_ = nh.subscribe("gridspace", 1, &ball::ballInfo, this);
//     vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 1 );
//     nh.param<std::string>("PotentialFieldControl/obstacle_list" , obstacle_avoidance, "/right_arm/PotentialFieldControl/obstacle_pose_right");
//     obstacles_subscribe_ = nh.subscribe(obstacle_avoidance.c_str(), 1, &ball::InfoGeometry, this); 
// }

// void ball::ballInfo(const std_msgs::Float64MultiArray::ConstPtr &msg)
// {
//     KDL::Vector ball_pos;
//     double radius,mass;
//     Eigen::Matrix<double,6,1> Force_attractive;
//     ball_pos.x() = msg->data[0];
//     ball_pos.y() = mag->data[1];
//     ball_pos.z() = msg->data[2];
//     radius = msg->data[3];
//     mass = msg->data[4];
//     KDL::Vector pos_des;
//     pos_des.x() = msg->data[5];
//     pos_des.y() = msg->data[6];
//     pos_des.z() = msg->data[7];

//     SeeMarker(pos_des);
//     KDL::JntArray Kp_,Kd_;
//     Eigen::Matrix<double,6,1> x_dot_(0,0,0,0,0,0); //e-e velocity

//     Eigen::Matrix<double,6,1> F_repulsive = GetFieldRep(ball_pos);
//     for(int i = 0; i < Force_attractive.size(); i++)
//     {
//         Kp(i) = 200;
//         Kd(i) = 100;
//         Force_attractive(i) =  -Kd_(i)*(x_dot_(i)) + Kp_(i)*diff(POS_init,pos_des);
//     }
    
//     GetVelocityAndPosition()
//     F_w_B = F_w_A.addDelta(t,timestep)

// }

// void ball::SeeMarker(KDL::Vector &Pos)
// {
//    std::string obst_name= "pose_desired" ;

//     tf::Transform tfGeomTRansform;
//     geometry_msgs::Pose p;
//     p.position.x = Pos.x();
//     p.position.y = Pos.y();
//     p.position.z = Pos.z();
//     p.orientation.x = 0;
//     p.orientation.y = 0;
//     p.orientation.z = 0;
//     p.orientation.w = 1; 
//     tf::poseMsgToTF( p, tfGeomTRansform );
//     tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "vito_anchor", obst_name.c_str()) );


// }



// void ball::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
// {
//       Object_radius.clear();
//       Object_height.clear();
//       Object_position.clear();
//       // Time_traj_rep = 0;
      
//       //get info for calculates objects surface
//       for(unsigned int i=0; i < msg->geometries.size(); i++)
//       {
//         KDL::Frame frame_obj;
//         Object_radius.push_back(msg->geometries[i].info[0]);  //radius
//         Object_height.push_back(msg->geometries[i].info[1]);  //height

//         tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
//         Object_position.push_back(frame_obj); 
//       }
// }

// void ball::DrawArrow( KDL::Vector &gridspace_Force, KDL::Vector &gridspace_point, int K, double Fmin, double Fmax )
// {
//     // std::cout<<"disegno"<<std::endl;
//     int32_t shape = visualization_msgs::Marker::ARROW;
//     visualization_msgs::Marker marker;

//     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
//     marker.header.frame_id = "vito_anchor";
//     marker.header.stamp = ros::Time::now();

//     marker.type = marker.SPHERE
//     marker.action = marker.ADD
//     marker.scale.x = 0.2
//     marker.scale.y = 0.2
//     marker.scale.z = 0.2
//     marker.color.a = 1.0
//     marker.pose.orientation.w = 1.0
//     marker.lifetime = ros::Duration(1000);

//     vis_pub.push_back(marker);
// }

//  Eigen::Matrix<double,6,1> ball::GetFieldRep(KDL::Vector &point_pos)
// {
//     std::vector<Eigen::Matrix<double,6,1> > F_rep;
//     //repulsive with obj
//     for(unsigned int i=0; i < Object_position.size(); i++)
//     {  
//         double influence = Object_radius[i] + 0.2;
//         std::vector<KDL::Vector> pos;
//         pos.push_back(point_pos);
//         std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex;
//         ForceAndIndex = pfc.GetRepulsiveForce(pos, influence, Object_position[i], Object_radius[i], Object_height[i]);
//         F_rep.push_back(ForceAndIndex.first);  
//     }
  
//     Eigen::Matrix<double,6,1> f = Eigen::Matrix<double,6,1>::Zero(); 
      
//     for(unsigned int k=0; k < F_rep.size();k++)
//     {
//         f = f + F_rep[k];
//     }

//     return f;

// }




















#endif