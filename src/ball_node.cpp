#include <test_ball.h>
// #include <grid.hpp>
// #include <utils/pseudo_inversion.h>
// #include <utils/skew_symmetric.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

// #include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
// #include <trajectory_msgs/JointTrajectory.h>

#include <math.h>

#define  treshold_influence  0.20


  ball::ball()
  {
      //ball   
      sub_grid_ = nh.subscribe("SphereInfo", 1, &ball::ballInfo, this);
      vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
      nh.param<std::string>("obstacle_list" , obstacle_avoidance, "obstacle_pose");
      obstacles_subscribe_ = nh.subscribe(obstacle_avoidance.c_str(), 1, &ball::InfoGeometry, this); 
      Force_attractive = Eigen::Matrix<double,6,1>::Zero();
      F_repulsive = Eigen::Matrix<double,6,1>::Zero();
      x_dot = Eigen::Matrix<double,6,1>::Zero();
      // pos = Eigen::Matrix<double,6,1>::Zero();   
  }

  void ball::ballInfo(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
      ball_pos = Eigen::Matrix<double,6,1>::Zero();
      pos_des = Eigen::Matrix<double,6,1>::Zero();

      //initial ball position
      ball_pos(0) = msg->data[0];
      ball_pos(1) = msg->data[1];
      ball_pos(2) = msg->data[2];
      radius = msg->data[3];
      mass = msg->data[4];
      //desired ball position 
      pos_des(0) = msg->data[5];
      pos_des(1) = msg->data[6];
      pos_des(2) = msg->data[7];

      SeeMarker(pos_des, "pose_desired");
      SeeMarker(ball_pos, "pose_init");

      Update();
  }

  void ball::Update()
  {   
      //if ball is not arrived, update the position and velocity 
      if( (ball_pos(0) != pos_des(0)) && (ball_pos(1) != pos_des(1)) && (ball_pos(2) != pos_des(2)) )
      {
        F_repulsive = GetFieldRep(ball_pos);
        double Kp,Kd;
        Kp = 200;
        Kd = 100;
        
        x_err_ = pos_des - ball_pos;

        for(int i = 0; i < Force_attractive.size(); i++)
        {   
            Force_attractive(i) =  (-Kd * (x_dot(i)) + Kp * x_err_(i)) / mass;
        }

        Eigen::Matrix<double,6,1> Force_tot = Force_attractive + F_repulsive;
        double delta = 0.1;
        GetVelocityAndPosition(Force_tot,delta); // Force is just divide by mass
        ros::spinOnce();
      }
      else
        ROS_INFO("ball arrived");
        
  }

  

  void ball::GetVelocityAndPosition(Eigen::Matrix<double,6,1> &Force,  double delta)
  {
    Eigen::Matrix<double,6,1> velocity_last = x_dot; 
    Eigen::Matrix<double,6,1> pos_last = ball_pos;
   
    x_dot = velocity_last + delta * Force;
    ball_pos = 0.5 * Force * delta * delta + x_dot * delta + pos_last;

    DrawSphere( ball_pos );
  }

  

  void ball::SeeMarker(Eigen::Matrix<double,6,1> &Pos, std::string obst_name)
  {
     // std::string obst_name= "pose_desired" ;
      tf::Transform tfGeomTRansform;
      geometry_msgs::Pose p;
      p.position.x = Pos(0);
      p.position.y = Pos(1);
      p.position.z = Pos(2);
      p.orientation.x = 0;
      p.orientation.y = 0;
      p.orientation.z = 0;
      p.orientation.w = 1; 
      tf::poseMsgToTF( p, tfGeomTRansform );
      // tf::poseMsgToKDL(p, frames); 
      tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "vito_anchor", obst_name.c_str()) );
  }


  void ball::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
  {
      Object_radius.clear();
      Object_height.clear();
      Object_position.clear();

      for(unsigned int i=0; i < msg->geometries.size(); i++)
      {
        KDL::Frame frame_obj;
        Object_radius.push_back(msg->geometries[i].info[0]);  //radius
        Object_height.push_back(msg->geometries[i].info[1]);  //height

        tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
        Object_position.push_back(frame_obj); 
      }
      
      DrawCylinder();
  }

  void ball::DrawCylinder()
  {
    for (unsigned int i = 0; i < Object_position.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "vito_anchor";
      marker.header.stamp = ros::Time();
      marker.ns = "";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      
      marker.pose.position.x = Object_position[i].p.x();
      marker.pose.position.y = Object_position[i].p.y();
      marker.pose.position.z = Object_position[i].p.z();
      double x,y,z,w;
      Object_position[i].M.GetQuaternion(x,y,z,w);

      marker.pose.orientation.x = x;
      marker.pose.orientation.y = y;
      marker.pose.orientation.z = z;
      marker.pose.orientation.w = w;

      marker.scale.x = Object_radius[i];
      marker.scale.y = Object_radius[i];
      marker.scale.z = Object_height[i];

      marker.color.a = 1.0; // for the clearness
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.lifetime = ros::Duration(100);
      marker_publisher_.publish(marker); 

      std::string obst_name= "obj_" + std::to_string(i);

      tf::Transform tfGeomTRansform;
      geometry_msgs::Pose p;
      tf::poseKDLToMsg (Object_position[i], p );
      tf::poseMsgToTF( p, tfGeomTRansform );
      tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "vito_anchor", obst_name.c_str()) );
 
    
    }
  }

  void ball::DrawSphere( Eigen::Matrix<double,6,1> &pos_ball )
  {
    // std::cout<<"disegno"<<std::endl;
    
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "vito_anchor";
    marker.header.stamp = ros::Time::now();

    marker.type = marker.SPHERE;
    marker.action = marker.ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = pos_ball(0);
    marker.pose.position.y = pos_ball(1);
    marker.pose.position.z = pos_ball(2);
    marker.lifetime = ros::Duration(1000);

    vis_pub.publish(marker);
  }

  Eigen::Matrix<double,6,1> ball::GetFieldRep(Eigen::Matrix<double,6,1> &point_pos)
  {

      std::vector<Eigen::Matrix<double,6,1> > F_rep;
      //repulsive with obj
      for(unsigned int i=0; i < Object_position.size(); i++)
      {  
          double influence = Object_radius[i] + 0.2;
          std::vector<KDL::Vector> pos;
          KDL::Vector ball_pos_kdl(point_pos(0),point_pos(1),point_pos(2));
          pos.push_back(ball_pos_kdl);
          std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex;
          ForceAndIndex = pfc.GetRepulsiveForce(pos, influence, Object_position[i], Object_radius[i], Object_height[i]);
          F_rep.push_back(ForceAndIndex.first);  
      }
    
      Eigen::Matrix<double,6,1> f = Eigen::Matrix<double,6,1>::Zero(); 
        
      for(unsigned int k=0; k < F_rep.size();k++)
      {
          f = f + F_rep[k]/mass;
      }

      return f;
  }

  // KDL::Frame FromEigenToKdl(Eigen::Matrix<double,6,1> &Force)
  // {
  //   KDL::Frame Force_kdl;
  //   Force_kdl.M = KDL::Rotation::Identity();
  //   Force_kdl.p = KDL::Vector::Zero();
  //   Force_kdl.p.data[0] = Force(0);
  //   Force_kdl.p.data[1] = Force(1);
  //   Force_kdl.p.data[2] = Force(2);

  //   return Force_kdl;
  // }

  // void ball::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
  // {
  //     if(msg->data.size() == 2*joint_handles_.size())
  //     {
  //       for(unsigned int i = 0; i < joint_handles_.size(); i++)
  //       {
  //         Kp_(i) = msg->data[i];
  //         Kd_(i) = msg->data[i + joint_handles_.size()];
  //       }
  //     }
  //     else
  //       ROS_INFO("Number of Joint handles = %lu", joint_handles_.size());

  //     ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());


  // }

