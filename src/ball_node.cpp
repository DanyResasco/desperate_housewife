#include <test_ball.h>

#define  treshold_influence  0.20


ball::ball()
{
  //ball
  sub_grid_ = nh.subscribe("SphereInfo", 1, &ball::ballInfo, this);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
  marker_publisher_ = nh.advertise<visualization_msgs::Marker>( "visualization_cylinder", 1 );
  nh.param<std::string>("topic_obstacle" , obstacle_avoidance, "obstacles");
  obstacles_subscribe_ = nh.subscribe(obstacle_avoidance.c_str(), 1, &ball::InfoGeometry, this);
  ROS_INFO_STREAM("Taking environment from topic: " << obstacle_avoidance);
  Force_attractive = Eigen::Matrix<double,6,1>::Zero();
  F_repulsive = Eigen::Matrix<double,6,1>::Zero();
  x_dot = Eigen::Matrix<double,6,1>::Zero();
  check_sms = false;
  count = 0;
  pub_Fa_ = nh.advertise<std_msgs::Float64MultiArray>("F_acttrative", 1000);
  pub_Fr_ = nh.advertise<std_msgs::Float64MultiArray>("F_repulsive", 1000);
  pub_Ft_ = nh.advertise<std_msgs::Float64MultiArray>("F_total", 1000);
  pfc.load_parameters(nh);
  pf_parameters = pfc.getParameters();
  F_total = Eigen::Matrix<double, 6,1>::Zero();
  F_total(0) = 1000;
}

void ball::ballInfo(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  // std::cout<<"ricevuto sms info palla"<<std::endl;
  ball_pos = Eigen::Matrix<double,6,1>::Zero();
  pos_des = Eigen::Matrix<double,6,1>::Zero();

  //initial ball position
  ball_pos(0) = msg->data[0];
  ball_pos(1) = msg->data[1];
  ball_pos(2) = msg->data[2];

  // std::cout<<"ball_pos: "<<ball_pos<<std::endl;
  radius = msg->data[3];
  mass = msg->data[4];
  ROS_INFO("mass: %f", mass);
  //desired ball position
  pos_des(0) = msg->data[5];
  pos_des(1) = msg->data[6];
  pos_des(2) = msg->data[7];

  SeeMarker(pos_des, "pose_desired");
  SeeMarker(ball_pos, "pose_init");
  check_sms = true;
  F_total = Eigen::Matrix<double, 6,1>::Zero();
  F_total(0) = 1000;
  // DrawSphere( ball_pos );
}

bool ball::Update(double delta)
{
  // return false;
  //if ball is not arrived, update the position and velocity
  // std::cout<<"dentro update"<<std::endl;
  if( (pos_des - ball_pos).isMuchSmallerThan(0.05) || (F_total.norm() < .05) )
    {
      // std::cout<<"pos_des: "<<pos_des(0)<<'\t'<<pos_des(1)<<'\t'<<pos_des(2)<<std::endl;
      // std::cout<<"ball_pos: "<<ball_pos(0)<<'\t'<<ball_pos(1)<<'\t'<<ball_pos(2)<<std::endl;
      ROS_INFO("Ball arrived or in local minima");
      // check_sms = false;
      return true;

    }
  else
    {
      // std::cout<<"ball_pos: "<<ball_pos(0)<<'\t'<<ball_pos(1)<<'\t'<<ball_pos(2)<<std::endl;
      // std::cout<<"dentro else"<<std::endl;
      std_msgs::Float64MultiArray Fa_msg;
      std_msgs::Float64MultiArray Fr_msg;
      std_msgs::Float64MultiArray Ft_msg;

      F_repulsive = GetFieldRep(ball_pos);
      // std::cout<<"F_repulsive: "<<F_repulsive<<std::endl;
      //publish repulsive force
      for(int i = 0; i < F_repulsive.size(); i++)
        {
          Fr_msg.data.push_back(F_repulsive(i));
        }
      pub_Fr_.publish(Fr_msg);

      double Kp,Kd;
      Kp = 200;
      Kd = 100;

      x_err_ = pos_des - ball_pos;

      for(int i = 0; i < Force_attractive.size(); i++)
        {
          Force_attractive(i) =  (-Kd * (x_dot(i)) + Kp * x_err_(i)) / mass;
          Fa_msg.data.push_back(Force_attractive(i));
        }

      pub_Fa_.publish(Fa_msg);

      F_total = Force_attractive + F_repulsive;

      for(int i = 0; i < F_repulsive.size(); i++)
        {
          Ft_msg.data.push_back(F_total(i));
        }
      pub_Ft_.publish(Ft_msg);
      //   std::cout<<"delta: "<<delta<<std::endl;
      double delta = 0.01;
      GetVelocityAndPosition(F_total,delta); // Force is just divide by mass
      
      SeeMarker(pos_des, "pose_desired");
      SeeMarker(ball_pos, "pose_init");


      return false;
    }

}



void ball::GetVelocityAndPosition(Eigen::Matrix<double,6,1> &Force,  double delta)
{
  Eigen::Matrix<double,6,1> velocity_last = x_dot;
  Eigen::Matrix<double,6,1> pos_last = ball_pos;

  x_dot = velocity_last + delta * Force;
  // std::cout<<"x_dot: "<<x_dot(0)<<'\t'<<x_dot(1)<<'\t'<<x_dot(2)<<std::endl;
  ball_pos = 0.5 * Force * delta * delta + velocity_last * delta + pos_last;

  DrawSphere( ball_pos );
  count = count + 1;
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
  tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "world", obst_name.c_str()) );
}


void ball::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  ROS_INFO("ricevuto sms info obj");
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
  ROS_INFO("Working with %ld cylinders", msg->geometries.size());


}

void ball::DrawCylinder()
{
  for (unsigned int i = 0; i < Object_position.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
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
      tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "world", obst_name.c_str()) );


    }
}

void ball::DrawSphere( Eigen::Matrix<double,6,1> &pos_ball )
{
  // std::cout<<"disegno"<<std::endl;

  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.id = count;
  marker.type = marker.SPHERE;
  marker.action = marker.ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1;
  marker.color.r = 0.1;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = pos_ball(0);
  marker.pose.position.y = pos_ball(1);
  marker.pose.position.z = pos_ball(2);
  marker.lifetime = ros::Duration(1000);

  vis_pub.publish(marker);
}

Eigen::Matrix<double,6,1> ball::GetFieldRep(Eigen::Matrix<double,6,1> &point_pos)
{

  Eigen::Matrix<double,6,1> ForceAndIndex = Eigen::Matrix<double,6,1>::Zero();
  //repulsive with obj
  for(unsigned int i=0; i < Object_position.size(); i++)
    {
      std::vector<KDL::Vector> pos;
      KDL::Vector ball_pos_kdl;
      ball_pos_kdl.data[0] = point_pos(0);
      ball_pos_kdl.data[1] = point_pos(1);
      ball_pos_kdl.data[2] = point_pos(2);
      pos.push_back(ball_pos_kdl);


      KDL::Frame temp_frame;
      temp_frame.p.data[0] = point_pos(0);
      temp_frame.p.data[1] = point_pos(1);
      temp_frame.p.data[2] = point_pos(2);

      ForceAndIndex += getRepulsiveForceObjects(temp_frame, pf_parameters.pf_dist_to_obstacles, Object_position[i], Object_radius[i], Object_height[i] );
      ROS_INFO_STREAM("Force" << ForceAndIndex.transpose());

    }

  Eigen::Matrix<double,6,1> f = Eigen::Matrix<double,6,1>::Zero();

  f += ForceAndIndex/mass;

  return f;
}

