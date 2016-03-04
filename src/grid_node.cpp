#include <grid.h>

grid::grid()
{
  //grid
  sub_grid_ = nh.subscribe("gridspace", 1, &grid::gridspace, this);
  // vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
  vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 1 );

  std::string control_topic_right;
  nh.param<std::string>("/right_arm/controller", control_topic_right, "PotentialFieldControl");
  obstacle_avoidance = "/right_arm/" + control_topic_right + "/obstacles";
  // nh.param<std::string>("topic_obstacle" , obstacle_avoidance, "obstacles");

  obstacles_subscribe_ = nh.subscribe(obstacle_avoidance.c_str(), 1, &grid::InfoGeometry, this);
  pfc.load_parameters(nh);

}

void grid::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  Object_radius.clear();
  Object_height.clear();
  Object_position.clear();

  // std::cout<<"msg->geometries.size(): "<<msg->geometries.size()<<std::endl;
  //get info for calculates objects surface
  for(unsigned int i=0; i < msg->geometries.size(); i++)
    {
      KDL::Frame frame_obj;
      Object_radius.push_back(msg->geometries[i].info[0]);  //radius
      Object_height.push_back(msg->geometries[i].info[1]);  //height

      tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
      Object_position.push_back(frame_obj);
    }
  ROS_INFO("Considering %ld Obstacles", msg->geometries.size());
}



void grid::gridspace(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  std::cout<<"sms ricevuto"<<std::endl;
  int count = 0;
  vect_pos.data[0] = 0;
  vect_pos.data[1] = 0;
  vect_pos.data[2] = 0;

  Force_norm.clear();
  Force.clear();
  Total_point.clear();
  vect_marker.markers.clear();

  for( double i = msg->data[0]; i <= msg->data[1]; i = i + msg->data[2] )
    {
      vect_pos.data[0] = i;
      for( double j = msg->data[3]; j <= msg->data[4]; j = j + msg->data[5] )
        {
          vect_pos.data[1] = j;
          for( double k = msg->data[6]; k <= msg->data[7]; k = k + msg->data[8] )
            {
              vect_pos.data[2] = k;
              GetForceAndDraw(vect_pos, count);
              count = count + 1 ;
            }

        }
    }
  std::pair<double,double> MinAndMAx;

  // ROS_INFO("num points tried: %d", count);
  if (Force.size()>0)
    {
      // ROS_INFO("Force_norm.size(): %ld", Force_norm.size());
      MinAndMAx = GetMinAndMax(Force_norm);
    }
  // ROS_INFO("Force.size(): %ld", Force.size());
  for(unsigned int i=0; i< Force.size(); i++)
    {
      DrawArrow( Force[i], Total_point[i], i, MinAndMAx.first, MinAndMAx.second  );
    }

  if (vect_marker.markers.size() > 0)
    {
      // ROS_INFO("vect_marker.size(): %ld", vect_marker.markers.size());
      vis_pub.publish( vect_marker );
    }
  ros::spinOnce();
  ROS_INFO("finito");
}

void grid::DrawArrow( KDL::Vector &gridspace_Force, KDL::Vector &gridspace_point, int K, double Fmin, double Fmax )
{
  // std::cout<<"disegno"<<std::endl;

  int32_t shape = visualization_msgs::Marker::ARROW;
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = K;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = gridspace_point.x();
  marker.pose.position.y = gridspace_point.y();
  marker.pose.position.z = gridspace_point.z();

  Eigen::Quaterniond quat;
  quat =  RotationMarker(gridspace_Force, gridspace_point);

  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  marker.scale.x = Scale(Fmin,Fmax, gridspace_Force.Norm());
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  // ROS_INFO("marker %d, r = %f", K, ((gridspace_Force.Norm()-Fmin)/(Fmax-Fmin)) );
  marker.color.r = ((gridspace_Force.Norm()-Fmin)/(Fmax-Fmin));
  marker.color.g = 1 - marker.color.r ;
  marker.color.b = 0.0f;
  marker.color.a = 1;
  marker.lifetime = ros::Duration(100);

  vect_marker.markers.push_back(marker);
}

double Scale(double Fmin, double Fmax, double Force_field_norm )
{
  //y=mx+q with P1=(F_min,0.01), P2=(F_max,1)
  double m = ((1-0.01)/(Fmax-Fmin));
  double y = ((Force_field_norm - Fmin)*m)+0.01;
  
  return y;

}
std::pair<double,double> grid::GetMinAndMax(std::vector<double> &field)
{
  std::pair<double,double> MinAndMAx;

  MinAndMAx.first = *std::min_element(std::begin(field), std::end(field));
  MinAndMAx.second = *std::max_element(std::begin(field), std::end(field));

  ROS_INFO("MinAndMAx.first : %f",MinAndMAx.first);
  ROS_INFO("MinAndMAx.second : %f",MinAndMAx.second);

  return MinAndMAx;
}


Eigen::Quaterniond grid::RotationMarker(KDL::Vector &ris_Force, KDL::Vector &point)
{

  Eigen::Vector3d  x(1,0,0);
  Eigen::Vector3d Force_eigen(ris_Force.x(),ris_Force.y(),ris_Force.z());
  double pi = 3.14159264;
  double angle = std::acos( (x.dot(Force_eigen))/(x.norm()*Force_eigen.norm()));
  // std::cout<<"angle: "<<angle<<std::endl;
  Eigen::Matrix3d transformation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d axis(0,0,0);
  axis = (x.cross(Force_eigen)) / (x.cross(Force_eigen)).norm();
  transformation_ = Eigen::AngleAxisd(angle, axis);

  Eigen::Quaterniond quat_eigen_hand(transformation_);
  
  return quat_eigen_hand.normalized();

}

void grid::GetForceAndDraw(KDL::Vector &point_pos, int num)
{
  // Eigen::Matrix<double,6,1>  F_rep = Eigen::Matrix<double,6,1>::Zero();

  Eigen::Matrix<double,6,1> ForceAndIndex_obstacles= Eigen::Matrix<double,6,1>::Zero();
  for(unsigned int i=0; i < Object_position.size(); i++)
  {
      double influence = Object_radius[i] + 0.2;
      std::vector<KDL::Vector> pos;
      pos.push_back(point_pos);
      KDL::Frame temp_frame = KDL::Frame::Identity();
      temp_frame.p.data[0] = point_pos.x();
      temp_frame.p.data[1] = point_pos.y();
      temp_frame.p.data[2] = point_pos.z();

      // double distance = std::sqrt(std::pow(-Object_position[i].p.x() + temp_frame.p.x(),2)+ std::pow(-Object_position[i].p.y() + temp_frame.p.y(),2), std::pow(-Object_position[i].p.z() + temp_frame.p.z(),2));
      double distance = (- Object_position[i].p + temp_frame.p).Norm();
      if(distance <= influence)
      {
        // ForceAndIndex_obstacles+= pfc.GetRepulsiveForce(temp_frame, influence, Object_position[i], Object_radius[i], Object_height[i] );
        Eigen::Vector3d distance_der_partial = pfc.GetPartialDerivate(Object_position[i], temp_frame.p, Object_radius[i],  Object_height[i]);
        ForceAndIndex_obstacles += pfc.GetFIRAS(distance, distance_der_partial, influence);
      }
  }

  //repulsive with table
  Eigen::Matrix<double,6,1> ForceAndIndex_table = Eigen::Matrix<double,6,1>::Zero();
  KDL::Vector Table_position(0,0,0.0);

  double distance_ = std::abs( -Table_position.z() + point_pos.z() ); //considered only the z position
  double Influence_table = 0.15;
  
  if(distance_ <= Influence_table )
  {
      Eigen::Vector3d distance_der_partial(0,0,1);
      ForceAndIndex_table = pfc.GetFIRAS( distance_, distance_der_partial, Influence_table );
  }

  Eigen::Matrix<double,6,1> Force_tot_grid;
  Force_tot_grid = ForceAndIndex_obstacles + .1*ForceAndIndex_table;

  KDL::Vector force_vect(Force_tot_grid(0), Force_tot_grid(1),Force_tot_grid(2));
  KDL::Vector null(0,0,0);

  if(!Equal(force_vect,null,0.05))
    {
      Force_norm.push_back(force_vect.Norm());
      Force.push_back(force_vect);
      Total_point.push_back(point_pos);
    }
}
