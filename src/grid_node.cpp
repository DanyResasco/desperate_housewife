#include <grid.h>

grid::grid()
{
    //grid   
    sub_grid_ = nh.subscribe("gridspace", 1, &grid::gridspace, this);
    // vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 1 );
    nh.param<std::string>("PotentialFieldControl/obstacle_list" , obstacle_avoidance, "/right_arm/PotentialFieldControl/obstacle_pose_right");
    obstacles_subscribe_ = nh.subscribe(obstacle_avoidance.c_str(), 1, &grid::InfoGeometry, this);
  
}

void grid::InfoGeometry(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
      Object_radius.clear();
      Object_height.clear();
      Object_position.clear();
   
      std::cout<<"msg->geometries.size(): "<<msg->geometries.size()<<std::endl;
      //get info for calculates objects surface
      for(unsigned int i=0; i < msg->geometries.size(); i++)
      {
        KDL::Frame frame_obj;
        Object_radius.push_back(msg->geometries[i].info[0]);  //radius
        Object_height.push_back(msg->geometries[i].info[1]);  //height

        tf::poseMsgToKDL(msg->geometries[i].pose, frame_obj);
        Object_position.push_back(frame_obj); 
      }

}



void grid::gridspace(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    std::cout<<"sms ricevuto"<<std::endl;
    int count = 0;
      
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
    MinAndMAx = GetMinAndMax(Force_norm);
    for(unsigned int i=0; i< Force.size(); i++)
    {

      DrawArrow( Force[i], Total_point[i], i, MinAndMAx.first, MinAndMAx.second  );
    }

    
    vis_pub.publish( vect_marker );
    // std::cout<<"finito"<<std::endl;
}

void grid::DrawArrow( KDL::Vector &gridspace_Force, KDL::Vector &gridspace_point, int K, double Fmin, double Fmax )
{
    // std::cout<<"disegno"<<std::endl;

    int32_t shape = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "vito_anchor";
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

    marker.color.r = ((gridspace_Force.Norm()-Fmin)/(Fmax-Fmin));
    marker.color.g = 1 - marker.color.r ;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
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
      // std::vector<double>::iterator result = std::min_element(std::begin(field), std::end(field));
      // int index_min = std::distance(std::begin(field), result);

      // std::vector<double>::iterator max = std::max_element(std::begin(field), std::end(field));
      // int index_max = std::distance(std::begin(field), max); 

      MinAndMAx.first = *std::min_element(std::begin(field), std::end(field));
      MinAndMAx.second = *std::max_element(std::begin(field), std::end(field));

      std::cout<<"MinAndMAx.first : "<<MinAndMAx.first <<std::endl;
      std::cout<<"MinAndMAx.second : "<<MinAndMAx.second <<std::endl;

      return MinAndMAx;
}


Eigen::Quaterniond grid::RotationMarker(KDL::Vector &ris_Force, KDL::Vector &point)
{
  Eigen::Vector3d  x(1,0,0);
  Eigen::Vector3d Force_eigen(ris_Force.x(),ris_Force.y(),ris_Force.z());
  // double pi = 3.14159264;
  double angle = std::acos( (x.dot(Force_eigen))/(x.norm()*Force_eigen.norm()));
  // std::cout<<"angle: "<<angle<<std::endl;
  Eigen::Matrix3d transformation_ = Eigen::Matrix3d::Identity();
  // Eigen::Quaterniond quat_eigen_hand(transformation_);
  // if(angle != 0)
  // {
    Eigen::Vector3d axis(0,0,0);
    axis = (x.cross(Force_eigen)) / (x.cross(Force_eigen)).norm();
    // std::cout<<"axis: "<<axis[0]<<'\t'<<axis[1]<<'\t'<<axis[2]<<std::endl;
    // Eigen::Matrix3d transformation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d row0(axis[0]*axis[0]*(1-cos(angle))+cos(angle), axis[0]*axis[1]*(1-cos(angle))-axis[2]*sin(angle), axis[0]*axis[2]*(1-cos(angle))+axis[1]*sin(angle));
    Eigen::Vector3d row1(axis[0]*axis[1]*(1-cos(angle))+axis[2]*sin(angle), axis[1]*axis[1]*(1-cos(angle))+cos(angle), axis[1]*axis[2]*(1-cos(angle))-axis[0]*sin(angle));
    Eigen::Vector3d row2(axis[0]*axis[2]*(1-cos(angle))-axis[1]*sin(angle), axis[1]*axis[2]*(1-cos(angle))+axis[0]*sin(angle),axis[2]*axis[2]*(1-cos(angle))+cos(angle));
    
    transformation_.row(0) << row0.transpose();
    transformation_.row(1) << row1.transpose();
    transformation_.row(2) << row2.transpose();
    Eigen::Quaterniond quat_eigen_hand(transformation_);
    // std::cout<<"quat_eigen_hand: "<<quat_eigen_hand.x()<<'\t'<<quat_eigen_hand.y()<<'\t'<<quat_eigen_hand.z()<<'\t'<<quat_eigen_hand.w()<<std::endl;
  // }
  
 return quat_eigen_hand.normalized();

}

void grid::GetForceAndDraw(KDL::Vector &point_pos, int num)
{
      std::vector<Eigen::Matrix<double,6,1> > F_rep;
      // std::cout<<"point: "<<point_pos[0]<<'\t'<<point_pos[1]<<'\t'<<point_pos[2]<<std::endl;
      //repulsive with obj
      for(unsigned int i=0; i < Object_position.size(); i++)
      {  
          double influence = Object_radius[i] + 0.2;
          std::vector<KDL::Vector> pos;
          pos.push_back(point_pos);
          std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex;
          ForceAndIndex = pfc.GetRepulsiveForce(pos, influence, Object_position[i], Object_radius[i], Object_height[i]);
          F_rep.push_back(ForceAndIndex.first);  
      }
  
      Eigen::Matrix<double,6,1> f = Eigen::Matrix<double,6,1>::Zero(); 
      
      for(unsigned int k=0; k < F_rep.size();k++)
      {
        f = f + F_rep[k];
      }
    
      //repulsive with table
      std::pair<Eigen::Matrix<double,6,1>, double> ForceAndIndex_table;
      KDL::Vector Table_position(0,0,0.15);  
      std::vector<double> dist;
      dist.push_back(- Table_position.z() + point_pos.z() );

      ForceAndIndex_table = pfc.RepulsiveWithTable(dist);
      
      Eigen::Matrix<double,6,1> Force_tot_grid;
      Force_tot_grid = f + ForceAndIndex_table.first;
      // std::cout<<"ForceAndIndex_table.first: "<<ForceAndIndex_table.first<<std::endl;
      // std::cout<<"f: "<<f<<std::endl;


      // std::cout<<"Force: "<< Force_tot_grid(0) <<'\t'<<Force_tot_grid(1)<<'\t'<<Force_tot_grid(2)<<std::endl;
      KDL::Vector force_vect(Force_tot_grid(0), Force_tot_grid(1),Force_tot_grid(2));
      KDL::Vector null(0,0,0);

      if(!Equal(force_vect,null,0.05))
      {
        Force_norm.push_back(force_vect.Norm());
        Force.push_back(force_vect);
        Total_point.push_back(point_pos);
      }
}