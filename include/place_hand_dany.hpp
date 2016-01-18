#include <hand_pose_generator.h>
#include <visualization_msgs/Marker.h>

/*** Code used to decide the desired pose. The hand pose is calculated respect the cylinder frame and after converted to base_fram (vito_anchor).
  * 
*/

geometry_msgs::Pose HandPoseGenerator::placeHand ( desperate_housewife::fittedGeometriesSingle geometry, int whichArm )
{

  Eigen::Matrix4d M_desired_local; // in cyl frame
  Eigen::Vector4d Point_desired, Pos_ori_hand; //in cyl frame
  Eigen::Vector4d translation; //in cyl frame
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0), z(0,0,1);
  Eigen::Matrix4d T_K_H;
  tf::StampedTransform T_K_vito_ancor;
  Eigen::Vector4d local;
  Eigen::Matrix4d T_w_h, T_vito_c;
  double max_radius = 0.1;

  Eigen::Affine3d mtemp;
 
  T_vito_c = FromMsgtoEigen( geometry.pose );
 
  Eigen::Matrix4d Rot_z;
  Rot_z.row(0)<< -1,0,0,0;
  Rot_z.row(1)<< 0,-1,0,0;
  Rot_z.row(2)<< 0,0,1,0;
  Rot_z.row(3)<< 0,0,0,1;
  Eigen::Matrix4d Rot_x;
  Rot_x.row(0)<< -1,0,0,0;
  Rot_x.row(1)<< 0,1,0,0;
  Rot_x.row(2)<< 0,0,-1,0;
  Rot_x.row(3)<< 0,0,0,1;


  //Rot_z = Rot_z*Rot_x;


  double  radius = geometry.info[0];
  double  height = geometry.info[1];
  double  isLying = geometry.info[2];
  double  isFull = geometry.info[3];
  ROS_DEBUG ("radius = %g", radius);
  ROS_DEBUG ("height = %g", height);
  ROS_DEBUG ("isLying = %g", isLying);
  ROS_DEBUG ("isFull = %g", isFull);

  // std::cout<<"retta_hand_obj: "<<retta_hand_obj<<std::endl;
  // retta_hand_obj.normalize();
  std::cout<<"retta_hand_obj normalize: "<< retta_hand_obj <<std::endl;
  // retta_hand_obj.normalize();
  Eigen::Vector3d projection(retta_hand_obj[0], retta_hand_obj[1],0);
  // projection.normalize();
  Eigen::Vector3d Liyng_projection(retta_hand_obj[0], 0, retta_hand_obj[2]);
  // Liyng_projection.normalize();
  
  Eigen::Vector3d y_l = projection.normalized();
  Eigen::Vector3d z_l = -z ;
  Eigen::Vector3d x_l = y_l.cross(z_l);

  M_desired_local = Eigen::Matrix4d::Identity();
  M_desired_local.col(0) << x_l, 0; 
  M_desired_local.col(1) << y_l, 0; //y_porojection
  M_desired_local.col(2) << z_l, 0; //z_cylinder

// M_desired_local = M_desired_local;

  // std::cout << "Det M_desired_local : " << M_desired_local.determinant() << std::endl;
  // std::cout<<"M_desired_local: "<<M_desired_local<<std::endl;
  if((isLying == 0) && (isFull == 0)) 
    {
      Point_desired(1) = 0;
      Point_desired(2) = height *0.5 + 0.05;	
      Point_desired(3) = 1;
      ROS_DEBUG("cyl upright and empty");
      
      if (whichArm == 1) //left arm
      {
        Point_desired(0) = -radius;
        M_desired_local.col(3) << Point_desired;
        T_w_h = T_vito_c * M_desired_local;
        std::cout<<"Not Lying, Empty, left Arm, "<< std::endl;
      }
      else
      {
        Point_desired(0) = radius;
        M_desired_local.col(3) << Point_desired;
        T_w_h = T_vito_c * M_desired_local* Rot_z;
        std::cout<<"Not Lying, Empty, right Arm, "<< std::endl;
      }   

    }

  else if(((isLying == 0) && (isFull != 0)) && (radius< max_radius))
    {
      Point_desired(0) = 0;
      Point_desired(1) = 0;
      Point_desired(2) = height *0.5 + 0.05; 
      Point_desired(3) = 1;
      ROS_DEBUG("cyl upright and full");
      
      M_desired_local.col(3) << Point_desired;

      if (whichArm == 1) //left arm
      {
         T_w_h = T_vito_c * M_desired_local;
         std::cout<<"Not Lying, full, left Arm"<< std::endl;
      }

      else
      {
        T_w_h = T_vito_c * M_desired_local* Rot_z;
        std::cout<<"Not Lying, full, rigth Arm"<< std::endl;
      }
     
    }

  //if cilynder is lying, the pose is calculates in vito frame
  else if ((isLying != 0) && (radius < max_radius))
    {
      Point_desired(0) = T_vito_c(0,3);
      Point_desired(1) = T_vito_c(1,3);
      Point_desired(2) = T_vito_c(2,3)+ radius + 0.05; 
      Point_desired(3) = 1;

      ROS_DEBUG("cyl is lying");

      if(whichArm == 1) //left to check
      {
          z_l = -z;
          x_l << T_vito_c(0,2), T_vito_c(1,2), T_vito_c(2,2) ;
          y_l = z_l.cross(x_l);

          T_w_h.col(0) << x_l,0;
          T_w_h.col(1) << y_l,0;
          T_w_h.col(2) << z_l,0;

          T_w_h.col(3) << Point_desired;
          // T_w_h = T_w_h*Rot_z;
         std::cout<<"Lying, left Arm"<< std::endl;
      
      }
      else //right
      { 
          z_l = -z;
          x_l << T_vito_c(0,2), T_vito_c(1,2), T_vito_c(2,2) ;
          y_l = z_l.cross(x_l);

          T_w_h.col(0) << x_l,0;
          T_w_h.col(1) << y_l,0;
          T_w_h.col(2) << z_l,0;

          T_w_h.col(3) << Point_desired;
          T_w_h = T_w_h*Rot_z;
          std::cout<<"Lying, right Arm"<< std::endl;
          // std::cout<<"T_w_h det: "<<T_w_h.determinant()<< std::endl;

      }
      // std::cout<<"M_desired_local: "<<M_desired_local<<std::endl;

    }
  else
    {
      ROS_DEBUG("case not covered");
    }

    // std::cout<<"M_desired_local: "<<M_desired_local<<std::endl;

  ROS_DEBUG("Set hand final position");
  geometry_msgs::Pose local_sh_pose;
  fromEigenToPose( T_w_h ,local_sh_pose);

  return local_sh_pose;
}

void HandPoseGenerator::fromEigenToPose(Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &Hand_pose)
{
  Eigen::Matrix<double,3,3> Tmatrix;
  Tmatrix = tranfs_matrix.block<3,3>(0,0) ;
  Eigen::Quaterniond quat_eigen_hand(Tmatrix);
  // quat_eigen_hand.normalized();
  Hand_pose.orientation.x = (quat_eigen_hand.normalized()).x();
  Hand_pose.orientation.y = (quat_eigen_hand.normalized()).y();
  Hand_pose.orientation.z = (quat_eigen_hand.normalized()).z();
  Hand_pose.orientation.w = (quat_eigen_hand.normalized()).w();
  Hand_pose.position.x = tranfs_matrix(0,3);
  Hand_pose.position.y = tranfs_matrix(1,3);
  Hand_pose.position.z = tranfs_matrix(2,3);

}


Eigen::Matrix4d HandPoseGenerator::FromMsgtoEigen(geometry_msgs::Pose &object)
{	

  Eigen::Quaterniond transf_quad(object.orientation.w, object.orientation.x, object.orientation.y, object.orientation.z);
  transf_quad.normalize();
  Eigen::Vector3d translation(object.position.x, object.position.y, object.position.z);
  Eigen::Matrix3d rotation(transf_quad.toRotationMatrix());

  Eigen::Matrix4d Matrix_transf;



  Matrix_transf.row(0) << rotation.row(0), translation[0];
  Matrix_transf.row(1) << rotation.row(1), translation[1];
  Matrix_transf.row(2) << rotation.row(2), translation[2];
  Matrix_transf.row(3) << 0,0,0,1;

  return Matrix_transf;
}


int HandPoseGenerator::whichArm( geometry_msgs::Pose object_pose )
{
  //We use vito frame for chose which arm use, while we use cylinder frame for calculates the straight line between hand frame and objects frame.
  int return_value;
	tf::StampedTransform hand_left, hand_rigth, hand_r_object,hand_l_object;

	listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

	listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

  //Ancora da vedere come fare la trasformazione della posa della mano al cilindro considerato 
  listener_info.waitForTransform("object_0", right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
  listener_info.lookupTransform("object_0", right_hand_frame_.c_str(), ros::Time(0), hand_r_object);

  listener_info.waitForTransform("object_0", left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
  listener_info.lookupTransform("object_0", left_hand_frame_.c_str(), ros::Time(0), hand_l_object);


	Eigen::Vector3d object_position(object_pose.position.x, object_pose.position.y, object_pose.position.z);
	Eigen::Vector3d hand_left_position(hand_left.getOrigin().x(),hand_left.getOrigin().y(),hand_left.getOrigin().z());
	Eigen::Vector3d hand_right_position(hand_rigth.getOrigin().x(),hand_rigth.getOrigin().y(),hand_rigth.getOrigin().z());

	double dist_to_left_hand = std::sqrt((object_position[0] - hand_left_position[0]) * (object_position[0] - hand_left_position[0]) +
				   (object_position[1] - hand_left_position[1]) * (object_position[1] - hand_left_position[1]) +
				   (object_position[2] - hand_left_position[2]) * (object_position[2] - hand_left_position[2]) );

	double dist_to_right_hand = std::sqrt((object_position[0] - hand_right_position[0]) * (object_position[0] - hand_right_position[0]) +
				   (object_position[1] - hand_right_position[1]) * (object_position[1] - hand_right_position[1]) +
				   (object_position[2] - hand_right_position[2]) * (object_position[2] - hand_right_position[2]) );

  //the straight line is calculates in cilynder frame
	if(dist_to_left_hand < dist_to_right_hand)
	{
	  ROS_INFO("Vito uses a: left arm because of distance");
    retta_hand_obj[0] =  hand_l_object.getOrigin().x();
    retta_hand_obj[1] =  hand_l_object.getOrigin().y();
    retta_hand_obj[2] =  hand_l_object.getOrigin().z();

    return_value = 1;
	}
  else
  {
	  ROS_INFO("Vito uses a: Right arm because of distance");
    retta_hand_obj[0] =  hand_r_object.getOrigin().x();
    retta_hand_obj[1] =  hand_r_object.getOrigin().y();
    retta_hand_obj[2] =  hand_r_object.getOrigin().z();

  	return_value = 0;
  }

  return return_value; 
}

// void HandPoseGenerator::DrawStraingLIne( Eigen::Vector3d &rett_pos )
//   {
//     // std::cout<<"disegno"<<std::endl;
    
//     visualization_msgs::Marker marker;

//     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
//     marker.header.frame_id = "vito_anchor";
//     marker.header.stamp = ros::Time::now();
//     marker.id = 10;
//     marker.type = marker.LINE_STRIP;
//     marker.action = marker.ADD;
//     marker.scale.x = .2;
//     marker.color.a = 1; 
//     marker.color.r = 1.0;
//     marker.color.g = 0.0;
//     marker.color.b = 0.0;
//     marker.pose.orientation.w = 1.0;

//     geometry_msgs::Point p;
//     p.x = rett_pos(0);
//     p.y = rett_pos(1);
//     p.z = rett_pos(2);

//     marker.points.push_back(p);

//     marker.lifetime = ros::Duration(1000);

//     vis_pub.publish(marker);
//     // ros::spinOnce();
//     std::cout << "Publishing line" << std::endl;

//   }








//Function for clear the table when the only object is obstacle
geometry_msgs::Pose HandPoseGenerator::ObstacleReject( desperate_housewife::fittedGeometriesSingle Pose_rej_obs, int arm_)
{
  Eigen::Matrix4d T_vito_c, T_w_ob;
  Eigen::Matrix4d M_desired_local; // in cyl frame
  Eigen::Vector4d Point_desired;
  geometry_msgs::Pose pose_obj_hand;
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0), z(0,0,1);

  Eigen::Matrix4d Rot_z;
  Rot_z.row(0)<< -1,0,0,0;
  Rot_z.row(1)<< 0,-1,0,0;
  Rot_z.row(2)<< 0,0,1,0;
  Rot_z.row(3)<< 0,0,0,1;

  Eigen::Matrix4d ROT_y;
  ROT_y.row(0)<< 0,0,1,0;
  ROT_y.row(1)<< 0,1,0,0;
  ROT_y.row(2)<< -1,0,0,0;
  ROT_y.row(3)<< 0,0,0,1;


  Eigen::Matrix4d ROT_x;
  ROT_x.row(0)<< 1,0,0,0;
  ROT_x.row(1)<< 0,0,-1,0;
  ROT_x.row(2)<< 0,1,0,0;
  ROT_x.row(3)<< 0,0,0,1;

  T_vito_c = FromMsgtoEigen( Pose_rej_obs.pose );

  Point_desired(0) = 0;
  Point_desired(1) = Pose_rej_obs.info[0] + 0.05;
  Point_desired(2) = 0;  
  Point_desired(3) = 1;
  ROS_DEBUG("obstacle to move");
      
  // M_desired_local.col(0) << -y.cross(z), 0; 
  // M_desired_local.col(1) << -y,0;
  // M_desired_local.col(2) << z , 0;
  
  M_desired_local.col(3) << Point_desired;

  if(arm_ == 1) //left
  { 
     M_desired_local.col(0) << -y.cross(z), 0; 
    M_desired_local.col(1) << -y,0;
    M_desired_local.col(2) << z , 0;
     // T_w_ob = T_vito_c * M_desired_local*ROT_y*ROT_x;
  }
  else
  {
    M_desired_local.col(0) << y.cross(z), 0; 
    M_desired_local.col(1) << y,0;
    M_desired_local.col(2) << z, 0;
    // T_w_ob = T_vito_c * M_desired_local*ROT_y*ROT_x;
    // *ROT_y*ROT_x;
  }
  
  T_w_ob = T_vito_c * M_desired_local*ROT_y*ROT_x;
  fromEigenToPose (T_w_ob, pose_obj_hand);

  // tf::Transform tfHandTrasform;
  // tf::poseMsgToTF( pose_obj_hand, tfHandTrasform);
  
  // tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"ObstacleReject") );

  // std::cout<<"pose_obj_hand: "<<pose_obj_hand<<std::endl;


  return pose_obj_hand;

}

void HandPoseGenerator::Overturn()
{
  // std::cout<<"overturn"<<std::endl;
  Eigen::Matrix3d rot_left;
  Eigen::Matrix3d rot_right;

  desperate_housewife::handPoseSingle DesiredHandPose_left;
  desperate_housewife::handPoseSingle DesiredHandPose_right;
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0), z(0,0,1);
  
  Eigen::Matrix4d Rot_z;
  Rot_z.row(0)<< -1,0,0,0;
  Rot_z.row(1)<< 0,-1,0,0;
  Rot_z.row(2)<< 0,0,1,0;
  Rot_z.row(3)<< 0,0,0,1;

  Eigen::Matrix3d ROT_y;
  ROT_y.row(0)<< 0,0,1;
  ROT_y.row(1)<< 0,1,0;
  ROT_y.row(2)<< -1,0,0;
  // ROT_y.row(3)<< 0,0,0,1;

  rot_left.col(0) << x;
  rot_left.col(1) << -z;
  rot_left.col(2) << y;


  rot_right.col(0) << -x;
  rot_right.col(1) << y.cross(x);
  rot_right.col(2) << -y;


  Eigen::Quaterniond quat_eigen_harm_left(rot_left);
  DesiredHandPose_left.pose.orientation.x = quat_eigen_harm_left.x();
  DesiredHandPose_left.pose.orientation.y = quat_eigen_harm_left.y();
  DesiredHandPose_left.pose.orientation.z = quat_eigen_harm_left.z();
  DesiredHandPose_left.pose.orientation.w = quat_eigen_harm_left.w();

  Eigen::Quaterniond quat_eigen_harm_right(rot_right);
  DesiredHandPose_right.pose.orientation.x = (quat_eigen_harm_right.normalized()).x();
  DesiredHandPose_right.pose.orientation.y = (quat_eigen_harm_right.normalized()).y();
  DesiredHandPose_right.pose.orientation.z = (quat_eigen_harm_right.normalized()).z();
  DesiredHandPose_right.pose.orientation.w = (quat_eigen_harm_right.normalized()).w();
  

  DesiredHandPose_left.pose.position.x = -1.0;
  DesiredHandPose_left.pose.position.y = -0.5;
  DesiredHandPose_left.pose.position.z = 0.15;

  tf::Transform tfHandTrasform;
  tf::poseMsgToTF( DesiredHandPose_left.pose, tfHandTrasform);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"left ribalto") );  

  DesiredHandPose_right.pose.position.x = -1.0;
  DesiredHandPose_right.pose.position.y = 0.5;
  DesiredHandPose_right.pose.position.z = 0.15;

  tf::Transform tfHandTrasform2;
  tf::poseMsgToTF( DesiredHandPose_right.pose, tfHandTrasform2);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str()," right ribalto") ); 
  ROS_INFO("BUTTO TUTTO");

  //send
  // desired_hand_publisher_left.publish(DesiredHandPose_left);
  // Obj_info.data = 2; //flag to grasp object in the desperate_mind code
  // objects_info_left_pub.publish(Obj_info);
  // stop = 1; // flag to stop this procedur
  // desired_hand_publisher_right.publish(DesiredHandPose_right);
  // Obj_info.data = 2;
  // objects_info_right_pub.publish(Obj_info);
}