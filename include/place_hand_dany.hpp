#include <HandPoseGenerator_state.h>
#include <visualization_msgs/Marker.h>

/*** Code used to decide the desired pose. 
*/

geometry_msgs::Pose HandPoseGenerator::placeHand ( desperate_housewife::fittedGeometriesSingle geometry, int whichArm )
{

  Eigen::Matrix4d M_desired_local; /*in cyl frame*/
  Eigen::Vector4d Point_desired, Pos_ori_hand; /*in cyl frame*/
  Eigen::Vector4d translation; /*in cyl frame*/
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


  double  radius = geometry.info[0];
  double  height = geometry.info[1];
  double  isLying = geometry.info[2];
  double  isFull = geometry.info[3];
  ROS_DEBUG ("radius = %g", radius);
  ROS_DEBUG ("height = %g", height);
  ROS_DEBUG ("isLying = %g", isLying);
  ROS_DEBUG ("isFull = %g", isFull);

  
  Eigen::Vector3d projection(retta_hand_obj[0], retta_hand_obj[1],0);

  Eigen::Vector3d Liyng_projection(retta_hand_obj[0], 0, retta_hand_obj[2]);
  
  Eigen::Vector3d y_l = projection.normalized();
  Eigen::Vector3d z_l = -z ;
  Eigen::Vector3d x_l = y_l.cross(z_l);

  M_desired_local = Eigen::Matrix4d::Identity();
  M_desired_local.col(0) << x_l, 0; 
  M_desired_local.col(1) << y_l, 0; /*y_porojection*/
  M_desired_local.col(2) << z_l, 0; /*z_cylinder*/

  /*takes the object on the top moved on the board*/
  if((isLying == 0) && (isFull == 0)) 
    {
      Point_desired(0) = 0;
      Point_desired(1) = 0;
      Point_desired(2) = height * 0.5 + 0.03; //0.05;	
      Point_desired(3) = 1;
      ROS_DEBUG("cyl upright and empty");
      
      // if (whichArm == 1) left arm
      // {
        Point_desired(1) =  radius;  /*along the y axis*/
      //   M_desired_local.col(3) << Point_desired;
      //   T_w_h = T_vito_c * M_desired_local* Rot_z;;
      //   std::cout<<"Not Lying, Empty, left Arm, "<< std::endl;
      // }

      // else
      // {
      //   Point_desired(1) = - 2*radius;  /*along the y axis*/
        M_desired_local.col(3) << Point_desired;

        T_w_h = T_vito_c * M_desired_local* Rot_z;
        ROS_INFO("Not Lying, Empty, right Arm, ");
      // }   

    }

  /*takes the object on the top if the object's radius is minus than treshold*/
  else if(((isLying == 0) && (isFull != 0)) && (radius< max_radius))
    {
      Point_desired(0) = 0;
      Point_desired(1) = 0;
      Point_desired(2) = height *0.5 + 0.03;  //0.05; 
      Point_desired(3) = 1;
      ROS_DEBUG("cyl upright and full");
      
      M_desired_local.col(3) << Point_desired;

      // if (whichArm == 1) //left arm
      // {
      //    T_w_h = T_vito_c * M_desired_local* Rot_z;
      //    std::cout<<"Not Lying, full, left Arm"<< std::endl;
      // }

      // else
      // {
      //   T_w_h = T_vito_c * M_desired_local* Rot_z;
      //   std::cout<<"Not Lying, full, rigth Arm"<< std::endl;
      // }
       T_w_h = T_vito_c * M_desired_local* Rot_z;
       ROS_INFO("Not Lying, full");
       // std::cout<<"Not Lying, full"<< std::endl;
     
    }

  /*if cilynder is lying, the pose is calculates in vito frame and we take it on the middle*/
  else if ((isLying != 0) && (radius < max_radius))
    {
      Point_desired(0) = T_vito_c(0,3);
      Point_desired(1) = T_vito_c(1,3);
      Point_desired(2) = T_vito_c(2,3)+ radius + 0.05; 
      Point_desired(3) = 1;

      ROS_DEBUG("cyl is lying");

      if(whichArm == 1) /*left to check*/
      {
          z_l = -z;
          x_l << T_vito_c(0,2), T_vito_c(1,2), T_vito_c(2,2) ;
          y_l = z_l.cross(x_l);

          T_w_h.col(0) << x_l,0;
          T_w_h.col(1) << y_l,0;
          T_w_h.col(2) << z_l,0;
          T_w_h.col(3) << Point_desired;
          std::cout<<"Lying, left Arm"<< std::endl;
     }

      else /*right*/
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
      }
    }

  else
    {
      ROS_DEBUG("case not covered");
    }

  ROS_DEBUG("Set hand final position");
  geometry_msgs::Pose local_sh_pose;
  fromEigenToPose( T_w_h ,local_sh_pose);

  tf::Transform tfHandTrasform;
  tf::poseMsgToTF( local_sh_pose, tfHandTrasform);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(), right_hand_frame_ .c_str()) );

  return local_sh_pose;
}

void HandPoseGenerator::fromEigenToPose(Eigen::Matrix4d &tranfs_matrix, geometry_msgs::Pose &Hand_pose)
{
  Eigen::Matrix<double,3,3> Tmatrix;
  Tmatrix = tranfs_matrix.block<3,3>(0,0) ;
  Eigen::Quaterniond quat_eigen_hand(Tmatrix);

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


int HandPoseGenerator::whichArm( geometry_msgs::Pose object_pose, int cyl_nbr )
{
  // ROS_INFO("inizio mano");
  /*We use vito frame for chose which arm use, while we use cylinder frame for calculates the straight line between hand frame and objects frame.*/
  int return_value;
	tf::StampedTransform hand_left, hand_rigth, hand_r_object,hand_l_object;

	// listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	// listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

	listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

  /*Get information about the distance between the cylinder and soft-hand*/
  listener_info.waitForTransform("object_"+ std::to_string(cyl_nbr), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
  listener_info.lookupTransform("object_"+std::to_string(cyl_nbr), right_hand_frame_.c_str(), ros::Time(0), hand_r_object);

  // listener_info.waitForTransform("object_"+std::to_string(cyl_nbr), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
  // listener_info.lookupTransform("object_"+std::to_string(cyl_nbr), left_hand_frame_.c_str(), ros::Time(0), hand_l_object);

	Eigen::Vector3d object_position(object_pose.position.x, object_pose.position.y, object_pose.position.z);
	// Eigen::Vector3d hand_left_position(hand_left.getOrigin().x(),hand_left.getOrigin().y(),hand_left.getOrigin().z());
	Eigen::Vector3d hand_right_position(hand_rigth.getOrigin().x(),hand_rigth.getOrigin().y(),hand_rigth.getOrigin().z());

  /*distance from each arm to cylinder. it use for decide wich arm to use*/
	// double dist_to_left_hand = std::sqrt((object_position[0] - hand_left_position[0]) * (object_position[0] - hand_left_position[0]) +
	// 			   (object_position[1] - hand_left_position[1]) * (object_position[1] - hand_left_position[1]) +
	// 			   (object_position[2] - hand_left_position[2]) * (object_position[2] - hand_left_position[2]) );

	double dist_to_right_hand = std::sqrt((object_position[0] - hand_right_position[0]) * (object_position[0] - hand_right_position[0]) +
				   (object_position[1] - hand_right_position[1]) * (object_position[1] - hand_right_position[1]) +
				   (object_position[2] - hand_right_position[2]) * (object_position[2] - hand_right_position[2]) );

  /*the straight line is calculates in cilynder frame*/
	// if(dist_to_left_hand < dist_to_right_hand)
	// {
	//   // ROS_INFO("Vito uses a: left arm because of distance");
 //    retta_hand_obj[0] =  hand_l_object.getOrigin().x();
 //    retta_hand_obj[1] =  hand_l_object.getOrigin().y();
 //    retta_hand_obj[2] =  hand_l_object.getOrigin().z();

 //    return_value = 1;
	// }
 //  else
 //  {
	//   // ROS_INFO("Vito uses a: Right arm because of distance");
    retta_hand_obj[0] =  hand_r_object.getOrigin().x();
    retta_hand_obj[1] =  hand_r_object.getOrigin().y();
    retta_hand_obj[2] =  hand_r_object.getOrigin().z();

  	return_value = 0;
  // }
  // ROS_INFO("finito mano");
  return return_value; 
}



/*Function for clear the table when the only object is obstacle*/
geometry_msgs::Pose HandPoseGenerator::ObstacleReject( desperate_housewife::fittedGeometriesSingle Pose_rej_obs, int arm_)
{
  Eigen::Matrix4d T_vito_c, T_w_ob;
  Eigen::Matrix4d M_desired_local; /* in cyl frame*/
  Eigen::Vector4d Point_desired;
  geometry_msgs::Pose pose_obj_hand;
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0), z(0,0,1);

  /*sends msgs to close hand*/
  trajectory_msgs::JointTrajectory msg_jointT_hand;
  msg_jointT_hand.joint_names.resize(1);
  msg_jointT_hand.points.resize(1);
  msg_jointT_hand.points[0].positions.resize(1);
  msg_jointT_hand.points[0].positions[0] = 1.0;
  msg_jointT_hand.points[0].time_from_start = ros::Duration(1); // 1s;

  T_vito_c = FromMsgtoEigen( Pose_rej_obs.pose );

  /*to remove object we decide to put with the back of the hand in the middle of the obstacles */
  Point_desired(0) = T_vito_c(0,3) + Pose_rej_obs.info[0] + 0.10;
  Point_desired(1) = T_vito_c(1,3) ;
  Point_desired(2) = T_vito_c(2,3) - 0.02; 
  Point_desired(3) = 1;

  M_desired_local = Eigen::Matrix4d::Identity();

  if(arm_ == 1) /*left*/
  { 
      T_w_ob.col(0) << -z, 0;  
      T_w_ob.col(1) << y, 0;
      T_w_ob.col(2) << y.cross(z) , 0;

      msg_jointT_hand.joint_names[0] = "left_hand_synergy_joint";
      hand_publisher_left.publish(msg_jointT_hand);
  }

  else  /*right*/
  {
    T_w_ob.col(0) << z, 0; 
    T_w_ob.col(1) << -y,0;
    T_w_ob.col(2) << y.cross(z), 0;

    msg_jointT_hand.joint_names[0] = "right_hand_synergy_joint";
    hand_publisher_right.publish(msg_jointT_hand);
  }

  T_w_ob.col(3) << Point_desired;  

  fromEigenToPose (T_w_ob, pose_obj_hand);

  ROS_DEBUG("obstacle to move");
      
  tf::Transform tfHandTrasform;
  tf::poseMsgToTF( pose_obj_hand, tfHandTrasform);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"ObstacleReject") );

  return pose_obj_hand;
}

void HandPoseGenerator::Overturn()
{
  /* std::cout<<"overturn"<<std::endl;*/
  Eigen::Matrix3d rot_left;
  Eigen::Matrix3d rot_right;

  desperate_housewife::handPoseSingle DesiredHandPose_left;
  desperate_housewife::handPoseSingle DesiredHandPose_right;
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0), z(0,0,1);

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
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"left_ribalto") );  

  DesiredHandPose_right.pose.position.x = -1.0;
  DesiredHandPose_right.pose.position.y = 0.5;
  DesiredHandPose_right.pose.position.z = 0.15;

  tf::Transform tfHandTrasform2;
  tf::poseMsgToTF( DesiredHandPose_right.pose, tfHandTrasform2);
  tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str()," right_ribalto") ); 
  // ROS_INFO("BUTTO TUTTO");
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

