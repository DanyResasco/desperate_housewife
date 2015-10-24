#include <hand_pose_generator.h>

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
  double max_radius=0.1;

  Eigen::Affine3d mtemp;
 
  T_vito_c = FromMsgtoEigen( geometry.pose );
 
  Eigen::Matrix4d Rot_z;
  Rot_z.row(0)<< -1,0,0,0;
  Rot_z.row(1)<< 0,-1,0,0;
  Rot_z.row(2)<< 0,0,1,0;
  Rot_z.row(3)<< 0,0,0,1;



  double  radius = geometry.info[0];
  double  height = geometry.info[1];
  double  isLying = geometry.info[2];
  double  isFull = geometry.info[3];
  ROS_DEBUG ("radius = %g", radius);
  ROS_DEBUG ("height = %g", height);
  ROS_DEBUG ("isLying = %g", isLying);
  ROS_DEBUG ("isFull = %g", isFull);

  
  Eigen::Vector3d projection(retta_hand_obj.position.x, retta_hand_obj.position.y,0);
  projection.normalize();
  //double y_projection = projection.norm();
  // std::cout<<"projectionNorm: "<<projection<<std::endl;

  M_desired_local.col(0) << z.cross(projection), 0; 
  M_desired_local.col(1) << projection,0; //y_porojection
  M_desired_local.col(2) << -z , 0; //z_cylinder


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
        T_w_h = T_vito_c * M_desired_local*Rot_z;
      }
      else
      {
        Point_desired(0) = radius;
        M_desired_local.col(3) << Point_desired;
        T_w_h = T_vito_c * M_desired_local;
      }

      
    }

  else if(((isLying == 0) && (isFull != 0)) && (radius< max_radius))
    {
      Point_desired(0) = 0;
      Point_desired(1) = 0;
      Point_desired(2) = height *0.5+ 0.05; 
      Point_desired(3) = 1;
      ROS_DEBUG("cyl upright and full");
      
      M_desired_local.col(3) << Point_desired;
      if (whichArm == 1) //left arm
      {
         T_w_h = T_vito_c * M_desired_local*Rot_z;
       }
      else
      {
         T_w_h = T_vito_c * M_desired_local;
      }
   
    }

  else if ((isLying != 0) && (radius < max_radius))
    {
      if(whichArm == 1) //left
      {
         M_desired_local.col(0) << -z, 0;
         M_desired_local.col(1) << -y.cross(-z), 0;	
      
      }
      else //right
      { 
          M_desired_local.col(0) << z, 0;	
          M_desired_local.col(1) << -y.cross(z), 0;	
      }
      
      Point_desired(0) = 0;
      Point_desired(1) = radius + 0.05;
      Point_desired(2) = 0; 
      Point_desired(3) = 1;
      ROS_DEBUG("cyl is lying");
      
      M_desired_local.col(2) << -y, 0;
      M_desired_local.col(3) << Point_desired;
      T_w_h = T_vito_c * M_desired_local*Rot_z ;
    }
  else
    {
      ROS_DEBUG("case not covered");
    }

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
  Hand_pose.orientation.x = quat_eigen_hand.x();
  Hand_pose.orientation.y = quat_eigen_hand.y();
  Hand_pose.orientation.z = quat_eigen_hand.z();
  Hand_pose.orientation.w = quat_eigen_hand.w();
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

	tf::StampedTransform hand_left, hand_rigth;

	listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

	listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

	Eigen::Vector3d object_position(object_pose.position.x, object_pose.position.y, object_pose.position.z);
	Eigen::Vector3d hand_left_position(hand_left.getOrigin().x(),hand_left.getOrigin().y(),hand_left.getOrigin().z());
	Eigen::Vector3d hand_right_position(hand_rigth.getOrigin().x(),hand_rigth.getOrigin().y(),hand_rigth.getOrigin().z());

	//double dist_to_left_hand = 
  dist_to_left_hand = std::sqrt((object_position[0] - hand_left_position[0]) * (object_position[0] - hand_left_position[0]) +
				   (object_position[1] - hand_left_position[1]) * (object_position[1] - hand_left_position[1]) +
				   (object_position[2] - hand_left_position[2]) * (object_position[2] - hand_left_position[2]) );

	//double dist_to_right_hand 
  dist_to_right_hand = std::sqrt((object_position[0] - hand_right_position[0]) * (object_position[0] - hand_right_position[0]) +
				   (object_position[1] - hand_right_position[1]) * (object_position[1] - hand_right_position[1]) +
				   (object_position[2] - hand_right_position[2]) * (object_position[2] - hand_right_position[2]) );


	// double dist_to_left_hand  = 0;
	// double dist_to_right_hand = 0;

	if(dist_to_left_hand <= dist_to_right_hand)
	{
	 
	  ROS_DEBUG("Vito uses a: left arm");
    retta_hand_obj.position.x = object_position[0] + (object_position[0] - hand_left_position[0]);
    retta_hand_obj.position.y = object_position[1] + (object_position[1] - hand_left_position[1]);
    retta_hand_obj.position.z = object_position[2] + (object_position[2] - hand_left_position[2]);
    return 1;
	}
  else
  {
	 	ROS_DEBUG("Vito uses a: Right arm");
    retta_hand_obj.position.x = object_position[0] + (object_position[0] - hand_right_position[0]);
    retta_hand_obj.position.y = object_position[1] + (object_position[1] - hand_right_position[1]);
    retta_hand_obj.position.z = object_position[2] + (object_position[2] - hand_right_position[2]);
	 return 0;
  }
}


//Function for clear the table when the only object is obstacle
geometry_msgs::Pose HandPoseGenerator::ObstacleReject( desperate_housewife::fittedGeometriesSingle Pose_rej_obs)
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
      
  M_desired_local.col(0) << -y.cross(z), 0; 
  M_desired_local.col(1) << -y,0;
  M_desired_local.col(2) << z , 0;
  
  M_desired_local.col(3) << Point_desired;

  T_w_ob = T_vito_c * M_desired_local*ROT_y*ROT_x;
  // *ROT_x*ROT_y ; 

  fromEigenToPose (T_w_ob, pose_obj_hand);

  // tf::Transform tfHandTrasform;
  // tf::poseMsgToTF( pose_obj_hand, tfHandTrasform);
  
  // tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform, ros::Time::now(), base_frame_.c_str(),"ObstacleReject") );

  // std::cout<<"pose_obj_hand: "<<pose_obj_hand<<std::endl;


  return pose_obj_hand;

}