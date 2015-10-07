#include <hand_pose_generator.h>


geometry_msgs::Pose HandPoseGenerator::placeHand ( desperate_housewife::fittedGeometriesSingle geometry, int whichArm )
{

  Eigen::Matrix4d M_desired_local; // in cyl frame
  Eigen::Vector4d Point_desired,Pos_ori_hand; //in cyl frame
  Eigen::Vector4d translation; //in cyl frame
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0), z_d, z(0,0,1);
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
  ROS_INFO ("radius = ", radius);
  ROS_INFO ("height = ", height);
  ROS_INFO ("isLying = ", isLying);
  ROS_INFO ("isFull = ", isFull);

  if((isLying == 0) && (isFull == 0))
    {
      if (whichArm == 1) //left arm
        {
          M_desired_local.col(0) << x, 0;
          M_desired_local.col(1) << -z.cross(x),0;
        }
      else
        {
          M_desired_local.col(0) << -x, 0;	//right arm
          M_desired_local.col(1) << -z.cross(-x),0;
        }
      Point_desired(0) = radius + 0.05;
      Point_desired(1) = 0;
      Point_desired(2) = height*0.5 + 0.05;	//da rivedere!!
      Point_desired(3) = 1;
      ROS_INFO("cyl dritto e vuoto");

      M_desired_local.col(2) << -z , 0;
      M_desired_local.col(3) << Point_desired;
      T_w_h = T_vito_c * M_desired_local*Rot_z ;

    }

  else if(((isLying == 0) && (isFull != 0)) && (radius< max_radius))
    {
      if (whichArm == 1) //left arm
        {
          M_desired_local.col(0) << x, 0;
          M_desired_local.col(1) << -z.cross(x),0;
        }

      else
        {
          M_desired_local.col(0) << -x, 0;	//right arm
          M_desired_local.col(1) << -z.cross(-x),0;
        }
      Point_desired(0) = 0;
      Point_desired(1) = 0;
      Point_desired(2) = height*0.5 + 0.05;; //da rivedere
      Point_desired(3) = 1;
      ROS_INFO("cyl dritto e pieno");
      M_desired_local.col(2) << -z, 0;
      M_desired_local.col(3) << Point_desired;
      T_w_h = T_vito_c * M_desired_local*Rot_z ;
    }

  else if ((isLying != 0) && (radius < max_radius))
    {
      if (whichArm == 1) //left arm
        {
          M_desired_local.col(0) << -z, 0;
          M_desired_local.col(1) << -y.cross(-z), 0;	//da rifare
        }

      else
        {
          M_desired_local.col(0) << z, 0;	//right arm
          M_desired_local.col(1) << -y.cross(z), 0;	//da rifare
        }
      Point_desired(0) = 0;
      Point_desired(1) = radius + 0.05;
      Point_desired(2) = 0; //da rivedere
      Point_desired(3) = 1;
      ROS_INFO("cyl piegato");
      // M_desired_local.col(1) << -y.cross(z), 0;	//da rifare
      M_desired_local.col(2) << -y, 0;
      M_desired_local.col(3) << Point_desired;
      T_w_h = T_vito_c * M_desired_local ;
    }
  else
    {
      ROS_INFO("caso non contemplato");
    }

  ROS_INFO("Set hand final position");
  geometry_msgs::Pose local_sh_pose;
  fromEigenToPose( T_w_h ,local_sh_pose);

    // geometry_msgs::Pose pose_local = geometry.pose;
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

  Eigen::Quaterniond transf_quad(object.orientation.x, object.orientation.y, object.orientation.z, object.orientation.w);
  transf_quad.normalize();
  Eigen::Vector3d translation(object.position.x, object.position.y, object.position.x);
  Eigen::Matrix3d rotation(transf_quad.toRotationMatrix());

  Eigen::Matrix4d Matrix_transf;

  Matrix_transf.row(0) << rotation.row(0), translation[0];
  Matrix_transf.row(1) << rotation.row(1), translation[1];
  Matrix_transf.row(2) << rotation.row(2), translation[2];
  Matrix_transf.row(3) << 0,0,0,1;

  return Matrix_transf;
}



int HandPoseGenerator::whichArm( geometry_msgs::Pose object_pose ){

	tf::StampedTransform hand_left, hand_rigth;

	listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

	listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
	listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

	Eigen::Vector3d object_position(object_pose.position.x, object_pose.position.y, object_pose.position.z);
	Eigen::Vector3d hand_left_position(hand_left.getOrigin().x(),hand_left.getOrigin().y(),hand_left.getOrigin().z());
	Eigen::Vector3d hand_right_position(hand_rigth.getOrigin().x(),hand_rigth.getOrigin().y(),hand_rigth.getOrigin().z());

	double dist_to_left_hand = std::sqrt((object_position[0] - hand_left_position[0]) * (object_position[0] - hand_left_position[0]) +
				   (object_position[1] - hand_left_position[1]) * (object_position[1] - hand_left_position[1]) +
				   (object_position[2] - hand_left_position[2]) * (object_position[2] - hand_left_position[2]) );

	double dist_to_right_hand = std::sqrt((object_position[0] - hand_right_position[0]) * (object_position[0] - hand_right_position[0]) +
				   (object_position[1] - hand_right_position[1]) * (object_position[1] - hand_right_position[1]) +
				   (object_position[2] - hand_right_position[2]) * (object_position[2] - hand_right_position[2]) );


	// double dist_to_left_hand  = 0;
	// double dist_to_right_hand = 0;

	if(dist_to_left_hand < dist_to_right_hand)
	{
	  return 0;
	  ROS_DEBUG("Vito uses a: left arm");
	}
	
	ROS_DEBUG("Vito uses a: Right arm");
	return 1;
}
