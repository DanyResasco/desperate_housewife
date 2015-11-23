#include <desperate_mind.h>

void DesperateDecisionMaker::SendHomeRobot_left()
{
    desperate_housewife::handPoseSingle home_robot_left;

    Eigen::Matrix4d ROT_y;
    ROT_y.row(0)<< -1,0,0,0;
    ROT_y.row(1)<< 0,1,0,0;
    ROT_y.row(2)<< 0,0,-1,0;
    ROT_y.row(3)<< 0,0,0,1;

    Eigen::Matrix4d Rot_z;
    Rot_z.row(0)<< -1,0,0,0;
    Rot_z.row(1)<< 0,-1,0,0;
    Rot_z.row(2)<< 0,0,1,0;
    Rot_z.row(3)<< 0,0,0,1;


    std::cout<<"arrivato in node home left"<<std::endl;
    home_robot_left.home = 1;
    home_robot_left.obj = 0;

    double roll,pitch,yaw;
    nh.param<double>("/home_left_arm_position_x", home_robot_left.pose.position.x, -0.75022);
    nh.param<double>("/home_left_arm_position_y",  home_robot_left.pose.position.y,  -0.47078);
    nh.param<double>("/home_left_arm_position_z", home_robot_left.pose.position.z, 0.74494);
    nh.param<double>("/home_left_arm_A", yaw,  -0.12690);
    nh.param<double>("/home_left_arm_B", pitch, -0.06571);
    nh.param<double>("/home_left_arm_C", roll, -0.11774);



    Eigen::Matrix3d Tmatrix;
    Tmatrix = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Matrix4d Vito_home_base = Eigen::Matrix4d::Identity(4,4);
    Vito_home_base.block<3,3>(0,0) = Tmatrix;
    Vito_home_base(0,3) = home_robot_left.pose.position.x;
    Vito_home_base(1,3) = home_robot_left.pose.position.y;
    Vito_home_base(2,3) = home_robot_left.pose.position.z;
    
     Eigen::Matrix4d Vito_home_base_rot = Vito_home_base*ROT_y;

    Eigen::Quaterniond quat_eigen_hand(Vito_home_base_rot.block<3,3>(0,0));
    home_robot_left.pose.orientation.x = quat_eigen_hand.x();
    home_robot_left.pose.orientation.y = quat_eigen_hand.y();
    home_robot_left.pose.orientation.z = quat_eigen_hand.z();
    home_robot_left.pose.orientation.w = quat_eigen_hand.w();

    //  tf::StampedTransform hand_left;
    // listener_info.waitForTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
    // listener_info.lookupTransform(base_frame_.c_str(), left_hand_frame_.c_str(), ros::Time(0), hand_left);

    
    // home_robot_left.pose.position.x = hand_left.getOrigin().x();
    // home_robot_left.pose.position.y = hand_left.getOrigin().y();
    // home_robot_left.pose.position.z = hand_left.getOrigin().z() - 0.1;
  
    // home_robot_left.pose.orientation.x = hand_left.getRotation().getX();
    // home_robot_left.pose.orientation.y = hand_left.getRotation().getY();
    // home_robot_left.pose.orientation.z = hand_left.getRotation().getZ();
    // home_robot_left.pose.orientation.w = hand_left.getRotation().getW();


      // std::cout<<"qui"<<std::endl;
    desired_hand_left_pose_publisher_.publish( home_robot_left );
    // std::cout<<"qui"<<std::endl;
    tf::Transform tfHandTrasform2;
    tf::poseMsgToTF( home_robot_left.pose, tfHandTrasform2);
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"home_robot_left") ); 

}

void DesperateDecisionMaker::SendHomeRobot_right()
{
     desperate_housewife::handPoseSingle home_robot_right;  
    Eigen::Matrix4d ROT_y;
    ROT_y.row(0)<< -1,0,0,0;
    ROT_y.row(1)<< 0,1,0,0;
    ROT_y.row(2)<< 0,0,-1,0;
    ROT_y.row(3)<< 0,0,0,1;

    Eigen::Matrix4d Rot_z;
    Rot_z.row(0)<< -1,0,0,0;
    Rot_z.row(1)<< 0,-1,0,0;
    Rot_z.row(2)<< 0,0,1,0;
    Rot_z.row(3)<< 0,0,0,1;
    
    home_robot_right.home = 1;
    home_robot_right.obj = 0; 
    // desired_hand_left_pose_publisher_.publish( home_robot_left );
    double roll_r,pitch_r,yaw_r;
    nh.param<double>("/home_right_arm_position_x", home_robot_right.pose.position.x, -0.75022);
    nh.param<double>("/home_right_arm_position_y",  home_robot_right.pose.position.y,  0.47078);
    nh.param<double>("/home_right_arm_position_z", home_robot_right.pose.position.z, 0.74494);
    nh.param<double>("/home_right_arm_A", yaw_r,  0.134);
    nh.param<double>("/home_right_arm_B", pitch_r, -0.08650);
    nh.param<double>("/home_right_arm_C", roll_r, -0.5108);

   
    Eigen::Matrix3d Tmatrix_right;
    Tmatrix_right = Eigen::AngleAxisd(roll_r, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch_r, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw_r, Eigen::Vector3d::UnitZ());

   
    Eigen::Matrix4d Vito_home_base_right = Eigen::Matrix4d::Identity(4,4);
    Vito_home_base_right.block<3,3>(0,0) = Tmatrix_right;
    Vito_home_base_right(0,3) = home_robot_right.pose.position.x;
    Vito_home_base_right(1,3) = home_robot_right.pose.position.y;
    Vito_home_base_right(2,3) = home_robot_right.pose.position.z;

    Eigen::Matrix4d Vito_home_base_right_rot = Vito_home_base_right*ROT_y*Rot_z;
    // *Rot_z;
   
    
    Eigen::Quaterniond quat_eigen_hand_right(Vito_home_base_right_rot.block<3,3>(0,0));
    home_robot_right.pose.orientation.x = quat_eigen_hand_right.x();
    home_robot_right.pose.orientation.y = quat_eigen_hand_right.y();
    home_robot_right.pose.orientation.z = quat_eigen_hand_right.z();
    home_robot_right.pose.orientation.w = quat_eigen_hand_right.w();

    // tf::StampedTransform  hand_rigth;
    // listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
    // listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

    // home_robot_right.pose.position.x = hand_rigth.getOrigin().x();
    // home_robot_right.pose.position.y = hand_rigth.getOrigin().y();
    // home_robot_right.pose.position.z = hand_rigth.getOrigin().z()-0.1;
  
    // home_robot_right.pose.orientation.x = hand_rigth.getRotation().getX();
    // home_robot_right.pose.orientation.y = hand_rigth.getRotation().getY();
    // home_robot_right.pose.orientation.z = hand_rigth.getRotation().getZ();
    // home_robot_right.pose.orientation.w = hand_rigth.getRotation().getW();

    desired_hand_right_pose_publisher_.publish( home_robot_right );
    
    // desired_hand_right_pose_publisher_.publish( home_robot_right );
      tf::Transform tfHandTrasform1;    
      tf::poseMsgToTF( home_robot_right.pose, tfHandTrasform1);    
      tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"home_robot_right") ); 


  
}


