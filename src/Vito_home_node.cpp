#include <ros/ros.h>
#include <ros/console.h>
#include <Vito_home.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Vito home position node");
  HomeVitoPosition node;

  ROS_INFO("[HomeVitoPosition] Node is ready");

  double spin_rate = 10;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);

  ros::Rate rate(spin_rate); 

  while (node.nh.ok())
  {
    ros::spinOnce(); 
    rate.sleep();
  }
  return 0;
}

HomeVitoPosition::HomeVitoPosition()
{
  nh.param<std::string>("PotentialFieldControl/home_desperate_left" , home_left_topic_, "/left_arm/PotentialFieldControl/home_left");
  left_home_subscribe_ = nh.subscribe(home_left_topic_, 1, &HomeVitoPosition::SendHomeRobot_left, this);

  nh.param<std::string>("PotentialFieldControl/home_desperate_right", home_right_topic_, "/right_arm/PotentialFieldControl/home_right");
  right_home_subscribe_ = nh.subscribe(home_right_topic_, 1, &HomeVitoPosition::SendHomeRobot_right, this);

    nh.param<std::string>("/PotentialFieldControl/base_frame", base_frame_, "vito_anchor");

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_right", desired_hand_right_pose_topic_, "/right_arm/PotentialFieldControl/desired_hand_right_pose");
  desired_hand_right_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  nh.param<std::string>("/PotentialFieldControl/desired_hand_pose_left", desired_hand_left_pose_topic_, "/left_arm/PotentialFieldControl/desired_hand_left_pose");
  desired_hand_left_pose_publisher_ = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_left_pose_topic_.c_str(),1);
}


void HomeVitoPosition::SendHomeRobot_left(const std_msgs::Bool::ConstPtr& home_msg)
{
    // desperate_housewife::handPoseSingle home_robot_left;

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



  if(home_msg->data == true)
  {
    // std::cout<<"arrivato in node home left"<<std::endl;
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

      // std::cout<<"qui"<<std::endl;
    desired_hand_left_pose_publisher_.publish( home_robot_left );
    // std::cout<<"qui"<<std::endl;
    tf::Transform tfHandTrasform2;
    tf::poseMsgToTF( home_robot_left.pose, tfHandTrasform2);
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"home_robot_left") ); 
  }
  else
  {
    home_robot_left.home = 0;
    home_robot_left.obj = 0;
    desired_hand_left_pose_publisher_.publish( home_robot_left );
  }
}

void HomeVitoPosition::SendHomeRobot_right(const std_msgs::Bool::ConstPtr& home_msg)
{
       
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

    std::cout << "test" << std::endl;
    ROS_INFO("publishing on %s", desired_hand_right_pose_topic_.c_str());



  // desperate_housewife::handPoseSingle home_robot_right;
  // if(home_msg->data == true)
  // {
    // std::cout<<"arrivato in node home right"<<std::endl;
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

    desired_hand_right_pose_publisher_.publish( home_robot_right );
    
    // desired_hand_right_pose_publisher_.publish( home_robot_right );
      tf::Transform tfHandTrasform1;    
      tf::poseMsgToTF( home_robot_right.pose, tfHandTrasform1);    
      tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"home_robot_right") ); 

  // }
  // else
  // {
  //   std::cout<<"invio data home a zero"<<std::endl;
  //   home_robot_right.home = 0;
  //   home_robot_right.obj = 0;
  //   desired_hand_right_pose_publisher_.publish( home_robot_right );
  //   std::cout<<"inviato a zero"<<std::endl;
  // }
  
}


