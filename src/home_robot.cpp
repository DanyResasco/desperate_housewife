#include <desperate_mind.h>

void DesperateDecisionMaker::SendHomeRobot_left()
{
    desperate_housewife::handPoseSingle home_robot_left;

    std::cout<<"arrivato in node home left"<<std::endl;
    home_robot_left.home = 1;
    home_robot_left.obj = 0;

    double roll,pitch,yaw;
    nh.param<double>("/desperate_mind_node/home_left_arm_position_x", home_robot_left.pose.position.x, -0.75022);
    nh.param<double>("/desperate_mind_node/home_left_arm_position_y",  home_robot_left.pose.position.y,  -0.47078);
    nh.param<double>("/desperate_mind_node/home_left_arm_position_z", home_robot_left.pose.position.z, 0.74494);
    nh.param<double>("/desperate_mind_node/home_left_arm_A_yaw", yaw,  -0.12690);
    nh.param<double>("/desperate_mind_node/home_left_arm_B_pitch", pitch, -0.06571);
    nh.param<double>("/desperate_mind_node/home_left_arm_C_roll", roll, -0.11774);


    KDL::Rotation Rot_matrix = KDL::Rotation::RPY(roll, pitch, yaw);

    Rot_matrix.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

    home_robot_left.pose.orientation.x = quat_des_.v(0);
    home_robot_left.pose.orientation.y = quat_des_.v(1);
    home_robot_left.pose.orientation.z = quat_des_.v(2);
    home_robot_left.pose.orientation.w = quat_des_.a;

    desired_hand_left_pose_publisher_.publish( home_robot_left );

    tf::Transform tfHandTrasform2;
    tf::poseMsgToTF( home_robot_left.pose, tfHandTrasform2);
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform2, ros::Time::now(), base_frame_.c_str(),"home_robot_left") ); 

}

void DesperateDecisionMaker::SendHomeRobot_right()
{
    desperate_housewife::handPoseSingle home_robot_right;  
    
    home_robot_right.home = 1;
    home_robot_right.obj = 0; 
    
    double roll_r,pitch_r,yaw_r;
    nh.param<double>("/desperate_mind_node/home_right_arm_position_x", home_robot_right.pose.position.x, -0.75022);
    nh.param<double>("/desperate_mind_node/home_right_arm_position_y",  home_robot_right.pose.position.y,  0.47078);
    nh.param<double>("/desperate_mind_node/home_right_arm_position_z", home_robot_right.pose.position.z, 0.74494);
    nh.param<double>("/desperate_mind_node/home_right_arm_A_yaw", yaw_r,  0.334);
    nh.param<double>("/desperate_mind_node/home_right_arm_B_pitch", pitch_r, -0.08650);
    nh.param<double>("/desperate_mind_node/home_right_arm_C_roll", roll_r, -0.5108);

  
    KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll_r,pitch_r,yaw_r);

    Rot_matrix_r.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

    home_robot_right.pose.orientation.x = quat_des_.v(0);
    home_robot_right.pose.orientation.y = quat_des_.v(1);
    home_robot_right.pose.orientation.z = quat_des_.v(2);
    home_robot_right.pose.orientation.w = quat_des_.a;

    desired_hand_right_pose_publisher_.publish( home_robot_right );

    tf::Transform tfHandTrasform1;    
    tf::poseMsgToTF( home_robot_right.pose, tfHandTrasform1);    
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"home_robot_right") );  
}

geometry_msgs::Pose DesperateDecisionMaker::TrashObjectPOsition(int Arm_, geometry_msgs::Quaternion &Quat_hand)
{
    double roll,pitch,yaw;
    geometry_msgs::Pose trash_robot_pose;
   
    if(Arm_ == 1)//left arm
    {
        nh.param<double>("/desperate_mind_node/trash_left_arm_position_x", trash_robot_pose.position.x, -0.75022);
        nh.param<double>("/desperate_mind_node/trash_left_arm_position_y",  trash_robot_pose.position.y,  -0.47078);
        nh.param<double>("/desperate_mind_node/trash_left_arm_position_z", trash_robot_pose.position.z, 0.74494);
        nh.param<double>("/desperate_mind_node//trash_left_arm_A_yaw", yaw,  -0.12690);
        nh.param<double>("/desperate_mind_node//trash_left_arm_B_pitch", pitch, -0.06571);
        nh.param<double>("/desperate_mind_node//trash_left_arm_C_roll", roll, -0.11774);
        // desired_hand_left_pose_publisher_.publish( trash_robot_pose );
        // tf::poseMsgToTF( home_robot_left.pose, tfHandTrasform1); 
    }
    else
    {
        nh.param<double>("/desperate_mind_node/trash_right_arm_position_x", trash_robot_pose.position.x, -0.75022);
        nh.param<double>("/desperate_mind_node/trash_right_arm_position_y",  trash_robot_pose.position.y,  -0.47078);
        nh.param<double>("/desperate_mind_node/trash_right_arm_position_z", trash_robot_pose.position.z, 0.74494);
        nh.param<double>("/desperate_mind_node//trash_right_arm_A_yaw", yaw,  -0.12690);
        nh.param<double>("/desperate_mind_node//trash_right_arm_B_pitch", pitch, -0.06571);
        nh.param<double>("/desperate_mind_node//trash_right_arm_C_roll", roll, -0.11774);
        // desired_hand_right_pose_publisher_.publish( trash_robot_pose );
        // tf::poseMsgToTF( home_robot_right.pose, tfHandTrasform1); 
    }


    KDL::Rotation Rot_matrix_r = KDL::Rotation::RPY(roll, pitch ,yaw);

    Rot_matrix_r.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

    trash_robot_pose.orientation.x = quat_des_.v(0);
    trash_robot_pose.orientation.y = quat_des_.v(1);
    trash_robot_pose.orientation.z = quat_des_.v(2);
    trash_robot_pose.orientation.w = quat_des_.a;

    // tf::Transform tfHandTrasform1;    
    // tf::poseMsgToTF( trash_robot_pose, tfHandTrasform1);    
    // tf_desired_hand_pose.sendTransform( tf::StampedTransform( tfHandTrasform1, ros::Time::now(), base_frame_.c_str(),"trash_robot_pos") );  
    
    return trash_robot_pose;
}
