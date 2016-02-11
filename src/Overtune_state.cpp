#include "overtune_state.h"

Overtune_state::Overtune_state()
{

	nh.param<std::string>("/right_arm/PotentialFieldControl/error_id", error_topic_right, "/right_arm/PotentialFieldControl/error_id");
  	error_sub_right = nh.subscribe(error_topic_right, 1, &Overtune_state::Error_info_right, this);

  	// nh.param<std::string>("/right_arm/PotentialFieldControl/topic_desired_reference", desired_hand_right_pose_topic_, "/right_arm/PotentialFieldControl/command");
  	// desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);

  	std::string string_temp;
    
	nh.param<std::string>("/right_arm/PotentialFieldControl/topic_desired_reference", string_temp, "command");
	desired_hand_right_pose_topic_ = std::string("/right_arm/PotentialFieldControl/") + string_temp;

	desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_right_pose_topic_.c_str(),1);




  	nh.param<std::string>("/right_arm/PotentialFieldControl/root_name", base_frame_, "world");

  	nh.param<std::string>("/right_arm/PotentialFieldControl/tip_name", right_hand_frame_, "right_hand_palm_ref_link");

  	id_class = static_cast<int>(transition_id::Vito_Overtune);
  	overturn_check = 0;
}

std::map< transition, bool > Overtune_state::getResults()
{
	std::map< transition, bool > results;
	
	if(finish == true)
	{
		results[transition::Error_arrived] = finish;
	}

	return results;
}


void Overtune_state::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
    tf::twistMsgToKDL (error_msg->error_, e_);

    id_error_msgs = error_msg->id;
    // std::cout<<"error: "<<error_msg->data[0]<<error_msg->data[1]<<error_msg->data[2]<<std::endl;
}


void Overtune_state::run()
{
	
	if(IsEqual(e_))
	{
		tf::StampedTransform hand_rigth;
		listener_info.waitForTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time::now(), ros::Duration(1));
		listener_info.lookupTransform(base_frame_.c_str(), right_hand_frame_.c_str(), ros::Time(0), hand_rigth);

		New_Hand_Position.pose.position.x = hand_rigth.getOrigin().x() - 0.20; /*move along the x axis for push down from the table*/
		New_Hand_Position.pose.position.y = hand_rigth.getOrigin().y();
		New_Hand_Position.pose.position.z = hand_rigth.getOrigin().z();
		KDL::Rotation matrix_temp;
		tf::quaternionTFToKDL(hand_rigth.getRotation(),matrix_temp);

		matrix_temp.GetQuaternion(New_Hand_Position.pose.orientation.x,New_Hand_Position.pose.orientation.y,
										New_Hand_Position.pose.orientation.z,New_Hand_Position.pose.orientation.w);


		switch(overturn_check)
	    {
	            case 0: //sends robot at position
	            {  
	                New_Hand_Position.pose.position.z = New_Hand_Position.pose.position.z + 0.20;
	                New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x - 0.20; /*move along the x axis for push down from the table*/
	                desired_hand_publisher_right.publish( New_Hand_Position );
	                overturn_check = 1;
	              break;
	            }

	            case 1: //replace table
	            {
	                
	                New_Hand_Position.pose.position.z = New_Hand_Position.pose.position.z - 0.20;
	                New_Hand_Position.pose.position.x = New_Hand_Position.pose.position.x + 0.20; /*move along the x axis for push down from the table*/
	                desired_hand_publisher_right.publish( New_Hand_Position );
	                
	                overturn_check = 2; 
	                break;  
	            }

	            case 2: //open hand and go to home
	            {
	            	overturn_check = 0; 
	                finish = true;
	                break;
	            }
	    }
	}


}

bool Overtune_state::isComplete()
{
    return finish;
}

std::string Overtune_state::get_type()
{
    return "Overtune_state";
}


bool Overtune_state::IsEqual(KDL::Twist E_pf)
  {
    KDL::Twist E_pf_abs;
    // std::cout<<"IsEqual"<<std::endl;
    E_pf_abs.vel.data[0] = std::abs(E_pf.vel.data[0] );
    E_pf_abs.vel.data[1] = std::abs(E_pf.vel.data[1] );
    E_pf_abs.vel.data[2] = std::abs(E_pf.vel.data[2] );
    E_pf_abs.rot.data[0] = std::abs(E_pf.rot.data[0] );
    E_pf_abs.rot.data[1] = std::abs(E_pf.rot.data[1] );
    E_pf_abs.rot.data[2] = std::abs(E_pf.rot.data[2] );


    if (  (E_pf_abs.vel.data[0] < E_t.vel.data[0]) &&
          (E_pf_abs.vel.data[1] < E_t.vel.data[1]) &&
          (E_pf_abs.vel.data[2] < E_t.vel.data[2]) &&
          (E_pf_abs.rot.data[0] < E_t.rot.data[0]) &&
          (E_pf_abs.rot.data[1] < E_t.rot.data[1]) &&
          (E_pf_abs.rot.data[2] < E_t.rot.data[2]) )
    {
       ROS_DEBUG("is equal");
      return true;
    }
    else
    {
      ROS_DEBUG("is not equal");
      ROS_DEBUG("error linear: E_pf_abs.vel.data[0] %g E_pf_abs.vel.data[1] %g  E_pf_abs.vel.data[2]: %g",  E_pf_abs.vel.data[0],E_pf_abs.vel.data[1], E_pf_abs.vel.data[2]);
      ROS_DEBUG("error agular: E_pf_abs.rot.data[0] %g E_pf_abs.rot.data[1] %g E_pf_abs.rot.data[2] %g", E_pf_abs.rot.data[0], E_pf_abs.rot.data[1], E_pf_abs.rot.data[2]);
      return false;
    }
  }