#include <grasp_state.h>


Grasp_move::Grasp_move()
{
	this->type=type;
	nh.param<std::string>("/right_arm/PotentialFieldControl/error_interpolate", error_topic_right, "/right_arm/PotentialFieldControl/error_interpolate");
  error_sub_right = nh.subscribe(error_topic_right, 1, &Grasp_move::Error_info_right, this);

  	msg_arrived = 0;
  	finish = false;
  	failed = false;

    double x,y,z,rotx,roty,rotz;
   /*treshold error*/
    nh.param<double>("/x_treshold",x,0.01);
    nh.param<double>("/y_treshold",y,0.01);
    nh.param<double>("/z_treshold",z,0.01);
    nh.param<double>("/rot_x_treshold",rotx,0.01);
    nh.param<double>("/rot_y_treshold",roty,0.01);
    nh.param<double>("/rot_z_treshold",rotz,0.01);

    KDL::Vector vel;
    KDL::Vector rot;
    vel.data[0] = x;
    vel.data[1] = y;
    vel.data[2] = z;
    rot.data[0] = rotx;
    rot.data[1] = roty;
    rot.data[2] = rotz;
    E_t.vel = vel;
    E_t.rot = rot;

    id_class = static_cast<int>(transition_id::Vito_grasp);
}

std::map< transition, bool > Grasp_move::getResults()
{
	std::map< transition, bool > results;
	
	if(finish == true)
	{
		results[transition::Error_arrived] = finish;
	}

	return results;
}


void Grasp_move::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
    tf::twistMsgToKDL (error_msg->error_, e_);

    id_msgs = error_msg->id;
    // std::cout<<"error: "<<error_msg->data[0]<<error_msg->data[1]<<error_msg->data[2]<<std::endl;
}


void Grasp_move::run()
{
   	if(IsEqual(e_))
   	{
   			finish = true;
   	}
   	else
   			finish = false;
}

bool Grasp_move::isComplete()
{
    return finish;
}

std::string Grasp_move::get_type()
{
    return "Grasp_move";
}

bool Grasp_move::IsEqual(KDL::Twist E_pf)
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