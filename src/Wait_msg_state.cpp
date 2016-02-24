
#include <Wait_msgs_.h>


Wait_msgs::Wait_msgs(const shared& m):data(m)
{  nh.param<std::string>("/right_arm/controller", control_topic_right, "PotentialFieldControl");
  nh.param<std::string>("/left_arm/controller", control_topic_left, "PotentialFieldControl");

  // nh.param<std::string>("/right_arm/objects_info", obj_info_topic_r, "/right_arm/objects_info");
  obj_info_topic_r = "/right_arm/" + control_topic_right + "/objects_info";
  objects_info_right_sub = nh.subscribe(obj_info_topic_r.c_str(),1, &Wait_msgs::ObjOrObst_right,this);

  arrived_r = 0;
}

void Wait_msgs::ObjOrObst_right(const std_msgs::UInt16::ConstPtr& obj_msg)
{
  // whichArm = 0;
  ObjOrObst = obj_msg->data;
  arrived_r = 1;
}




std::map< transition, bool > Wait_msgs::getResults()
{
  std::map< transition, bool > results;
  // std::cout<<"send results"<<std::endl;
  if(arrived_r == 1)
    {
      switch(ObjOrObst)
        {
        case 0:
          {
            results[transition::Grasp_Obj] = true;
            break;
          }
        case 1:
          {
            results[transition::Removed_Obj] = true;
            break;
          }
        case 2:
          {
            results[transition::Overtune_table] = true;
            break;
          }

        }
    }

  return results;
}

void Wait_msgs::run()
{	    
  usleep(200000);
}

bool Wait_msgs::isComplete()
{
  if(arrived_r == 1)
    return true;
  else
    return false;
}

std::string Wait_msgs::get_type()
{
  return "Wait_msgs";
}

// void Wait_msgs::reset()
// {
// }
