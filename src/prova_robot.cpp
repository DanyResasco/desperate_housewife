#include <prova_mp.h>



// bool phobic_mp::GetVitoUrdf()
// {
//   //take the vito urdf file
//   gazebo::physics::ModelPtr parent;
//   robot_namespace_ = parent->GetName(); // default
//   model_nh_ = ros::NodeHandle(robot_namespace_);
//   robot_description_ = "robot_description"; // default
//   link_names_= "link_names";

//   const std::string urdf_string = getURDF(robot_description_);

//   // if (!parseTransmissionsFromURDF(urdf_string))
//   // {
//   //   //ROS_ERROR_NAMED("gazebo_ros_soft_hand", "Error parsing URDF in gazebo_ros_soft_hand plugin, plugin not active.\n");
//   //   return;
//   // }

//   //make a model
//   urdf::Model urdf_model;
//   //check
//   const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

//   //make a robot tree
//   //KDL::Tree Robot_tree;
//   if (!kdl_parser::treeFromUrdfModel(urdf_model, Vito_desperate.Robot_tree))
//   {
//     ROS_ERROR("Failed to construct kdl tree");
//     return false;
//   }
//   else
//   {
//     return true;
//   }
// }

// std::string phobic_mp::getURDF(std::string param_name) 
// {
//   std::string urdf_string;

//   // search and wait for robot_description on param server
//   while (urdf_string.empty())
//   {
//     std::string search_param_name;
//     if (model_nh_.searchParam(param_name, search_param_name))
//     {
//       ROS_INFO("Vito_desperate is waiting for model", search_param_name.c_str());

//       model_nh_.getParam(search_param_name, urdf_string);
//     }
//     else
//     {
//       ROS_INFO("Vito_desperate is waiting for model", robot_description_.c_str());

//       model_nh_.getParam(param_name, urdf_string);
//     }

//     usleep(100000);
//   }
  
//   ROS_INFO("Recieved urdf from param server, create a urdf_model");

//   return urdf_string;
// }


void phobic_mp::Robot_Callback_left(sensor_msgs::JointState msg)
{
  
  for (int i=0; i < 7; i++)
  {
    Vito_desperate.robot_position_left[i] = msg.position[i]; 
    Vito_desperate.Vel_l[i] = msg.velocity[i]; //q_point
  }
  
}

void phobic_mp::Robot_Callback_right(sensor_msgs::JointState msg)
{
  
  for (int i=0; i < 7; i++)
  {
    Vito_desperate.robot_position_right[i] = msg.position[i]; //q
    Vito_desperate.Vel_r[i] = msg.velocity[i]; //q_point
  }
  
}


void phobic_mp::GetJacobian(int p)
{
    Vito_desperate.link_jac_solver_.resize(7);
    Vito_desperate.link_fk_solver_.resize(7);
    Vito_desperate.link_joints_.resize(7);
    Vito_desperate.link_chain_.resize(7);
    
    //link_names_[1] =""  //link_names_[j] = (link->GetName());
    //std::vector<KDL::Chain> link_chain_;

    for(int i=0; i<7; i++)
    {
      // get the chains
      Vito_desperate.Robot_tree.getChain(root_name.c_str(), link_names_[i].c_str(), Vito_desperate.link_chain_[i]);
      // init the jacobian solvers
      Vito_desperate.link_jac_solver_[i] = new KDL::ChainJntToJacSolver(Vito_desperate.link_chain_[i]);
      // init the forward kinematic solvers
      Vito_desperate.link_fk_solver_[i] = new KDL::ChainFkSolverPos_recursive(Vito_desperate.link_chain_[i]);
      // resize the complete jntarrays
      Vito_desperate.link_joints_[i] = KDL::JntArray(Vito_desperate.link_chain_[i].getNrOfJoints());

      // if(p == 0) //left
      // {
        Vito_desperate.link_joints_.at(i)(0) = Vito_desperate.robot_position_left[i];
        Vito_desperate.link_jac_l[i]= KDL::Jacobian(Vito_desperate.link_chain_[i].getNrOfJoints());
        //UPDATE KINEMATICS
        Vito_desperate.link_jac_solver_[i]->JntToJac(Vito_desperate.link_joints_[i], Vito_desperate.link_jac_l[i]); //jac
        // update frames
        Vito_desperate.link_fk_solver_[i]->JntToCart(Vito_desperate.link_joints_[i], Vito_desperate.link_frame_l[i]); //x
        Vito_desperate.link_jac_l[i].changeRefFrame(Vito_desperate.link_frame_l[i]);
    //  }

      // else
      // {
      //   Vito_desperate.link_joints_.at(i)(0) = Vito_desperate.robot_position_right[i];
      //   Vito_desperate.link_jac_r[i]= KDL::Jacobian(Vito_desperate.link_chain_[i].getNrOfJoints());
      //   //UPDATE KINEMATICS
      //   Vito_desperate.link_jac_solver_[i]->JntToJac(Vito_desperate.link_joints_[i], Vito_desperate.link_jac_r[i]); //jac
      //   // update frames
      //   Vito_desperate.link_fk_solver_[i]->JntToCart(Vito_desperate.link_joints_[i], Vito_desperate.link_frame_r[i]); //x
      //   Vito_desperate.link_jac_r[i].changeRefFrame(Vito_desperate.link_frame_r[i]);
      // }
    
    }

    Vito_desperate.link_jac_solver_.clear();
    Vito_desperate.link_fk_solver_.clear();
    Vito_desperate.link_joints_.clear();
    Vito_desperate.link_chain_.clear();
}


void phobic_mp::SetPotentialField_robot(std::vector<Eigen::VectorXd> &Force_repulsion, int p)
{
    std::vector<Eigen::Vector3d> distance_link; //Only position

    std::vector<Eigen::VectorXd> robot_link_position1;
    std::vector<Eigen::VectorXd> robot_link_position2;

    Eigen::VectorXd HAND;
    Eigen::VectorXd base_link;
    //robot_link_position->resize(Vito_desperate.robot_position_left.size());
    
     //p==0 is left arm, else is right arm
    // if(p ==0)
    // {
      for(int i=0; i< Vito_desperate.link_frame_l.size();i++)
      { 
        GetEuleroAngle(Vito_desperate.link_frame_l[i], robot_link_position1[i]); //take's the eulero angles
        //GetEuleroAngle(Vito_desperate.link_frame_r[i], robot_link_position2[i]);   
  
      }

        HAND =  Vito_desperate.Pos_HAND_l_x;     

        base_link = Vito_desperate.pos_base_l ;
        
     //}
     
     // else //right arm
     // {
     //    for(int i=0; i< Vito_desperate.link_frame_r.size();i++)
     //   { 
     //      GetEuleroAngle(Vito_desperate.link_frame_r[i], robot_link_position1[i]);
     //      GetEuleroAngle(Vito_desperate.link_frame_l[i], robot_link_position2[i]);   

     //    }

     //     HAND = Vito_desperate.Pos_HAND_r_x;
        

     //    base_link = Vito_desperate.pos_base_r; 
     //} 

    //Self collision
    // distance_link.push_back(GetDistance(robot_link_position1[1], robot_link_position2[5]));
    // distance_link.push_back(GetDistance(robot_link_position1[1], robot_link_position2[6]));
    // distance_link.push_back(GetDistance(robot_link_position1[1], robot_link_position2[7]));
    // distance_link.push_back(GetDistance(robot_link_position1[2], robot_link_position2[5]));
    // distance_link.push_back(GetDistance(robot_link_position1[2], robot_link_position2[6]));
    // distance_link.push_back(GetDistance(robot_link_position1[2], robot_link_position2[7])); 
    // distance_link.push_back(GetDistance(robot_link_position1[3], robot_link_position2[4]));
    // distance_link.push_back(GetDistance(robot_link_position1[3], robot_link_position2[5]));
    // distance_link.push_back(GetDistance(robot_link_position1[3], robot_link_position2[6]));
    // distance_link.push_back(GetDistance(robot_link_position1[3], robot_link_position2[7]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[3]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[4]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[5]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[6]));
    // distance_link.push_back(GetDistance(robot_link_position1[4], robot_link_position2[7]));

    // for(int i=5;i<=7;i++)
    // {
    //   for (int k=1; k<=7; k++)
    //   {
    //     distance_link.push_back(GetDistance(robot_link_position1[i], robot_link_position2[k]));
    //   }
    // }  

    //collision with softhand
    distance_link.push_back(GetDistance(HAND, base_link));

    for(int j = 3; j >= 1; j--) //with himself
    {
      distance_link.push_back(GetDistance(HAND, robot_link_position1[j]));
    }

    // for(int i=1; i<=7; i++)//with other arm
    // {
    //   distance_link.push_back(GetDistance(HAND,robot_link_position2[i]));
    // }

    // Repulsive fields = K/distance^2 (1/distance -1/influence) partial_derivative_vector

    for (int i=0; i <= distance_link.size(); i++)
    {
       double local_distance;
       local_distance  = distance_link[i].norm();

      if(local_distance <= influence) //if in minus than influence area
      {
        Eigen::Vector3d distance_der_partial;
        distance_der_partial[0] =  distance_link[i](0) / sqrt(pow(distance_link[i](0),2)+ pow(distance_link[i](1),2) +pow(distance_link[i](2),2));
        distance_der_partial[1] =  distance_link[i](1) / sqrt(pow(distance_link[i](0),2)+ pow(distance_link[i](1),2) +pow(distance_link[i](2),2));
        distance_der_partial[2] =  distance_link[i](2) / sqrt(pow(distance_link[i](0),2)+ pow(distance_link[i](1),2) +pow(distance_link[i](2),2));

        Eigen::Vector3d vec_Temp;
        vec_Temp << (P_obj/pow(distance_link[i].norm(),2)) * (1/distance_link[i].norm() - 1/influence) *distance_der_partial[0];

        Force_repulsion_left << vec_Temp; //Ricorda che se devi metterne più di uno devi mettere le parentesi
      }
      else
      {
        //Force_repulsion.push_back(0);
        continue;
      }
    }
}