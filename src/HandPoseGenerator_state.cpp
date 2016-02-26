#include "HandPoseGenerator_state.h"
#include <place_hand_dany.hpp>
#include <getClosestObject.hpp>
// #include <visualization_msgs/MarkerArray.h>

HandPoseGenerator::HandPoseGenerator(shared& m): data(m)
{
  nh.param<int>("/demo", demo, 0);
  nh.param<int>("/max_number_obj", Number_obj, 5);

  nh.param<std::string>("/right_arm/controller", control_topic_right, "PotentialFieldControl");
  nh.param<std::string>("/left_arm/controller", control_topic_left, "PotentialFieldControl");

  /*reads geometry information*/
  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  stream_subscriber_ = nh.subscribe(geometries_topic_.c_str(), 1, &HandPoseGenerator::HandPoseGeneratorCallback, this);

  /*sends obstacle informations*/
  obstacles_topic_right = "/right_arm/" + control_topic_right + "/obstacles"; 
  obstacles_publisher_right = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstacles_topic_right.c_str(), 1);

  obstacles_topic_left = "/left_arm/" + control_topic_left + "/obstacles";
  obstacles_publisher_left = nh.advertise<desperate_housewife::fittedGeometriesArray > (obstacles_topic_left.c_str(), 1);

  /*sends information about the remove or grasp objects */
  obj_info_topic_r = "/right_arm/" + control_topic_right + "/objects_info";
  objects_info_right_pub = nh.advertise<std_msgs::UInt16 > (obj_info_topic_r.c_str(), 1, this);
  
  obj_info_topic_l = "/left_arm/" + control_topic_left + "/objects_info";
  objects_info_left_pub = nh.advertise<std_msgs::UInt16 > (obj_info_topic_l.c_str(), 1, this);

  /*config parameteres*/
  nh.param<std::string>("/right_arm/" + control_topic_right + "/tip_name", right_hand_frame_ , "right_hand_palm_ref_link");
  nh.param<std::string>("/right_arm/" + control_topic_right + "/root_name", base_frame_, "world");
  nh.param<std::string>("/left_arm/" + control_topic_right + "/tip_name", left_hand_frame_, "left_hand_palm_ref_link");

  /*sends hand pose*/
  desired_hand_pose_right_topic_ = "/right_arm/" + control_topic_right + "/command";
  desired_hand_publisher_right = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_right_topic_.c_str(), 1);

  desired_hand_pose_left_topic_ = "/left_arm/" + control_topic_left + "/command";
  desired_hand_publisher_left = nh.advertise<desperate_housewife::handPoseSingle > (desired_hand_pose_left_topic_.c_str(), 1);

  nh.param<double>("/SoftHandDistanceFrame", SoftHandDistanceFrame, 0.2);

  id_class = static_cast<int>(transition_id::Gen_pose);

  ROS_INFO("HandPoseGenerator: %d", id_class);

  /*read the error*/
  error_topic_right = "/right_arm/" + control_topic_right + "/error_id";
  error_sub_right = nh.subscribe(error_topic_right.c_str(), 1, &HandPoseGenerator::Error_info_right, this);

  error_topic_left = "/left_arm/" + control_topic_left + "/error_id";
  error_sub_left = nh.subscribe(error_topic_left.c_str(), 1, &HandPoseGenerator::Error_info_left, this);

  /*send the msg to close hand*/
  nh.param<std::string>("/right_hand/joint_trajectory_controller/command", hand_close_right, "/right_hand/joint_trajectory_controller/command");
  hand_publisher_right = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_right.c_str(), 1000);

  nh.param<std::string>("/left_hand/joint_trajectory_controller/command", hand_close_left, "/left_hand/joint_trajectory_controller/command");
  hand_publisher_left = nh.advertise<trajectory_msgs::JointTrajectory>(hand_close_left.c_str(), 1000);

  pub_aux_projection = nh.advertise<visualization_msgs::Marker>("aux_projection", 10);
  pub_aux_objects_sorted = nh.advertise<desperate_housewife::fittedGeometriesArray>("aux_objects_sorted", 10);
  pub_aux_graspable_object = nh.advertise<desperate_housewife::fittedGeometriesSingle>("aux_graspable_object", 10);
  pub_aux_obstacles = nh.advertise<desperate_housewife::fittedGeometriesArray>("aux_obstacles", 10);

  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>( "obstacles_in_control", 1 );

  /*to stop*/
  srv_reset = nh.subscribe("/reset",1, &HandPoseGenerator::resetCallBack, this);

  /*treshold error*/
  double x, y, z, rotx, roty, rotz;

  nh.param<double>("/error/pos/x", x, 0.01);
  nh.param<double>("/error/pos/y", y, 0.01);
  nh.param<double>("/error/pos/z", z, 0.01);
  nh.param<double>("/error/rot/x", rotx, 0.01);
  nh.param<double>("/error/rot/y", roty, 0.01);
  nh.param<double>("/error/rot/z", rotz, 0.01);

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
  Arm_r = false;
  Arm_l = false;
  finish = false;
  failed = false;
  /*to initialize the error*/
  vect_error.resize(2);
  KDL::Twist temp;
  SetToZero(temp);
  vect_error[0] = temp;
  vect_error[1] = temp;
  send_msg = 0;
  home_reset = false;
}


void HandPoseGenerator::Error_info_right(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  tf::twistMsgToKDL (error_msg->error_, e_r);

  id_msgs = error_msg->id;
  vect_error[0] = e_r;
  Arm_r = true;
}


void HandPoseGenerator::Error_info_left(const desperate_housewife::Error_msg::ConstPtr& error_msg)
{
  tf::twistMsgToKDL (error_msg->error_, e_l);

  id_msgs = error_msg->id;
  vect_error[1] = e_l;
  Arm_l = true;
}



void HandPoseGenerator::HandPoseGeneratorCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  // if (id_msgs == static_cast<int>(transition_id::Vito_home))
  // {
    cylinder_geometry.geometries.clear();
    if (id_msgs == static_cast<int>(transition_id::Vito_home))
    {
      cylinder_geometry.geometries.resize(msg->geometries.size());

      for (unsigned int i = 0; i < msg->geometries.size(); i++ )
      {
        cylinder_geometry.geometries[i] = msg->geometries[i];
      }
      // ROS_INFO_STREAM("New environment received and considered");
    }
    // else
    // {
    //   ROS_INFO_STREAM("New environment received but not considered");
    // }
    
}

std::map< transition, bool > HandPoseGenerator::getResults()
{
  std::map< transition, bool > results;

  if (failed == true)
  {
    results[transition::failed] = true;
     send_msg = 0;
  } 
  else
  {
    if (home_reset == true)
    {
      results[transition::home_reset] = true;
      home_reset = false;
      send_msg = 0;
      ROS_DEBUG("RESET");
      // id_class = static_cast<int>(transition_id::Vito_trash);
    }
    else
    {
      results[transition::Error_arrived] = true;
    }
  }

 
  finish = false;
  return results;
}

bool HandPoseGenerator::isComplete()
{
  return finish;
}

std::string HandPoseGenerator::get_type()
{
  return "HandPoseGenerator";
}


void HandPoseGenerator::run()
{
  // std::cout<<"start controller"<<std::endl;
  ROS_DEBUG("start controller");
  desperate_housewife::handPoseSingle DesiredHandPose;

  // DesiredHandPose.home = 0;
  std_msgs::UInt16 Obj_info;  /*msg for desperate_mind with object's information*/

  desperate_housewife::fittedGeometriesArray obstaclesMsg;

  e_ = vect_error[0] + vect_error[1]; //sum of error norma uno

  // ROS_INFO_STREAM("id_class: " << id_class << "\tid_msgs: " << id_msgs << "\tsend_msg: " << send_msg << "finish = "  << (finish ? "true" : "false"));
  if ((id_msgs != id_class) && IsEqual(e_) && (send_msg == 0))
  {
    id_class = static_cast<int>(transition_id::Gen_pose);
    if ( cylinder_geometry.geometries.size() <= 0)
    {
      failed = true;
      finish = true;
      ROS_INFO("No objects on table");
      return;
    }
    else if (static_cast<int>(cylinder_geometry.geometries.size()) >= Number_obj)
    {
      if ((Arm_l == true) && (Arm_r == true))
      {
        Overturn();
        data.arm_to_use = 2;
      }
      else
      {
        failed = true;
        //to Change to a primitive
        ROS_ERROR("There are more than: %d ", Number_obj);
      }
    }
    else
    {
      std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec;
      send_msg = 1;
      desperate_housewife::fittedGeometriesArray obstaclesMsg;
      std_msgs::UInt16 Obj_info;

      for (unsigned int i = 0; i < cylinder_geometry.geometries.size(); i++)
      {
        objects_vec.push_back(cylinder_geometry.geometries[i]);
        objects_vec[i].id = i;
      }

      std::vector< desperate_housewife::fittedGeometriesSingle > objects_vec_sorted = getClosestObject( objects_vec);

      desperate_housewife::fittedGeometriesArray objects_sorted_msg;
      for (unsigned int i = 0; i < objects_vec.size(); ++i)
      {
        objects_sorted_msg.geometries.push_back(objects_vec_sorted[i]);
      }
      pub_aux_objects_sorted.publish(objects_sorted_msg);

      switch (demo)
      {
      case 0:
      {
        DesperateDemo0(objects_vec_sorted);
        break;
      }
      case 1:
      {
        DesperateDemo1(objects_vec_sorted);
        break;
      }
      }
    }
  }
  else if ((id_msgs == id_class)  && (IsEqual(e_)))
  {
    // ROS_INFO("Sens msgs to whait_msg_state");
    Obj_info.data = ObjorObst;
    // ROS_INFO_STREAM("ObjorObst: " << ObjorObst);
    objects_info_right_pub.publish(Obj_info);
    finish = true;
    send_msg = 0;
    failed = false;
  }
  else if ((id_msgs == id_class) && (!IsEqual(e_)))
  {
    finish = false;
  }
}


desperate_housewife::handPoseSingle HandPoseGenerator::generateHandPose( desperate_housewife::fittedGeometriesSingle geometry, int cyl_nbr )
{
  desperate_housewife::handPoseSingle hand_pose_local;
  hand_pose_local.whichArm = whichArm(  geometry.id);
  hand_pose_local.pose = placeHand( geometry, hand_pose_local.whichArm );
  hand_pose_local.isGraspable = true;

  return hand_pose_local;
}

bool HandPoseGenerator::isGeometryGraspable ( desperate_housewife::fittedGeometriesSingle geometry )
{
  /*comparision between ration and treshold and cylinder radius with another threshold*/
  // if ( (geometry.info[geometry.info.size() - 1] >= 55) && (geometry.info[0] < 0.10))
  if ( geometry.info[0] < 0.10 )
  {
    return true;
  }

  return false;
}


bool HandPoseGenerator::IsEqual(KDL::Twist E_pf)
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
    // ROS_DEBUG("is equal");
    return true;
  }
  else
  {
    return false;
  }
}

void HandPoseGenerator::DesperateDemo0( std::vector<desperate_housewife::fittedGeometriesSingle> cyl_)
{
  ROS_INFO("*** DEMO 0, take first graspable object with obstacle avoidance ***");

  desperate_housewife::fittedGeometriesArray obstaclesMsg;
  desperate_housewife::handPoseSingle DesiredHandPose;

  desperate_housewife::fittedGeometriesSingle grapableObjectMsg;

  obstaclesMsg.geometries.clear();

  bool graspable_object_exist = false;
  int index_grasp = 0;
  for (unsigned int k = 0; k < cyl_.size(); k++)
  {
    failed = false;
    if ( isGeometryGraspable( cyl_[k] ) && !(graspable_object_exist) )
    {
      DesiredHandPose = generateHandPose( cyl_[k], cyl_[k].id );
      graspable_object_exist = true;
      grapableObjectMsg = cyl_[k];
      index_grasp = cyl_[k].id;
    }
    else
    {
      obstaclesMsg.geometries.push_back( cyl_[k] );
    }

  }
  DesiredHandPose.id = id_class;
  if (graspable_object_exist)
  {
    data.arm_to_use = DesiredHandPose.whichArm;
    ObjorObst = 0;
    if (DesiredHandPose.whichArm == 1) /*left arm*/
    {

      desired_hand_publisher_left.publish( DesiredHandPose );
      obstacles_publisher_left.publish(obstaclesMsg);
      ROS_INFO_STREAM("There is a graspable object, No. objects in obstaclesMsg: " << obstaclesMsg.geometries.size() );
      // finish = false;
    }

    else /*right arm*/
    {
      // DesiredHandPose.id = id_class;
      desired_hand_publisher_right.publish( DesiredHandPose );
      obstacles_publisher_right.publish(obstaclesMsg);
      // finish = false;
    }
    pub_aux_graspable_object.publish(grapableObjectMsg);
    pub_aux_obstacles.publish(obstaclesMsg);
    obstaclesMsg.geometries.push_back(grapableObjectMsg);
    plotObstacles(obstaclesMsg, index_grasp);
  }
  else
  {
    ObjorObst = 1;
    // DesiredHandPose.id = id_class;
    DesiredHandPose = generateHandPose( obstaclesMsg.geometries[0], obstaclesMsg.geometries[0].id );
    DesiredHandPose.pose = ObstacleReject( obstaclesMsg.geometries[0], DesiredHandPose.whichArm);

    desperate_housewife::fittedGeometriesArray obstaclesMsg_local;
    obstaclesMsg_local.geometries.clear();
    obstaclesMsg_local = obstaclesMsg;


    data.arm_to_use = DesiredHandPose.whichArm;

    if (DesiredHandPose.whichArm == 1) /*left arm*/
    {
      // DesiredHandPose.id = id_class;
      desired_hand_publisher_left.publish( DesiredHandPose );
      index_grasp = obstaclesMsg_local.geometries[0].id;
      obstaclesMsg_local.geometries.erase(obstaclesMsg_local.geometries.begin());
      obstacles_publisher_left.publish(obstaclesMsg_local);
      // finish = false;
    }

    else /*right arm*/
    {
      DesiredHandPose.id = id_class;
      desired_hand_publisher_right.publish( DesiredHandPose );
      index_grasp = obstaclesMsg_local.geometries[0].id;
      obstaclesMsg_local.geometries.erase(obstaclesMsg_local.geometries.begin());
      obstacles_publisher_right.publish(obstaclesMsg_local);

    }
    pub_aux_obstacles.publish(obstaclesMsg);
    plotObstacles(obstaclesMsg, index_grasp);

  }
  finish = false;
  ROS_INFO_STREAM((graspable_object_exist ? "There is" : "There is not") << " a graspable object" );
}


void  HandPoseGenerator::DesperateDemo1(std::vector<desperate_housewife::fittedGeometriesSingle> cyl_)
{
  ROS_INFO("*** DEMO1, take first graspable object without obstacles avoidance ***");

  desperate_housewife::handPoseSingle DesiredHandPose;
  desperate_housewife::fittedGeometriesArray obstaclesMsg;
  failed = false;
  for (unsigned int k = 0; k < cyl_.size(); k++)
  {
    obstaclesMsg.geometries.push_back( cyl_[k] );
  }

  DesiredHandPose = generateHandPose( cyl_[0], cyl_[0].id );
  ObjorObst = 0;
  if (!isGeometryGraspable( cyl_[0] ))
  {
    ROS_INFO("Object to Reject");
    DesiredHandPose.pose = ObstacleReject(cyl_[0], DesiredHandPose.whichArm);
    ObjorObst = 1;
  }

  data.arm_to_use = DesiredHandPose.whichArm;
  DesiredHandPose.id = id_class;
  finish = false;
  if (DesiredHandPose.whichArm == 1)
  {
    desired_hand_publisher_left.publish(DesiredHandPose);
  }
  else
  {
    desired_hand_publisher_right.publish(DesiredHandPose);
  }


  plotObstacles(obstaclesMsg, cyl_[0].id);


}


void HandPoseGenerator::plotObstacles( desperate_housewife::fittedGeometriesArray Obstacles, int index_grasp )
{
  visualization_msgs::MarkerArray marker_total;
  std::string obst_name;

  for (unsigned int i = 0; i < Obstacles.geometries.size(); ++i)
  {
    visualization_msgs::Marker marker_local;

    marker_local.header.frame_id = "world";
    marker_local.header.stamp = ros::Time();
    marker_local.ns = "";
    marker_local.id = Obstacles.geometries[i].id;
    marker_local.type = visualization_msgs::Marker::CYLINDER;
    marker_local.action = visualization_msgs::Marker::ADD;

    marker_local.pose.position = Obstacles.geometries[i].pose.position;
    marker_local.pose.orientation = Obstacles.geometries[i].pose.orientation;

    marker_local.scale.x = Obstacles.geometries[i].info[0] * 2;
    marker_local.scale.y = Obstacles.geometries[i].info[0] * 2;
    marker_local.scale.z = Obstacles.geometries[i].info[1];

    if (Obstacles.geometries[i].id == index_grasp )
    {
      marker_local.color.r = 0.0;
      marker_local.color.g = 1.0;
      marker_local.color.b = 0.0;
    }
    else
    {
      marker_local.color.r = 1.0;
      marker_local.color.g = 0.0;
      marker_local.color.b = 0.0;
    }

    marker_local.color.a = 1.0; // for the clearness
    // obst_name = "obstacle_" + std::to_string(i);
    marker_local.lifetime = ros::Duration(100);
    tf::Transform tfGeomTRansform;
    tf::poseMsgToTF(Obstacles.geometries[i].pose, tfGeomTRansform );
    tf_geometriesTransformations_.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "world", "obstacle_" + std::to_string(Obstacles.geometries[i].id) ) );
    marker_total.markers.push_back(marker_local);

  }


  marker_publisher_.publish(marker_total);

}

void HandPoseGenerator::resetCallBack(const std_msgs::Bool::ConstPtr msg)
{
  ROS_INFO("HandPoseGenerator::Reset called");
  home_reset = msg->data;
  finish = true;
  failed = false;
  // return true;
}


void HandPoseGenerator::reset()
{
  home_reset = false;
  finish = false;
  failed = false;
}





// bool HandPoseGenerator::resetCallBack(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
// {
//   ROS_INFO("HandPoseGenerator::Reset called");
//   home_reset = true;
//   finish = true;
//   failed = false;
//   return true;
// }
