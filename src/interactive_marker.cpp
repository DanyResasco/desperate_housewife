
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "desperate_housewife/handPoseSingle.h"

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
bool enable_tracking;
ros::Publisher pub_reference;
interactive_markers::MenuHandler menu_handler;

void enabletrackingArm(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  interactive_markers::MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  interactive_markers::MenuHandler::CheckState state;
  menu_handler.getCheckState( handle, state );
  if ( state == interactive_markers::MenuHandler::CHECKED )
  {
    menu_handler.setCheckState( handle, interactive_markers::MenuHandler::UNCHECKED );
    enable_tracking = false;
  }
  else
  {
    menu_handler.setCheckState( handle, interactive_markers::MenuHandler::CHECKED );
    enable_tracking = true;
  }
  menu_handler.reApply( *server );
  server->applyChanges();

}


void moveArm(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  desperate_housewife::handPoseSingle HandPose;
  HandPose.pose = feedback->pose;
  pub_reference.publish(HandPose);
}


void trackArm(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (enable_tracking)
  {
    moveArm(feedback);
  }
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "DesiredPoseInteractiveMarker");

  ros::NodeHandle node;
  std::string control_topic;
  node.param<std::string>("controller", control_topic, "PotentialFieldControlKinematicReverse");

  pub_reference = node.advertise<desperate_housewife::handPoseSingle>(control_topic + "/command", 10);

  server.reset( new interactive_markers::InteractiveMarkerServer("ArmMenu", "", false) );

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/world";
  // int_marker.header.stamp = ros::Time::now();
  int_marker.name = "end_effector";

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  enable_tracking = false;
  visualization_msgs::InteractiveMarkerControl control;

  // control.markers.push_back(marker);

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);


  server->insert(int_marker, &trackArm);

  menu_handler.setCheckState( menu_handler.insert( "track Arm", &enabletrackingArm ), interactive_markers::MenuHandler::UNCHECKED );
  menu_handler.insert( "move Arm", &moveArm );
  menu_handler.apply( *server, "right" );
  

  server->applyChanges();
  ros::spin();
  server.reset();
}



