#include <ros/ros.h>
#include <keyboard/Key.h>
#include <std_msgs/Bool.h>
#include <controller_manager_msgs/SwitchController.h>


uint key_pressed;

void getKeyCallback(const keyboard::Key::ConstPtr& msg)
{
  key_pressed = msg->code;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GetCommandsKeyboard");
  ros::NodeHandle node;

  ROS_INFO("[GetCommandsKeyboard] Node is ready");

  double spin_rate = 100;
  ros::param::get("~spin_rate", spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);

  key_pressed = keyboard::Key::KEY_UNKNOWN;

  ros::Subscriber sub_keyboard = node.subscribe("keyboard/keydown", 10, getKeyCallback);

  std_msgs::Bool bool_msg;
  ros::Publisher pub_reset = node.advertise<std_msgs::Bool>("/reset", 10);
  ros::Publisher pub_start_robots = node.advertise<std_msgs::Bool>("/desperate_housewife/start_controller", 10);

  std::string control_topic_right, control_topic_left;
  node.param<std::string>("/right_arm/controller", control_topic_right, "PotentialFieldControlKinematicReverse");
  node.param<std::string>("/left_arm/controller", control_topic_left, "PotentialFieldControlKinematicReverse");


  bool right_arm_enabled, left_arm_enabled;


  ros::Rate rate(spin_rate);

  while (node.ok())
  {

    node.param<bool>("/right_arm/enabled", right_arm_enabled, "false");
    node.param<bool>("/left_arm/enabled", left_arm_enabled, "false");

    ros::spinOnce();

    switch (key_pressed) {

    case keyboard::Key::KEY_r: // Reset
      bool_msg.data = true;
      pub_reset.publish(bool_msg);
      ROS_INFO_STREAM("Reseting System");
      break;
    case keyboard::Key::KEY_g: //Start Robots
      bool_msg.data = true;
      pub_start_robots.publish(bool_msg);
      ROS_INFO_STREAM("Starting Robots");
      break;
    case keyboard::Key::KEY_s: //Stop Robots
      bool_msg.data = false;
      pub_start_robots.publish(bool_msg);
      ROS_INFO_STREAM("Stopping Robots");
      break;
    case keyboard::Key::KEY_i: //Stop Robots
      ROS_INFO_STREAM("Initializing Controllers");
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.start_controllers.push_back(control_topic_right);
      switch_srv.request.stop_controllers.push_back("joint_trajectory_controller");
      switch_srv.request.strictness = 2;

      ros::ServiceClient client = node.serviceClient<controller_manager_msgs::SwitchController>("/right_arm/controller_manager/switch_controller");
      if (right_arm_enabled)
      {
        if (!client.call(switch_srv))
        {
          ROS_ERROR_STREAM("Not possible to switch right_arm control to: " << control_topic_right.c_str());
        }
        else
        {
          if (switch_srv.response.ok == true)
          {
            ROS_INFO_STREAM("right_arm control switched to: " << control_topic_right.c_str());
          }
          else
          {
            ROS_ERROR_STREAM("Not possible to switch right_arm control to: " << control_topic_right.c_str());
          }
        }
      }
      switch_srv.request.start_controllers.push_back(control_topic_left);
      client = node.serviceClient<controller_manager_msgs::SwitchController>("/left_arm/controller_manager/switch_controller");
      if (left_arm_enabled)
      {
        if (!client.call(switch_srv))
        {
          ROS_ERROR_STREAM("Not possible to switch right_arm control to: " << control_topic_right.c_str());
        }
        else
        {
          if (switch_srv.response.ok == true)
          {
            ROS_INFO_STREAM("left_arm control switched to: " << control_topic_right.c_str());
          }
          else
          {
            ROS_ERROR_STREAM("Not possible to switch left_arm control to: " << control_topic_right.c_str());
          }
        }
      }
      break;
    }
    key_pressed = keyboard::Key::KEY_UNKNOWN;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
