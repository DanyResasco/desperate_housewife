#include <ros/node_handle.h>
#include <ros/ros.h>
#include <Eigen/Core> 

Eigen::MatrixXd getGainMatrix(std::string parameter, ros::NodeHandle n, int dimension = 6);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sendObj_obst_test");

  ros::NodeHandle n;

  XmlRpc::XmlRpcValue my_list;
  n.getParam("links_with_potential_field", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for ( int i = 0; i < my_list.size(); ++i)
    {
      ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      ROS_INFO("%s", static_cast<std::string>(my_list[i]).c_str());
      
    }

  // Eigen::Matrix<double, 6, 6> KP = Eigen::Matrix<double, 6, 6>::Zero();
  // // XmlRpc::XmlRpcValue my_list;
  // n.getParam("k_p", my_list);
  // ROS_INFO("%d", my_list.getType());
  // ROS_INFO("%d", my_list.size());
  // ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // for (int i = 0; i < std::max(my_list.size(), 6); ++i)
  // {
  //   KP(i,i) = static_cast<double>(my_list[i]);
  //   ROS_INFO("%f", static_cast<double>(my_list[i]));
  // }

  Eigen::Matrix<double, 6, 6> KP = getGainMatrix(std::string("k_p"), n);
  std::cout << KP << std::endl;


  return 0;
}

Eigen::MatrixXd getGainMatrix(std::string parameter, ros::NodeHandle n, int dimension)
{
  XmlRpc::XmlRpcValue my_list;
  n.getParam(parameter.c_str(), my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  Eigen::MatrixXd K(dimension, dimension);
  K = Eigen::MatrixXd::Zero(dimension, dimension);
  for (int i = 0; i < std::max(my_list.size(), dimension); ++i)
    {
      K(i,i) = static_cast<double>(my_list[i]);
    }
  return K;
}
