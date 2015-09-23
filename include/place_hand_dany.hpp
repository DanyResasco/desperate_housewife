#include <hand_pose_generator.h>


geometry_msgs::Pose HandPoseGenerator::placeHand ( desperate_housewife::fittedGeometriesSingle geometry )
{

  
  geometry_msgs::Pose pose_local = geometry.pose;
  return pose_local;

}
