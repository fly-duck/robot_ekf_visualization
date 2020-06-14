#include <ros/ros.h>
#include <robot_pose_ekf/Kalman.h>





int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "fusion_demo");
  


  Fusion::Kalman kf_filter;

  // create filter class
//   Kalman_Filter  

  ros::spin();
  
  return 0;
}