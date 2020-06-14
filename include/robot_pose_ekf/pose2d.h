#ifndef POSE2D_H
#define POSE2D_H

#include <eigen3/Eigen/src/Core/CoreEvaluators.h>
#include <geometry_msgs/Pose.h>
#include<geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

namespace local_planner {

class Pose2D
{
public:
  Pose2D(const geometry_msgs::Pose& pose)
  {
        position_.coeffRef(0)=pose.position.x;
        position_.coeffRef(1)=pose.position.y;
        theta_=tf::getYaw(pose.orientation);
  }
  Pose2D(const geometry_msgs::Pose2D& pose)
  {

      position_.coeffRef(0)=pose.x;
      position_.coeffRef(1)=pose.y;
      theta_=pose.theta;


  }

   inline Eigen::Vector2d GetPos2D() const // compiler will complain for not const
   {
       return position_;
   }
  void toPoseMsg(geometry_msgs::Pose& pose) const
  {
    pose.position.x = position_.x();
    pose.position.y = position_.y();
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
  }

private:
  Eigen::Vector2d position_;
  double theta_;


};


}
#endif // POSE2D_H
