#ifndef PATH_H
#define PATH_H
#include <nav_msgs/Path.h>
#include <robot_pose_ekf/odomsub.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace  local_planner  {

        void FormatingPathmsg(  nav_msgs::Path & path ,OdomSub * ptr,const std::string&odom_frame)
        {
                const std::string frame =odom_frame;
                 path.header.stamp=ros::Time::now();
                 path.header.frame_id=frame;
                  geometry_msgs::PoseStamped currentpose ;
               currentpose.pose.position.x = ptr->GetPose().x;
               currentpose.pose.position.y = ptr->GetPose().y;

               geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(ptr->GetPose().theta);
               currentpose.pose.orientation.x = goal_quat.x;
               currentpose.pose.orientation.y = goal_quat.y;
               currentpose.pose.orientation.z = goal_quat.z;
               currentpose.pose.orientation.w = goal_quat.w;
               currentpose.header.stamp=ros::Time::now();
               currentpose.header.frame_id=frame;
               path.poses.emplace_back(currentpose);


        }

        void FormatingCombinedPathmsg(  nav_msgs::Path & path ,FusionSub* ptr,const std::string&odom_frame)

        {
                const std::string frame =odom_frame;
                 path.header.stamp=ros::Time::now();
                 path.header.frame_id=frame;
                  geometry_msgs::PoseStamped currentpose ;
               currentpose.pose.position.x = ptr->GetPose().x;
               currentpose.pose.position.y = ptr->GetPose().y;

               geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(ptr->GetPose().theta);
               currentpose.pose.orientation.x = goal_quat.x;
               currentpose.pose.orientation.y = goal_quat.y;
               currentpose.pose.orientation.z = goal_quat.z;
               currentpose.pose.orientation.w = goal_quat.w;
               currentpose.header.stamp=ros::Time::now();
               currentpose.header.frame_id=frame;
               path.poses.emplace_back(currentpose);



        }
        void FormatingLocalPathmsg(  nav_msgs::Path & path ,OdomSub * ptr,const std::string&odom_frame)
        {

            const std::string frame =odom_frame;
             path.header.stamp=ros::Time::now();
             path.header.frame_id=frame;
              geometry_msgs::PoseStamped currentpose ;
           currentpose.pose.position.x = ptr->GetPose().x;
           currentpose.pose.position.y = ptr->GetPose().y;

           geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(ptr->GetPose().theta);
           currentpose.pose.orientation.x = goal_quat.x;
           currentpose.pose.orientation.y = goal_quat.y;
           currentpose.pose.orientation.z = goal_quat.z;
           currentpose.pose.orientation.w = goal_quat.w;
           currentpose.header.stamp=ros::Time::now();
           currentpose.header.frame_id=frame;
           path.poses.emplace_back(currentpose);

        }

        class PathBuilder
        {




        };



}
#endif // PATH_H
