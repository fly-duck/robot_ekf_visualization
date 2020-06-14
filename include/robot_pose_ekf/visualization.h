#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <base_local_planner/goal_functions.h>
#include <visualization_msgs/Marker.h>
#include <robot_pose_ekf/pose2d.h>
#include <robot_pose_ekf/robot_model.h>
#include <robot_pose_ekf/odomsub.h>
//#include <nav_2d_msgs/Polygon2D.h>


namespace local_planner
{

class Visualization
{


public:
    explicit Visualization(ros::NodeHandle & nh )
    {

            marker_pub_=nh.advertise<visualization_msgs::Marker>("footprintmarker",1000);

    }
    //got configuration
    //Todo: implement the initialize when you got configuration
   // void initialize (ros::NodeHandle & nh  )

    void PublishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& localplan) const
    {

        base_local_planner::publishPlan(localplan,localpub_);

    }

    void PublishPlanandPose();
    void PublishFootprint(const Pose2D& current_pose, const BaseRobotFootprintModel & robotmodel , const std::string & ns    )
    {

        std::cout<< "current pose " << current_pose.GetPos2D()<<"ns "<<ns<<std::endl;
        std::vector<visualization_msgs::Marker> markers;
        robotmodel.visualizeRobot(current_pose,markers);
        if(markers.empty()  )
        {
            std::cout<< " can not  visualize the markers"<<'\n';
            return ;
        }
        int idx =0;
        std::string map_frame="odom_combined";
        for(std::vector<visualization_msgs::Marker>::iterator marker_it=markers.begin(); marker_it
            != markers.end() ; marker_it++,idx++)
        {

            marker_it->header.frame_id =map_frame;
            marker_it->header.stamp = ros::Time::now();
            marker_it->action = visualization_msgs::Marker::ADD;
            //# Namespace to place this object in... used in conjunction with id to create a unique name for the object
            marker_it->ns = ns;
            marker_it->id = idx;
            marker_it->lifetime = ros::Duration(2.0);
            marker_pub_.publish(*marker_it);
        }



    }

 protected:
    ros::Publisher localpub_;
    ros::Publisher marker_pub_;
};



}



#endif // UTILS_H
