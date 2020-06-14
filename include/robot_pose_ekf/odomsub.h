#ifndef ODOMSUB_H
#define ODOMSUB_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <robot_pose_ekf/time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <robot_pose_ekf/mathutils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
namespace local_planner {

class OdomSub
{
public:

explicit OdomSub(ros::NodeHandle & nh ,std::string default_topic = "odom")
    {
        std::string odom_topic;
          nh.param("odom_topic", odom_topic, default_topic);
          odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&OdomSub::odomCallback, this, _1));


    }

    inline geometry_msgs::Pose2D  GetPose()
    {
        return pose_;
    }

    
//
protected:

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
      {
          //std::put_time(std::localtime(&now_c), "%F %T")
      //  const common::Time time=  FromRos(msg->header.stamp);

       // std::cout<<"TIME"<<std::put_time(std::localtime(FromRos(msg->header.stamp)), "%F %T")<<std::endl;

        ROS_INFO_ONCE("odom received!");
        boost::mutex::scoped_lock lock(odom_mutex_);
       // odom_vel_.header = msg->header;
       // odom_vel_.velocity = twist3Dto2D(msg->twist.twist);
        pose_.x=msg->pose.pose.position.x;
        pose_.y=msg->pose.pose.position.y;
        pose_.theta=tf::getYaw(msg->pose.pose.orientation);

}


    ros::Subscriber odom_sub_;
    boost::mutex odom_mutex_;
    geometry_msgs::Pose2D pose_;
};



class FusionSub
{
public:

explicit FusionSub(ros::NodeHandle & nh ,std::string default_topic = "odom")
    {
        std::string odom_topic;
          nh.param("odom_topic", odom_topic, default_topic);
          odom_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(odom_topic, 1, boost::bind(&FusionSub::FusionCallback, this, _1));


    }

        inline geometry_msgs::Pose2D  GetPose()
    {
        return pose_;
    }



//
protected:

    void FusionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
      {
          //std::put_time(std::localtime(&now_c), "%F %T")
      //  const common::Time time=  FromRos(msg->header.stamp);

       // std::cout<<"TIME"<<std::put_time(std::localtime(FromRos(msg->header.stamp)), "%F %T")<<std::endl;

        ROS_INFO_ONCE("odom received!");
        boost::mutex::scoped_lock lock(odom_mutex_);
       // odom_vel_.header = msg->header;
       // odom_vel_.velocity = twist3Dto2D(msg->twist.twist);
        pose_.x=msg->pose.pose.position.x;
        pose_.y=msg->pose.pose.position.y;
        pose_.theta=tf::getYaw(msg->pose.pose.orientation);

        msg_=*msg;
}

    geometry_msgs::PoseWithCovarianceStamped msg_;
    ros::Subscriber odom_sub_;
    boost::mutex odom_mutex_;
    geometry_msgs::Pose2D pose_;

};

class LaserSub
{
  public:
 explicit   LaserSub(ros::NodeHandle &nh , std::string default_topic ="/base_scan")

    {
        std::string scan_topic ;
        nh.param("scan_topic",scan_topic ,default_topic);// "scan_topic " can not have space " " it will throw an exception
        lasersub=nh.subscribe<sensor_msgs::LaserScan> (scan_topic,1,boost::bind(&LaserSub::Scancallback2,this,_1));
     //   tf::TransformListener listener_;
        point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
         //listener_.setExtrapolationLimit(ros::Duration(0.1));
    }
    void Scancallback(const sensor_msgs::LaserScan::ConstPtr & scan_in  )
    {
        std::cout<<"fov"<<scan_in->angle_max-scan_in->angle_min<<"\n";
        std::cout<<"increment"<<scan_in->angle_increment<<"\n";

        if(!listener_.waitForTransform(
                scan_in->header.frame_id,
                "/base_link",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0))){
             return;
          }
//        for(const auto & range :scan_in->ranges)
//        {
//            std::cout<<range<<std::endl;
//        }
       // std::cout<<scan_in->time_increment<<std::endl;
       // std::cout<<scan_in->scan_time<<std::endl;
        //projector_.transformLaserScanToPointCloud("/base_link",*scan_in,cloud,listener_);

    }
    //laser_geometry::LaserProjection projector_;

    void Scancallback2 (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {

        std::cout<<"fov"<<common::RadToDeg(scan_in->angle_max-scan_in->angle_min)<<"\n";
        std::cout<<"increment"<<common::RadToDeg(scan_in->angle_increment)<<"\n";
        std::cout<<"ranges"<<scan_in->ranges.size()<<"\n";
       sensor_msgs::PointCloud2 cloud;
//        projector_.transformLaserScanToPointCloud("base_laser_link", *scan, cloud, listener_);

        if(!listener_.waitForTransform(
                scan_in->header.frame_id,
                "/base_link",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0))){
             return;
          }
        ROS_INFO_STREAM("RECEIVED  LASERSCAN");
        projector_.transformLaserScanToPointCloud("/base_link",*scan_in,cloud,listener_);
        point_cloud_publisher_.publish(cloud);

     // projector_.projectLaser(*scan_in, cloud);

//      for(const auto point : cloud.points)
//      {
//          std::cout<<point<<std::endl;
//      }
      // Do something with cloud.
    }

private :
    laser_geometry::LaserProjection projector_;
    tf::TransformListener  listener_;
   //sensor_msgs::PointCloud cloud ;
    ros::Publisher point_cloud_publisher_;
    ros::Subscriber  lasersub;
};

class GeneralPublisher
{
public:
   explicit  GeneralPublisher(ros::NodeHandle& nh)
        :nh_(nh)
    {

    }
    void PublisherOccgrid ()
    {
        occ_pub_=nh_.advertise<nav_msgs::OccupancyGrid>("map",1);
    }
    void init()
    {
        nav_msgs::OccupancyGrid info;
        info.header.frame_id="odom";
        ros::Rate rate(20);
        while(ros::ok())
        {
        occ_pub_.publish(info);
        rate.sleep();
        }


    }

private:
    ros::Publisher occ_pub_;
    ros::NodeHandle nh_;
};



}





#endif // ODOMSUB_H
