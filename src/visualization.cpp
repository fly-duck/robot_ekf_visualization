#include <ros/ros.h>
// #include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <memory>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <robot_pose_ekf/robot_model.h>
#include <robot_pose_ekf/visualization.h>
#include <robot_pose_ekf/path.h>
//#include <robot_pose_ekf/occupancy_grid.h>


//rerun rebuild in qt to include msg files;
//#include <robot_pose_ekf/msg_demo.h>

//#include <robot_pose_ekf/PersonArray.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "visualization");
    ros::NodeHandle nh;
//    std::string odom_topic ="odom";
//    std::string topic ="base_scan";
    std::string topic ="scan";
    std::string combined_frame;
    std::string fusion_topic;
    nh.param("output_frame", combined_frame, std::string("odom_combined"));
    nh.param("fusion_topic", fusion_topic, std::string("/robot_pose_ekf/odom_combined"));
    std::string odom_topic ="/base_odometry/odom";
   // local_planner::LaserSub *lasersub =new local_planner::LaserSub(nh,topic);
    //trying without topic to initialize odomsub
    // auto lasersub =std::make_unique<local_planner::LaserSub>(nh,topic);
    local_planner::OdomSub *odom_sub =new local_planner::OdomSub(nh,odom_topic );

    std::string funsion_topic;
    
    local_planner::FusionSub* fusion_sub= new local_planner::FusionSub(nh,fusion_topic);
    //sleep(20.0);
     local_planner::Visualization visualization(nh) ;
     nav_msgs::Path path;
     nav_msgs::Path fusion_path;
     ros::Publisher path_publisher;
     ros::Publisher combined_path_publisher;

     ros::Publisher local_path_publisher;
    //  local_path_publisher=nh.advertise<nav_msgs::Path>("local_trajectory",1,true);
     path_publisher=nh.advertise<nav_msgs::Path>("trajectory",1,true);
     combined_path_publisher=nh.advertise<nav_msgs::Path>("fusion_trajectory",1,true);



//    local_planner::GeneralPublisher  nav_pub(nh);
//    nav_pub.PublisherOccgrid();
//    nav_pub.init();


    ros::Rate loop_rate(20.0f);
    std::vector<local_planner::Pose2D> poses;
    while(ros::ok())
    {
    local_planner::FormatingPathmsg(path,odom_sub,combined_frame);
    path_publisher.publish(path);
    local_planner::FormatingCombinedPathmsg(fusion_path,fusion_sub,combined_frame);
    combined_path_publisher.publish(fusion_path);
    local_planner::Pose2D pose2d(odom_sub->GetPose());
    poses.emplace_back(pose2d);
    local_planner::CircularRobotFootprint robotmodel(0.3);
//    const std::string globle_ns="base_scan";
    const std::string globle_ns="scan";
    //vector::at prevent out of range
    visualization.PublishFootprint(poses.at(0),robotmodel,globle_ns);
    poses.pop_back();
    ros::spinOnce();
    loop_rate.sleep();

    }
//    ros::Publisher cloudpub;
//    cloudpub=nh.advertise<sensor_msgs::PointCloud>("pointcloud",1);
//    for(const auto point : lasersub->cloud.points)
//    {
//        std::cout<<point<<std::endl;
//    }
//    cloudpub.publish(lasersub->cloud);

//    ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

//    PointCloud::Ptr msg (new PointCloud);
//    msg->header.frame_id = "base_link";
//    msg->height = msg->width = 1;
//    msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

//    ros::Rate loop_rate(4);
//    while (nh.ok())
//    {
//      pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
//      pub.publish (msg);
//      ros::spinOnce ();
//      loop_rate.sleep ();
//    }
    //ros::Rate rate(20.0f);
    /*
    while(ros::ok())
    {
    cloudpub.publish(lasersub->cloud);
    ros::spinOnce();
    }
    */
    //ros::spin();

    //ROS_INFO("Hello world!");
}

