#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <robot_pose_ekf/pose2d.h>

namespace local_planner {
class BaseRobotFootprintModel
{
public:
    BaseRobotFootprintModel()
    {
    }

    /**
     * @brief Virtual destructor.
     */
    virtual ~BaseRobotFootprintModel()
    {
    }


    /**
      * @brief Calculate the distance between the robot and an obstacle
      * @param current_pose Current robot pose
      * @param obstacle Pointer to the obstacle
      * @return Euclidean distance to the robot
      */
    //virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const = 0;

    /**
      * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
      * @param current_pose robot pose, from which the distance to the obstacle is estimated
      * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
      * @param t time, for which the predicted distance to the obstacle is calculated
      * @return Euclidean distance to the robot
      */
//    virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const = 0;

    /**
      * @brief Visualize the robot using a markers
      *
      * Fill a marker message with all necessary information (type, pose, scale and color).
      * The header, namespace, id and marker lifetime will be overwritten.
      * @param current_pose Current robot pose
      * @param[out] markers container of marker messages describing the robot shape
      */
    virtual void visualizeRobot(const Pose2D& current_pose, std::vector<visualization_msgs::Marker>& markers ) const {}


    /**
     * @brief Compute the inscribed radius of the footprint model
     * @return inscribed radius
     */
    virtual double getInscribedRadius() = 0;



public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};






class CircularRobotFootprint
        : public BaseRobotFootprintModel


{
public:

  /**
    * @brief Default constructor of the abstract obstacle class
    * @param radius radius of the robot
    */
  CircularRobotFootprint(double radius) : radius_(radius) { }

  /**
   * @brief Virtual destructor.
   */
  virtual ~CircularRobotFootprint() { }

  /**
    * @brief Set radius of the circular robot
    * @param radius radius of the robot
    */
  void setRadius(double radius) {radius_ = radius;}
  virtual void visualizeRobot(const Pose2D& current_pose, std::vector<visualization_msgs::Marker>& markers ) const
  {
    markers.resize(1);
    visualization_msgs::Marker& marker = markers.back();
    marker.type = visualization_msgs::Marker::CYLINDER;

    //translate current pose to marker 's pose
    current_pose.toPoseMsg(marker.pose);
    marker.scale.x = marker.scale.y = 2*radius_; // scale = diameter
    marker.scale.z = 0.05;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.0;
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() {return radius_;}

private:

  double radius_;
};
}
#endif // ROBOT_MODEL_H
