
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>

using std::string;

#ifndef EUCLID_PLANNER_CPP
#define EUCLID_PLANNER_CPP

namespace global_planner
{
class EuclidPlanner : public nav_core::BaseGlobalPlanner
{
public:
  EuclidPlanner();
  EuclidPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
  bool Is_Area_Free(double wx, double wy, costmap_2d::Costmap2D* costmap);
  void Bounds_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  bool Is_Inside_Rectangle(double x, double y, double xmin, double xmax, double ymin, double ymax);
  void Calculate_Rectangle_Bounds(const std::vector<geometry_msgs::Point32>& points, 
                              double& xmin, double& xmax, double& ymin, double& ymax);
  void Visualize_Markers(geometry_msgs::Point p1,
                        geometry_msgs::Point p2,
                        geometry_msgs::Point p3,
                        geometry_msgs::Point p4,
                        costmap_2d::Costmap2D* costmap);

void Scan_Area(geometry_msgs::Point p1, 
               geometry_msgs::Point p2,
               geometry_msgs::Point p3,
               geometry_msgs::Point p4,
               costmap_2d::Costmap2D* costmap,
               std::vector<geometry_msgs::PoseStamped>& plan);

void Add_Point_To_Plan(unsigned int mx, unsigned int my,
                      costmap_2d::Costmap2D* costmap,
                      std::vector<geometry_msgs::PoseStamped>& plan);

void Calculate_Grid_Bounds(costmap_2d::Costmap2D* costmap,
                           geometry_msgs::Point p1,
                           geometry_msgs::Point p2,
                           geometry_msgs::Point p3,
                           geometry_msgs::Point p4,
                           unsigned int& start_x, unsigned int& start_y,
                           unsigned int& end_x, unsigned int& end_y);

private:
  ros::Publisher plan_pub_;
  ros::Publisher marker_pub_;
  ros::Subscriber bounds_sub_;
  geometry_msgs::Polygon polygon_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  double xmin_, xmax_, ymin_, ymax_;
  bool bounds_received_;
};
};  // namespace global_planner
#endif