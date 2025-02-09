
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

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
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

private:
  ros::Publisher plan_pub_;
};
};  // namespace global_planner
#endif