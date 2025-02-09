#include <euclid_planner/euclid_planner.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::EuclidPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

// Default Constructor
namespace global_planner
{
EuclidPlanner::EuclidPlanner()
{
}

EuclidPlanner::EuclidPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

void EuclidPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  ros::NodeHandle private_nh("~/" + name);
  this->plan_pub_ = private_nh.advertise<nav_msgs::Path>("euclid_plan", 1);
}

bool EuclidPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
  
  plan.push_back(start);
  float dist;
  dist = sqrt(start.pose.position.x * start.pose.position.x + start.pose.position.y * start.pose.position.y);
  geometry_msgs::PoseStamped new_goal;
  new_goal.pose.position.x  = start.pose.position.x ;
  new_goal.pose.position.y  = start.pose.position.y ;
  float yaw = atan2(goal.pose.position.y-start.pose.position.y,goal.pose.position.x-start.pose.position.x);
  tf::Quaternion goal_quat = tf::createQuaternionFromYaw(yaw);
  new_goal.pose.orientation.x = goal_quat.x();
  new_goal.pose.orientation.y = goal_quat.y();
  new_goal.pose.orientation.z = goal_quat.z();
  new_goal.pose.orientation.w = goal_quat.w();
  for (int i = 0; dist > (i * dist / 0.2); i++)
  {
    new_goal.pose.position.x += 0.2* cos(yaw);
    new_goal.pose.position.y += 0.2* sin(yaw);

    plan.push_back(new_goal);
  }
  plan.push_back(goal);
  nav_msgs::Path new_path;
  new_path.poses = plan;
  new_path.header.frame_id = "map";
  plan_pub_.publish(new_path);
  return true;
}
};  // namespace global_planner