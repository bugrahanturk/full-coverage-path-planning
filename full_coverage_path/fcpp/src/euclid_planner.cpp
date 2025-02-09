#include <euclid_planner/euclid_planner.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>

// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::EuclidPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner
{

EuclidPlanner::EuclidPlanner() : bounds_received_(false)
{
}

EuclidPlanner::EuclidPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}

void EuclidPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle private_nh("~/" + name);
    this->costmap_ros_ = costmap_ros;
    this->plan_pub_ = private_nh.advertise<nav_msgs::Path>("euclid_plan", 1);
    this->marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    this->bounds_sub_ = private_nh.subscribe("/rectangle_points", 1, &EuclidPlanner::Bounds_Callback, this);
    ROS_INFO("EuclidPlanner initialized.");
}

void EuclidPlanner::Bounds_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() == 8) // 4 noktadan oluşan dikdörtgen için
    {
        std::vector<geometry_msgs::Point32> points;
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            geometry_msgs::Point32 point;
            point.x = msg->data[i];
            point.y = msg->data[i + 1];
            points.push_back(point);
        }

        // Dikdörtgen sınırlarını hesapla
        Calculate_Rectangle_Bounds(points, xmin_, xmax_, ymin_, ymax_);
        bounds_received_ = true;
        ROS_INFO("Rectangle bounds calculated: xmin=%f, xmax=%f, ymin=%f, ymax=%f", xmin_, xmax_, ymin_, ymax_);
    }
    else
    {
        ROS_WARN("Invalid rectangle points received. Expected 4 points (8 values). Received %lu values.", msg->data.size());
    }
}

void EuclidPlanner::Visualize_Markers(geometry_msgs::Point p1,
                                      geometry_msgs::Point p2,
                                      geometry_msgs::Point p3,
                                      geometry_msgs::Point p4,
                                      costmap_2d::Costmap2D* costmap)
{
    // Costmapdeki engel ve engel olmayan alanlari gorsellesitrmek icin
    // Dikdörtgen sınırlarını görselleştir
    visualization_msgs::Marker rectangle_marker;
    rectangle_marker.header.frame_id = "map";
    rectangle_marker.header.stamp = ros::Time::now();
    rectangle_marker.ns = "rectangle";
    rectangle_marker.id = 0;
    rectangle_marker.type = visualization_msgs::Marker::LINE_STRIP;
    rectangle_marker.action = visualization_msgs::Marker::ADD;
    rectangle_marker.scale.x = 0.1; // Çizgi kalınlığı
    rectangle_marker.color.r = 1.0;
    rectangle_marker.color.g = 0.0;
    rectangle_marker.color.b = 0.0;
    rectangle_marker.color.a = 1.0;

    rectangle_marker.points = {p1, p2, p3, p4, p1}; // Dikdörtgeni kapat
    marker_pub_.publish(rectangle_marker);

    // Grid hücrelerini görselleştir
    double resolution = costmap->getResolution();
    unsigned int size_x = costmap->getSizeInCellsX();
    unsigned int size_y = costmap->getSizeInCellsY();

    for (unsigned int y = 0; y < size_y; y++)
    {
        for (unsigned int x = 0; x < size_x; x++)
        {
            double wx, wy;
            costmap->mapToWorld(x, y, wx, wy);

            if (Is_Inside_Rectangle(wx, wy, p1.x, p2.x, p1.y, p4.y))
            {
                visualization_msgs::Marker cell_marker;
                cell_marker.header.frame_id = "map";
                cell_marker.header.stamp = ros::Time::now();
                cell_marker.ns = "grid";
                cell_marker.id = y * size_x + x;
                cell_marker.type = visualization_msgs::Marker::CUBE;
                cell_marker.action = visualization_msgs::Marker::ADD;
                cell_marker.scale.x = resolution;
                cell_marker.scale.y = resolution;
                cell_marker.scale.z = 0.01; // Hücre yüksekliği
                cell_marker.pose.position.x = wx;
                cell_marker.pose.position.y = wy;
                cell_marker.pose.position.z = 0;

                if (Is_Area_Free(wx, wy, costmap))
                {
                    cell_marker.color.r = 0.0;
                    cell_marker.color.g = 1.0; // Geçilebilir alan: yeşil
                    cell_marker.color.b = 0.0;
                }
                else
                {
                    cell_marker.color.r = 1.0;
                    cell_marker.color.g = 0.0; // Engelli alan: kırmızı
                    cell_marker.color.b = 0.0;
                }
                cell_marker.color.a = 0.6; // Şeffaflık
                marker_pub_.publish(cell_marker);
            }
        }
    }
}

void EuclidPlanner::Add_Point_To_Plan(unsigned int mx, unsigned int my,
                                      costmap_2d::Costmap2D* costmap,
                                      std::vector<geometry_msgs::PoseStamped>& plan)
{
    // Engel degilse plana ekleyelim
    double wx, wy;
    costmap->mapToWorld(mx, my, wx, wy);

    if (Is_Area_Free(wx, wy, costmap))
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
    }
}

void EuclidPlanner::Calculate_Grid_Bounds(costmap_2d::Costmap2D* costmap,
                                          geometry_msgs::Point p1,
                                          geometry_msgs::Point p2,
                                          geometry_msgs::Point p3,
                                          geometry_msgs::Point p4,
                                          unsigned int& start_x, unsigned int& start_y,
                                          unsigned int& end_x, unsigned int& end_y)
{
    // Her köşe noktasının grid koordinatlarını hesapla
    unsigned int x1, y1, x2, y2, x3, y3, x4, y4;
    costmap->worldToMap(p1.x, p1.y, x1, y1);
    costmap->worldToMap(p2.x, p2.y, x2, y2);
    costmap->worldToMap(p3.x, p3.y, x3, y3);
    costmap->worldToMap(p4.x, p4.y, x4, y4);

    // Minimum ve maksimum sınırları belirle
    start_x = std::min({x1, x2, x3, x4});
    start_y = std::min({y1, y2, y3, y4});
    end_x = std::max({x1, x2, x3, x4});
    end_y = std::max({y1, y2, y3, y4});
}

void EuclidPlanner::Scan_Area(geometry_msgs::Point p1, 
                              geometry_msgs::Point p2,
                              geometry_msgs::Point p3,
                              geometry_msgs::Point p4,
                              costmap_2d::Costmap2D* costmap,
                              std::vector<geometry_msgs::PoseStamped>& plan)
{
    // Robot yarıçapı bir parametreden veya direkt sabit olarak tanimlanabilir.
    double robot_radius = 0.15; 
    double resolution   = costmap->getResolution();

    // Tarama yapılacak "grid step size"
    double diameter_in_cells = (2.0 * robot_radius) / resolution;
    unsigned int step_size   = static_cast<unsigned int>(std::ceil(diameter_in_cells));
    ROS_INFO("Tarama step_size: %u (robot_radius=%.2f, res=%.2f)", step_size, robot_radius, resolution);
    
    // Grid sınırlarını hesapla
    unsigned int start_x, start_y, end_x, end_y;
    Calculate_Grid_Bounds(costmap, p1, p2, p3, p4, start_x, start_y, end_x, end_y);
    
    // İlk yön sağa doğru
    bool left_to_right = true; 
    for (unsigned int y = start_y; y <= end_y; y += step_size)
    {
        if (left_to_right)
        {
            for (unsigned int x = start_x; x <= end_x; x += step_size)
            {
                Add_Point_To_Plan(x, y, costmap, plan);
            }
        }
        else
        {
            // Burada dikkat, x>=start_x koşulunu koruyacak şekilde step_size kullanırken 
            // unsigned int ve 0 durumu var. Min boundary'yi aştığımızda döngü bozulabilir.
            for (int x_i = static_cast<int>(end_x); x_i >= static_cast<int>(start_x); x_i -= step_size)
            {
                Add_Point_To_Plan(static_cast<unsigned int>(x_i), y, costmap, plan);
            }
        }
        left_to_right = !left_to_right; // Yön değiştir
    }
}

bool EuclidPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!bounds_received_)
    {
        ROS_WARN("Rectangle bounds not received yet!");
        return false;
    }

    // Costmap bilgileri
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (!costmap)
    {
        ROS_ERROR("Costmap is not initialized!");
        return false;
    }

    // Dikdörtgenin köşe noktalarını ekle
    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = xmin_; p1.y = ymin_;
    p2.x = xmax_; p2.y = ymin_;
    p3.x = xmax_; p3.y = ymax_;
    p4.x = xmin_; p4.y = ymax_;

    // P1 ve P2 aynı yatay hizada olmalı
    p2.y = p1.y;
    // P3 ve P4 aynı yatay hizada olmalı
    p4.y = p3.y;
    // P1 ve P4 aynı dikey hizada olmalı
    p4.x = p1.x;
    // P2 ve P3 aynı dikey hizada olmalı
    p3.x = p2.x;

    // Görselleştirme işlemi
    Visualize_Markers(p1, p2, p3, p4, costmap);
    
    // Alanı tarama
    Scan_Area(p1, p2, p3, p4, costmap, plan);

    // Planı RViz'de görselleştirir
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.poses = plan;
    plan_pub_.publish(path_msg);

    return true;
}

void EuclidPlanner::Calculate_Rectangle_Bounds(const std::vector<geometry_msgs::Point32>& points, 
                                               double& xmin, double& xmax, double& ymin, double& ymax) 
{
    xmin = xmax = points[0].x;
    ymin = ymax = points[0].y;

    for (const auto& point : points)
    {
        if (point.x < xmin) xmin = point.x;
        if (point.x > xmax) xmax = point.x;
        if (point.y < ymin) ymin = point.y;
        if (point.y > ymax) ymax = point.y;
    }
}

bool EuclidPlanner::Is_Inside_Rectangle(double x, double y, double xmin, double xmax, double ymin, double ymax) 
{
    return (x >= xmin && x <= xmax && y >= ymin && y <= ymax);
}

bool EuclidPlanner::Is_Area_Free(double wx, double wy, costmap_2d::Costmap2D* costmap)
{
    unsigned int mx, my;

    if (!costmap->worldToMap(wx, wy, mx, my)) 
    {
        ROS_INFO("Koordinat harita disinda: wx=%f, wy=%f", wx, wy);
        return false;
    }

    // Hücrenin maliyetini kontrol et
    unsigned char cost = costmap->getCost(mx, my);
    if (cost >= 140) 
    { 
        return false; 
    }
    
    return true;
}
}
