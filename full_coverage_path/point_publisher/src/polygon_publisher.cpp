#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32MultiArray.h> 

class PointPublisher
{
public:
    PointPublisher()
    {
        // Publisher: Noktaları yayınlar
        points_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("rectangle_points", 1);

        // Subscriber: RViz'den gelen tıklama noktalarını dinler
        clicked_point_sub_ = nh_.subscribe("clicked_point", 10, &PointPublisher::pointCallback, this);

        ROS_INFO("Point Publisher Node Initialized. Click 4 points in RViz using the Publish Point tool.");
    }

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        // Noktayı alın ve listeye ekleyin
        points_.push_back(msg->point);

        ROS_INFO("Point added: (%f, %f, %f)", msg->point.x, msg->point.y, msg->point.z);

        // 4. noktadan sonra mesajı yayınla
        if (points_.size() == 4)
        {
            std_msgs::Float32MultiArray points_msg;
            for (const auto& point : points_)
            {
                points_msg.data.push_back(point.x);
                points_msg.data.push_back(point.y);
            }
            points_pub_.publish(points_msg);
            ROS_INFO("4 Points published.");

            points_.clear();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher points_pub_;
    ros::Subscriber clicked_point_sub_;
    std::vector<geometry_msgs::Point> points_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_publisher");
    PointPublisher point_publisher;

    ros::spin();
    return 0;
}
