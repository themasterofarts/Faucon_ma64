#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


namespace faucon::algorithm
{

    class ClusterMethodNode : public rclcpp::Node
    {
    public:
        explicit ClusterMethodNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()); 

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clustered_;
    };
         
    
} // namespace faucon::algorithm    