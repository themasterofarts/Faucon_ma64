#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>



namespace faucon::algorithm
{

    class RansacNode : public rclcpp::Node
    {
    public:
        explicit RansacNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_inliers_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_outliers_;

    };

} // namespace faucon::algorithm