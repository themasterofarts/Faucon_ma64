#include "faucon_perception/ransac_node.hpp"

#include "faucon_perception/algo/ransac_method.hpp"


namespace faucon::algorithm
{

    RansacNode::RansacNode(const rclcpp::NodeOptions &options)
        : Node("ransac_node", options)
    {
        
        sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {    
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*msg, *cloud);

                // Appliquer le filtre RANSAC
                faucon::filter::RansacConfig cfg;
                cfg.max_iterations = 1000;
                cfg.distance_threshold = 0.09;
                faucon::filter::RansacFilter ransac_filter(cfg);
                faucon::filter::RansacResult result = ransac_filter.apply(cloud);

                
                sensor_msgs::msg::PointCloud2 inliers_msg;
                pcl::toROSMsg(*result.cloud_inliers, inliers_msg);
                inliers_msg.header = msg->header;

                sensor_msgs::msg::PointCloud2 outliers_msg;
                pcl::toROSMsg(*result.cloud_outliers, outliers_msg);
                outliers_msg.header = msg->header;

                
                pub_inliers_->publish(inliers_msg);
                pub_outliers_->publish(outliers_msg);
            });

        pub_inliers_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "inliers_point_cloud", 10);

        pub_outliers_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "outliers_point_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "RANSAC Node initialized.");
    }

} // namespace faucon::algorithm


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<faucon::algorithm::RansacNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}