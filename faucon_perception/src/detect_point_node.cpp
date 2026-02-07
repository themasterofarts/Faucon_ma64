#include "faucon_perception/pcl/model_detect.hpp"
#include "rclcpp/rclcpp.hpp"
#include  "faucon_perception/detect_point_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>



namespace faucon::detection
{

    DetectPointNode::DetectPointNode() : rclcpp::Node("detect_point_node")
    {
        
        RoiSphere default_roi{3.5, 5.0};   // 1.75 devant le robot, 5m max
        point_detector_ = std::make_unique<PointDetector>(default_roi);

        
        sub_cluster_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/crop_clusters", rclcpp::SensorDataQoS(),
            std::bind(&DetectPointNode::onCluster, this, std::placeholders::_1));

        
        pub_roisphere_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_sphere_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "Detect Point Node initialized with ROI sphere: r_min=%.2f m, r_max=%.2f m",
                        default_roi.r_min, default_roi.r_max);
    }

    void DetectPointNode::onCluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        CloudPtr cloud(new Cloud);
        CloudPtr cloud_filtered(new Cloud);

        pcl::fromROSMsg(*msg, *cloud);
        if (!cloud || cloud->empty())
            return; 
        
        RoiSphere roi_sphere = point_detector_->getRoiSphere();

        cloud_filtered = point_detector_->filterSphere(cloud, RobotPose{0,0,0,0,0,0}, roi_sphere);

        publishFilteredCloud(cloud_filtered, msg->header);

    }

    void DetectPointNode::publishFilteredCloud(const CloudPtr& cloud, const std_msgs::msg::Header& header) const
    {
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header = header;

        pub_roisphere_->publish(output_msg);
    }

} // namespace faucon::detection


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<faucon::detection::DetectPointNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}