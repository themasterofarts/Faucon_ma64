#include "faucon_perception/pcl/model_detect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace faucon::detection
{

  class DetectPointNode final : public rclcpp::Node
  {
  public:
    explicit DetectPointNode();
 

    private:
        std::unique_ptr<PointDetector> point_detector_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cluster_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_roisphere_;

        void onCluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void publishFilteredCloud(const CloudPtr& cloud, const std_msgs::msg::Header& header) const;
        
  };    

} // namespace faucon::detection