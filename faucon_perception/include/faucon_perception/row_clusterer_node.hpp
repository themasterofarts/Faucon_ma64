#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "faucon_perception/pcl/row_pipeline.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace faucon::ros
{

    class RowClustererNode final : public rclcpp::Node
    {
    public:
        RowClustererNode();

    private:
        void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        faucon::pclrow::RowDetectionConfig loadConfig();

        void publishClusters(const std::vector<faucon::pclrow::CloudPtr> &clusters,
                             const std_msgs::msg::Header &header) const;
        void publishMarkers(const std::vector<faucon::pclrow::CropRow> &rows,
                            const std_msgs::msg::Header &header) const;
            

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
        

        faucon::pclrow::RowPipeline pipeline_; 
    };

} // namespace faucon::ros
