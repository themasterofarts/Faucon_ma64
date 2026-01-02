#include "faucon_perception/row_clusterer_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>

namespace faucon::ros
{

    RowClustererNode::RowClustererNode()
        : rclcpp::Node("crop_row_detector"),
          pipeline_(loadConfig())
    {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_calib_out", rclcpp::SensorDataQoS(),
            std::bind(&RowClustererNode::onCloud, this, std::placeholders::_1));

        pub_clusters_ = create_publisher<sensor_msgs::msg::PointCloud2>("/crop_clusters", 10);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/crop_row_markers", 10);
    }

    void RowClustererNode::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        faucon::pclrow::CloudPtr cloud(new faucon::pclrow::Cloud);
        pcl::fromROSMsg(*msg, *cloud);
        if (!cloud || cloud->empty())
            return;

        const auto result = pipeline_.process(cloud);

        publishClusters(result.clusters, msg->header);
        publishMarkers(result.rows, msg->header);
    }

    faucon::pclrow::RowDetectionConfig RowClustererNode::loadConfig() 
    {
        using namespace faucon::pclrow;
        RowDetectionConfig cfg;

        declare_parameter("cluster_tolerance", cfg.clustering.tolerance);
        declare_parameter("min_cluster_size", cfg.clustering.min_size);
        declare_parameter("max_cluster_size", cfg.clustering.max_size);

        declare_parameter("ransac_distance_threshold", cfg.ransac.distance_threshold);
        declare_parameter("ransac_max_iterations", cfg.ransac.max_iterations);
        declare_parameter("min_inlier_ratio", cfg.ransac.min_inlier_ratio);

        declare_parameter("row_detection_method", cfg.method);
        declare_parameter("min_row_length", cfg.row_filter.min_row_length);
        //declare_parameter("max_row_distance", cfg.row_filter.max_row_distance);

        declare_parameter("use_roi", cfg.roi.enabled);
        declare_parameter("roi_x_min", cfg.roi.x_min);
        declare_parameter("roi_x_max", cfg.roi.x_max);
        declare_parameter("roi_y_min", cfg.roi.y_min);
        declare_parameter("roi_y_max", cfg.roi.y_max);
        declare_parameter("roi_z_min", cfg.roi.z_min);
        declare_parameter("roi_z_max", cfg.roi.z_max);

        declare_parameter("use_axis_constraint", cfg.axis_constraint.enabled);
        declare_parameter("principal_axis_x", cfg.axis_constraint.axis_x);
        declare_parameter("principal_axis_y", cfg.axis_constraint.axis_y);
        declare_parameter("principal_axis_z", cfg.axis_constraint.axis_z);
        declare_parameter("axis_angle_tolerance", cfg.axis_constraint.angle_tolerance_deg);

        // lecture effective
        cfg.clustering.tolerance = get_parameter("cluster_tolerance").as_double();
        cfg.clustering.min_size = get_parameter("min_cluster_size").as_int();
        cfg.clustering.max_size = get_parameter("max_cluster_size").as_int();

        cfg.ransac.distance_threshold = get_parameter("ransac_distance_threshold").as_double();
        cfg.ransac.max_iterations = get_parameter("ransac_max_iterations").as_int();
        cfg.ransac.min_inlier_ratio = get_parameter("min_inlier_ratio").as_double();

        cfg.method = get_parameter("row_detection_method").as_string();
        cfg.row_filter.min_row_length = get_parameter("min_row_length").as_double();
        //cfg.row_filter.max_row_distance = get_parameter("max_row_distance").as_double();

        cfg.roi.enabled = get_parameter("use_roi").as_bool();
        cfg.roi.x_min = get_parameter("roi_x_min").as_double();
        cfg.roi.x_max = get_parameter("roi_x_max").as_double();
        cfg.roi.y_min = get_parameter("roi_y_min").as_double();
        cfg.roi.y_max = get_parameter("roi_y_max").as_double();
        cfg.roi.z_min = get_parameter("roi_z_min").as_double();
        cfg.roi.z_max = get_parameter("roi_z_max").as_double();

        cfg.axis_constraint.enabled = get_parameter("use_axis_constraint").as_bool();
        cfg.axis_constraint.axis_x = get_parameter("principal_axis_x").as_double();
        cfg.axis_constraint.axis_y = get_parameter("principal_axis_y").as_double();
        cfg.axis_constraint.axis_z = get_parameter("principal_axis_z").as_double();
        cfg.axis_constraint.angle_tolerance_deg = get_parameter("axis_angle_tolerance").as_double();

        return cfg;
    }

    void RowClustererNode::publishClusters(
        const std::vector<faucon::pclrow::CloudPtr> &clusters,
        const std_msgs::msg::Header &header) const
    {
        // similaire à ton publishClusters actuel, mais sans accès à des membres “algo”.
        // Tu peux soit publier chaque cluster séparément, soit concaténer.

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < clusters.size(); ++i)
        {

            uint8_t r = (i * 50) % 255;
            uint8_t g = (i * 100) % 255;
            uint8_t b = (i * 150) % 255;

            for (const auto &pt : clusters[i]->points)
            {
                pcl::PointXYZRGB colored_pt;
                colored_pt.x = pt.x;
                colored_pt.y = pt.y;
                colored_pt.z = pt.z;
                colored_pt.r = r;
                colored_pt.g = g;
                colored_pt.b = b;
                colored_cloud->points.push_back(colored_pt);
            }
        }

        colored_cloud->width = colored_cloud->points.size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = false;

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*colored_cloud, output_msg);
        output_msg.header = header;

        pub_clusters_->publish(output_msg);
    }

    void RowClustererNode::publishMarkers(
        const std::vector<faucon::pclrow::CropRow> &rows,
        const std_msgs::msg::Header &header) const
    {
        // similaire à ton publishRowMarkers, mais basé sur row.start_point/end_point
        // et sans logique PCL interne.

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < rows.size(); ++i)
        {
            const auto &row = rows[i];

            visualization_msgs::msg::Marker line_marker;
            line_marker.header = header;
            line_marker.ns = "crop_rows";
            line_marker.id = i;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.05;

            line_marker.color.r = (i * 0.3f);
            line_marker.color.g = (i * 0.5f);
            line_marker.color.b = 1.0f;
            line_marker.color.a = 1.0;

            geometry_msgs::msg::Point start_pt, end_pt;

            double extension = row.length / 2.0;

            start_pt.x = row.start_point.x - row.direction.x;
            start_pt.y = row.start_point.y - row.direction.y;
            start_pt.z = row.start_point.z;

            end_pt.x = row.start_point.x + row.direction.x * extension;
            end_pt.y = row.start_point.y + row.direction.y * extension;
            end_pt.z = row.start_point.z;

            line_marker.points.push_back(start_pt);
            line_marker.points.push_back(end_pt);

            marker_array.markers.push_back(line_marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = header;
            text_marker.ns = "crop_row_labels";
            text_marker.id = i + 1000;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = row.centroid.x;
            text_marker.pose.position.y = row.centroid.y;
            text_marker.pose.position.z = row.centroid.z + 0.5;
            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = "Row " + std::to_string(i + 1);

            marker_array.markers.push_back(text_marker);
        }

        pub_markers_->publish(marker_array);
    }

} // namespace faucon::ros

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<faucon::ros::RowClustererNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}