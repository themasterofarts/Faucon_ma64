#ifndef CLUSTER_ROWS_HPP
#define CLUSTER_ROWS_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>

class RowClusterer : public rclcpp::Node
{

public:
    RowClusterer();

    struct CropRow
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
        pcl::ModelCoefficients::Ptr coefficients;
        pcl::PointXYZ start_point;
        pcl::PointXYZ direction;
        pcl::PointXYZ centroid;
        double length;
        int num_points;
    };

    

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Méthode 1: Clustering euclidien (détection des groupes de plantes)
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> performEuclideanClustering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

    // Méthode 2: Détection des lignes (RANSAC) dans chaque cluster
    std::vector<CropRow> detectCropRows(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters);

    // Méthode 3: Groupement des rangs parallèles
    std::vector<std::vector<CropRow>> groupParallelRows(
        const std::vector<CropRow> &crop_rows);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterROICropBox(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);


    // Méthodes de publication
    void publishClusters(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters,
        const std_msgs::msg::Header &header);

    void publishRowMarkers(
        const std::vector<CropRow> &crop_rows,
        const std_msgs::msg::Header &header);

    // ROS2
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;

    // Paramètres de clustering
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

    // Paramètres RANSAC pour détection de lignes
    double ransac_distance_threshold_;
    int ransac_max_iterations_;

    // Paramètres de détection de rangs
    std::string row_detection_method_;
    double min_row_length_;
    double max_row_distance_;

    // Paramètres ROI
    bool use_roi_;
    double roi_x_min_;
    double roi_x_max_;
    double roi_y_min_;
    double roi_y_max_;
    double roi_z_min_;
    double roi_z_max_;

    // Paramètres de contrainte d'axe
    bool use_axis_constraint_;
    double principal_axis_x_;
    double principal_axis_y_;
    double principal_axis_z_;
    double axis_angle_tolerance_;
    double min_inlier_ratio_;
};

#endif // CLUSTER_ROWS_HPP