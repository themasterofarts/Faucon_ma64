#ifndef LIDAR_CALIBRATOR_HPP
#define LIDAR_CALIBRATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class LidarCalibrator : public rclcpp::Node
{
public:
    LidarCalibrator();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_publisher_;
  

    /**
     * \brief Transforme un nuage de points vers le repère du robot
     * \param cloud_in Le nuage de points d'entrée
     * \return Le nuage de points transformé
     */
    sensor_msgs::msg::PointCloud2 transformToRobotFrame(
        const sensor_msgs::msg::PointCloud2 &cloud_in);
    /**
     * \brief Filtre le sol d'un nuage de points en utilisant RANSAC
     * \param input_cloud Le nuage de points d'entrée
     * \param ground_cloud  Le nuage de points du sol détecté
     * \return Le nuage de points sans le sol
     */
    std::pair<
        pcl::PointCloud<pcl::PointXYZ>::Ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr>
    filterGroundRANSAC(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

    /**
     * \brief Regroupe les points du nuage en lignes (rows) basées sur leur coordonnée Y
     * \param input_cloud Le nuage de points d'entrée
     * \return Clusters indices pour chaque ligne détectée
     */
    std::vector<pcl::PointIndices>
    clusterRows(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

    /**
     * \brief Construit un message PointCloud2 avec des couleurs différentes pour chaque cluster
     * \param input_cloud Le nuage de points d'entrée
     * \param clusters Les indices des clusters
     * \param header L'en-tête du message
     * \return Le message PointCloud2 coloré
     */
    sensor_msgs::msg::PointCloud2
    buildColoredClustersMsg(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
        const std::vector<pcl::PointIndices> &clusters,
        const std_msgs::msg::Header &header);

    std::string target_frame_ = "base_link";

    double ransac_distance_threshold_;
    int ransac_max_iterations_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // LIDAR_CALIBRATOR_HPP
