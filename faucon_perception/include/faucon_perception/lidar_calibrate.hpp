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

class LidarCalibrator : public rclcpp::Node
{
public:
    LidarCalibrator();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    /**
     * \brief Transforme un nuage de points vers le repère du robot
     * \param cloud_in Le nuage de points d'entrée
     * \return Le nuage de points transformé
     */
    sensor_msgs::msg::PointCloud2 transformToRobotFrame(
        const sensor_msgs::msg::PointCloud2 &cloud_in);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGround(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

    std::string target_frame_ = "base_link"; 
    double ground_min_z_ = -0.5; 
    double ground_max_z_ = 0.3;  

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


};

#endif // LIDAR_CALIBRATOR_HPP
