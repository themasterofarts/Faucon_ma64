#include "faucon_perception/lidar_calibrate.hpp"

#include <pcl/memory.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

LidarCalibrator::LidarCalibrator() : Node("lidar_calibrator")
{

  this->declare_parameter("ransac_distance_threshold", 0.04);
  this->declare_parameter("ransac_max_iterations", 100);

  ransac_distance_threshold_ = this->get_parameter("ransac_distance_threshold").as_double();
  ransac_max_iterations_ = this->get_parameter("ransac_max_iterations").as_int();

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud", 10,
      std::bind(&LidarCalibrator::pointCloudCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_calib_out", 10);

  ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_cloud", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Lidar Ground Filter initialized - Method: %s",
              "RANSAC" );
}

void LidarCalibrator::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto cloud_robot = transformToRobotFrame(*msg);
  // Convertir le message ROS2 en nuage PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_robot, *pcl_cloud_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud;

  cloud_no_ground = filterGroundRANSAC(pcl_cloud_, ground_cloud);

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_no_ground, output_msg);
  output_msg.header = cloud_robot.header;
  publisher_->publish(output_msg);

  
  if (ground_cloud && ground_publisher_->get_subscription_count() > 0)
  {
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    ground_msg.header = cloud_robot.header;
    ground_publisher_->publish(ground_msg);
  }
}



pcl::PointCloud<pcl::PointXYZ>::Ptr LidarCalibrator::filterGroundRANSAC(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &pt : input_cloud->points)
  {
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
      clean_cloud->points.push_back(pt);
  }
  clean_cloud->width = clean_cloud->points.size();
  clean_cloud->height = 1;
  clean_cloud->is_dense = true;

  
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_max_iterations_);
  seg.setDistanceThreshold(ransac_distance_threshold_);

  seg.setInputCloud(clean_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Could not detect ground plane");
    return clean_cloud;
  }

  
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(clean_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true); 

  auto filtered_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  extract.filter(*filtered_cloud);

  
  if (ground_cloud)
  {
    extract.setNegative(false);
    extract.filter(*ground_cloud);
  }

  RCLCPP_DEBUG(this->get_logger(),
               "Ground removal: %zu points -> %zu objects, %zu ground",
               clean_cloud->points.size(),
               filtered_cloud->points.size(),
               inliers->indices.size());

  return filtered_cloud;
}

sensor_msgs::msg::PointCloud2 LidarCalibrator::transformToRobotFrame(
    const sensor_msgs::msg::PointCloud2 &cloud_in)
{
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform(
        target_frame_,
        cloud_in.header.frame_id,
        tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(),
                "Cannot transform cloud from %s to %s: %s",
                cloud_in.header.frame_id.c_str(),
                target_frame_.c_str(),
                ex.what());
    return cloud_in;
  }

  sensor_msgs::msg::PointCloud2 cloud_out;
  tf2::doTransform(cloud_in, cloud_out, tf);
  cloud_out.header.frame_id = target_frame_;

  return cloud_out;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarCalibrator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
