#include "faucon_perception/lidar_calibrate.hpp"

#include <pcl/memory.h>


LidarCalibrator::LidarCalibrator() : Node("lidar_calibrator")
{
    
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud", 10,
        std::bind(&LidarCalibrator::pointCloudCallback, this, std::placeholders::_1));

    
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_calib_out", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void LidarCalibrator::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convertir le message ROS2 en nuage PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud_);

    auto cloud_no_ground = filterGround(pcl_cloud_);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_no_ground, output_msg);
    output_msg.header = msg->header;


    auto cloud_robot = transformToRobotFrame(*msg);
    publisher_->publish(output_msg);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr LidarCalibrator::filterGround(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
 
  auto filtered_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  filtered_cloud->header   = input_cloud->header;
  filtered_cloud->is_dense = false;
  filtered_cloud->points.reserve(input_cloud->points.size());

  for (const auto &pt : input_cloud->points)
  {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
          continue;

      // filtre sol
      if (pt.z >= ground_min_z_ && pt.z <= ground_max_z_)
          continue;

      filtered_cloud->points.push_back(pt);
  }

  filtered_cloud->width  = filtered_cloud->points.size();
  filtered_cloud->height = 1;

  return filtered_cloud;
}




sensor_msgs::msg::PointCloud2 LidarCalibrator::transformToRobotFrame(
    const sensor_msgs::msg::PointCloud2 & cloud_in)
{
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform(
      target_frame_,                      
      cloud_in.header.frame_id,           
      tf2::TimePointZero                  
    );
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN(this->get_logger(),
                "Cannot transform cloud from %s to %s: %s",
                cloud_in.header.frame_id.c_str(),
                target_frame_.c_str(),
                ex.what());
    return cloud_in;  // on renvoie le nuage d'origine si échec
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
