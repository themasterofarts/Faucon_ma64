#include "faucon_perception/cluster_method_node.hpp" 
#include "faucon_perception/algo/cluster_method.hpp"


namespace faucon::algorithm
{
    ClusterMethodNode::ClusterMethodNode(const rclcpp::NodeOptions &options)
        : Node("cluster_method_node", options)
    {
        
        sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/crop_clusters", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
            {  
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*msg, *cloud);

                // Appliquer le clustering euclidien
                faucon::filter::ClusterConfig cfg;
                cfg.cluster_tolerance = 0.3;
                cfg.min_cluster_size = 10;
                cfg.max_cluster_size = 15000;
                faucon::filter::ClusterMethod cluster_method(cfg);
                auto clusters = cluster_method.CreateEuclideanClustering(cloud);

                
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
                output_msg.header = msg->header;

                pub_clustered_->publish(output_msg);

             });    

        pub_clustered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/clustered_point_cloud", 10);  

        RCLCPP_INFO(this->get_logger(), "Cluster Method Node initialized.");

    }





      


}      

// namespace faucon::algorithm
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<faucon::algorithm::ClusterMethodNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}