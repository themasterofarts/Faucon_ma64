#include "faucon_perception/algo/cluster_method.hpp"

namespace faucon::filter
{

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ClusterMethod::CreateEuclideanClustering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) const
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

        if (!input_cloud || input_cloud->empty())
            return clusters;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(input_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cfg_.cluster_tolerance);
        ec.setMinClusterSize(cfg_.min_cluster_size);
        ec.setMaxClusterSize(cfg_.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud);
        ec.extract(cluster_indices);

        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));

            extract.setInputCloud(input_cloud);
            extract.setIndices(indices_ptr);
            extract.setNegative(false);
            extract.filter(*cluster);

            clusters.push_back(cluster);
        }

        return clusters;
    }

} // namespace faucon::filter
