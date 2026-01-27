#include "faucon_perception/algo/ransac_method.hpp"

namespace faucon::filter
{


    RansacResult RansacFilter::apply(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) const
    {
        RansacResult result;
        if (!cloud || cloud->empty())
            return result;

        // Nettoyer le nuage des points non finis

        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &pt : cloud->points)
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
        seg.setMaxIterations(cfg_.max_iterations);
        seg.setDistanceThreshold(cfg_.distance_threshold);

        seg.setInputCloud(clean_cloud);
        seg.segment(*inliers, *coefficients); 

        if (inliers->indices.empty())
        {
            // result.cloud_inliers = cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
            // result.cloud_outliers = cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);    
            return result;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(clean_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);

        auto filtered_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        extract.filter(*filtered_cloud);

        auto ground_cloud_filter = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        extract.setNegative(false);
        extract.filter(*ground_cloud_filter);

        result.cloud_inliers = filtered_cloud;
        result.cloud_outliers = ground_cloud_filter;

        return result;
    }

}