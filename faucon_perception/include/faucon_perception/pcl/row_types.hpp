#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace faucon::pclrow
{

    using Cloud = pcl::PointCloud<pcl::PointXYZ>;
    using CloudPtr = Cloud::Ptr;

    struct RoiBox
    {
        bool enabled{true};
        double x_min{0.0}, x_max{7.0};
        double y_min{-1.5}, y_max{1.5};
        double z_min{-0.5}, z_max{2.0};
    };

    struct AxisConstraint
    {
        bool enabled{true};
        double axis_x{1.0}, axis_y{0.0}, axis_z{0.0};
        double angle_tolerance_deg{5.0};
    };

    struct ClusteringConfig
    {
        double tolerance{0.3};
        int min_size{10};
        int max_size{15000};
    };

    struct RansacConfig
    {
        double distance_threshold{0.03};
        int max_iterations{1000};
        double min_inlier_ratio{0.1};
    };

    struct RowFilterConfig
    {
        double min_row_length{1.0};
        //double max_row_distance{0.6};   // to remove 
    };

    struct RowDetectionConfig
    {
        std::string method{"euclidean"}; // future extension OCP  "euclidean" ou "region_growing"
        RoiBox roi;
        ClusteringConfig clustering;
        RansacConfig ransac;
        AxisConstraint axis_constraint;
        RowFilterConfig row_filter;
    };

    struct CropRow
    {
        CloudPtr cluster;
        pcl::ModelCoefficients::Ptr coefficients;

        pcl::PointXYZ start_point;
        pcl::PointXYZ end_point;
        pcl::PointXYZ direction; // unité
        pcl::PointXYZ centroid;

        double length{0.0};
        int num_points{0};
        int inlier_count{0};
    };

    struct RowPipelineResult
    {
        std::vector<CloudPtr> clusters;
        std::vector<CropRow> rows;
    };

} // namespace faucon::pclrow
