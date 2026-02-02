#include "faucon_perception/pcl/row_pipeline.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>
#include <limits>
#include <cmath>

#include "faucon_perception/pcl/hough_method.hpp"

namespace faucon::pclrow
{

    RowPipeline::RowPipeline(RowDetectionConfig cfg) : cfg_(std::move(cfg)) {}

    RowPipelineResult RowPipeline::process(const CloudPtr &input) const
    {
        RowPipelineResult out;
        if (!input || input->empty())
            return out;

        // HoughLineDetector hough_detector; // Utilisation de la méthode de Hough

        auto filtered = applyRoi(input);
        out.clusters = clusterize(filtered);
        out.metrics.cluster_count = out.clusters.size();
        out.rows = detectRows(out.clusters);

        // out.rows = hough_detector.detectLines(out.clusters); // Détection des lignes avec Hough
        out.metrics.rows_count = out.rows.size();

        return out;
    }

    CloudPtr RowPipeline::applyRoi(const CloudPtr &input) const
    {
        if (!cfg_.roi.enabled)
            return input;

        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(input);
        crop.setMin(Eigen::Vector4f(cfg_.roi.x_min, cfg_.roi.y_min, cfg_.roi.z_min, 1.0f));
        crop.setMax(Eigen::Vector4f(cfg_.roi.x_max, cfg_.roi.y_max, cfg_.roi.z_max, 1.0f));

        auto out = CloudPtr(new Cloud);
        crop.filter(*out);
        return out;
    }

    std::vector<CloudPtr> RowPipeline::clusterize(const CloudPtr &cloud) const
    {
        std::vector<CloudPtr> clusters;
        if (!cloud || cloud->empty())
            return clusters;

        auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cfg_.clustering.tolerance);
        ec.setMinClusterSize(cfg_.clustering.min_size);
        ec.setMaxClusterSize(cfg_.clustering.max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        clusters.reserve(cluster_indices.size());
        for (const auto &indices : cluster_indices)
        {
            auto c = CloudPtr(new Cloud);
            c->points.reserve(indices.indices.size());
            for (int idx : indices.indices)
                c->points.push_back(cloud->points[idx]);
            c->width = static_cast<uint32_t>(c->points.size());
            c->height = 1;
            c->is_dense = false;
            clusters.push_back(c);
        }
        return clusters;
    }

    std::vector<CropRow> RowPipeline::detectRows(const std::vector<CloudPtr> &clusters) const
    {
        std::vector<CropRow> rows;
        rows.reserve(clusters.size());

        for (const auto &cluster : clusters)
        {
            if (!cluster || cluster->empty())
                continue;

            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            const double length_xy = std::hypot(max_pt.x - min_pt.x, max_pt.y - min_pt.y);
            if (length_xy < cfg_.row_filter.min_row_length)
                continue;

            // Appliquer RANSAC pour ajuster une ligne
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PARALLEL_LINE); // SACMODEL_PARALLEL_LINE  SACMODEL_LINE
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(cfg_.ransac.max_iterations);
            seg.setDistanceThreshold(cfg_.ransac.distance_threshold);
            seg.setInputCloud(cluster);

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            seg.segment(*inliers, *coeff);

            if (inliers->indices.empty())
                continue;

            // 3) Filtre axis constraint (optionnel)
            if (cfg_.axis_constraint.enabled && !passesAxisConstraint(*coeff))
            {
                continue;
            }

            // 4) Gating ratio inliers
            const double ratio = static_cast<double>(inliers->indices.size()) / static_cast<double>(cluster->size());
            if (ratio < cfg_.ransac.min_inlier_ratio)
                continue;

            // 5) Extraire inliers
            pcl::ExtractIndices<pcl::PointXYZ> ex;
            ex.setInputCloud(cluster);
            ex.setIndices(inliers);
            ex.setNegative(false);

            CloudPtr inlier_cloud(new Cloud);
            ex.filter(*inlier_cloud);

            // 6) Refit PCA sur inliers pour direction stable + segment tmin/tmax
            Eigen::Vector3f p0, dir;
            fitLinePcaXY(*inlier_cloud, p0, dir);

            float tmin = std::numeric_limits<float>::infinity();
            float tmax = -std::numeric_limits<float>::infinity();
            for (const auto &pt : inlier_cloud->points)
            {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                const float t = dir.dot(p - p0);
                tmin = std::min(tmin, t);
                tmax = std::max(tmax, t);
            }

            const Eigen::Vector3f p_start = p0 + tmin * dir;
            const Eigen::Vector3f p_end = p0 + tmax * dir;

            const double seg_len = (p_end - p_start).head<2>().norm();
            if (seg_len < cfg_.row_filter.min_row_length)
                continue;

            // 7) Construire CropRow (DTO)
            CropRow row;
            row.cluster = cluster;
            row.coefficients = coeff;
            row.start_point = pcl::PointXYZ(p_start.x(), p_start.y(), p_start.z());
            row.end_point = pcl::PointXYZ(p_end.x(), p_end.y(), p_end.z());
            row.direction = pcl::PointXYZ(dir.x(), dir.y(), dir.z());
            row.length = seg_len;
            row.num_points = static_cast<int>(cluster->size());
            row.inlier_count = static_cast<int>(inliers->indices.size());

            Eigen::Vector4f centroid4;
            pcl::compute3DCentroid(*cluster, centroid4);
            row.centroid = pcl::PointXYZ(centroid4.x(), centroid4.y(), centroid4.z());

            rows.push_back(std::move(row));
        }

        return rows;
    }

    void RowPipeline::fitLinePcaXY(const Cloud &cloud, Eigen::Vector3f &p0, Eigen::Vector3f &dir_unit)
    {
        Eigen::Vector4f centroid4;
        pcl::compute3DCentroid(cloud, centroid4);
        Eigen::Vector3f c(centroid4.x(), centroid4.y(), centroid4.z());

        Eigen::Matrix3f cov;
        pcl::computeCovarianceMatrixNormalized(cloud, centroid4, cov);

        // option “XY dominant”: on peut annuler la composante Z si souhaité
        cov(2, 0) = cov(0, 2) = 0.0f;
        cov(2, 1) = cov(1, 2) = 0.0f;
        cov(2, 2) = 0.0f;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        const auto ev = solver.eigenvectors();
        Eigen::Vector3f dir = ev.col(2); // plus grande valeur propre (trié croissant)
        dir.z() = 0.0f;

        if (dir.norm() < 1e-6f)
            dir = Eigen::Vector3f(1.f, 0.f, 0.f);
        dir_unit = dir.normalized();
        p0 = c;
    }

    bool RowPipeline::passesAxisConstraint(const pcl::ModelCoefficients &coeff) const
    {
        // coeff.values = [x0 y0 z0 dx dy dz]
        if (coeff.values.size() < 6)
            return false;

        Eigen::Vector3d dir(coeff.values[3], coeff.values[4], coeff.values[5]);
        if (dir.norm() < 1e-9)
            return false;
        dir.normalize();

        Eigen::Vector3d axis(cfg_.axis_constraint.axis_x,
                             cfg_.axis_constraint.axis_y,
                             cfg_.axis_constraint.axis_z);
        if (axis.norm() < 1e-9)
            return true;
        axis.normalize();

        const double dot = std::clamp(dir.dot(axis), -1.0, 1.0);
        const double angle = std::acos(dot) * 180.0 / M_PI;

        return angle <= cfg_.axis_constraint.angle_tolerance_deg || (180.0 - angle) <= cfg_.axis_constraint.angle_tolerance_deg;
    }

} // namespace faucon::pclrow
