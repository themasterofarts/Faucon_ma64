#include "faucon_perception/cluster_rows.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

RowClusterer::RowClusterer() : Node("crop_row_detector")
{
    // Déclaration des paramètres
    this->declare_parameter("cluster_tolerance", 0.3);
    this->declare_parameter("min_cluster_size", 30);
    this->declare_parameter("max_cluster_size", 15000);
    this->declare_parameter("ransac_distance_threshold", 0.05);
    this->declare_parameter("ransac_max_iterations", 1000);
    this->declare_parameter("row_detection_method", "euclidean"); // "euclidean" ou "region_growing"
    this->declare_parameter("min_row_length", 3.0);
    this->declare_parameter("max_row_distance", 1.0); // Distance max entre rangs

    this->declare_parameter("use_roi", true);
    this->declare_parameter("roi_x_min", 0.0);
    this->declare_parameter("roi_x_max", 7.0);
    this->declare_parameter("roi_y_min", -1.5);
    this->declare_parameter("roi_y_max", 1.5);
    this->declare_parameter("roi_z_min", -0.5);
    this->declare_parameter("roi_z_max", 2.0);

    this->declare_parameter("use_axis_constraint", false); // Contraindre la direction de la ligne
    this->declare_parameter("principal_axis_x", 1.0);     // Direction principale des rangs
    this->declare_parameter("principal_axis_y", 0.0);
    this->declare_parameter("principal_axis_z", 0.0);
    this->declare_parameter("axis_angle_tolerance", 20.0); // Degrés
    this->declare_parameter("min_inlier_ratio", 0.2);      // Ratio minimum de points sur la ligne

    use_axis_constraint_ = this->get_parameter("use_axis_constraint").as_bool();
    principal_axis_x_ = this->get_parameter("principal_axis_x").as_double();
    principal_axis_y_ = this->get_parameter("principal_axis_y").as_double();
    principal_axis_z_ = this->get_parameter("principal_axis_z").as_double();
    axis_angle_tolerance_ = this->get_parameter("axis_angle_tolerance").as_double();
    min_inlier_ratio_ = this->get_parameter("min_inlier_ratio").as_double();

    // Récupération des paramètres
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    ransac_distance_threshold_ = this->get_parameter("ransac_distance_threshold").as_double();
    ransac_max_iterations_ = this->get_parameter("ransac_max_iterations").as_int();
    row_detection_method_ = this->get_parameter("row_detection_method").as_string();
    min_row_length_ = this->get_parameter("min_row_length").as_double();
    max_row_distance_ = this->get_parameter("max_row_distance").as_double();

    // Paramètres ROI
    use_roi_ = this->get_parameter("use_roi").as_bool();
    roi_x_min_ = this->get_parameter("roi_x_min").as_double();
    roi_x_max_ = this->get_parameter("roi_x_max").as_double();
    roi_y_min_ = this->get_parameter("roi_y_min").as_double();
    roi_y_max_ = this->get_parameter("roi_y_max").as_double();
    roi_z_min_ = this->get_parameter("roi_z_min").as_double();
    roi_z_max_ = this->get_parameter("roi_z_max").as_double();

    // Subscribers et Publishers
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_calib_out", 10,
        std::bind(&RowClusterer::pointCloudCallback, this, std::placeholders::_1));

    clusters_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/crop_clusters", 10);
    markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/crop_row_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Crop Row Detector initialized");
    RCLCPP_INFO(this->get_logger(), "  Method: %s", row_detection_method_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Cluster tolerance: %.2f m", cluster_tolerance_);

    if (use_roi_)
    {
        RCLCPP_INFO(this->get_logger(), "  ROI enabled: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
                    roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
    }
}

void RowClusterer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    if (input_cloud->points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
        return;
    }

    // create a ROI crop box filter if needed

    if (use_roi_)
    {
        input_cloud = filterROICropBox(input_cloud);
        RCLCPP_DEBUG(this->get_logger(), "ROI filter: %zu -> %zu points",
                     input_cloud->points.size(), input_cloud->points.size());
    }

    // Étape 1: Clustering euclidien
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters =
        performEuclideanClustering(input_cloud);

    if (clusters.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No clusters detected");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Detected %zu clusters", clusters.size());

    // Étape 2: Détecter les lignes (rangs) dans chaque cluster
    std::vector<CropRow> crop_rows = detectCropRows(clusters);

    RCLCPP_INFO(this->get_logger(), "Detected %zu crop rows", crop_rows.size());

    // Étape 3: Grouper les rangs parallèles
    std::vector<std::vector<CropRow>> row_groups = groupParallelRows(crop_rows);

    RCLCPP_INFO(this->get_logger(), "Grouped into %zu row groups", row_groups.size());

    publishClusters(clusters, msg->header);
    publishRowMarkers(crop_rows, msg->header);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RowClusterer::filterROICropBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Définir les coins de la boîte
    Eigen::Vector4f min_point(roi_x_min_, roi_y_min_, roi_z_min_, 1.0);
    Eigen::Vector4f max_point(roi_x_max_, roi_y_max_, roi_z_max_, 1.0);

    // Créer et configurer le filtre CropBox
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(input_cloud);
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);
    crop_box.filter(*cloud_filtered);

    return cloud_filtered;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
RowClusterer::performEuclideanClustering(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_cloud);

    // Configuration du clustering euclidien
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    // Extraire chaque cluster
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

std::vector<RowClusterer::CropRow>
RowClusterer::detectCropRows(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
    std::vector<CropRow> crop_rows;

    for (size_t i = 0; i < clusters.size(); ++i)
    {
        const auto &cluster = clusters[i];

        // Vérifier que le cluster est assez long pour être un rang
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);

        double length = std::sqrt(
            std::pow(max_pt.x - min_pt.x, 2) +
            std::pow(max_pt.y - min_pt.y, 2));

        if (length < min_row_length_)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Cluster %zu too short (%.2f m), skipping", i, length);
            continue;
        }

        // Ajuster une ligne avec RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_distance_threshold_);
        seg.setMaxIterations(ransac_max_iterations_);

        // Contrainte optionnelle sur la direction de la ligne
        if (use_axis_constraint_)
        {
            Eigen::Vector3f axis(principal_axis_x_, principal_axis_y_, principal_axis_z_);
            axis.normalize();
            seg.setAxis(axis);
            seg.setEpsAngle(axis_angle_tolerance_ * M_PI / 180.0); // Convertir en radians

            RCLCPP_DEBUG(this->get_logger(),
                         "Using axis constraint: (%.2f, %.2f, %.2f) ±%.1f°",
                         axis[0], axis[1], axis[2], axis_angle_tolerance_);
        }

        seg.setInputCloud(cluster);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "No line found in cluster %zu", i);
            continue;
        }

        // Vérifier que la ligne a assez d'inliers (au moins 50% des points)
        double inlier_ratio = static_cast<double>(inliers->indices.size()) / cluster->points.size();
        if (inlier_ratio < min_inlier_ratio_)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Cluster %zu: insufficient inliers (%.1f%% < %.1f%%)",
                         i, inlier_ratio * 100, min_inlier_ratio_ * 100);
            continue;
        }

        // Créer l'objet CropRow
        CropRow row;
        row.cluster = cluster;
        row.coefficients = coefficients;

        // Point de départ de la ligne (point sur la ligne le plus proche de l'origine)
        row.start_point.x = coefficients->values[0];
        row.start_point.y = coefficients->values[1];
        row.start_point.z = coefficients->values[2];

        // Direction de la ligne
        row.direction.x = coefficients->values[3];
        row.direction.y = coefficients->values[4];
        row.direction.z = coefficients->values[5];

        // Calculer le centroïde du cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        row.centroid.x = centroid[0];
        row.centroid.y = centroid[1];
        row.centroid.z = centroid[2];

        row.length = length;
        row.num_points = cluster->points.size();

        crop_rows.push_back(row);
    }

    return crop_rows;
}

std::vector<std::vector<RowClusterer::CropRow>>
RowClusterer::groupParallelRows(const std::vector<CropRow> &crop_rows)
{
    std::vector<std::vector<CropRow>> row_groups;

    if (crop_rows.empty())
        return row_groups;

    // Algorithme simple de regroupement basé sur la position Y et l'orientation
    std::vector<bool> assigned(crop_rows.size(), false);

    for (size_t i = 0; i < crop_rows.size(); ++i)
    {
        if (assigned[i])
            continue;

        std::vector<CropRow> group;
        group.push_back(crop_rows[i]);
        assigned[i] = true;

        // Chercher les rangs parallèles proches
        for (size_t j = i + 1; j < crop_rows.size(); ++j)
        {
            if (assigned[j])
                continue;

            // Vérifier la distance entre les centroïdes (perpendiculaire aux rangs)
            double y_distance = std::abs(crop_rows[i].centroid.y - crop_rows[j].centroid.y);

            // Vérifier le parallélisme (produit scalaire des directions)
            double dot_product =
                crop_rows[i].direction.x * crop_rows[j].direction.x +
                crop_rows[i].direction.y * crop_rows[j].direction.y +
                crop_rows[i].direction.z * crop_rows[j].direction.z;

            double angle_diff = std::acos(std::abs(dot_product)) * 180.0 / M_PI;

            // Si les rangs sont parallèles (angle < 15°) et proches
            if (y_distance < max_row_distance_ && angle_diff < 15.0)
            {
                group.push_back(crop_rows[j]);
                assigned[j] = true;
            }
        }

        row_groups.push_back(group);
    }

    return row_groups;
}

void RowClusterer::publishClusters(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters,
    const std_msgs::msg::Header &header)
{
    // Créer un nuage avec des couleurs différentes pour chaque cluster
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < clusters.size(); ++i)
    {
        // Générer une couleur unique pour chaque cluster
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
    output_msg.header = header;

    clusters_publisher_->publish(output_msg);
}

void RowClusterer::publishRowMarkers(
    const std::vector<CropRow> &crop_rows,
    const std_msgs::msg::Header &header)
{
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < crop_rows.size(); ++i)
    {
        const auto &row = crop_rows[i];

        // Marker de ligne pour chaque rang
        visualization_msgs::msg::Marker line_marker;
        line_marker.header = header;
        line_marker.ns = "crop_rows";
        line_marker.id = i;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.05; // Épaisseur de la ligne

        // Couleur unique pour chaque rang
        line_marker.color.r = (i * 0.3f);
        line_marker.color.g = (i * 0.5f);
        line_marker.color.b = 1.0f;
        line_marker.color.a = 1.0;

        // Calculer les extrémités de la ligne
        geometry_msgs::msg::Point start_pt, end_pt;

        // Prolonger la ligne dans les deux directions
        double extension = row.length / 2.0;

        start_pt.x = row.start_point.x - row.direction.x * extension;
        start_pt.y = row.start_point.y - row.direction.y * extension;
        start_pt.z = row.start_point.z;

        end_pt.x = row.start_point.x + row.direction.x * extension;
        end_pt.y = row.start_point.y + row.direction.y * extension;
        end_pt.z = row.start_point.z;

        line_marker.points.push_back(start_pt);
        line_marker.points.push_back(end_pt);

        marker_array.markers.push_back(line_marker);

        // Marker de texte avec le numéro du rang
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = header;
        text_marker.ns = "crop_row_labels";
        text_marker.id = i + 1000;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = row.centroid.x;
        text_marker.pose.position.y = row.centroid.y;
        text_marker.pose.position.z = row.centroid.z + 0.5;
        text_marker.scale.z = 0.3;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "Row " + std::to_string(i + 1);

        marker_array.markers.push_back(text_marker);
    }

    markers_publisher_->publish(marker_array);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RowClusterer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}