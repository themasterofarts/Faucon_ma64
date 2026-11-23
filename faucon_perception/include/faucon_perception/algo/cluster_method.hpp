#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>



namespace faucon::filter
{
    struct ClusterConfig
    {
        // Configuration du clustering euclidien
        double cluster_tolerance = 0.3;
        int min_cluster_size = 10;
        int max_cluster_size = 15000;
    };

    class ClusterMethod
    {
    public:
        explicit ClusterMethod(ClusterConfig cfg) : cfg_(std::move(cfg)) {}
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CreateEuclideanClustering(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) const;
    private:
        ClusterConfig cfg_;
    };

    

} // namespace faucon::filter