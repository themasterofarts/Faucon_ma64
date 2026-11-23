#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace faucon::filter
{

    struct RansacConfig
    {
        
        int max_iterations = 1000;
        double distance_threshold = 0.09;
        double min_inlier_ratio = 0.5;
    };

    struct RansacResult
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers;
    };

    class RansacFilter
    {
    public:
        explicit RansacFilter(RansacConfig cfg) : cfg_(std::move(cfg)) {}

        RansacResult apply(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) const;
    private:
        RansacConfig cfg_;

    };

    } // namespace faucon::filter