#pragma once

#include "faucon_perception/pcl/row_types.hpp"

namespace faucon::pclrow
{

    class RowPipeline
    {
    public:
        explicit RowPipeline(RowDetectionConfig cfg);

        // input: cloud PCL (XYZ)
        RowPipelineResult process(const CloudPtr &input) const;

    private:
        RowDetectionConfig cfg_;

        CloudPtr applyRoi(const CloudPtr &input) const;
        std::vector<CloudPtr> clusterize(const CloudPtr &cloud) const;

        std::vector<CropRow> detectRows(const std::vector<CloudPtr> &clusters) const;

        // algos internes
        static void fitLinePcaXY(const Cloud &cloud, Eigen::Vector3f &p0, Eigen::Vector3f &dir_unit);
        bool passesAxisConstraint(const pcl::ModelCoefficients &coeff) const;
    };

} // namespace faucon::pclrow
