#pragma once

#include "faucon_perception/pcl/row_types.hpp"


namespace faucon::pclrow
{

    class HoughLineDetector
    {
    public:
        explicit HoughLineDetector() = default;

        std::vector<CropRow> detectLines(
            const std::vector<CloudPtr> &clusters) const;
    };

} // namespace faucon::pclrow