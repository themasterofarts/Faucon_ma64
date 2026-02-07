#include "faucon_perception/pcl/model_detect.hpp"


namespace faucon::detection
{

    void PointDetector::setRoiSphere(const RoiSphere& roi_sphere) 
    {
        PointDetector::roi_sphere_ = roi_sphere;
    }

    RoiSphere PointDetector::getRoiSphere() const
    {
        return roi_sphere_;
    }

    CloudPtr PointDetector::filterSphere(const CloudPtr& input_cloud, const RobotPose& robot_pose, RoiSphere roi_sphere) const
    {
        CloudPtr cloud_filtered(new Cloud);
        cloud_filtered->reserve(input_cloud->size());

        const float rmin2 = roi_sphere.r_min * roi_sphere.r_min;
        const float rmax2 = roi_sphere.r_max * roi_sphere.r_max;

        for (const auto& pt : input_cloud->points)
        {
            // Calculer la distance du point au robot
            float dx = pt.x - robot_pose.x;
            float dy = pt.y - robot_pose.y;
            float dz = pt.z - robot_pose.z;
            float distance = dx*dx + dy*dy + dz*dz;

            // Vérifier si la distance est dans la plage du ROI
            if (distance >= rmin2 && distance <= rmax2)
            {
                cloud_filtered->points.push_back(pt);
            }
        }  

        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = false;

        return cloud_filtered;
    }

} // namespace faucon::detection

