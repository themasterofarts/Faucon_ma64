/**
 * @file model.hpp
 * @author Franklin Y 
 * @brief Modèle pour la detection de points dans le rang
 * @version 0.1
 * @date 2026-06-02 
 * 
 */

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

#include <string>
#include <vector>


namespace faucon::detection
{

    using Cloud = pcl::PointCloud<pcl::PointXYZ>;
    using CloudPtr = Cloud::Ptr;

        struct RobotPose
        {
            float x;
            float y;
            float z;
            float roll;
            float pitch;
            float yaw;
        };

        struct RoiSphere
        {
            float r_min;
            float r_max;
            
        };


        class PointDetector
        {
        public:

            explicit PointDetector(RoiSphere roi_sphere) : roi_sphere_(roi_sphere) {};

            /**
             * @brief Set the Roi Sphere object
             */
            void setRoiSphere(const RoiSphere& roi_sphere);
            
            /**
             * @brief Get the Roi Sphere object
             * @return RoiSphere 
             */
            RoiSphere getRoiSphere() const;
            
            /**
             * @brief Filter the input point cloud based on the ROI sphere and robot pose
             * @param input_cloud The input point cloud to be filtered
             * @param robot_pose The current pose of the robot
             * @param roi_sphere The region of interest defined as a sphere
             * @return CloudPtr The filtered point cloud containing points within the ROI sphere
             */
            CloudPtr filterSphere(const CloudPtr& input_cloud, const RobotPose& robot_pose, RoiSphere roi_sphere) const;
            

        private:
            RoiSphere roi_sphere_;
        };

        
}