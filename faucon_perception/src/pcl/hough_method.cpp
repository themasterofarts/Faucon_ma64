#include "faucon_perception/pcl/hough_method.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <cmath>
#include <limits>

namespace faucon::pclrow
{

    
    namespace
    {

        struct GridMapping
        {
            float x_min{0.f};
            float y_min{0.f};
            float resolution_m_per_px{0.02f}; 
            int width{0};
            int height{0};
        };

        // Construit une grille basée sur la bounding box XY du cluster
        GridMapping buildMappingFromCluster(const Cloud &cluster,
                                            float resolution_m_per_px,
                                            int padding_px,
                                            int max_img_dim)
        {
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(cluster, min_pt, max_pt);

            const float x_min = min_pt.x;
            const float x_max = max_pt.x;
            const float y_min = min_pt.y;
            const float y_max = max_pt.y;

            const float span_x = std::max(0.001f, x_max - x_min);
            const float span_y = std::max(0.001f, y_max - y_min);

            int w = static_cast<int>(std::ceil(span_x / resolution_m_per_px)) + 1 + 2 * padding_px;
            int h = static_cast<int>(std::ceil(span_y / resolution_m_per_px)) + 1 + 2 * padding_px;

            // Clamp pour éviter des images énormes
            const int scale = std::max(1, std::max(w, h) / std::max(1, max_img_dim));
            if (scale > 1)
            {
                // on augmente la taille des pixels (moins précis mais stable)
                resolution_m_per_px *= static_cast<float>(scale);
                w = static_cast<int>(std::ceil(span_x / resolution_m_per_px)) + 1 + 2 * padding_px;
                h = static_cast<int>(std::ceil(span_y / resolution_m_per_px)) + 1 + 2 * padding_px;
            }

            GridMapping m;
            m.x_min = x_min;
            m.y_min = y_min;
            m.resolution_m_per_px = resolution_m_per_px;
            m.width = w;
            m.height = h;
            return m;
        }

        inline bool worldToPixel(const GridMapping &m, float x, float y, int padding_px, int &u, int &v)
        {
            u = static_cast<int>(std::round((x - m.x_min) / m.resolution_m_per_px)) + padding_px;
            v = static_cast<int>(std::round((y - m.y_min) / m.resolution_m_per_px)) + padding_px;
            return (u >= 0 && u < m.width && v >= 0 && v < m.height);
        }

        // Convertit un (rho, theta) image en (rho, theta) world
        // Ici, HoughLines est appliqué sur un repère pixel.
        // On convertit ensuite la droite dans le repère monde XY.
        struct WorldLinePolar
        {
            float rho_m; // distance à l'origine (dans monde)
            float theta_rad;
        };

        // Droite en pixel : rho_px = u*cos(theta) + v*sin(theta)
        // avec u,v en pixels. On veut : rho_m = x*cos(theta) + y*sin(theta)
        // sachant : x = x_min + (u - padding)*res, y = y_min + (v - padding)*res
        WorldLinePolar pixelLineToWorld(const GridMapping &m, int padding_px, float rho_px, float theta)
        {
            const float c = std::cos(theta);
            const float s = std::sin(theta);
            const float res = m.resolution_m_per_px;

            // x = x_min + (u - pad)*res
            // y = y_min + (v - pad)*res
            // rho_px = u*c + v*s
            // rho_m  = x*c + y*s
            //       = [x_min*c + y_min*s] + [(u-pad)*res*c + (v-pad)*res*s]
            //       = [x_min*c + y_min*s - pad*res*(c+s)] + res*(u*c + v*s)
            //       = offset + res*rho_px
            const float offset = (m.x_min * c + m.y_min * s) - static_cast<float>(padding_px) * res * (c + s);
            WorldLinePolar out;
            out.rho_m = offset + res * rho_px;
            out.theta_rad = theta;
            return out;
        }

        // Distance point -> ligne (rho, theta) dans repère monde XY
        inline float pointLineDistanceXY(const pcl::PointXYZ &p, float rho_m, float theta)
        {
            const float c = std::cos(theta);
            const float s = std::sin(theta);
            const float v = p.x * c + p.y * s - rho_m;
            return std::fabs(v);
        }

        // Construit coeff PCL (x0,y0,z0, dx,dy,dz) à partir de (rho,theta) monde
        pcl::ModelCoefficients::Ptr makePclLineCoefficients(float rho_m, float theta)
        {
            // Droite: x*cosθ + y*sinθ = rho
            // Point le plus proche de l'origine sur la droite:
            // p0 = rho*[cosθ, sinθ]
            const float c = std::cos(theta);
            const float s = std::sin(theta);

            const float x0 = rho_m * c;
            const float y0 = rho_m * s;

            // Direction dans le plan: d = [-sinθ, cosθ]
            const float dx = -s;
            const float dy = c;

            auto coeff = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
            coeff->values.resize(6);
            coeff->values[0] = x0;
            coeff->values[1] = y0;
            coeff->values[2] = 0.0f;
            coeff->values[3] = dx;
            coeff->values[4] = dy;
            coeff->values[5] = 0.0f;
            return coeff;
        }

    } // namespace

    
    std::vector<CropRow> HoughLineDetector::detectLines(const std::vector<CloudPtr> &clusters) const
    {
        std::vector<CropRow> detected_rows;
        detected_rows.reserve(clusters.size());

        
        constexpr float kResolution_m_per_px = 0.02f; // 2 cm/pixel
        constexpr int kPadding_px = 10;
        constexpr int kMaxImgDim = 800;

        constexpr int kHoughThreshold = 50;     // votes min
        constexpr float kInlierDist_m = 0.05f;  // 5 cm
        constexpr float kMinRowLength_m = 1.0f; // gating longueur

        for (const auto &cluster : clusters)
        {
            if (!cluster || cluster->empty())
                continue;

            //  Mapping monde -> image basé sur bounding box
            const GridMapping map = buildMappingFromCluster(*cluster, kResolution_m_per_px, kPadding_px, kMaxImgDim);

            cv::Mat binary = cv::Mat::zeros(map.height, map.width, CV_8UC1);

            //  Rasterisation (points -> pixels)
            for (const auto &pt : cluster->points)
            {
                int u = 0, v = 0;
                if (worldToPixel(map, pt.x, pt.y, kPadding_px, u, v))
                {
                    binary.at<uchar>(v, u) = 255;
                }
            }

            
            // pour augmenter la cohérence des votes
            cv::dilate(binary, binary, cv::Mat(), cv::Point(-1, -1), 1);

            //  Hough
            std::vector<cv::Vec2f> lines_px;
            cv::HoughLines(binary, lines_px, 1.0, CV_PI / 180.0, kHoughThreshold);

            if (lines_px.empty())
                continue;

            //  Prendre la meilleure ligne (la première = souvent la plus votée)
            const float rho_px = lines_px.front()[0];
            const float theta = lines_px.front()[1];
            const WorldLinePolar line_world = pixelLineToWorld(map, kPadding_px, rho_px, theta);

            //  Inliers + segment tmin/tmax le long de la direction
            // direction unitaire d = [-sinθ, cosθ]
            const float dx = -std::sin(theta);
            const float dy = std::cos(theta);

            // point p0 proche de l’origine : rho*[cosθ, sinθ]
            const float x0 = line_world.rho_m * std::cos(theta);
            const float y0 = line_world.rho_m * std::sin(theta);

            // Projection scalaire t = d · (p - p0)
            float tmin = std::numeric_limits<float>::infinity();
            float tmax = -std::numeric_limits<float>::infinity();

            int inlier_count = 0;
            for (const auto &pt : cluster->points)
            {
                const float dist = pointLineDistanceXY(pt, line_world.rho_m, theta);
                if (dist > kInlierDist_m)
                    continue;

                ++inlier_count;

                const float px = pt.x - x0;
                const float py = pt.y - y0;
                const float t = dx * px + dy * py;

                tmin = std::min(tmin, t);
                tmax = std::max(tmax, t);
            }

            if (inlier_count == 0 || !std::isfinite(tmin) || !std::isfinite(tmax))
            {
                continue;
            }

            const float seg_len = std::fabs(tmax - tmin);
            if (seg_len < kMinRowLength_m)
            {
                continue;
            }

            
            CropRow row;
            row.cluster = cluster;
            row.coefficients = makePclLineCoefficients(line_world.rho_m, theta);

            row.start_point = pcl::PointXYZ(x0 + dx * tmin, y0 + dy * tmin, 0.0f);
            row.end_point = pcl::PointXYZ(x0 + dx * tmax, y0 + dy * tmax, 0.0f);
            row.direction = pcl::PointXYZ(dx, dy, 0.0f);
            row.length = seg_len;
            row.inlier_count = inlier_count;
            row.num_points = static_cast<int>(cluster->size());

            Eigen::Vector4f c4;
            pcl::compute3DCentroid(*cluster, c4);
            row.centroid = pcl::PointXYZ(c4.x(), c4.y(), c4.z());

            detected_rows.push_back(std::move(row));
        }

        return detected_rows;
    }

} // namespace faucon::pclrow
