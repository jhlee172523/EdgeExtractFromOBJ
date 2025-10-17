#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <cstdint> // for uint8_t
#include <Eigen/Dense> // Eigen 
#include <pcl/point_types.h> // for pcl::PointXYZ
#include <pcl/ModelCoefficients.h> // for pcl::ModelCoefficients
#include <pcl/point_cloud.h> // for pcl::PointCloud

/**
 * @brief vertex structure
 * xyz, color, label ID
 */
struct Vertex {
    Eigen::Vector3d pos; // (x, y, z)
    Eigen::Vector3i color; // (r, g, b)
    int label_id = -1; // label ID from color
};

/**
 * @brief mesh face structure
 * 3 indice of vertex
 */
struct Face {
    std::vector<int> vertex_indices; 
};

/**
 * @brief plane segment structure form RANSAC
 */
struct PlaneSegment {
    int id = -1; // plane ID
    pcl::ModelCoefficients::Ptr coefficients; // (Ax + By + Cz + D = 0)
    std::vector<int> inlier_indices; // indices of plane inlier
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // point cloud on plane 
};

#endif // TYPES_H
