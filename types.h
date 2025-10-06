#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <cstdint> // for uint8_t
#include <Eigen/Dense> // Eigen 라이브러리 포함
#include <pcl/point_types.h> // for pcl::PointXYZ
#include <pcl/ModelCoefficients.h> // for pcl::ModelCoefficients
#include <pcl/point_cloud.h> // for pcl::PointCloud

/**
 * @brief 3D 정점(Vertex)을 나타내는 구조체.
 * 위치, 색상, 라벨 ID를 포함합니다.
 */
struct Vertex {
    Eigen::Vector3d pos; // 정점의 3D 위치 (x, y, z)
    Eigen::Vector3i color; // 정점의 색상 (r, g, b)
    int label_id = -1; // 색상으로부터 변환된 고유 라벨 ID
};

/**
 * @brief 메쉬의 면(Face)을 나타내는 구조체.
 * 3개의 정점 인덱스로 구성됩니다.
 */
struct Face {
    std::vector<int> vertex_indices; // 면을 구성하는 정점들의 인덱스
};

/**
 * @brief RANSAC으로 검출된 평면 세그먼트를 나타내는 구조체.
 */
struct PlaneSegment {
    int id = -1; // 평면의 고유 ID
    pcl::ModelCoefficients::Ptr coefficients; // 평면의 방정식 (Ax + By + Cz + D = 0)
    std::vector<int> inlier_indices; // 평면에 속하는 원본 정점들의 인덱스
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // 평면에 속하는 점들의 포인트 클라우드
};

#endif // TYPES_H
