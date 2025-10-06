#ifndef PLANE_EXTRACTOR_H
#define PLANE_EXTRACTOR_H

#include "types.h"
#include <map>
#include <set>
#include <string>

/**
 * @brief Class : extract plane, generate Patch Adjacency Graph, caculate edges
 */
class PlaneExtractor {
public:
    /**
     * @brief Constructor
     * @param vertices Vertices of Input Mesh
     * @param faces Faces of Input Mesh
     */
    PlaneExtractor(std::vector<Vertex> vertices, std::vector<Face> faces);

    /**
     * @brief Entire Pipeline
     * (Group the Vertices -> Extract the Planes -> Generate the Patch Adjacency Graph -> Calculate the edges)
     * @param ransac_distance_threshold Threshold of RANSAC plane model
     */
    void process(double ransac_distance_threshold);

    /**
     * @brief Generate point cloud of result (extract plane)
     * @return Each planes(patches) and outliers -> different color point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredResultCloud();

    /**
     * @brief Return the calculated Edges
     * @return Vectors of Edges
     */
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& getIntersectionEdges() const;

private:
    // Input Data (Mesh)
    std::vector<Vertex> vertices_;
    std::vector<Face> faces_;

    // Check Data and Output Data
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> grouped_points_by_label_;
    std::map<int, std::set<int>> plane_adjacency_graph_; // Plane Adjacency Graph
    std::map<int, PlaneSegment> planes_by_label_; // Label ID and Plane
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> intersection_edges_;

    /**
     * @brief Group the Vertices by lablelled ID
     */
    void groupPointsByLabel();

    /**
     * @brief Build Plane Adjacency Graph
     */
    void buildAdjacencyGraph();

    /**
     * @brief Extract the plane using RANSAC on grouped Vertices
     * @param distance_threshold Threshold of RANSAC
     */
    void detectPlanes(double distance_threshold);


    /**
     * @brief 인접한 평면들 사이의 교선을 계산합니다.
     */
    void calculateIntersectionEdges();

    /**
     * @brief 시각화를 위해 고유하고 구별되는 색상들을 생성합니다.
     * @param num_colors 생성할 색상의 개수.
     * @return (r,g,b) 색상 값의 벡터.
     */
    std::vector<Eigen::Vector3i> generateDistinctColors(int num_colors);
};

#endif // PLANE_EXTRACTOR_H
