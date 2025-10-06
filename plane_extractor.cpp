#include "plane_extractor.h"
#include <iostream>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

PlaneExtractor::PlaneExtractor(std::vector<Vertex> vertices, std::vector<Face> faces)
    : vertices_(std::move(vertices)), faces_(std::move(faces)) {}

void PlaneExtractor::process(double ransac_distance_threshold) {
    std::cout << "\n--- Starting Processing ---" << std::endl;
    groupPointsByLabel();
    buildAdjacencyGraph();
    detectPlanes(ransac_distance_threshold);
    calculateIntersectionEdges();
    std::cout << "--- Processing Finished ---\n" << std::endl;
}

void PlaneExtractor::groupPointsByLabel() {
    std::cout << "Step 1: Grouping points by label..." << std::endl;
    for (const auto& v : vertices_) {
        if (grouped_points_by_label_.find(v.label_id) == grouped_points_by_label_.end()) {
            grouped_points_by_label_[v.label_id] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }
        grouped_points_by_label_[v.label_id]->push_back(pcl::PointXYZ(v.pos.x(), v.pos.y(), v.pos.z()));
    }
}


void PlaneExtractor::buildAdjacencyGraph() {
    std::cout << "Step 2: Building plane adjacency graph from input mesh..." << std::endl;
    for (const auto& face : faces_) {
        // label ID from vertices of plane
        int label1 = vertices_[face.vertex_indices[0]].label_id;
        int label2 = vertices_[face.vertex_indices[1]].label_id;
        int label3 = vertices_[face.vertex_indices[2]].label_id;
        
        // 한 면에 서로 다른 라벨이 존재하면, 해당 라벨들은 인접한 것입니다.
        auto add_edge = [&](int u, int v) {
            if (u != v) {
                 plane_adjacency_graph_[u].insert(v);
                 plane_adjacency_graph_[v].insert(u);
            }
        };
        add_edge(label1, label2);
        add_edge(label1, label3);
        add_edge(label2, label3);
    }

    // return result(graph)
    std::cout << "Label Adjacency Graph:" << std::endl;
    for (auto const& [label_id, neighbors] : plane_adjacency_graph_) {
        std::cout << "  Label " << label_id << " is adjacent to: ";
        for (int neighbor_id : neighbors) {
            std::cout << neighbor_id << " ";
        }
        std::cout << std::endl;
    }
}


void PlaneExtractor::detectPlanes(double distance_threshold) {
    std::cout << "Step 3: Detecting one plane per label group using RANSAC..." << std::endl;
    for (auto const& [label_id, cloud] : grouped_points_by_label_) {
        if (cloud->points.size() < 3) continue; // 평면을 정의하기에 점이 너무 적음

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cerr << "Warning: Could not estimate a planar model for label " << label_id << std::endl;
            continue;
        }

        PlaneSegment plane;
        plane.id = label_id;
        plane.coefficients = coefficients;
        plane.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        
        // mapping for vertex indices of Input Mesh
        std::vector<int> original_indices;
        for (size_t i = 0; i < vertices_.size(); ++i) {
            if (vertices_[i].label_id == label_id) {
                original_indices.push_back(i);
            }
        }
        
        // Inlier에 해당하는 정점들의 원본 인덱스를 저장합니다.
        for (int index : inliers->indices) {
            plane.inlier_indices.push_back(original_indices[index]);
            plane.cloud->push_back(cloud->points[index]);
        }
        planes_by_label_[label_id] = plane;
    }
    std::cout << "Detected " << planes_by_label_.size() << " planes." << std::endl;
}

void PlaneExtractor::calculateIntersectionEdges() {
    std::cout << "Step 4: Calculating intersection edges for adjacent planes..." << std::endl;
    
    // 라벨 인접 그래프를 순회하며 교선을 계산합니다.
    for (const auto& [label1, neighbors] : plane_adjacency_graph_) {
        for (int label2 : neighbors) {
            // 중복 계산을 피하기 위해 (label1 < label2) 조건 사용
            if (label1 < label2) {
                // 두 라벨에 해당하는 평면이 모두 RANSAC으로 검출되었는지 확인
                if (planes_by_label_.count(label1) == 0 || planes_by_label_.count(label2) == 0) {
                    continue;
                }
                const auto& plane1 = planes_by_label_.at(label1);
                const auto& plane2 = planes_by_label_.at(label2);

                // 두 평면의 법선 벡터와 D 계수 추출
                Eigen::Vector3d n1(plane1.coefficients->values[0], plane1.coefficients->values[1], plane1.coefficients->values[2]);
                double d1 = plane1.coefficients->values[3];
                Eigen::Vector3d n2(plane2.coefficients->values[0], plane2.coefficients->values[1], plane2.coefficients->values[2]);
                double d2 = plane2.coefficients->values[3];

                // 교선의 방향 벡터 계산 (두 법선 벡터의 외적)
                Eigen::Vector3d dir = n1.cross(n2);
                if (dir.norm() < 1e-6) continue; // 평면들이 거의 평행함
                dir.normalize();

                // 교선 위의 한 점 계산 (연립 방정식 풀이)
                Eigen::Matrix<double, 3, 2> A; A << n1, n2;
                Eigen::Matrix2d AtA = A.transpose() * A;
                if (AtA.determinant() < 1e-6) continue;
                Eigen::Vector3d p0 = A * AtA.inverse() * Eigen::Vector2d(-d1, -d2);

                // 교선을 클리핑할 범위 계산 (두 평면의 점들을 모두 사용)
                pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>(*plane1.cloud + *plane2.cloud));
                if (combined_cloud->empty()) continue;

                double min_proj = std::numeric_limits<double>::max();
                double max_proj = std::numeric_limits<double>::lowest();
                for (const auto& pt : combined_cloud->points) {
                    Eigen::Vector3d p(pt.x, pt.y, pt.z);
                    double proj = (p - p0).dot(dir);
                    min_proj = std::min(min_proj, proj);
                    max_proj = std::max(max_proj, proj);
                }

                if (min_proj < max_proj) {
                    Eigen::Vector3d start_pt = p0 + min_proj * dir;
                    Eigen::Vector3d end_pt = p0 + max_proj * dir;
                    intersection_edges_.push_back({start_pt, end_pt});
                }
            }
        }
    }
}

std::vector<Eigen::Vector3i> PlaneExtractor::generateDistinctColors(int num_colors) {
    std::vector<Eigen::Vector3i> colors;
    if (num_colors == 0) return colors;
    for (int i = 0; i < num_colors; ++i) {
        double hue = (360.0 / num_colors) * i;
        double s = 0.9, v = 0.9;
        int hi = static_cast<int>(floor(hue / 60.0)) % 6;
        double f = hue / 60.0 - floor(hue / 60.0);
        int p = static_cast<int>(v * (1 - s) * 255);
        int q = static_cast<int>(v * (1 - f * s) * 255);
        int t = static_cast<int>(v * (1 - (1 - f) * s) * 255);
        int V = static_cast<int>(v * 255);

        switch (hi) {
            case 0: colors.emplace_back(V, t, p); break;
            case 1: colors.emplace_back(q, V, p); break;
            case 2: colors.emplace_back(p, V, t); break;
            case 3: colors.emplace_back(p, q, V); break;
            case 4: colors.emplace_back(t, p, V); break;
            case 5: colors.emplace_back(V, p, q); break;
        }
    }
    return colors;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneExtractor::getColoredResultCloud() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<bool> is_inlier(vertices_.size(), false);
    
    int max_label_id = 0;
    for (const auto& pair : planes_by_label_) {
        max_label_id = std::max(max_label_id, pair.first);
    }
    auto colors = generateDistinctColors(max_label_id + 1);

    for (const auto& [label_id, plane] : planes_by_label_) {
        const auto& color = colors[label_id];
        for (int v_idx : plane.inlier_indices) {
            pcl::PointXYZRGB point;
            point.x = vertices_[v_idx].pos.x();
            point.y = vertices_[v_idx].pos.y();
            point.z = vertices_[v_idx].pos.z();
            point.r = color.x(); point.g = color.y(); point.b = color.z();
            colored_cloud->push_back(point);
            is_inlier[v_idx] = true;
        }
    }

    // RANSAC의 Inlier에 속하지 않은 모든 점들을 회색으로 표시
    for (size_t i = 0; i < vertices_.size(); ++i) {
        if (!is_inlier[i]) {
            pcl::PointXYZRGB point;
            point.x = vertices_[i].pos.x();
            point.y = vertices_[i].pos.y();
            point.z = vertices_[i].pos.z();
            point.r = 128; point.g = 128; point.b = 128; // 아웃라이어는 회색
            colored_cloud->push_back(point);
        }
    }
    return colored_cloud;
}

const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& PlaneExtractor::getIntersectionEdges() const {
    return intersection_edges_;
}

