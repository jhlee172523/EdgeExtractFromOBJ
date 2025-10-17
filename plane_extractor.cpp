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
    /*
    for (const auto& face : faces_) {
        // label ID from vertices of plane
        int label1 = vertices_[face.vertex_indices[0]].label_id;
        int label2 = vertices_[face.vertex_indices[1]].label_id;
        int label3 = vertices_[face.vertex_indices[2]].label_id;
        
        // 
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
    */
    
    // 1단계: 두 라벨 사이의 공유 엣지 개수를 센다.
    // std::map<정렬된 라벨 쌍, 엣지 개수>
    std::map<std::pair<int, int>, int> edge_counts;

    for (const auto& face : faces_) {
        for (int i = 0; i < 3; ++i) {
            int v1_idx = face.vertex_indices[i];
            int v2_idx = face.vertex_indices[(i + 1) % 3]; // 면의 다음 정점

            int label1 = vertices_[v1_idx].label_id;
            int label2 = vertices_[v2_idx].label_id;

            if (label1 != label2) {
                // 항상 (작은 라벨, 큰 라벨) 순서로 키를 만들어 중복을 방지합니다.
                if (label1 > label2) std::swap(label1, label2);
                edge_counts[{label1, label2}]++;
            }
        }
    }

    // 2단계: 엣지 개수가 임계값을 넘는 경우에만 최종 인접 그래프에 추가한다.
    const int adjacency_threshold = 10; // 임계값. 10개 이상의 엣지를 공유해야 인접한 것으로 간주.
                                        // 이 값을 조절하여 인접성의 강도를 제어할 수 있습니다.

    std::cout << "Filtering adjacencies with threshold: " << adjacency_threshold << " shared edges" << std::endl;

    for (const auto& [labels, count] : edge_counts) {
        if (count >= adjacency_threshold) {
            int label1 = labels.first;
            int label2 = labels.second;
            plane_adjacency_graph_[label1].insert(label2);
            plane_adjacency_graph_[label2].insert(label1);
        }
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
        if (cloud->points.size() < 10) continue; // 평면을 정의하기에 점이 너무 적음

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
        
        // load Inlier points to original indices
        for (int index : inliers->indices) {
            plane.inlier_indices.push_back(original_indices[index]);
            plane.cloud->push_back(cloud->points[index]);
        }
        planes_by_label_[label_id] = plane;
    }
    std::cout << "Detected " << planes_by_label_.size() << " planes." << std::endl;
}

/**
 * @brief calculate edges using vicinity clipping 
 */
void PlaneExtractor::calculateIntersectionEdges() {
    std::cout << "Step 4: Calculating intersection edges for adjacent planes using vicinity clipping..." << std::endl;
    
    // vicinity threshold
    const double clipping_vicinity_threshold = 0.1; // 10cm 이내

    // calculate edges using PAG
    for (const auto& [label1, neighbors] : plane_adjacency_graph_) {
        for (int label2 : neighbors) {
            if (label1 < label2) {
                // 2 labels  are they extracted by RANSAC
                if (planes_by_label_.count(label1) == 0 || planes_by_label_.count(label2) == 0) {
                    continue;
                }
                const auto& plane1 = planes_by_label_.at(label1);
                const auto& plane2 = planes_by_label_.at(label2);

                // --- 1. calculate infinity line
                Eigen::Vector3d n1(plane1.coefficients->values[0], plane1.coefficients->values[1], plane1.coefficients->values[2]);
                double d1 = plane1.coefficients->values[3];
                Eigen::Vector3d n2(plane2.coefficients->values[0], plane2.coefficients->values[1], plane2.coefficients->values[2]);
                double d2 = plane2.coefficients->values[3];

                Eigen::Vector3d dir = n1.cross(n2);
                if (dir.norm() < 1e-6) continue;
                dir.normalize();

                Eigen::Matrix<double, 3, 2> A; A << n1, n2;
                Eigen::Matrix2d AtA = A.transpose() * A;
                if (AtA.determinant() < 1e-6) continue;
                Eigen::Vector3d p0 = A * AtA.inverse() * Eigen::Vector2d(-d1, -d2);

                // --- 2. collect vicinity point
                pcl::PointCloud<pcl::PointXYZ>::Ptr vicinity_points(new pcl::PointCloud<pcl::PointXYZ>);

                // add point of Plane 1 (close to Plane 2)
                for (const auto& pt : plane1.cloud->points) {
                    double dist_to_plane2 = std::abs(pt.x * n2.x() + pt.y * n2.y() + pt.z * n2.z() + d2);
                    if (dist_to_plane2 < clipping_vicinity_threshold) {
                        vicinity_points->push_back(pt);
                    }
                }
                // add point of Plane 1 (close to Plane 2)
                for (const auto& pt : plane2.cloud->points) {
                    double dist_to_plane1 = std::abs(pt.x * n1.x() + pt.y * n1.y() + pt.z * n1.z() + d1);
                    if (dist_to_plane1 < clipping_vicinity_threshold) {
                        vicinity_points->push_back(pt);
                    }
                }

                if (vicinity_points->empty()) continue;

                // --- 3. t_max t_min
                double min_proj = std::numeric_limits<double>::max();
                double max_proj = std::numeric_limits<double>::lowest();
                for (const auto& pt : vicinity_points->points) {
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

    // outlier gray
    for (size_t i = 0; i < vertices_.size(); ++i) {
        if (!is_inlier[i]) {
            pcl::PointXYZRGB point;
            point.x = vertices_[i].pos.x();
            point.y = vertices_[i].pos.y();
            point.z = vertices_[i].pos.z();
            point.r = 128; point.g = 128; point.b = 128; // outlier gray
            colored_cloud->push_back(point);
        }
    }
    return colored_cloud;
}

const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& PlaneExtractor::getIntersectionEdges() const {
    return intersection_edges_;
}

