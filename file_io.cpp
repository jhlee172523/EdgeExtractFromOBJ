#include "file_io.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <pcl/io/pcd_io.h>
#include <iomanip> // for std::fixed and std::setprecision

/**
 * @brief Load OBJ (Mesh, colored vertices)
 * @param filename file name
 * @param out_vertices vertices of loaded mesh
 * @param out_faces faces of loaded mesh
 * @param out_min_bb min XYZ
 * @param out_max_bb Max XYZ
 */
bool loadObjWithColors(const std::string& filename, std::vector<Vertex>& out_vertices, std::vector<Face>& out_faces, Eigen::Vector3d& out_min_bb, Eigen::Vector3d& out_max_bb) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    bool is_first_vertex = true;
    std::string line;
    std::map<int, int> color_to_label_map;
    int next_label_id = 0;
    int vertex_count = 0;

    std::cout << "Starting to parse OBJ file: " << filename << std::endl;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string keyword;
        ss >> keyword;

        if (keyword == "v") {
            Vertex v;
            
            // 1. Read XYZ
            if (!(ss >> v.pos.x() >> v.pos.y() >> v.pos.z())) {
                continue;
            }

            // bounding box 
            if (is_first_vertex) {
                out_min_bb = v.pos;
                out_max_bb = v.pos;
                is_first_vertex = false;
            } else {
                out_min_bb.x() = std::min(out_min_bb.x(), v.pos.x());
                out_min_bb.y() = std::min(out_min_bb.y(), v.pos.y());
                out_min_bb.z() = std::min(out_min_bb.z(), v.pos.z());
                out_max_bb.x() = std::max(out_max_bb.x(), v.pos.x());
                out_max_bb.y() = std::max(out_max_bb.y(), v.pos.y());
                out_max_bb.z() = std::max(out_max_bb.z(), v.pos.z());

            } 

            // 2. Read Color
            double r_val, g_val, b_val;
            if (ss >> r_val >> g_val >> b_val) {
                // if there is color data
                // lower than 1 -> 0.0-1.0 scale 
                if (r_val >= 0.0 && r_val <= 1.0 && g_val >= 0.0 && g_val <= 1.0 && b_val >= 0.0 && b_val <= 1.0) {
                    // 0.0-1.0 -> 0-255
                    v.color.x() = static_cast<int>(r_val * 255.0);
                    v.color.y() = static_cast<int>(g_val * 255.0);
                    v.color.z() = static_cast<int>(b_val * 255.0);
                } else {
                    // 0-255 
                    v.color.x() = static_cast<int>(r_val);
                    v.color.y() = static_cast<int>(g_val);
                    v.color.z() = static_cast<int>(b_val);
                }
            } else {
                // if there is no color data
                v.color.x() = 255;
                v.color.y() = 255;
                v.color.z() = 255;
            }

            // 3. Generate Label ID
            int color_key = (v.color.x() << 16) | (v.color.y() << 8) | v.color.z();
            if (color_to_label_map.find(color_key) == color_to_label_map.end()) {
                color_to_label_map[color_key] = next_label_id++;
            }
            v.label_id = color_to_label_map[color_key];
            out_vertices.push_back(v);

            // [Debug] first 5 vertices
            if (vertex_count < 5) {
                 std::cout << "  - Parsed vertex " << vertex_count << ": "
                          << "pos(" << std::fixed << std::setprecision(3) << v.pos.x() << ", " << v.pos.y() << ", " << v.pos.z() << "), "
                          << "read_color(" << r_val << ", " << g_val << ", " << b_val << "), "
                          << "final_color(" << v.color.x() << ", " << v.color.y() << ", " << v.color.z() << "), "
                          << "label_id(" << v.label_id << ")" << std::endl;
            }
            vertex_count++;

        } else if (keyword == "f") {
            Face f;
            std::string face_val;
            while (ss >> face_val) {
                f.vertex_indices.push_back(std::stoi(face_val.substr(0, face_val.find('/'))) - 1);
            }
            out_faces.push_back(f);
        }
    }
    std::cout << "\nLoaded " << out_vertices.size() << " vertices and " << out_faces.size() << " faces." << std::endl;
    std::cout << "IMPORTANT: Found " << next_label_id << " unique labels (colors)." << std::endl;
    return true;
}

bool saveResultPcd(const std::string& filename, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if (pcl::io::savePCDFileBinary(filename, *cloud) == -1) {
        std::cerr << "Error: Could not save PCD file " << filename << std::endl;
        return false;
    }
    std::cout << "Saved RANSAC result to " << filename << std::endl;
    return true;
}

bool saveEdgesObj(const std::string& filename, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
        return false;
    }
    int vertex_index = 1;
    for (const auto& edge : edges) {
        file << "v " << edge.first.x() << " " << edge.first.y() << " " << edge.first.z() << std::endl;
        file << "v " << edge.second.x() << " " << edge.second.y() << " " << edge.second.z() << std::endl;
        file << "l " << vertex_index << " " << vertex_index + 1 << std::endl;
        vertex_index += 2;
    }
    std::cout << "Saved " << edges.size() << " intersection edges to " << filename << std::endl;
    return true;
}

/**
 * @brief save 12 Edges of bounding box
 */
bool saveBoundingBoxObj(const std::string& filename, const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
        return false;
    }
    Eigen::Vector3d size = 0.05 * (max_bb - min_bb);
    
    // 1. calculate 8 vertices of bounding box
    std::array<Eigen::Vector3d, 8> vertices;
    vertices[0] = Eigen::Vector3d(min_bb.x() - size.x(), min_bb.y() - size.y(), min_bb.z() - size.z());
    vertices[1] = Eigen::Vector3d(max_bb.x() + size.x(), min_bb.y() - size.y(), min_bb.z() - size.z());
    vertices[2] = Eigen::Vector3d(max_bb.x() + size.x(), max_bb.y() + size.y(), min_bb.z() - size.z());
    vertices[3] = Eigen::Vector3d(min_bb.x() - size.x(), max_bb.y() + size.y(), min_bb.z() - size.z());
    vertices[4] = Eigen::Vector3d(min_bb.x() - size.x(), min_bb.y() - size.y(), max_bb.z() + size.z());
    vertices[5] = Eigen::Vector3d(max_bb.x() + size.x(), min_bb.y() - size.y(), max_bb.z() + size.z());
    vertices[6] = Eigen::Vector3d(max_bb.x() + size.x(), max_bb.y() + size.y(), max_bb.z() + size.z());
    vertices[7] = Eigen::Vector3d(min_bb.x() - size.x(), max_bb.y() + size.y(), max_bb.z() + size.z());

    // 2. write 8 vertices
    file << "# Bounding Box Vertices" << std::endl;
    for (const auto& v : vertices) {
        file << "v " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    }

    // 3. write 12 edges 
    file << "\n# Bounding Box Edges" << std::endl;
    // Bottom face
    file << "l 1 2" << std::endl;
    file << "l 2 3" << std::endl;
    file << "l 3 4" << std::endl;
    file << "l 4 1" << std::endl;
    // Top face
    file << "l 5 6" << std::endl;
    file << "l 6 7" << std::endl;
    file << "l 7 8" << std::endl;
    file << "l 8 5" << std::endl;
    // Vertical edges
    file << "l 1 5" << std::endl;
    file << "l 2 6" << std::endl;
    file << "l 3 7" << std::endl;
    file << "l 4 8" << std::endl;

    std::cout << "Saved bounding box to " << filename << std::endl;
    return true;
}
