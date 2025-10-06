#include <iostream>
#include <string>
#include "file_io.h"
#include "plane_extractor.h"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file.obj>" << std::endl;
        return -1;
    }
    std::string input_filename = argv[1];

    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    Eigen::Vector3d min_bb, max_bb;
    if (!loadObjWithColors(input_filename, vertices, faces, min_bb, max_bb)) {
        return -1;
    }

    // Bounding Box
    if (!vertices.empty()) {
        std::cout << "\n--- Mesh Bounding Box ---" << std::endl;
        std::cout << "Min corner (X, Y, Z): (" << min_bb.x() << ", " << min_bb.y() << ", " << min_bb.z() << ")" << std::endl;
        std::cout << "Max corner (X, Y, Z): (" << max_bb.x() << ", " << max_bb.y() << ", " << max_bb.z() << ")" << std::endl;
        Eigen::Vector3d size = max_bb - min_bb;
        std::cout << "Size       (X, Y, Z): (" << size.x() << ", " << size.y() << ", " << size.z() << ")" << std::endl;
        
        // --- 바운딩 박스 OBJ 파일 저장 ---
        saveBoundingBoxObj("bounding_box.obj", min_bb, max_bb);
        // ---------------------------------

        std::cout << "-------------------------" << std::endl;
    }

    // PlaneExtractor
    PlaneExtractor extractor(vertices, faces);
    // RANSAC Threshold
    double ransac_threshold = 0.01; 
    extractor.process(ransac_threshold);

    // Colored Point Cloud as plane extraction by RANSAC 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud = extractor.getColoredResultCloud();
    saveResultPcd("ransac_result.pcd", result_cloud);

    // Edge
    const auto& edges = extractor.getIntersectionEdges();
    saveEdgesObj("intersection_edges.obj", edges);

    std::cout << "\nAll processes are complete. Check 'ransac_result.pcd' and 'intersection_edges.obj'." << std::endl;

    return 0;
}