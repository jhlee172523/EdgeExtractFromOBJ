#ifndef FILE_IO_H
#define FILE_IO_H

#include <string>
#include <vector>
#include "types.h" 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Load OBJ (Mesh, colored vertices)
 * @param filename file name
 * @param out_vertices vertices of loaded mesh
 * @param out_faces faces of loaded mesh
 * @param out_min_bb min XYZ
 * @param out_max_bb Max XYZ
 */
bool loadObjWithColors(const std::string& filename, std::vector<Vertex>& out_vertices, std::vector<Face>& out_faces, Eigen::Vector3d& out_min_bb, Eigen::Vector3d& out_max_bb);

/**
 * @brief save Point Cloud as PCD
 * @param filename file name
 * @param cloud point cloud
 */
bool saveResultPcd(const std::string& filename, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

/**
 * @brief save Edges as OBJ
 * @param filename file name
 * @param edges edges
 */
bool saveEdgesObj(const std::string& filename, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges);

/**
 * @brief save modified mesh as OBJ
 * @param filename file name
 * @param vertices modified
 * @param faces origin
 */
bool saveModifiedMeshOBJ(const std::string& filename, const std::vector<Vertex>& vertices, const std::vector<Face>& faces);

/**
 * @brief save Bounding Box as obj
 * @param filename file name
 * @param min_bb min XYZ
 * @param max_bb Max XYZ
 */
bool saveBoundingBoxObj(const std::string& filename, const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb);


#endif // FILE_IO_H
