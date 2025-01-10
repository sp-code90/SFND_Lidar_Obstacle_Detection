#ifndef __HELPER_FUNCTIONS__
#include <chrono>
#include <string>
#include "kdtree.h"
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Defined in cluster.cpp
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol);

// Defined in ransac.cpp
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);

#endif