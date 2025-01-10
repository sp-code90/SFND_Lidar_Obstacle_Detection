#include "helper_functions.h"

void check_proximity(std::vector<int>& cluster, const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, std::vector<bool> &processed_pt, int idx)
{
    // Mark point ID as processed
    processed_pt[idx] = true;
  
    // Add point to the cluster
    cluster.push_back(idx);

    // Search for nearby points to the point defined by idx
    std::vector<int> nearby_points = tree->search(points[idx], distanceTol);
    for (auto id : nearby_points)
    {
        // If point is not processed earlier check nearby points to the point
        if (!processed_pt[id])
        {
            // Check nearby point within distance tolerance
            check_proximity(cluster, points, tree, distanceTol, processed_pt, id);
        }
    }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed_points(points.size(), false);
    int idx = 0;
    // Process all points in the vector
    while (idx < points.size())
    {
        // If point is already part of another cluster skip it
        if (processed_points[idx])
        {
            idx++;
            continue;
        }
        std::vector<int> cluster;
        
        // Check nearby points which are within distance tolerance to the point defined by idx
        check_proximity(cluster, points, tree, distanceTol, processed_points, idx);
        
        // Push point cluster to the vector of point clusters
        clusters.push_back(cluster);
        idx++;
    }
    return clusters;
}
