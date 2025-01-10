#include <unordered_set>
#include "helper_functions.h"

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	// Perform defined numder of iteration to find the plane
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
        
        //Add points to form a plane 
		while (inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size()));

		float x1, x2, y1, y2, z1, z2, x3, y3, z3;

		auto iter = inliers.begin();

      	// Extract values to compute the coefficients
        // This is efficient way than accessing point struct every time
		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;
		iter++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;
		iter++;
		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;

		std::vector<float[3]> v1, v2;
      
		// Calculate coefficients of the plane
		float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float D = -1 * ((A * x1) + (B * y1) + (C * z1));

		for (int idx = 0; idx < cloud->points.size(); idx++)
		{
            // If point is alreadypart of plane skip it
			if (inliers.count(idx) > 0) 
				continue;

			float px, py, pz;
			px = cloud->points[idx].x;
			py = cloud->points[idx].y;
			pz = cloud->points[idx].z;
          
			// Calulate distance to plane for the point 
			float d = fabs((A * px + B * py + C * pz + D) / sqrt(A * A + B * B + C * C));
			
          	// If point ditance from the plane is with tolerance add it as inlier
			if (d <= distanceTol)
				inliers.insert(idx);
		}
		
        // Retain set of points closest to the plane  
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	return inliersResult;
}
