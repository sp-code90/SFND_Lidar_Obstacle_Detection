/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include <unordered_set>
#include "render/box.h"
#include "user_defined/helper_functions.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidarptr = new Lidar(cars, 0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = lidarptr->scan();
    // renderRays(viewer, lidarptr->position, pointcloud);
    renderPointCloud(viewer, pointcloud, "Point Cloud");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointprocessor;
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentCloud = pointprocessor.SegmentPlane(pointcloud,100,0.2);
    // renderPointCloud(viewer,SegmentCloud.first,"ObstCloud",Color(1,0,0));
    // renderPointCloud(viewer,SegmentCloud.second,"planeCloud",Color(0,1,0));

    //Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudclusters = pointprocessor.Clustering(SegmentCloud.first,1.0,3,30);
    int clusterid = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster:cloudclusters)
    {
        pointprocessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"ObstCloud"+std::to_string(clusterid),colors[clusterid]);
        Box bbox= pointprocessor.BoundingBox(cluster);
        renderBox(viewer, bbox, clusterid);
        ++clusterid;
    }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}



void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr  filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f , Eigen::Vector4f (-30.0f, -6.0f, -2.0f, 1), Eigen::Vector4f ( 100.0f, 6.0f, 5.0f, 1));
    /*
    Steps:
    1. Segment the filtered cloud into two parts, road and obstacles.
    2. Find object Clusters from the obstacle cloud
    3. Add bounding boxes to the object clusters
    */

    /* Step 1. Use user defined RANSAC plane segmentation method*/
    std::unordered_set<int> inliers = RansacPlane(filterCloud, 100, 0.2f);
    if(inliers.size() == 0)
        std::cout << "No inliers found in plane" << std::endl;

    // Extract Plane and Obstacle cloud based on inliers 
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudObstacle(new pcl::PointCloud<pcl::PointXYZI>());
    for (int index = 0; index < filterCloud->points.size(); index++)
    {
        pcl::PointXYZI point = filterCloud->points[index];
        if (inliers.count(index))
            cloudPlane->points.push_back(point);    // Add point to plane(road) cloud
        else
            cloudObstacle->points.push_back(point); // Add point to obstacle cloud
    }
  
    // Render pointclouds
    // renderPointCloud(viewer,SegmentCloud.first,"ObstCloud",Color(1,0,0));
    renderPointCloud(viewer,cloudPlane,"planeCloud",Color(0,1,0));

    /* Step 2: Use user defined clustering method*/
    KdTree *tree = new KdTree;
    std::vector<std::vector<float>> points;

    // Extract points from obstacle cloud and push to insert into Kd tree 
    for (int index = 0; index < cloudObstacle->points.size(); index++)
    {
        // Extract point details
        pcl::PointXYZI point = cloudObstacle->points[index];
        std::vector<float> p{point.x,point.y,point.z, point.intensity};
        points.push_back(p);         // Push point details to points Vector
        tree->insert(p, index);      // Add point details to Kd Tree
    }
    
    // Perform clustering of the obstacles
    std::vector<std::vector<int>> clusters_vec = euclideanCluster(points, tree, 0.5f);
    std::cout << "clustering found " << clusters_vec.size() << " clusters" << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudclusters;

    // Process each cluster found and extract point details
    for (const auto& cluster : clusters_vec)
    {
        // Ignore clusters with low point count
        if(cluster.size() < 10) // MinSize = 10
            continue;
        
        // process all point indices in the cluster and extract points from the obstacle cluster
        pcl::PointCloud<pcl::PointXYZI>::Ptr l_clustor(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto indice : cluster)
        {
            l_clustor->points.push_back(cloudObstacle->points[indice]);
        }
      
        // Add extracted points to vector of obstacle cloud cluster
        cloudclusters.push_back(l_clustor);
    }

    // Step 3: Add Bounding Box to object clusters
    int clusterid = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster:cloudclusters)
    {
        // Render each obstacle cloud individually
        renderPointCloud(viewer,cluster,"ObstCloud"+std::to_string(clusterid), Color(0,0,1));
      
        // Add bounding box to cluster
        Box bbox= pointProcessorI->BoundingBox(cluster);
       // Render bounding box 
        renderBox(viewer, bbox, clusterid);
        ++clusterid;
    }
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	// viewer->getRenderWindow()->GlobalWarningDisplayOff(); 

    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
//     simpleHighway(viewer);

#ifdef SINGLE_FRAME
	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
	cityBlock(viewer, pointProcessorI, inputCloudI);
  
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
#else
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }
#endif
}