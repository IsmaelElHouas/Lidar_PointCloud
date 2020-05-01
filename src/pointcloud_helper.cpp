#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include "pointcloud_helper.h"



pcl::PCLPointCloud2 *voxelGrid(const pcl::PCLPointCloud2ConstPtr &cloudPtr, pcl::PCLPointCloud2 *cloud_filtered)
{
   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
   sor.setInputCloud(cloudPtr);
   sor.setLeafSize(0.01f, 0.01f, 0.01f);
   sor.filter(*cloud_filtered);
   return cloud_filtered;
}

pcl::PCLPointCloud2 *passThrough(const pcl::PCLPointCloud2ConstPtr &cloudPtr, pcl::PCLPointCloud2 *cloud_passFiltered)
{
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 13.0);
    pass.filter(*cloud_passFiltered);
    return cloud_passFiltered;
}  


pcl::PCLPointCloud2 *removeOutlier(const pcl::PCLPointCloud2ConstPtr &cloudPtr, pcl::PCLPointCloud2 *cloud_outlierFiltered)
{
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setMeanK(10);
    sor.setStddevMulThresh(0.2);
    sor.filter(*cloud_outlierFiltered);
    return cloud_outlierFiltered; 
}  



pcl::PointCloud<pcl::PointXYZRGB>::Ptr euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloudPtr, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_segmented)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(point_cloudPtr);
   
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08); 
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25);
    ec.setSearchMethod(tree);
    ec.setInputCloud(point_cloudPtr);
    ec.extract(cluster_indices);

    int j= 0;
 
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	  {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                pcl::PointXYZRGB point;
                point.x = point_cloudPtr->points[*pit].x;
                point.y = point_cloudPtr->points[*pit].y;
                point.z = point_cloudPtr->points[*pit].z;

              
                point_cloud_segmented->push_back(point);
			
            }
        j++;
    }
   
   
   return point_cloud_segmented;
  
}



pcl::PCLPointCloud2 *segmentation(const pcl::PCLPointCloud2ConstPtr &cloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_SAC(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::fromPCLPointCloud2(cloud_filtered,*cloud_SAC);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold(0.01);

    int i=0, nr_points = (int) cloud_SAC->points.size ();
    while (cloud_SAC->points.size () > 0.3 * nr_points)
    {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_SAC);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_SAC);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the line surface
    extract.filter (*cloud_line);
    std::cout << "PointCloud representing the line component: " << cloud_line->points.size () << " data points." << std::endl;

    // Remove the line inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_SAC = *cloud_f;
    }
}  

