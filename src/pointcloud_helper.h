#ifndef POINTCLOUD_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define POINTCLOUD_H

pcl::PCLPointCloud2 *voxelGrid (const pcl::PCLPointCloud2ConstPtr &cloudPtr, pcl::PCLPointCloud2 *cloud_filtered); 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloudPtr, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_segmented);
pcl::PCLPointCloud2 *passThrough(const pcl::PCLPointCloud2ConstPtr &cloudPtr, pcl::PCLPointCloud2 *cloud_passFiltered);
pcl::PCLPointCloud2 *removeOutlier(const pcl::PCLPointCloud2ConstPtr &cloudPtr, pcl::PCLPointCloud2 *cloud_outlierFiltered);
pcl::PCLPointCloud2 *segmentation(const pcl::PCLPointCloud2ConstPtr &cloudPtr, pcl::PCLPointCloud2 *cloud_segmented);
#endif
