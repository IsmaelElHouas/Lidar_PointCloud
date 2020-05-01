#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "sensor_msgs/PointCloud.h"
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/search/search.h>
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl/filters/frustum_culling.h>
#include <pcl/common/eigen.h>
#include <iostream>
#include <Eigen/Dense>
#include "pointcloud_helper.h"

using namespace Eigen;


class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  ros::Subscriber scan_sub_ ;  
  ros::Publisher pcl_pub_;
  
  LaserScanToPointCloud(ros::NodeHandle n) {
    scan_sub_ = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LaserScanToPointCloud::scanCallback, this);
    pcl_pub_ = n.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);  
}


  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    
    //Convert LaserScan to PointCloud 
    projector_.projectLaser(*scan_in, cloud);
    
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
 
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2* cloud_outlierFiltered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2* cloud_passFiltered = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(cloud, *cloud2);

    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromPCLPointCloud2(*cloud2, input_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Frustrum  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(input_cloud, *cloud_Frustrum);
 
    //FrustrumCulling filter
    pcl::FrustumCulling<pcl::PointXYZ> fc ;

    fc.setInputCloud(cloud_Frustrum);

    fc.setVerticalFOV(60);
    fc.setHorizontalFOV(90);
    fc.setNearPlaneDistance(0.0);
    fc.setFarPlaneDistance(13);  
/*
    Eigen::Matrix4f camera_pose(4,4);

    camera_pose.setZero ();
    Eigen::Matrix3f R;
    Eigen::Vector3f theta(0.0,0.0,0.0);
    R = Eigen::AngleAxisf (theta[0] * M_PI / 180, Eigen::Vector3f::UnitX ()) *
            Eigen::AngleAxisf (theta[1] * M_PI / 180, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (theta[2] * M_PI / 180, Eigen::Vector3f::UnitZ ());
    camera_pose.block (0, 0, 3, 3) = R;
    Eigen::Vector3f T;
    T (0) = -4; T (1) = 0; T (2) = 0;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
*/
//    Eigen::Vector4f pose_orig(0,0,0,0);

    Eigen::Matrix4f cam2robot;
    cam2robot << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    //Eigen::Matrix4f pose_new = pose_orig * cam2robot;    

    fc.setCameraPose(cam2robot);

    fc.filter(*target);
  
    pcl::toPCLPointCloud2(*target, *cloud2);
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);

   
    //Pass Through filter    
    cloud_passFiltered = passThrough(cloudPtr, cloud_passFiltered);
    pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud_passFiltered);

    //Remove outlier filter
    cloud_outlierFiltered = removeOutlier(cloud_ptr, cloud_outlierFiltered);
    pcl::PCLPointCloud2ConstPtr cloud_Ptr(cloud_outlierFiltered);

    // Perform Voxel Grid actual filtering
    cloud_filtered = voxelGrid(cloud_Ptr, cloud_filtered);

   
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(*cloud_filtered, point_cloud);

    pcl::copyPointCloud(point_cloud, *point_cloudPtr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

    //Euclidean Clustering
    point_cloud_segmented = euclideanCluster(point_cloudPtr, point_cloud_segmented);
    
    // Convert to ROS data type
    
    point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;
   
    if(point_cloud_segmented->size()) 
    {
       pcl::toPCLPointCloud2(*point_cloud_segmented, *cloud_filtered);
    }
    else
    {
       pcl::toPCLPointCloud2(*point_cloudPtr, *cloud_filtered);
    }

    sensor_msgs::PointCloud2 output;

    pcl_conversions::fromPCL(*cloud_filtered, output);
    //pcl_conversions::fromPCL(*cloud2, output);
    
    pcl_pub_.publish(output);

  }

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}
