#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/MarkerArray.h>

class sdc{
private:

  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Subscriber pcl_filtered_sub;
  ros::Subscriber pcl_seg_sub;

  ros::Publisher pcl_filtered_pub;
  ros::Publisher pcl_ground_removal_pub;
  ros::Publisher pcl_seg_pub;
  ros::Publisher vis_pub;

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
  void groundRemovalCB(const sensor_msgs::PointCloud2ConstPtr& input);
  void segmentationCB(const sensor_msgs::PointCloud2ConstPtr& input);

  void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  // void visualizeObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);


public:
  // Constructor

  sdc(ros::NodeHandle nh)
  {
    pcl_sub =  nh.subscribe("/velodyne_points", 10, &sdc::cloudCB, this);
    pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/sdc/filteredPointCloud", 1);

    pcl_filtered_sub =  nh.subscribe("/sdc/filteredPointCloud", 10, &sdc::groundRemovalCB, this);
    pcl_ground_removal_pub = nh.advertise<sensor_msgs::PointCloud2>("/sdc/GroundRemoved", 1);

    pcl_seg_sub = nh.subscribe("sdc/GroundRemoved", 10, &sdc::segmentationCB, this);
    pcl_seg_pub = nh.advertise<sensor_msgs::PointCloud2>("/sdc/EuclideanExtraction", 1);

    vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "/sdc/visualization_marker", 1 );

  }

};
