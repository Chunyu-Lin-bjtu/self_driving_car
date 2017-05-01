#include <iostream>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <stereo_msgs/DisparityImage.h>

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
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>


#include <visualization_msgs/MarkerArray.h>

class sdc{
private:

  ros::NodeHandle nh;

  /****** Image Transport for subcribing and Publishing Images****/
  image_transport::ImageTransport it;
  image_transport::Subscriber m_left_sub;

  image_transport::Publisher m_left_pub;

  /****** ROS subscriber for Disparity Images****/
  ros::Subscriber m_disp_sub;

  /****** Subscribers and Publishers for Lidar Pointcloud ****/
  ros::Subscriber pcl_sub;
  ros::Subscriber pcl_filtered_sub;
  ros::Subscriber pcl_seg_sub;
  ros::Subscriber pcl_assembled_cloud_sub;

  ros::Publisher pcl_assembled_cloud_pub;
  ros::Publisher pcl_filtered_pub;
  ros::Publisher pcl_ground_removal_pub;
  ros::Publisher pcl_seg_pub;
  ros::Publisher vis_pub;

  // image_transport::Publisher m_right_pub;

  /****** Image callback for Left camera Color****/
  void leftImgCB(const sensor_msgs::ImageConstPtr& image);

  /****** Callback for Disparity Images ****/
  void disparityCB(stereo_msgs::DisparityImageConstPtr msg);

  /****** Callback for Pointcloud from LIDAR****/
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);

  /****** Callback for Assembled Lidar Point Cloud****/
  void assembledCloudCB(const sensor_msgs::PointCloud2ConstPtr& input);


  /****** Callback for Removing the Ground Plane ****/
  void groundRemovalCB(const sensor_msgs::PointCloud2ConstPtr& input);

  /****** Callback  for Euclidean Cluster Extraction****/
  void segmentationCB(const sensor_msgs::PointCloud2ConstPtr& input);

  /****** Function for performing Voxel Grid Filtering  ****/
  void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /****** Function for performing Pass Through Filter  ****/
  void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);


public:
  /****** Function for generating a Organized Point cloud with the help of reprojectImageTo3D function in OpenCV  ****/
  void generateOrganizedCloud(const cv::Mat &dispImage, const cv::Mat &colorImage, const cv::Mat Qmat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  /****** Function for getting the Location of the Object with respect to the Camera Coordinates  ****/
  void getWorldCordinates(geometry_msgs::Point pixel_location);

  sdc() : it(nh)
  {
    /****** Left Camera Image Raw  ****/
    m_left_sub = it.subscribe("/sdc/camera/left/image_raw",1, &sdc::leftImgCB, this);

    /****** Disparity Image  ****/
    m_disp_sub = nh.subscribe("/sdc/camera/disparity", 1, &sdc::disparityCB, this);

    /****** Processed Image  ****/
    m_left_pub = it.advertise("/sdc/camera/left/image_processed", 1);

    /****** LIDAR Pointcloud  ****/
    pcl_sub =  nh.subscribe("/velodyne_points", 10, &sdc::cloudCB, this);

    /****** Transforming Lidar Data to Camera Frame ********/
    pcl_assembled_cloud_sub = nh.subscribe("/velodyne_points", 10, &sdc::assembledCloudCB, this);

    pcl_assembled_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/sdc/assembled_cloud", 1);

    /****** Filtered Pointcloud (Voxel grid filtering and Pass Through Filter)  ****/
    pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/sdc/filteredPointCloud", 1);

    pcl_filtered_sub =  nh.subscribe("/sdc/filteredPointCloud", 10, &sdc::groundRemovalCB, this);

    /****** Ground Plane Removed  ****/
    pcl_ground_removal_pub = nh.advertise<sensor_msgs::PointCloud2>("/sdc/GroundRemoved", 1);

    pcl_seg_sub = nh.subscribe("/sdc/GroundRemoved", 10, &sdc::segmentationCB, this);

    /****** Euclidean Cluster Extraction ****/
    pcl_seg_pub = nh.advertise<sensor_msgs::PointCloud2>("/sdc/EuclideanExtraction", 1);

    /****** Drawing Markers on the Segmented Point Cloud  ****/
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "/sdc/visualization_marker", 1 );

  }

};
