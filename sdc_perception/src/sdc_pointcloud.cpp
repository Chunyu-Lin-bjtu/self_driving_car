#include "sdc_pointcloud.h"

void sdc::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  sensor_msgs::PointCloud2 output;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*input, *cloud);

  voxelGridFilter(cloud);
  passThroughFilter(cloud);

  pcl::toROSMsg(*cloud, output);

  output.header.frame_id = "velodyne";

  pcl_filtered_pub.publish(output);

}

void sdc::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

  pcl::VoxelGrid<pcl::PointXYZ> vox_obj;

  vox_obj.setInputCloud(cloud->makeShared());

  vox_obj.setLeafSize(0.1f, 0.1f, 0.1f);

  vox_obj.filter(*cloud);

}

void sdc::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

  // pcl::PassThrough<pcl::PointXYZ> pass_x;
  // pass_x.setInputCloud(cloud);
  // pass_x.setFilterFieldName("x");
  // pass_x.setFilterLimits(-5, 30.0);
  // pass_x.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-1,5);
  pass_y.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(cloud);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(-2,3);
  pass_z.filter(*cloud);

}

void sdc::groundRemovalCB(const sensor_msgs::PointCloud2ConstPtr& input){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  sensor_msgs::PointCloud2 output;

  pcl::fromROSMsg(*input, *cloud);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.4);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud);

  pcl::toROSMsg(*cloud, output);

  output.header.frame_id = "velodyne";

  pcl_ground_removal_pub.publish(output);

}


void sdc::segmentationCB(const sensor_msgs::PointCloud2ConstPtr& input){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  sensor_msgs::PointCloud2 output;

  pcl::fromROSMsg(*input, *cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.3);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (900);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int centroid_size = cluster_indices.size();
  ROS_INFO(" %d", centroid_size);

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  int index = 0;
  double x = 0;
  double y = 0;
  double z = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_points (new pcl::PointCloud<pcl::PointXYZ>);

  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
      cloud_cluster->points.push_back(cloud->points[*pit]);

      }

      for(int i = 0; i < cloud_cluster->points.size(); i++) {

        x+=cloud_cluster->points[i].x;
        y+=cloud_cluster->points[i].y;
        z+=cloud_cluster->points[i].z;

      }

      centroid_points->width  = centroid_size;
      centroid_points->height = 1;
      centroid_points->points.resize (centroid_points->width * centroid_points->height);

      centroid_points->points[index].x = x / cloud_cluster->points.size();
      centroid_points->points[index].y = y / cloud_cluster->points.size();
      centroid_points->points[index].z = z / cloud_cluster->points.size();

      ROS_ERROR("Centroid Values : Index := %d   X : %0.2f, Y : %0.2f, Z : %0.2f", index, centroid_points->points[index].x, centroid_points->points[index].y, centroid_points->points[index].z);

      x = 0;
      y = 0;
      z = 0;

      index = index + 1;

      *cloud_out += *cloud_cluster;

    }

    ROS_WARN(" %d ", (int)centroid_points->points.size());


    visualization_msgs::MarkerArray marker_array_msg;

    marker_array_msg.markers.resize((int)centroid_points->points.size());
    for ( int i = 0; i < (int)centroid_points->points.size(); i++)
    {
        ROS_INFO("Hello I am here");
        marker_array_msg.markers[i].header.frame_id = "velodyne";
        marker_array_msg.markers[i].header.stamp = ros::Time();
        marker_array_msg.markers[i].ns = "Obstacles";
        marker_array_msg.markers[i].id = i;
        marker_array_msg.markers[i].type = visualization_msgs::Marker::CUBE;
        marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array_msg.markers[i].pose.position.x = centroid_points->points[i].x;
        marker_array_msg.markers[i].pose.position.y = centroid_points->points[i].y;
        marker_array_msg.markers[i].pose.position.z = centroid_points->points[i].z;
        marker_array_msg.markers[i].pose.orientation.x = 0.0;
        marker_array_msg.markers[i].pose.orientation.y = 0.0;
        marker_array_msg.markers[i].pose.orientation.z = 0.0;
        marker_array_msg.markers[i].pose.orientation.w = 1.0;
        marker_array_msg.markers[i].scale.x = 2.5;
        marker_array_msg.markers[i].scale.y = 1.5;
        marker_array_msg.markers[i].scale.z = 1.5;
        marker_array_msg.markers[i].color.a = 1.0;
        marker_array_msg.markers[i].color.r = 1.0;
        marker_array_msg.markers[i].color.g = 0.0;
        marker_array_msg.markers[i].color.b = 0.0;
        marker_array_msg.markers[i].lifetime = ros::Duration(0.1);
        ROS_INFO("Hello I am here");


    pcl::toROSMsg(*cloud_out, output);

    output.header.frame_id = "velodyne";
    pcl_seg_pub.publish(output);

    vis_pub.publish(marker_array_msg);

}

}

int main(int argc, char** argv){
  ros::init(argc, argv, "sdc_pointcloud");
  ros::NodeHandle nh;

  sdc obj(nh);

  while(ros::ok()){

    ros::spinOnce();
  }

  return 0;
}
