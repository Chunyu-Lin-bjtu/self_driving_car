#include "sdc_perception.h"


std::vector <cv::Mat> disparityImageVect;
std::vector<cv::Mat> colorImageVect;
cv::Mat_<double> qMatrix;
bool read_dispImg = false;

void sdc::leftImgCB(const sensor_msgs::ImageConstPtr& msg){

  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat image;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  image = cv_ptr->image.clone();
	colorImageVect.push_back(image);

  m_left_pub.publish(cv_ptr->toImageMsg());

}

void sdc::disparityCB(stereo_msgs::DisparityImageConstPtr msg){

  std_msgs::Header disp_header;
	cv::Mat disparity_;
	bool new_disp;
	uint8_t depth=sensor_msgs::image_encodings::bitDepth(msg->image.encoding);  //the size of the disparity data can be 16 or 32

	if (depth == 32)

  {
	    cv::Mat_<float> disparity(msg->image.height, msg->image.width,
	                              const_cast<float*>(reinterpret_cast<const float*>(&msg->image.data[0])));

	    disparity_=disparity.clone();
	    disp_header=msg->image.header;

  }

	else if(depth == 16)

	{

	    cv::Mat_<uint16_t> disparityOrigP(msg->image.height, msg->image.width,const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&msg->image.data[0])));
	    cv::Mat_<float>   disparity(msg->image.height, msg->image.width);
	    disparity_ = disparityOrigP / 16.0f;
	    disparity_=disparity.clone();
	    disp_header=msg->image.header;

	}

	disparityImageVect.push_back(disparity_);
	if(disparityImageVect.size() != 0){
	read_dispImg = true;
	}

// ROS_INFO("Got disparity image");
}

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
void sdc::assembledCloudCB(const sensor_msgs::PointCloud2ConstPtr& input){

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();

    transform_mat(0,0) = 0.007533745;
    transform_mat(0,1) = -0.9999714;
    transform_mat(0,2) = -0.0006166020;
    transform_mat(0,3) = -0.004069766;
    transform_mat(1,0) = 0.01480249;
    transform_mat(1,1) = 0.00072807323;
    transform_mat(1,2) = -0.9998902;
    transform_mat(1,3) = -0.07631618;
    transform_mat(2,0) = 0.9998621;
    transform_mat(2,1) = 0.007523790;
    transform_mat(2,2) = 0.01480755;
    transform_mat(2,3) = -0.2717806;
    transform_mat(3,0) = 0;
    transform_mat(3,1) = 0;
    transform_mat(3,2) = 0;
    transform_mat(3,3) = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr assembled_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*cloud, *assembled_cloud, transform_mat);

    pcl::toROSMsg(*assembled_cloud, output);

    output.header.frame_id = "cam02";

    pcl_assembled_cloud_pub.publish(output);

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

      // ROS_ERROR("Centroid Values : Index := %d   X : %0.2f, Y : %0.2f, Z : %0.2f", index, centroid_points->points[index].x, centroid_points->points[index].y, centroid_points->points[index].z);

      x = 0;
      y = 0;
      z = 0;

      index = index + 1;

      *cloud_out += *cloud_cluster;

    }

    // ROS_WARN(" %d ", (int)centroid_points->points.size());


    visualization_msgs::MarkerArray marker_array_msg;

    marker_array_msg.markers.resize((int)centroid_points->points.size());
    for ( int i = 0; i < (int)centroid_points->points.size(); i++)
    {
        // ROS_INFO("Hello I am here");
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
        // ROS_INFO("Hello I am here");


    pcl::toROSMsg(*cloud_out, output);

    output.header.frame_id = "velodyne";
    pcl_seg_pub.publish(output);

    vis_pub.publish(marker_array_msg);

  }

}

void sdc::generateOrganizedCloud( const cv::Mat &dispImage, const cv::Mat &colorImage, const cv::Mat Qmat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

	cv::Mat xyz;

  int width=dispImage.cols;
  int height=dispImage.rows;

	cloud->height=height;
	cloud->width=width;
	cloud->resize(width*height);

	cv::reprojectImageTo3D(dispImage, xyz, Qmat, false);
	for(int u=0;u<dispImage.rows;u++)
		for(int v=0;v<dispImage.cols;v++)
		{
			if(dispImage.at<float>(cv::Point(v,u))==0.0)
				continue;
			cv::Vec3f cv_pt=xyz.at<cv::Vec3f>(cv::Point(v,u));
			pcl::PointXYZRGB pt;
			pt.x=cv_pt.val[0];
			pt.y=cv_pt.val[1];
			pt.z=cv_pt.val[2];
			cloud->at(v,u)=pt;
		}
}


void sdc::getWorldCordinates(geometry_msgs::Point pixel_location){

    pcl::PointXYZRGB pcl_point;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr organized_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    generateOrganizedCloud(disparityImageVect[0],colorImageVect[0],qMatrix,organized_cloud);
    pcl_point = organized_cloud->at(pixel_location.x, pixel_location.y);
    ROS_INFO("World Coordinates: X = %.2f, Y = %.2f, Z = %.2f", pcl_point.x, pcl_point.y, pcl_point.z);

}


int main(int argc, char** argv){

  ros::init(argc, argv, "sdc_perception");
  //*****Q-Matrix Calculation
  double fy = 721.5377;
  double fx = 721.5377;
  double cx = 609.559;
  double cy = 172.854;
  double tx = -0.4705;

  qMatrix=cv::Mat_<double>(4,4,0.0);
  qMatrix(0,0) =  fy*tx;
  qMatrix(1,1) =  fx*tx;
  qMatrix(0,3) = -fy*cx*tx;
  qMatrix(1,3) = -fx*cy*tx;
  qMatrix(2,3) =  fx*fy*tx;
  qMatrix(3,2) = -fy;
  qMatrix(3,3) = 0.0f;
  //**********

  sdc obj;
  geometry_msgs::Point pixel_location;
  pixel_location.x = 350;
  pixel_location.y = 300;

  while(ros::ok()){

    if (read_dispImg){
				// ROS_INFO("Going to Calculate Location of the Object");
        obj.getWorldCordinates(pixel_location);
			}
    ros::spinOnce();
  }

  return 0;
}
