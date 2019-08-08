
#ifndef BIOTRACKING_H
#define BIOTRACKING_H

#include <algorithm>
#include <vector>
#include <iostream>
#include <math.h>
#include <cstdlib>
#include <fstream>
#include <tuple>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <opencv2/video/background_segm.hpp>
// #include "opencv2/bgsegm.hpp"
#include "opencv2/bgsegm.hpp"

#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>


static const std::string OPENCV_WINDOW = "Image window";

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Biotracking
{
private:
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  ros::NodeHandle nh_;
  
  
  bool isAvgCalculated;
  bool isCalculateAvgSrvCalled;
  int remainedImagesToCalcAvg;
  
  cv::Mat avg_image;
  
  double lower_limit;
  double upper_limit;
  
  double person_hips;
  double person_neck;
  
  bool useCentroid;
  
  int background_threshold;
  double person_distance;
  
  bool shouldOutput;
  bool usePCL;
  
  std::string topic_point_cloud;
  std::string rgb_image_topic;
  std::string depth_image_topic;
  std::string depth_image_pub;
  
  double bottom_factor;

  
  double line_px, line_py, line_qx, line_qy;
  
  
  std::string frame_id;
  
  int left_r, left_c, right_r, right_c;
  
  int bottom_right_r, bottom_left_r, bottom_right_c, bottom_left_c;
//   
//   cv::Mat frame; //current frame
//   cv::Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
//   cv::Ptr<bgsegm::BackgroundSubtractorMOG> pMOG2; //MOG2 Background subtractor
  
  
  void processPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void rgbImageCb(const sensor_msgs::ImageConstPtr& msg);
  bool calculateAvgImage(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  visualization_msgs::Marker getRectangleMarker(double x, double y, double z);
  
  
  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);
  ros::Subscriber sub_camera_info_;
  std::string camera_info_topic;
  image_geometry::PinholeCameraModel model_;
  bool hasCameraInfo;
  
  
  ros::Subscriber pc2_subscriber;
  
  ros::Publisher pcl_cloud_publisher;
  ros::Publisher hips_plane_pub_;
  ros::Publisher neck_plane_pub_;
  ros::Publisher calculated_point_cloud_publisher;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber rgb_image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher subtract_image_pub_;
  image_transport::Publisher working_image_pub_;
  image_transport::Publisher avg_image_pub_;
  image_transport::Publisher raw_image_8u_pub_;
  image_transport::Publisher rgb_image_pub_;
  image_transport::Publisher mog2_pub_;
  image_transport::Publisher erosion_image_pub_;
  
  
  
  

public:
  ros::ServiceServer calculateAvgService;
  Biotracking(ros::NodeHandle nh);
  
  ~Biotracking() {}
  
};

#endif
