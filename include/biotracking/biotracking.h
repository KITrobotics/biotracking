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
#include "opencv2/bgsegm.hpp"

#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>

#include <ros/package.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <iirob_filters/kalman_filter.h>
#include <biotracking/BioFeedbackMsg.h>

typedef iirob_filters::MultiChannelKalmanFilter<double> KalmanFilter;
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
  bool has_avg_image;
  bool hasCameraInfo;

  int remainedImagesToCalcAvg;
  int num_images_for_background;
  int R;
  int C;
  int rows_rollator_offset;

  float center_x;
  float center_y;
  float constant_x;
  float constant_y;

  double bad_point;
  double person_distance;
  double bottom_factor;
  double camera_angle_radians;
  double min_distance_near_camera;

  std::string cv_fs_image_id;
  std::string topic_point_cloud;
  std::string avg_image_path;
  std::string biotracking_path;
  std::string camera_info_topic;
  std::string from_pc2_depth_image_topic;
  std::string depth_image_sub_topic;

  cv::Mat avg_image;

  KalmanFilter* kalman_left_shoulder;
  KalmanFilter* kalman_right_shoulder;

  image_geometry::PinholeCameraModel model_;

  ros::ServiceServer calculateAvgService_;

  ros::Subscriber pc2_subscriber_;
  ros::Subscriber sub_camera_info_;

  ros::Publisher biofeedback_pub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_image_sub_;
  image_transport::Subscriber rgb_image_sub_;
  image_transport::Publisher subtract_image_pub_;
  image_transport::Publisher working_image_pub_;
  image_transport::Publisher avg_image_pub_;
  image_transport::Publisher rgb_image_pub_;
  image_transport::Publisher mog2_pub_;
  image_transport::Publisher erosion_image_pub_;
  image_transport::Publisher from_pc2_depth_image_pub_;

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void rgbImageCb(const sensor_msgs::ImageConstPtr& msg);
  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);
  bool calculateAvgImageCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

public:
  Biotracking(ros::NodeHandle nh);
  ~Biotracking() {}

};

#endif
