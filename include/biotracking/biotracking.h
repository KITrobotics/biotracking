
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

#include <ros/package.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <iirob_filters/kalman_filter.h>
#include <biotracking/BioFeedbackMsg.h>

typedef iirob_filters::MultiChannelKalmanFilter<double> KalmanFilter;
static const std::string OPENCV_WINDOW = "Image window";

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;


struct SLine
{
    SLine():
        numOfValidPoints(0),
        params(-1.f, -1.f, -1.f, -1.f)
    {}
    cv::Vec4f params;//(cos(t), sin(t), X0, Y0)
    int numOfValidPoints;
};


class Biotracking
{
private:
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  ros::NodeHandle nh_;
  
  KalmanFilter* kalman_left_shoulder;
  KalmanFilter* kalman_right_shoulder;
  
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
  
  geometry_msgs::Point shoulder_left_pt;
  geometry_msgs::Point shoulder_right_pt;
  geometry_msgs::Point hips_left_pt;
  geometry_msgs::Point hips_right_pt;
  
  bool has_avg_image;
  
  int shoulder_window_size;
  
  std::string topic_point_cloud;
  std::string rgb_image_topic;
  std::string depth_image_topic;
  std::string depth_image_pub;
  
  double bottom_factor;
  double camera_angle_radians;
  
  double line_px, line_py, line_qx, line_qy;
  double hips_height_world;
  double camera_height_world;
  
  float center_x;
  float center_y;

  float constant_x;
  float constant_y;
  
  int hips_height;
  int hips_left_x;
  int hips_right_x;
  
  std::vector<Point> left_points;
	std::vector<Point> left_points_positions;
	std::vector<Point> right_points;
	std::vector<Point> right_points_positions;
	std::vector<int> left_slopes_indices;
	std::vector<int> right_slopes_indices;
  
  
  float left_line_px;
  float left_line_py;
  float left_line_qx;
  float left_line_qy;
    
  int shoulder_left_r;
  int shoulder_left_c;
  int shoulder_right_r;
  int shoulder_right_c;
  
  
  std::string camera_frame_id;
  std::string person_hips_frame_id;
  
  std::string avg_image_path;
  
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
  int showHorizontalPlane(cv::Mat& depth_image, cv::Mat& black_white_image);
  int calculateHipsHeight(cv::Mat& depth_image, cv::Mat& black_white_image);
  int showFirstFromLeftPoints(cv::Mat& depth_image, cv::Mat& black_white_image, std::string frame_id);
  void calculateHipsLeftRightX(cv::Mat& depth_image, cv::Mat& black_white_image);
  void drawHipsCirles(cv::Mat& image);
  void calculateSlopeLines(cv::Mat& black_white_image, cv::Mat& depth_image);
  void drawShoulderLine(cv::Mat& image);
  void drawSlopeCircles(cv::Mat& image);
  cv::Mat calculateHorizontalLines(cv::Mat& black_white_image);
  void clearVectors();
  void setUpVariables();
  cv::Mat calculateTopPoints(cv::Mat& black_white_image);
  void drawFirstLineWithEnoughPoints(cv::Mat black_white_image);
  void drawLeftLine(cv::Mat& image);
  void drawShoulderCircles(cv::Mat& image);
  
  void calculateShoulderPoints(cv::Mat& black_white_image, cv::Mat& result, int& shoulder_x, int& shoulder_y, bool isRightShoulder);
  void updateKalman(float x, float y, int& shoulder_x, int& shoulder_y);
  void drawFitLine(std::vector<cv::Point>& nzPoints, cv::Mat& black_white_image);
  
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
  ros::Publisher biofeedback_pub;

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
  image_transport::Publisher horizontal_pub_;
  
  
  

public:
  ros::ServiceServer calculateAvgService;
  Biotracking(ros::NodeHandle nh);
  cv::Vec4f TotalLeastSquares(
    std::vector<cv::Point>& nzPoints,
    std::vector<int> ptOnLine);
  SLine LineFitRANSAC(
    float t,//distance from main line
    float p,//chance of hitting a valid pair
    float e,//percentage of outliers
    int T,//number of expected minimum inliers 
    std::vector<cv::Point>& nzPoints);
  ~Biotracking() {}
  
};

static std::array<double, 3> cross(const std::array<double, 3> &a, 
	const std::array<double, 3> &b)
{
	std::array<double, 3> result;
	result[0] = a[1] * b[2] - a[2] * b[1];
	result[1] = a[2] * b[0] - a[0] * b[2];
	result[2] = a[0] * b[1] - a[1] * b[0];
	return result;
}

static double point_to_line_distance(const cv::Point &p, const cv::Vec4f &line)
{
	std::array<double, 3> pa{ { line[0], line[1], 1 } };
	std::array<double, 3> pb{ { line[2], line[3], 1 } };
	std::array<double, 3> l = cross(pa, pb);
	return std::abs((p.x * l[0] + p.y * l[1] + l[2])) * 1.0 /
		std::sqrt(double(l[0] * l[0] + l[1] * l[1]));
}

#endif
