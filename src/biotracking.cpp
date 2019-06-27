
#include <biotracking/biotracking.h>


Biotracking::Biotracking(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer)
{
    nh_.param("topic_point_cloud", topic_point_cloud, std::string("/camera/depth_registered/points"));
    nh_.param("topic_image_rgb", topic_image_rgb, std::string("/camera/rgb/image_rect_color"));
    
    nh_.param("lower_limit", lower_limit, 0.0);
    nh_.param("upper_limit", upper_limit, 0.5);
    
    pcl_cloud_publisher = nh_.advertise<PointCloud>("pcl_point_cloud", 10);
    pc2_subscriber = nh_.subscribe<sensor_msgs::PointCloud2>(topic_point_cloud, 1, 
			&Biotracking::processPointCloud2, this);
	calculateAvgService = n.advertiseService("calculateAvg", &Biotracking::calculateAvgImage, this);
    
    image_sub_ = nh_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = nh_.advertise("/biotracking/raw_image", 1);
    subtract_image_pub_ = it_.advertise("/biotracking/subtract_image", 1);
	working_image_pub_ = nh_.advertise("/biotracking/working_image", 1);

    cv::namedWindow(OPENCV_WINDOW);
	
	remainedImagesToCalcAvg = 100;
	int R = 640, C = 480;
	avg_image.create(R,C,TYPE_32FC1);
}

void calculateAvgImage(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	isCalculateAvgSrvCalled = true;
}

void Biotracking::processPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloud_in, *pcl_pc2);
    PointCloud cloudXYZRGB;
    pcl::fromPCLPointCloud2(*pcl_pc2, cloudXYZRGB);
    
    PointCloud pass_through_filtered;
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(cloudXYZRGB.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(lower_limit, upper_limit);
    pass.setFilterLimitsNegative (false);
    pass.filter(pass_through_filtered);
    
    pcl_cloud_publisher.publish(pass_through_filtered);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	if (!isCalculateAvgSrvCalled) { return; }
	
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::CV_16UC1);
      // image_encodings: rostopic echo /camera/depth/image --noarr
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	
	if (remainedImagesToCalcAvg > 0) 
	{ 
		avg_image += 0.01 * cv_ptr->image;
		remainedImagesToCalcAvg--;
	}
	
	if (remainedImagesToCalcAvg == 0)
	{ 
		std::string file = "/home/azanov/avg_image.jpg";
        cv::imwrite(file, avg_image);
		remainedImagesToCalcAvg--;
	}

	if (remainedImagesToCalcAvg > 0) { return; }
	
	cv::Mat workingImg = avg_image - cv_ptr->image;
	cv::Mat erosion_dst;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
              cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
              cv::Point(erosion_size, erosion_size) );
 
	erode(workingImg, erosion_dst, element);
	cv::Mat subtract_dst = workingImg - erosion_dst;
	
	
    // cv::normalize(cv_ptr->image, depthImg->image, 1, 0, cv::NORM_MINMAX);
    
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", workingImg).toImageMsg();
	working_image_pub_.publish(msg);
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", subtract_dst).toImageMsg();
	subtract_image_pub_.publish(msg);
}
