

#include <biotracking/biotracking.h>


Biotracking::Biotracking(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer), it_(nh)
{
    nh_.param("topic_point_cloud", topic_point_cloud, std::string("/camera/depth_registered/points"));
    nh_.param("rgb_image_topic", rgb_image_topic, std::string("/camera/rgb/image_rect_color"));
    nh_.param("depth_image_topic", depth_image_topic, std::string("/camera/depth_registered/image_raw"));
    nh_.param("depth_image_pub", depth_image_pub, std::string("/biotracking/raw_image"));
    
    nh_.param("lower_limit", lower_limit, 0.0);
    nh_.param("upper_limit", upper_limit, 1.0);
    
    pcl_cloud_publisher = nh_.advertise<PointCloud>("pcl_point_cloud", 10);
    pc2_subscriber = nh_.subscribe<sensor_msgs::PointCloud2>(topic_point_cloud, 1, 
			&Biotracking::processPointCloud2, this);
	calculateAvgService = nh_.advertiseService("calculateAvg", &Biotracking::calculateAvgImage, this);
    
    image_sub_ = it_.subscribe(depth_image_topic, 1, &Biotracking::imageCb, this);
    image_pub_ = it_.advertise(depth_image_pub, 1);
	working_image_pub_ = it_.advertise("/biotracking/working_image", 1);

//     cv::namedWindow(e);
	
	remainedImagesToCalcAvg = 100;
	int R = 640, C = 480;
	avg_image.create(R,C,CV_32FC1);
    
    isCalculateAvgSrvCalled = isAvgCalculated = false;
}

bool Biotracking::calculateAvgImage(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	isCalculateAvgSrvCalled = true;
    return true;
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

void Biotracking::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
	sensor_msgs::ImagePtr workingImgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", workingImg).toImageMsg();

	
    // cv::normalize(cv_ptr->image, depthImg->image, 1, 0, cv::NORM_MINMAX);
    
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
	working_image_pub_.publish(workingImgMsg);
}
