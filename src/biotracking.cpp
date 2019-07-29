

#include <biotracking/biotracking.h>

int C = 640, R = 480;
int out = 1;

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
    subtract_image_pub_ = it_.advertise("/biotracking/subtract_image", 1);
	working_image_pub_ = it_.advertise("/biotracking/working_image", 1);
	avg_image_pub_ = it_.advertise("/biotracking/avg_image", 1);

//     cv::namedWindow(e);
	
	remainedImagesToCalcAvg = 100;
	
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
		std::string file = "/home/student1/avg_image.jpg";
        cv::imwrite(file, avg_image);
		remainedImagesToCalcAvg--;
        ROS_INFO("Average image is calculated!");
	}

	if (remainedImagesToCalcAvg > 0) { return; }
	
	
	/*
     * workingImg
     */
// 	cv::Mat workingImg = cv_ptr->image - avg_image;

    cv::Mat workingImg;
    if (out == 1) {
        workingImg.create(R,C,CV_32FC1);
        std::string s = "";
        for(int r = 0; r < workingImg.rows; r++) {
            float* raw_image_ptr = cv_ptr->image.ptr<float>(r);
            float* avg_image_ptr = avg_image.ptr<float>(r);
            
            for(int c = 0; c < workingImg.cols; c++) {
                s += "[" + std::to_string(raw_image_ptr[c]) + ", " + std::to_string(avg_image_ptr[c]) + "(" + std::to_string(r) + ", " + std::to_string(c) + ")]";
            }
            s += "\n";
        }
        out = 0;
        ROS_INFO("image: %s", s.c_str());
    }
    else { workingImg = cv_ptr->image - avg_image; }
    
    
    
    
    
	cv::Mat erosion_dst;
    int erosion_size = 0;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
            cv::Point(erosion_size, erosion_size) );

	erode(workingImg, erosion_dst, element);
	cv::Mat subtract_dst = workingImg - erosion_dst;
	
	int left_r, left_c, right_r, right_c;
	left_r = left_c = right_r = right_c = -1;
	for(int r = 0; r < subtract_dst.rows; r++) {
        // We obtain a pointer to the beginning of row r
        // cv::Vec3b* ptr = subtract_dst.ptr<cv::Vec3b>(r);
		float* ptr = subtract_dst.ptr<float>(r);

		int left, right;
        for(int c = 0; c < subtract_dst.cols; c++) {
            // We invert the blue and red values of the pixel
            // ptr[c] = cv::Vec3b(ptr[c][2], ptr[c][1], ptr[c][0]);
			if (ptr[c] > 0.1) {
				left = c;
				break;
			}
        }
		for (int c = subtract_dst.cols - 1; c >= 0; c--) {
			if (ptr[c] > 0.1) {
				right = c;
				break;
			}
		}
		if (right - left > subtract_dst.cols / 2) {
			left_r = right_r = r;
			left_c = left;
			right_c = right;
		}
    }


    // cv::normalize(cv_ptr->image, depthImg->image, 1, 0, cv::NORM_MINMAX);
    
    // Draw an example circle on the video stream
//     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0),CV_FILLED, 8,0);
      cv::circle(cv_ptr->image, cv::Point(left_r, left_c), 10, CV_RGB(255,0,0),CV_FILLED, 8,0);
      cv::circle(cv_ptr->image, cv::Point(right_r, right_c), 10, CV_RGB(255,0,0),CV_FILLED, 8,0);

    // Update GUI Window
//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
	sensor_msgs::ImagePtr msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", workingImg).toImageMsg();
	working_image_pub_.publish(msg_to_pub);
	msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", subtract_dst).toImageMsg();
	subtract_image_pub_.publish(msg_to_pub);
	msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", avg_image).toImageMsg();
	avg_image_pub_.publish(msg_to_pub);
}
