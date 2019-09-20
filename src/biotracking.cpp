

#include <biotracking/biotracking.h>
#include <fstream>
#include <iostream>


int C = 640, R = 480; // x = c, y = r
int out = 1;
int ToCalcAvg = 10;
int bad_point = 9.9;



Biotracking::Biotracking(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer), it_(nh)
{
    
    
    nh_.param("topic_point_cloud", topic_point_cloud, std::string("/camera/depth_registered/points"));
    nh_.param("rgb_image_topic", rgb_image_topic, std::string("/camera/rgb/image_rect_color"));
    nh_.param("depth_image_topic", depth_image_topic, std::string("/camera/depth_registered/image_raw"));
    nh_.param("depth_image_pub", depth_image_pub, std::string("/biotracking/raw_image"));
    nh_.param("camera_frame_id", camera_frame_id, std::string("camera_depth_frame"));
    nh_.param("person_hips_frame_id", person_hips_frame_id, std::string("base_link"));
    nh_.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
    nh_.param("background_threshold", background_threshold, 3);
    nh_.param("person_distance", person_distance, 1.);
    
    nh_.param("shouldOutput", shouldOutput, false);
    nh_.param("usePCL", usePCL, true);
    nh_.param("useCentroid", useCentroid, true);
    
    nh_.param("lower_limit", lower_limit, 0.0);
    nh_.param("upper_limit", upper_limit, 1.0);
    
    nh_.param("person_hips", person_hips, 0.15);
    nh_.param("person_neck", person_neck, 0.52);
    
    nh_.param("hips_height_world", hips_height_world, 0.8);
    nh_.param("camera_height_world", camera_height_world, 0.6);
    
    nh_.param("camera_angle_radians", camera_angle_radians, 0.26);
    
    hips_plane_pub_ = nh_.advertise<visualization_msgs::Marker>("/biotracking/hips_plane_pub_", 10);
    neck_plane_pub_ = nh_.advertise<visualization_msgs::Marker>("/biotracking/neck_plane_pub_", 10);
    
    if (usePCL)
    {
        
        pcl_cloud_publisher = nh_.advertise<PointCloud>("pcl_point_cloud", 10);
        pc2_subscriber = nh_.subscribe<sensor_msgs::PointCloud2>(topic_point_cloud, 1, 
                                                                 &Biotracking::processPointCloud2, this);
    }
    else
    {
        calculated_point_cloud_publisher = nh_.advertise<PointCloud>("calculated_point_cloud", 10);
        calculateAvgService = nh_.advertiseService("calculateAvg", &Biotracking::calculateAvgImage, this);
        
        image_sub_ = it_.subscribe(depth_image_topic, 1, &Biotracking::imageCb, this);
        rgb_image_sub_ = it_.subscribe(rgb_image_topic, 1, &Biotracking::rgbImageCb, this);
        image_pub_ = it_.advertise(depth_image_pub, 1);
        subtract_image_pub_ = it_.advertise("/biotracking/subtract_image", 1);
        working_image_pub_ = it_.advertise("/biotracking/working_image", 1);
        avg_image_pub_ = it_.advertise("/biotracking/avg_image", 1);
        raw_image_8u_pub_ = it_.advertise("/biotracking/raw_image_8u", 1);
        rgb_image_pub_ = it_.advertise("/biotracking/rgb_image_", 1);
        mog2_pub_ = it_.advertise("/biotracking/mog2_image_", 1);
        erosion_image_pub_ = it_.advertise("/biotracking/erosion_image_", 1);
        
        
        sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, &Biotracking::cameraInfoCb, this);
        hasCameraInfo = false;
        
        
        remainedImagesToCalcAvg = ToCalcAvg;
        
        avg_image.create(R,C,CV_32FC1);
        
        isCalculateAvgSrvCalled = isAvgCalculated = false;
        
        bottom_right_r = bottom_left_r = bottom_right_c = bottom_left_c = -1;
        left_r = left_c = right_r = right_c = -1;
        line_px = -1;
    }
}


void Biotracking::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    if (!hasCameraInfo)
    {
        model_.fromCameraInfo(info_msg);
        center_x = model_.cx();
        center_y = model_.cy();
        
        double unit_scaling = depth_image_proc::DepthTraits<float>::toMeters( float(1) );
        constant_x = unit_scaling / model_.fx();
        constant_y = unit_scaling / model_.fy();
        hasCameraInfo = true;
    }
}


void Biotracking::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    if (!isCalculateAvgSrvCalled) { return; }
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    if (remainedImagesToCalcAvg > 0) 
    { 
        avg_image = (1 / ToCalcAvg) * cv_ptr->image;
        remainedImagesToCalcAvg--;
    }
    
    if (remainedImagesToCalcAvg == 0)
    { 
        std::string file = "/home/student1/avg_image.jpg";
        cv::imwrite(file, avg_image);
        remainedImagesToCalcAvg--;
        
        for(int r = 0; r < avg_image.rows; r++) {
            float* avg_image_raw_ptr = avg_image.ptr<float>(r);
            float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);
            
            for(int c = 0; c < avg_image.cols; c++) {
                // if NaN
                if (avg_image_raw_ptr[c] != avg_image_raw_ptr[c]) { 
                    avg_image_raw_ptr[c] = bad_point;
                }
                if (cv_ptr_image_ptr[c] != cv_ptr_image_ptr[c]) {
                    cv_ptr_image_ptr[c] = bad_point;
                }
            }
        }
        
        
        ROS_INFO("Average image is calculated!");
    }
    
    if (remainedImagesToCalcAvg > 0) { return; }
    
    cv::Mat avg_image_8u;
    //     avg_image.convertTo(avg_image_8u, CV_8UC1, 255, 0);
    avg_image.convertTo(avg_image_8u, CV_8UC1);
    
    cv::Mat raw_image_8u;
    cv_ptr->image.convertTo(raw_image_8u, CV_8UC1, 255, 0);
    
    /*
     * workingImg
     */
    // 	cv::Mat workingImg = cv_ptr->image - avg_image;
    cv::Mat diffMat;
    cv::absdiff(avg_image,cv_ptr->image,diffMat);
    
    cv::Mat workingImg;
    workingImg.create(R,C,CV_8UC1);
    std::string s = "", s2 = "";
    for(int r = 0; r < workingImg.rows; r++) {
        //         uchar* raw_image_ptr = raw_image_8u.ptr<uchar>(r);
        uchar* avg_image_ptr = avg_image_8u.ptr<uchar>(r);
        uchar* workingImg_ptr = workingImg.ptr<uchar>(r);
        float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);
        float* avg_image_raw_ptr = avg_image.ptr<float>(r);
        float* diffMat_ptr = diffMat.ptr<float>(r);
        
        for(int c = 0; c < workingImg.cols; c++) {
            //             if (std::abs(raw_image_ptr[c] - avg_image_ptr[c]) > background_threshold) {
            //                 workingImg_ptr[c] = 255;
            //             }
            //             else {
            //                 workingImg_ptr[c] = 0;
            //             }
            //             workingImg_ptr[c] = 255 * std::abs(avg_image_raw_ptr[c] - cv_ptr_image_ptr[c]);
            
            //             float diff = avg_image_raw_ptr[c] - cv_ptr_image_ptr[c];
            
            if (diffMat_ptr[c] > 0.1 && cv_ptr_image_ptr[c] < person_distance && cv_ptr_image_ptr[c] > 0.06)
            {
                workingImg_ptr[c] = 255;
            } else {
                workingImg_ptr[c] = 0;
            }
            
            if (out == 1) {
                s += "[actual: " + std::to_string(cv_ptr_image_ptr[c]) + ", absdiff: " + std::to_string(diffMat_ptr[c]) + ", avg: " + std::to_string(avg_image_raw_ptr[c]) + ", pos: (" + std::to_string(r) + ", " + std::to_string(c) + ")]";
                //                 s2 += "[" + std::to_string(raw_image_ptr[c]) + ", " + std::to_string(avg_image_ptr[c]) + "(" + std::to_string(r) + ", " + std::to_string(c) + ")]";
                
            }
        }
        
        if (out == 1) {
            s += "\n";
            
        }
    }
    if (out == 1) {
        std::ofstream ofstream("/home/student1/output.txt");
        ofstream << s;
        ofstream.close();
        out = 0;
        if (shouldOutput) {
            out = 1;
        }
    }
    
    
    cv::Mat erosion_dst;
    int erosion_size = 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size) );
    
    erode(workingImg, erosion_dst, element);
    cv::Mat subtract_dst = workingImg - erosion_dst;
    
    int bottom_r = subtract_dst.rows - 10;
    bottom_right_r = bottom_left_r = bottom_r; 
    uchar* bottom_ptr = subtract_dst.ptr<uchar>(bottom_r);
    for (int c = subtract_dst.cols / 2; c < subtract_dst.cols; c++)
    {
        if (bottom_ptr[c] == 0) 
        {
            bottom_right_c = c;
            break;
        }
    }
    
    for (int c = subtract_dst.cols / 2; c > 0; c--)
    {
        if (bottom_ptr[c] == 0) 
        {
            bottom_left_c = c;
            break;
        }
    }
    
    
    
    for(int r = 5; r < subtract_dst.cols / 2; r++) {
        uchar* ptr = subtract_dst.ptr<uchar>(r);
        int left, right;
        left = right = -1;
        for(int c = 0; c < subtract_dst.rows; c++) {
            if (ptr[c] > 0) {
                left = c;
                break;
            }
        }
        for (int c = subtract_dst.rows - 1; c >= 0; c--) {
            if (ptr[c] > 0) {
                right = c;
                break;
            }
        }
        
        if (left == -1 || right == -1) { continue; }
        
        if (right - left > bottom_factor * (bottom_right_c - bottom_left_c)) {
            left_r = right_r = r;
            left_c = left;
            right_c = right;
            
            break;
        }
    }
    
    
    
    int remove_left, remove_right;
    remove_left = remove_right = -1;
    for (int m = 0; m < avg_image.rows; m++)
    {
        for (int n = 0; n < avg_image.cols; n++)
        {
            uchar black_white = subtract_dst.ptr<uchar>(m)[n];
            if (black_white > 0) {
                remove_left = n;
            }
        }
        for (int n = avg_image.cols - 1; n > remove_left; n--)
        {
            uchar black_white = subtract_dst.ptr<uchar>(m)[n];
            if (black_white > 0) {
                remove_right = n;
            }
        }
        for (int n = remove_left + 1; n < remove_right; n++)
        {
            uchar* black_white_ptr = subtract_dst.ptr<uchar>(m);
            black_white_ptr[n] = 0;
        }
    }
    
    
    
    
    /**
     * Retrieving camera point of person hips height
     */
    //     geometry_msgs::TransformStamped transformStamped;
    //     geometry_msgs::PointStamped person_hips_height_world_point, person_hips_height_camera_point;
    //     person_hips_height_world_point.header.frame_id = camera_frame_id;
    //     person_hips_height_world_point.header.stamp = ros::Time::now();
    // 
    //     bool is_transformed = false;
    //     try{
    //         transformStamped = tfBuffer.lookupTransform(camera_frame_id, person_hips_frame_id, ros::Time(0));
    //         is_transformed = true;
    //     }
    //     catch (tf2::TransformException &ex) {
    //         ROS_WARN("Failure to lookup the transform for a point! %s\n", ex.what());
    //     }
    // 
    //     if (is_transformed) 
    //     {
    //         person_hips_height_world_point.point.x = 0.;
    //         person_hips_height_world_point.point.y = 0.;
    //         person_hips_height_world_point.point.z = person_hips;
    // 
    //         tf2::doTransform(person_hips_height_world_point, person_hips_height_camera_point, transformStamped);
    //     }
    /**
     * Retrieving camera point of person hips height
     */
    
    int horizontal_plane_y_int = showHorizontalPlane(cv_ptr->image, subtract_dst);
    
    if (horizontal_plane_y_int != -1) 
    {
        hips_height = calculateHipsHeight(cv_ptr->image, horizontal_plane_y_int, subtract_dst);
        calculateHipsLeftRightX(subtract_dst);
    }
    
    
    //     showFirstFromLeftPoints(cv_ptr->image, subtract_dst, msg->header.frame_id);
    
    
    
    if (line_px != -1) {
        cv::line(subtract_dst, cv::Point(line_px, line_py), cv::Point(line_qx, line_qy), cv::Scalar(255,255,255));
    }
    
    double x = upper_limit - lower_limit;
    double y = 0.0;
    visualization_msgs::Marker marker = getRectangleMarker(x, y, person_hips);
    hips_plane_pub_.publish(marker);
    marker = getRectangleMarker(x, y, person_neck);
    neck_plane_pub_.publish(marker);
    
    
    image_pub_.publish(cv_ptr->toImageMsg());
    sensor_msgs::ImagePtr msg_to_pub;
    
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", raw_image_8u).toImageMsg();
    raw_image_8u_pub_.publish(msg_to_pub);
    //     msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", avg_image_8u).toImageMsg();
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", avg_image).toImageMsg();
    avg_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", workingImg).toImageMsg();
    working_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", subtract_dst).toImageMsg();
    subtract_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", erosion_dst).toImageMsg();
    erosion_image_pub_.publish(msg_to_pub);
    // 	msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", fgMaskMOG2).toImageMsg();
    // 	mog2_pub_.publish(msg_to_pub);
    
    
    //     int waitInt;
    //     std::cin >> waitInt;
}

void Biotracking::calculateHipsLeftRightX(cv::Mat& black_white_image)
{   
    bool found_white = false;
    for (int n = 0; n < black_white_image.cols; n++)
    {
        uchar black_white = black_white_image.ptr<uchar>(hips_height)[n];
        if (black_white > 0) {
            hips_left_x = n;
            found_white = true;
        }
    }
    
    if (!found_white) {
        hips_left_x = hips_right_x = -1;
        return;
    }
    
    for (int n = black_white_image.cols - 1; n > hips_left_x; n--)
    {
        uchar black_white = black_white_image.ptr<uchar>(hips_height)[n];
        if (black_white > 0) {
            hips_right_x = n;
        }
    }
}

int Biotracking::calculateHipsHeight(cv::Mat& depth_image, int horizontal_plane_y_int, cv::Mat& black_white_image)
{
    int center_x_int = static_cast<int>(model_.cx());
    
    float hips_height_diff = hips_height_world;
    int hips_height_row = -1;
    
    float horizontal_plane_y = depth_image.ptr<float>(horizontal_plane_y_int)[center_x_int];
    
    float height_diff = camera_height_world - hips_height_world;
    
    int for_begin = horizontal_plane_y_int;
    int for_boundary = depth_image.rows;
    int for_increment = 1;
    
    if (height_diff < 0) // hips are higher than camera 
    {
        for_begin = 0;
        for_boundary = horizontal_plane_y_int;
        for_increment = -1;
    }
    
    for (int m = for_begin; m < for_boundary; m += for_increment)
    {
        float d = depth_image.ptr<float>(m)[center_x_int];
        
        if (d == 0 || d != d)
        {
            continue;
        }
        
        int image_row = m;
        if (height_diff < 0) // hips are higher than camera 
        {
            image_row = horizontal_plane_y_int - m;
        }
        
        float x = (center_x_int - center_x) * d * constant_x;
        float y = (image_row - center_y) * d * constant_y;
        
        float current_hips_height_diff = std::abs(std::abs(y - horizontal_plane_y) - std::abs(height_diff));
        
        if (current_hips_height_diff < hips_height_diff)
        {
            hips_height_diff = current_hips_height_diff;
            hips_height_row = m;
        }
    }
    
    cv::putText(black_white_image, "hips: (" + std::to_string(center_x_int) + ", " + 
    std::to_string(hips_height_row) + ")", cv::Point(center_x_int,hips_height_row), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);
    
    return hips_height_row;
}

int Biotracking::showFirstFromLeftPoints(cv::Mat& depth_image, cv::Mat& black_white_image, std::string frame_id)
{
    PointCloud cloud;
    cloud.header.frame_id = frame_id;
    
    bool hasText = false;
    int textEvery50 = 0;
    
    double camera_zero_plane_x, camera_zero_plane_y;
    double camera_zero_plane_z = -1;
    double horizontal_plane_y = -1;
    
    for (int m = 0; m < depth_image.rows; m++)
    {
        for (int n = 0; n < depth_image.cols; n++)
        {
            float d = depth_image.ptr<float>(m)[n];
            
            if (d == 0 || d != d)
            {
                continue;
            }
            
            uchar black_white = black_white_image.ptr<uchar>(m)[n];
            
            double z = d;
            double x = (n - center_x) * z * constant_x;
            double y = (m - center_y) * z * constant_y;
            
            if (!hasText && black_white > 0 && textEvery50 == 0) {
                cv::putText(black_white_image, "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")", cv::Point(n,m), 
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);
                textEvery50 = 50;
            }
            
            Point p;
            p.x = x; p.y = y; p.z = z;
            cloud.points.push_back(p);
            
        }
        
        if (textEvery50 > 0) {
            textEvery50--;
        }
    }
    
    if (line_py == avg_image.rows - 1) {
        line_px = -1;
    }
    
    calculated_point_cloud_publisher.publish(cloud);
}

int Biotracking::showHorizontalPlane(cv::Mat& depth_image, cv::Mat& black_white_image)
{
    int center_x_int = static_cast<int>(center_x);
    int center_y_int = static_cast<int>(center_y);
    
    float zero_plane_d = depth_image.ptr<float>(center_y_int)[center_x_int];
    
    float horizontal_plane_y = -1;
    float horizontal_plane_z = -1;
    
    float angle_diff = camera_angle_radians;
    
    for (int m = center_y_int + 1; m < depth_image.rows; m++)
    {
        float d = depth_image.ptr<float>(m)[center_x_int];
        
        if (d == 0 || d != d)
        {
            continue;
        }
        
        float y = (m - center_y) * d * constant_y;
        
        float angle = std::asin(y / d);
        
        float current_angle_diff = std::abs(angle - camera_angle_radians);
        
        if (current_angle_diff < angle_diff)
        {
            angle_diff = current_angle_diff;
            horizontal_plane_y = m;
            horizontal_plane_z = d;
        }
        
        if (current_angle_diff > angle_diff) { break; }
    }
    
    cv::putText(black_white_image, "zero plane: (" + std::to_string(center_x_int) + ", " + 
    std::to_string(center_y_int) + ", " + std::to_string(zero_plane_d) + ")", cv::Point(center_x_int,center_y_int), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);
    
    cv::putText(black_white_image, "horizontal plane: (" + std::to_string(center_x_int) + ", " + 
    std::to_string(horizontal_plane_y) + ", " + std::to_string(horizontal_plane_z) + ")", cv::Point(center_x_int,horizontal_plane_y), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);
    
    return horizontal_plane_y;
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
    
    double x = upper_limit - lower_limit;
    double y = 0.0;
    
    if (useCentroid) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(pass_through_filtered, centroid);
        x = centroid(2);
        y = -centroid(0);
    }
    
    visualization_msgs::Marker marker = getRectangleMarker(x, y, person_hips);
    hips_plane_pub_.publish(marker);
    marker = getRectangleMarker(x, y, person_neck);
    neck_plane_pub_.publish(marker);
}

void Biotracking::drawHipsCirles(cv::Mat& image)
{
    if (hips_height < 0 || hips_height >= image.rows || hips_left_x == -1 || hips_right_x == -1)
    {
        return;
    }
    
    cv::circle(image, cv::Point(hips_left_x, hips_height), 10, CV_RGB(255,0,0), CV_FILLED, 10,0);
    cv::circle(image, cv::Point((hips_left_x + hips_right_x) / 2, hips_height), 10, CV_RGB(255,0,0), CV_FILLED, 10,0);
    cv::circle(image, cv::Point(hips_right_x, hips_height), 10, CV_RGB(255,0,0), CV_FILLED, 10,0);
}

void Biotracking::rgbImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    if (!isCalculateAvgSrvCalled) { return; }
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    if (hips_height != -1)
    {
        drawHipsCirles(cv_ptr->image);
    }
    
    if (line_px != -1) {
        cv::line(cv_ptr->image, cv::Point(line_px, line_py), cv::Point(line_qx, line_qy), cv::Scalar(0,0,255));
    }
    
    //     if (left_c != -1 && left_r != -1) 
    //         cv::circle(cv_ptr->image, cv::Point(left_c, left_r), 10, CV_RGB(255,0,0), CV_FILLED, 10,0);
    //     
    //     if (right_c != -1 && right_r != -1) 
    //         cv::circle(cv_ptr->image, cv::Point(right_c, right_r), 10, CV_RGB(255,0,0), CV_FILLED, 10,0);
    //     
    //     if (bottom_left_c != -1 && bottom_left_r != -1) 
    //         cv::circle(cv_ptr->image, cv::Point(bottom_left_c, bottom_left_r), 10, CV_RGB(0,255,0), CV_FILLED, 10,0);
    //     
    //     if (bottom_right_c != -1 && bottom_right_r != -1) 
    //         cv::circle(cv_ptr->image, cv::Point(bottom_right_c, bottom_right_r), 10, CV_RGB(0,255,0), CV_FILLED, 10,0);
    
    rgb_image_pub_.publish(cv_ptr->toImageMsg());
}

visualization_msgs::Marker Biotracking::getRectangleMarker(double x, double y, double z)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = camera_frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = nh_.getNamespace();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale.x = 0.9;
    marker.scale.y = 0.9;
    marker.scale.z = 0.001;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    return marker;
}

