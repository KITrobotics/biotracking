

#include <biotracking/biotracking.h>
#include <fstream>
#include <iostream>


const int C = 640, R = 480; // x = c, y = r
// int out = 1;
// const int ToCalcAvg = 10;
const float bad_point = 9.9;
const int white = 255;
std::string cv_fs_image_id = "avg_image";


Biotracking::Biotracking(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer), it_(nh)
{
    
    
    nh_.param("topic_point_cloud", topic_point_cloud, std::string("/camera/depth_registered/points"));
    nh_.param("rgb_image_topic", rgb_image_topic, std::string("/camera/rgb/image_rect_color"));
    nh_.param("depth_image_topic", depth_image_topic, std::string("/camera/depth_registered/image_raw"));
    nh_.param("depth_image_pub", depth_image_pub, std::string("/biotracking/raw_image"));
    nh_.param("camera_frame_id", camera_frame_id, std::string("camera_depth_frame"));
    nh_.param("person_hips_frame_id", person_hips_frame_id, std::string("base_link"));
    nh_.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
    nh_.param("num_images_for_background", num_images_for_background, 3);
    nh_.param("person_distance", person_distance, 1.);
    nh_.param("min_distance_near_camera", min_distance_near_camera, 0.6);
    
    nh_.param("shoulder_window_size", shoulder_window_size, 10);
    nh_.param("rows_rollator_offset", rows_rollator_offset, 140);
    
    nh_.param("out", out, 0);
    
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
    
    std::string avg_image_path_param;
    nh_.param("avg_image_path", avg_image_path_param, std::string("background_image/avg_image.yml"));
    biotracking_path = ros::package::getPath("biotracking");
    avg_image_path = biotracking_path + "/" + avg_image_path_param;
    
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
        rgb_image_pub_ = it_.advertise("/biotracking/rgb_image", 1);
        mog2_pub_ = it_.advertise("/biotracking/mog2_image", 1);
        erosion_image_pub_ = it_.advertise("/biotracking/erosion_image", 1);
        horizontal_pub_ = it_.advertise("/biotracking/horizontal_lines", 1);
        
        
        biofeedback_pub = nh_.advertise<biotracking::BioFeedbackMsg>("/biotracking/biofeedback", 10);
        
        sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, &Biotracking::cameraInfoCb, this);
        hasCameraInfo = has_avg_image = false;
        
        kalman_left_shoulder = new KalmanFilter();
        kalman_right_shoulder = new KalmanFilter();
        
        remainedImagesToCalcAvg = num_images_for_background;
        
        avg_image.create(R,C,CV_32FC1);
        
        isCalculateAvgSrvCalled = isAvgCalculated = false;
        
        bottom_right_r = bottom_left_r = bottom_right_c = bottom_left_c = -1;
        left_r = left_c = right_r = right_c = -1;
        line_px = -1;
    }
}

void Biotracking::drawFirstLineWithEnoughPoints(cv::Mat black_white_image)
{
    int max_points = 0;
    int max_left_point = black_white_image.rows - 1;
    int max_right_ponit = -1;
    for(int global_r = 0; global_r < black_white_image.rows * 0.4; global_r++) {
        int current_points = 0;
        int left_point = black_white_image.rows - 1;
        int right_point = -1;
        for(int r = global_r; r < global_r + 10; r++) {
            for(int c = 0; c < black_white_image.cols; c++) {
                uchar black_white = black_white_image.ptr<uchar>(r)[c];
                if (black_white > 0) 
                {
                    if (c < left_point) { left_point = c; }
                    if (c > right_point) { right_point = c; }
                    current_points++;
                }
            }
        }
        if (current_points > max_points)
        {
            max_points = current_points;
            max_left_point = left_point;
            max_right_ponit = right_point;
            continue;
        }
        if (left_point != black_white_image.rows - 1 && right_point != -1 && hips_left_x != -1 && hips_right_x != -1)
        {
            if (max_points > 90 && max_left_point < hips_left_x && max_right_ponit > hips_right_x)
            {
                cv::line(black_white_image, cv::Point(max_left_point, global_r + 5), cv::Point(max_right_ponit, global_r + 5), cv::Scalar(255,255,255), 1);
                break;
            }
        }
    }
}

void Biotracking::calculateShoulderPoints(cv::Mat& black_white_image, cv::Mat& result, int& shoulder_x, int& shoulder_y, bool isRightShoulder)
{
    if (result.empty()) {
        result = cv::Mat(black_white_image.rows, black_white_image.cols, CV_8UC1, cv::Scalar(0));
    }
    shoulder_y = -1;
    std::vector<cv::Point> nzPoints;
    
    int prev_c = -1;
    int prev_r = -1;
    bool found_arm = false;
    int remaining_points = shoulder_window_size;
    for(int r = black_white_image.rows - 1; r > 0; r--) {
        if (found_arm && remaining_points == 0) { break; }
        if (found_arm && remaining_points > 0) { remaining_points--; }
        int nearest_c = black_white_image.rows;
        int c_diff = shoulder_window_size + 1;
        int lower_bound = std::max(0, prev_c - shoulder_window_size);
        int upper_bound = std::min(prev_c + shoulder_window_size, black_white_image.cols - 1);
        if (prev_c == -1) {
            upper_bound = black_white_image.cols - 1;
        }
        for(int col = lower_bound; col < upper_bound; col++) {
            int c = col;
            if (isRightShoulder) {
                c = std::min(lower_bound + (upper_bound - col), black_white_image.cols - 1);
            }
            uchar black_white = black_white_image.ptr<uchar>(r)[c];
            if (black_white > 0) 
            {
                if (prev_c != -1) {
                    if (std::abs(c - prev_c) < c_diff) {
                        c_diff = std::abs(c - prev_c);
                        nearest_c = c;
                    }
                } else {
                    nearest_c = c;
                    break;
                }
            }
        }
        if (nearest_c != black_white_image.rows) {
            nzPoints.push_back(cv::Point(nearest_c,r));
            result.ptr<uchar>(r)[nearest_c] = white;
            prev_c = nearest_c;
            if (prev_r != -1 && std::abs(prev_r - r) > 18 && !found_arm) {
                found_arm = true;
            }
            prev_r = r;
        }
    }
    
    shoulder_x = prev_c;
    shoulder_y = prev_r;
    //     updateKalman(prev_c, prev_r, shoulder_x, shoulder_y);
    drawFitLine(nzPoints, black_white_image);
}


void Biotracking::updateKalman(float x, float y, int& shoulder_x, int& shoulder_y)
{
    if (x == -1 && y == -1) { return; }
    
    std::vector<double> in, out;
    in.push_back(x); in.push_back(y);
    
    if (kalman_left_shoulder->isInitializated()) {
        kalman_left_shoulder->update(in, out);
        shoulder_x = out[0];
        shoulder_y = out[1];
    } else {
        for (int i = 0; i < 4; i++) { in.push_back(0.0); }
        kalman_left_shoulder->configure(in);
    }
}

// void Biotracking::drawNeckLine(cv::Mat& black_white_image)
// {
//     for (int r = 0; r < black_white_image.rows; r++)
//         for(int c = 0; c < black_white_image.cols; c++) {
//             uchar black_white = black_white_image.ptr<uchar>(r)[c];
//             if (black_white > 0) 
//             {
//                 result.ptr<uchar>(r)[c] = black_white;
//                 break;
//             }
//         }
//     }
// }

void Biotracking::drawFitLine(std::vector<cv::Point>& nzPoints, cv::Mat& black_white_image)
{
    if (nzPoints.size() < 2) { return; }
    
    cv::Vec4f line;
    cv::fitLine(nzPoints, line, cv::DIST_L1, 1, 0.001, 0.001);
    float d = std::sqrt((double) line[0] * line[0] + (double) line[1] * line[1]);
    line[0] /= d;
    line[1] /= d;
    float t = (float) (black_white_image.cols + black_white_image.rows);
    cv::Point pt1, pt2;
    pt1.x = cvRound(line[2] - line[0] * t);
    pt1.y = cvRound(line[3] - line[1] * t);
    pt2.x = cvRound(line[2] + line[0] * t);
    pt2.y = cvRound(line[3] + line[1] * t);
    cv::line(black_white_image, pt1, pt2, cv::Scalar(255, 255, 255), 3, CV_AA, 0);
    
    cv::Point p(hips_left_x, hips_height);
    double dist = point_to_line_distance(p, line);
    cv::putText(black_white_image, "(" + std::to_string(dist) + ")", cv::Point(300,20), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(255,255,255), 1, CV_AA);
}


cv::Mat Biotracking::calculateTopPoints(cv::Mat& black_white_image)
{
    cv::Mat result = cv::Mat(black_white_image.rows, black_white_image.cols, CV_8UC1, cv::Scalar(0));
    if (hips_height < 0 || hips_height >= black_white_image.rows || hips_left_x == -1 || hips_right_x == -1)
    {
        return result;
    }
    for(int c = 0; c < black_white_image.cols; c++) {
        for(int r = hips_left_x; r < hips_right_x; r++) {
            uchar black_white = black_white_image.ptr<uchar>(r)[c];
            if (black_white > 0) 
            {
                result.ptr<uchar>(r)[c] = black_white;
                break;
            }
        }
    }
    return result;
}

cv::Mat Biotracking::calculateHorizontalLines(cv::Mat& black_white_image)
{
    cv::Mat horizontal = black_white_image.clone();
    int horizontalsize = horizontal.cols / 30;
    cv::Mat horizontalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horizontalsize,1));
    erode(horizontal, horizontal, horizontalStructure, cv::Point(-1, -1));
    dilate(horizontal, horizontal, horizontalStructure, cv::Point(-1, -1));
    return horizontal;
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


void Biotracking::setUpVariables()
{
    hips_height = hips_left_x = hips_right_x = -1;
    left_line_px = left_line_py = left_line_qx = left_line_qy = -1;
}

void Biotracking::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
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
    
    if (!has_avg_image) {
        cv::Mat read_image;
        cv::FileStorage fs(avg_image_path, cv::FileStorage::READ);
        fs[cv_fs_image_id] >> read_image;
        
        if (!read_image.empty()) {
            for(int r = 0; r < read_image.rows; r++) {
                float* avg_image_raw_ptr = avg_image.ptr<float>(r);
                float* read_image_ptr = read_image.ptr<float>(r);
                
                for(int c = 0; c < read_image.cols; c++) {
                    avg_image_raw_ptr[c] = read_image_ptr[c];
                }
            }
            
            has_avg_image = true;
            ROS_INFO("Average image is read from '%s'!", avg_image_path.c_str());
            
        } else {
            
            if (remainedImagesToCalcAvg == num_images_for_background) {
                ROS_INFO("Call service 'calculateAvg' to calculate average image.");
            }
            
            if (!isCalculateAvgSrvCalled) { return; }
            
            if (remainedImagesToCalcAvg == num_images_for_background) {
                ROS_INFO("Average image was not found in '%s', calculating with %d images!", avg_image_path.c_str(), num_images_for_background);
            }
            
            if (remainedImagesToCalcAvg > 0) 
            { 
                for(int r = 0; r < avg_image.rows; r++) {
                    float* avg_image_raw_ptr = avg_image.ptr<float>(r);
                    float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);
                    
                    for(int c = 0; c < avg_image.cols; c++) {
                      
                        if (cv_ptr_image_ptr[c] == cv_ptr_image_ptr[c]) { // if not NaN
                            if (avg_image_raw_ptr[c] > 0. || remainedImagesToCalcAvg == num_images_for_background) {
                                avg_image_raw_ptr[c] += (1. / (float) num_images_for_background) * cv_ptr_image_ptr[c];
                            } else {
                                avg_image_raw_ptr[c] = ((num_images_for_background - remainedImagesToCalcAvg + 1) / num_images_for_background) * cv_ptr_image_ptr[c];
                            }
                        } else if (avg_image_raw_ptr[c] > 0.) {
                            avg_image_raw_ptr[c] += (1 / (num_images_for_background - remainedImagesToCalcAvg)) * avg_image_raw_ptr[c];
                        }
                    }
                }
                remainedImagesToCalcAvg--;
                return;
            }
            
            if (remainedImagesToCalcAvg == 0)
            { 
                remainedImagesToCalcAvg--;
                
                for(int r = 0; r < avg_image.rows; r++) {
                    float* avg_image_raw_ptr = avg_image.ptr<float>(r);
                    float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);
                    
                    for(int c = 0; c < avg_image.cols; c++) {
                        // if NaN
                        if (avg_image_raw_ptr[c] != avg_image_raw_ptr[c] || avg_image_raw_ptr[c] > bad_point) { 
                            avg_image_raw_ptr[c] = bad_point;
                        }
                        if (cv_ptr_image_ptr[c] != cv_ptr_image_ptr[c] || cv_ptr_image_ptr[c] > bad_point) {
                            cv_ptr_image_ptr[c] = bad_point;
                        }
                    }
                }
                cv::FileStorage fs(avg_image_path, cv::FileStorage::WRITE);
                fs << cv_fs_image_id << avg_image;


                has_avg_image = true;
                ROS_INFO("Average image is calculated!");
            }
        }
    }
    
    if (!has_avg_image) { return; }
    
    setUpVariables();
    
    cv::Mat avg_image_8u;
    avg_image.convertTo(avg_image_8u, CV_8UC1);
    
    cv::Mat raw_image_8u;
    cv_ptr->image.convertTo(raw_image_8u, CV_8UC1, 255, 0);
    
    cv::Mat diffMat;
    diffMat.create(R,C,CV_32FC1);
    for(int r = 0; r < avg_image.rows; r++) {
        float* avg_image_raw_ptr = avg_image.ptr<float>(r);
        float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);
        float* diffMat_ptr = diffMat.ptr<float>(r);
        
        for(int c = 0; c < avg_image.cols; c++) {
            // if NaN
            if (avg_image_raw_ptr[c] == bad_point || cv_ptr_image_ptr[c] == bad_point) {
                diffMat_ptr[c] = 0.;
            } else {
                diffMat_ptr[c] = std::abs(avg_image_raw_ptr[c] - cv_ptr_image_ptr[c]);
            }
        }
    }
    
    cv::Mat workingImg;
    workingImg = cv::Mat::zeros(R,C,CV_8UC1);
    std::string s = "", s2 = "";
    for(int r = 0; r < workingImg.rows - rows_rollator_offset; r++) {
        uchar* workingImg_ptr = workingImg.ptr<uchar>(r);
        uchar* avg_image_ptr = avg_image_8u.ptr<uchar>(r);
        float* cv_ptr_image_ptr = cv_ptr->image.ptr<float>(r);
        float* avg_image_raw_ptr = avg_image.ptr<float>(r);
        float* diffMat_ptr = diffMat.ptr<float>(r);
        
        for(int c = 0; c < workingImg.cols; c++) {
            
            if (diffMat_ptr[c] > 0.1 && cv_ptr_image_ptr[c] < person_distance && cv_ptr_image_ptr[c] > min_distance_near_camera 
                && avg_image_raw_ptr[c] > min_distance_near_camera && avg_image_raw_ptr[c] > cv_ptr_image_ptr[c])
            {
                workingImg_ptr[c] = 255;
            }
        }
    }
    
    
    cv::Mat erosion_dst;
    int erosion_size = 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size) );
    
    erode(workingImg, erosion_dst, element);
    cv::Mat subtract_dst = workingImg - erosion_dst;
    
    for (int c = 0; c < subtract_dst.cols; c++) {
        subtract_dst.ptr<uchar>(workingImg.rows - rows_rollator_offset - 1)[c] = 0;
    }
    
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
    
    //     int horizontal_plane_y_int = showHorizontalPlane(cv_ptr->image, subtract_dst);
    
    //     if (horizontal_plane_y_int != -1) 
    //     {
    hips_height = calculateHipsHeight(cv_ptr->image, subtract_dst);
    calculateHipsLeftRightX(cv_ptr->image, subtract_dst);
    //     }
    
    
    //     calculateSlopeLines(subtract_dst, cv_ptr->image);
    
    
    //     cv::Mat horizontal = calculateHorizontalLines(subtract_dst);
    
    //     calculateTopPoints(subtract_dst);
    cv::Mat shoulder_points;
    calculateShoulderPoints(subtract_dst, shoulder_points, shoulder_left_c, shoulder_left_r, false);
    calculateShoulderPoints(subtract_dst, shoulder_points, shoulder_right_c, shoulder_right_r, true);
    
    //     drawNeckLine(subtract_dst);
    
    
    biotracking::BioFeedbackMsg biofeedback_msg;
    biofeedback_msg.header = msg->header;
    bool should_pub_biofeedback = false;
    
    if (hips_left_x != -1 && hips_right_x != -1) {
        biofeedback_msg.hips_left_pt = hips_left_pt;
        biofeedback_msg.hips_right_pt = hips_right_pt;
        should_pub_biofeedback = true;
    }
    
    if (hips_height != -1 && shoulder_left_c != -1 && shoulder_right_c != -1 && shoulder_left_r != -1 && shoulder_right_r != -1) {
        shoulder_left_pt.z = cv_ptr->image.ptr<float>(shoulder_left_r)[shoulder_left_c];
        shoulder_left_pt.x = (shoulder_left_c - center_x) * shoulder_left_pt.z * constant_x;
        shoulder_left_pt.y = (shoulder_left_r - center_y) * shoulder_left_pt.z * constant_y;
        biofeedback_msg.shoulder_left_pt = shoulder_left_pt;
        
        shoulder_right_pt.z = cv_ptr->image.ptr<float>(shoulder_right_r)[shoulder_right_c];
        shoulder_right_pt.x = (shoulder_right_c - center_x) * shoulder_right_pt.z * constant_x;
        shoulder_right_pt.y = (shoulder_right_r - center_y) * shoulder_right_pt.z * constant_y;
        biofeedback_msg.shoulder_right_pt = shoulder_right_pt;
    }
    
    if (should_pub_biofeedback) {
        biofeedback_pub.publish(biofeedback_msg);
    }
    
    
    //     showFirstFromLeftPoints(cv_ptr->image, subtract_dst, msg->header.frame_id);
    
    
    
    //     if (line_px != -1) {
    //         cv::line(subtract_dst, cv::Point(line_px, line_py), cv::Point(line_qx, line_qy), cv::Scalar(255,255,255));
    //     }
    
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
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", shoulder_points).toImageMsg();
    horizontal_pub_.publish(msg_to_pub);
    // 	msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", fgMaskMOG2).toImageMsg();
    // 	mog2_pub_.publish(msg_to_pub);
    
    
    //     int waitInt;
    //     std::cin >> waitInt;
}

void Biotracking::calculateHipsLeftRightX(cv::Mat& depth_image, cv::Mat& black_white_image)
{   
    bool found_white = false;
    for (int n = 1; n < black_white_image.cols; n++)
    {
        uchar black_white = black_white_image.ptr<uchar>(hips_height)[n];
        if (black_white > 0) {
            hips_left_x = n;
            found_white = true;
        }
    }
    
    //     ROS_INFO("hips_left_x: %d", hips_left_x);
    if (!found_white) {
        hips_left_x = hips_right_x = -1;
        return;
    }
    
    for (int n = black_white_image.cols - 1; n > 0; n--)
    {
        uchar black_white = black_white_image.ptr<uchar>(hips_height)[n];
        if (black_white > 0) {
            hips_right_x = n;
        }
    }
    //     ROS_INFO("hips_right_x: %d", hips_right_x);
    
    if (std::abs(hips_left_x - hips_right_x) < 10)
    {
        hips_left_x = hips_right_x = -1;
    }
    
    if (hips_left_x != -1 && hips_right_x != -1) {
        hips_left_pt.z = depth_image.ptr<float>(hips_height)[hips_left_x];
        hips_left_pt.x = (hips_left_x - center_x) * hips_left_pt.z * constant_x;
        hips_left_pt.y = (hips_height - center_y) * hips_left_pt.z * constant_y;
        
        hips_right_pt.z = depth_image.ptr<float>(hips_height)[hips_right_x];
        hips_right_pt.x = (hips_right_x - center_x) * hips_right_pt.z * constant_x;
        hips_right_pt.y = (hips_height - center_y) * hips_right_pt.z * constant_y;
    }
}

int Biotracking::calculateHipsHeight(cv::Mat& depth_image, cv::Mat& black_white_image)
{
    int center_x_int = static_cast<int>(model_.cx());
    int center_y_int = static_cast<int>(model_.cy());
    float height_diff = camera_height_world - hips_height_world;
    
    float min_diff = bad_point;
    int hips_height_row = -1;
    
    for (int r = depth_image.rows - 1; r > 0; r--)
    {
        for (int c = 0; c < depth_image.cols; c++)
        { 
            uchar black_white = black_white_image.ptr<uchar>(r)[c];
            if (black_white > 0) {
                float d = depth_image.ptr<float>(r)[c];
                float needed_y = d * std::sin(camera_angle_radians) + height_diff;
                float current_y = (r - center_y) * d * constant_y;
                float current_diff = std::abs(current_y - needed_y);
                if (current_diff < min_diff) {
                    hips_height_row  = r;
                    min_diff = current_diff;
                    break;
                }
            }
        }
    }
    
    
    //     int fst_c_center = -1;
    //     for (int c = center_y_int; c < black_white_image.cols; c++)
    //     {
    //         uchar black_white = black_white_image.ptr<uchar>(center_x_int)[c]; 
    //         if (black_white > 0) {
    //             fst_c_center = c;
    //             break;
    //         }
    //     }
    
    
    //     float hips_height_diff = hips_height_world;
    //     
    //     float horizontal_plane_d = depth_image.ptr<float>(horizontal_plane_y_int)[center_x_int];
    //     if (horizontal_plane_d == 0 || horizontal_plane_d != horizontal_plane_d)
    //     {
    //         horizontal_plane_d = 1.;
    //     }
    //     float horizontal_plane_y = (horizontal_plane_y_int - center_y) * horizontal_plane_d * constant_y;
    //     
    //     float height_diff = camera_height_world - hips_height_world;
    //     
    //     float hips_height_in_camera = horizontal_plane_y + height_diff * horizontal_plane_d;
    //     
    //     float hips_height_row_float = hips_height_in_camera * (1. / constant_y) * (1. / horizontal_plane_d) + center_y;
    //     hips_height_row = static_cast<int>(hips_height_row_float);
    //     ROS_INFO("horizontal_plane_d: %f, horizontal_plane_y: %f, height_diff: %f, hips_height_in_camera: %f, hips_height_row_float: %f, hips_height_row: %d",
    //              horizontal_plane_d, horizontal_plane_y, height_diff, hips_height_in_camera, hips_height_row_float, hips_height_row);
    
    /*
     *    int for_begin = horizontal_plane_y_int;
     *    int for_boundary = depth_image.rows;
     *    
     *    if (height_diff < 0) // hips are higher than camera 
     *    {
     *        for_begin = 0;
     *        for_boundary = horizontal_plane_y_int;
}

for (int m = for_begin; m < for_boundary; m++)
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

if (hips_height_row != -1) 
{
cv::putText(black_white_image, "hips: (" + std::to_string(center_x_int) + ", " + 
std::to_string(hips_height_row) + ")", cv::Point(center_x_int,hips_height_row), 
cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);
}*/
    
    return hips_height_row;
}

void Biotracking::calculateSlopeLines(cv::Mat& black_white_image, cv::Mat& depth_image)
{
    clearVectors();
    for (int m = 0; m < black_white_image.rows * 0.7; m++)
    {
        for (int n = 0; n < black_white_image.cols; n++)
        {
            uchar black_white = black_white_image.ptr<uchar>(m)[n];
            if (black_white > 0)
            {
                Point p;
                p.z = depth_image.ptr<float>(m)[n]; 
                if (p.z == 0 || p.z != p.z) { p.z = 1.; }
                p.x = (n - center_x) * p.z * constant_x;
                p.y = (m - center_y) * p.z * constant_y;
                left_points.push_back(p);
                Point r;
                r.x = n;
                r.y = m;
                left_points_positions.push_back(r);
            }
        }
    }
    
    for (int m = 0; m < black_white_image.rows * 0.7; m++)
    {
        for (int n = black_white_image.cols - 1; n > 0; n--)
        {
            uchar black_white = black_white_image.ptr<uchar>(m)[n];
            if (black_white > 0)
            {
                Point p;
                p.z = depth_image.ptr<float>(m)[n]; 
                if (p.z == 0 || p.z != p.z) { p.z = 1.; }
                p.x = (n - center_x) * p.z * constant_x;
                p.y = (m - center_y) * p.z * constant_y;
                right_points.push_back(p);
                Point r;
                r.x = n;
                r.y = m;
                right_points_positions.push_back(r);
            }
        }
    }
    
    // int min_points_size = std::min(left_points.size(), right_points.size());
    for (int i = 0; i < left_points.size() - 10; i++)
    {
        Point& fst_pt = left_points[i];
        Point& snd_pt = left_points[i + 10];
        if (snd_pt.x - fst_pt.x == 0) { continue; }
        float slope = (snd_pt.y - fst_pt.y) / (snd_pt.x - fst_pt.x);
        if (slope == 1)
        {
            left_slopes_indices.push_back(i);
        }
    }
    
    for (int i = 0; i < right_points.size() - 10; i++)
    {
        Point& fst_pt = right_points[i];
        Point& snd_pt = right_points[i + 10];
        if (snd_pt.x - fst_pt.x == 0) { continue; }
        float slope = (snd_pt.y - fst_pt.y) / (snd_pt.x - fst_pt.x);
        if (slope == -1)
        {
            right_slopes_indices.push_back(i);
        }
    }
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
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255,255,255), 1, CV_AA);
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
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255,255,255), 1, CV_AA);
    
    cv::putText(black_white_image, "horizontal plane: (" + std::to_string(center_x_int) + ", " + 
    std::to_string(horizontal_plane_y) + ", " + std::to_string(horizontal_plane_z) + ")", cv::Point(center_x_int,horizontal_plane_y), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255,255,255), 1, CV_AA);
    
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
    
    
    cv::putText(image, "hips_y: (" + std::to_string(hips_height) + ")", cv::Point(hips_left_x, hips_height - 10), 
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,0), 1, CV_AA);
}

void Biotracking::drawShoulderLine(cv::Mat& image)
{
    if (hips_height < 0 || hips_height >= image.rows || hips_left_x == -1 || hips_right_x == -1)
    {
        return;
    }
    
    // 	for (int i = 0; i < left_points_positions.size(); i++)
    // 	{
    //         for (int j = 0; j < right_points_positions.size(); j++)
    //             if (std::abs(right_points_positions[i].y - left_points_positions[i].y) > 3
    // 		Point& left_pt = left_points_positions[i];
    // 		Point& right_pt = right_points_positions[i];
    //         ROS_INFO("line %d from (%f, %f) to (%f, %f)", i, left_pt.x, left_pt.y, right_pt.x, right_pt.y);
    // 			cv::line(image, cv::Point(left_pt.x, left_pt.y), 
    // 				 cv::Point(right_pt.x, right_pt.y), cv::Scalar(0,0,0));
    // 		if (left_pt.x < hips_left_x && right_pt.x > hips_right_x)
    // 		{
    // 			break;
    // 		}
    // 		
    // 	}
    
    for (int i = 0; i < std::min(left_points_positions.size(), right_points_positions.size()); i++)
    {
        Point& left_pt = left_points_positions[i];
        Point& right_pt = right_points_positions[i];
        ROS_INFO("line %d from (%f, %f) to (%f, %f)", i, left_pt.x, left_pt.y, right_pt.x, right_pt.y);
        cv::line(image, cv::Point(left_pt.x, left_pt.y), 
                 cv::Point(right_pt.x, right_pt.y), cv::Scalar(0,0,0));
        if (left_pt.x < hips_left_x && right_pt.x > hips_right_x)
        {
            break;
        }
        
    }
}

void Biotracking::drawShoulderCircles(cv::Mat& image)
{
    if (shoulder_left_c != -1 && shoulder_left_r != -1) {
        cv::circle(image, cv::Point(shoulder_left_c, shoulder_left_r), 10, CV_RGB(0,255,0), CV_FILLED, 10, 0);
    }
    
    if (shoulder_right_c != -1 && shoulder_right_r != -1) {
        cv::circle(image, cv::Point(shoulder_right_c, shoulder_right_r), 10, CV_RGB(0,255,0), CV_FILLED, 10, 0);
    }
}

void Biotracking::drawSlopeCircles(cv::Mat& image)
{
    for (int i = 0; i < left_slopes_indices.size(); i++)
    {
        Point& p = left_points_positions[left_slopes_indices[i]];
        cv::circle(image, cv::Point(p.x, p.y), 5, CV_RGB(0,255,0), CV_FILLED, 10, 0);
    }
    
    for (int i = 0; i < right_slopes_indices.size(); i++)
    {
        Point& p = right_points_positions[right_slopes_indices[i]];
        cv::circle(image, cv::Point(p.x, p.y), 5, CV_RGB(255,255,0), CV_FILLED, 10, 0);
    }
}

void Biotracking::drawLeftLine(cv::Mat& image)
{
    if (left_line_px != -1)
    {
        cv::line(image, cv::Point(left_line_px, left_line_py), cv::Point(left_line_qx, left_line_qy), cv::Scalar(0,0,255));
    }
}

void Biotracking::clearVectors()
{
    left_points.clear();
    left_points_positions.clear();
    right_points.clear();
    right_points_positions.clear();
    left_slopes_indices.clear();
    right_slopes_indices.clear();
}

void Biotracking::rgbImageCb(const sensor_msgs::ImageConstPtr& msg)
{
    if (!has_avg_image) { return; }
    
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
    
    //     drawSlopeCircles(cv_ptr->image);
    
    //     drawShoulderLine(cv_ptr->image);
    
    drawShoulderCircles(cv_ptr->image);
    
    //     drawLeftLine(cv_ptr->image);
    
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

