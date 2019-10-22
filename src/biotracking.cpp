#include <biotracking/biotracking.h>
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>

Biotracking::Biotracking(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer), it_(nh)
{
    nh_.param("topic_point_cloud", topic_point_cloud, std::string("/camera/depth_registered/points"));
    nh_.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
    nh_.param("cv_fs_image_id", cv_fs_image_id, std::string("avg_image"));
    nh_.param("depth_image_sub_topic", depth_image_sub_topic, std::string("/biotracking/from_pc2_depth_image"));
    nh_.param("rgb_image_sub_topic", rgb_image_sub_topic, std::string("/camera_body/rgb/image_raw"));
    nh_.param("image_cols", C, 640);
    nh_.param("image_rows", R, 480);
    nh_.param("num_images_for_background", num_images_for_background, 3);
    nh_.param("person_distance", person_distance, 1.);
    nh_.param("min_distance_near_camera", min_distance_near_camera, 0.4);
    nh_.param("bad_point", bad_point, 9.9);
    nh_.param("rows_rollator_offset", rows_rollator_offset, 140);
    nh_.param("erosion_size", erosion_size, 2);
    nh_.param("dilation_size", dilation_size, 2);

    std::string avg_image_path_param;
    nh_.param("avg_image_path", avg_image_path_param, std::string("background_image/avg_image.yml"));
    biotracking_path = ros::package::getPath("biotracking");
    avg_image_path = biotracking_path + "/" + avg_image_path_param;

    depth_image_sub_ = it_.subscribe(depth_image_sub_topic, 1, &Biotracking::depthImageCb, this);
    sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, &Biotracking::cameraInfoCb, this);
    rgb_image_sub_ = it_.subscribe(rgb_image_sub_topic, 1, &Biotracking::rgbImageCb, this);

    calculateAvgService_ = nh_.advertiseService("calculateAvg", &Biotracking::calculateAvgImageCb, this);
    avg_image_pub_ = it_.advertise("/biotracking/avg_image", 1);
    biofeedback_pub_ = nh_.advertise<biotracking::BioFeedbackMsg>("/biotracking/biofeedback", 10);
    rgb_image_pub_ = it_.advertise("/biotracking/rgb_image", 1);
    diffMat_image_pub_ = it_.advertise("/biotracking/diffMat", 1);

    remainedImagesToCalcAvg = num_images_for_background;
    avg_image.create(R,C,CV_32FC1);
    isCalculateAvgSrvCalled = isAvgCalculated = hasCameraInfo = has_avg_image = false;

    kalman_left_shoulder = new KalmanFilter();
    kalman_right_shoulder = new KalmanFilter();
}

void Biotracking::publishBiofeedbackMsg(const sensor_msgs::ImageConstPtr& msg)
{
    biotracking::BioFeedbackMsg biofeedback_msg;
    biofeedback_msg.header = msg->header;
    biofeedback_msg.sinister_schoulder_pt = sinister_schoulder_3d_pt;
    biofeedback_msg.dexter_schoulder_pt = dexter_schoulder_3d_pt;
    biofeedback_pub_.publish(biofeedback_msg);
}

void Biotracking::rgbImageCb(const sensor_msgs::ImageConstPtr& msg)
{
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
    cv::circle(cv_ptr->image, moments_center, 15, CV_RGB(255,0,0), -1);
    cv::circle(cv_ptr->image, sinister_schoulder_cv_pt, 15, CV_RGB(255,0,0), -1);
    cv::circle(cv_ptr->image, dexter_schoulder_cv_pt, 15, CV_RGB(255,0,0), -1);
    rgb_image_pub_.publish(cv_ptr->toImageMsg());
    publishBiofeedbackMsg(msg);
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
        ROS_INFO("Get camera info.");
    }
}

void Biotracking::depthImageCb(const sensor_msgs::ImageConstPtr& msg)
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
    for(int r = 0; r < workingImg.rows - rows_rollator_offset; r++) {
        uchar* workingImg_ptr = workingImg.ptr<uchar>(r);
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
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));
    erode(workingImg, erosion_dst, element);
    cv::Moments moments = cv::moments(erosion_dst);
    moments_center = cv::Point2f(static_cast<float>(moments.m10 / (moments.m00 + 1e-5)),
                                 static_cast<float>(moments.m01 / (moments.m00 + 1e-5)));
    cv::Mat dilation_dst;
    element = cv::getStructuringElement(cv::MORPH_RECT,
                                        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                        cv::Point(dilation_size, dilation_size));
    dilate(erosion_dst, dilation_dst, element);
    calculateSinisterLine(dilation_dst);
    calculateDexterLine(dilation_dst);
    findCorrespondingSinisterDepthPoint(cv_ptr->image, workingImg);
    findCorrespondingDexterDepthPoint(cv_ptr->image, workingImg);

    cv::Mat subtract_dst = workingImg - erosion_dst;

    sensor_msgs::ImagePtr msg_to_pub;

    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", avg_image).toImageMsg();
    avg_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", diffMat).toImageMsg();
    diffMat_image_pub_.publish(msg_to_pub);
}

void Biotracking::findCorrespondingDexterDepthPoint(cv::Mat& depth_image, cv::Mat& black_white_image)
{
    float min_distance = bad_point;
    cv::Point min_distance_pt = cv::Point(0, 0);
    for(int r = dexter_schoulder_cv_pt.y; r < dexter_schoulder_cv_pt.y + 2 * dilation_size + 1; r++) {
        for(int c = dexter_schoulder_cv_pt.x; c < dexter_schoulder_cv_pt.x + 2 * dilation_size + 1; c++) {
            uchar black_white = black_white_image.ptr<uchar>(r)[c];
            if (black_white > 0) {
                float current_distance = distance(dexter_schoulder_cv_pt.x, dexter_schoulder_cv_pt.y, c, r);
                if (current_distance < min_distance) {
                    min_distance = current_distance;
                    min_distance_pt.x = c;
                    min_distance_pt.y = r;
                }
            }
        }
    }
    if (min_distance != bad_point) {
        dexter_schoulder_cv_pt.x = min_distance_pt.x;
        dexter_schoulder_cv_pt.y = min_distance_pt.y;
        dexter_schoulder_3d_pt.z = depth_image.ptr<float>(min_distance_pt.y)[min_distance_pt.x];
        dexter_schoulder_3d_pt.y = (min_distance_pt.y - center_y) * dexter_schoulder_3d_pt.z * constant_y;
        dexter_schoulder_3d_pt.x = (min_distance_pt.x - center_x) * dexter_schoulder_3d_pt.z * constant_x;
    }
}

void Biotracking::findCorrespondingSinisterDepthPoint(cv::Mat& depth_image, cv::Mat& black_white_image)
{
    float min_distance = bad_point;
    cv::Point min_distance_pt = cv::Point(0, 0);
    for(int r = sinister_schoulder_cv_pt.y; r < sinister_schoulder_cv_pt.y + 2 * dilation_size + 1; r++) {
        for(int c = sinister_schoulder_cv_pt.x; c > sinister_schoulder_cv_pt.x - 2 * dilation_size - 1; c--) {
            uchar black_white = black_white_image.ptr<uchar>(r)[c];
            if (black_white > 0) {
                float current_distance = distance(sinister_schoulder_cv_pt.x, sinister_schoulder_cv_pt.y, c, r);
                if (current_distance < min_distance) {
                    min_distance = current_distance;
                    min_distance_pt.x = c;
                    min_distance_pt.y = r;
                }
            }
        }
    }
    if (min_distance != bad_point) {
        sinister_schoulder_cv_pt.x = min_distance_pt.x;
        sinister_schoulder_cv_pt.y = min_distance_pt.y;
        sinister_schoulder_3d_pt.z = depth_image.ptr<float>(min_distance_pt.y)[min_distance_pt.x];
        sinister_schoulder_3d_pt.y = (min_distance_pt.y - center_y) * sinister_schoulder_3d_pt.z * constant_y;
        sinister_schoulder_3d_pt.x = (min_distance_pt.x - center_x) * sinister_schoulder_3d_pt.z * constant_x;
    }
}

void Biotracking::fitAndDrawLine(cv::Mat& black_white_image, std::vector<cv::Point>& points)
{
    if (points.size() < 2) { return; }
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L1, 1, 0.001, 0.001);
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
}

void Biotracking::calculateDexterLine(cv::Mat& black_white_image)
{
    std::vector<cv::Point> points;
    int prev_col = -1;
    int printed = false;
    int moments_center_x = static_cast<int>(moments_center.x);
    int max_allowed_distance = 10;
    for(int r = black_white_image.rows - rows_rollator_offset; r > 0; r--) {
        uchar black_white = black_white_image.ptr<uchar>(r)[moments_center_x];
        if (black_white == 0) { continue; }

        bool found_something = false;

        int begin_col, end_col;
        if (prev_col == -1) {
            begin_col = moments_center_x;
            end_col = 0;
        } else {
            begin_col = std::min(prev_col + max_allowed_distance, black_white_image.cols - 1);
            end_col = std::max(0, prev_col - max_allowed_distance);
        }
        for(int c = begin_col; c > end_col; c--) {
            uchar black_white_right = black_white_image.ptr<uchar>(r)[c];
            if (black_white_right == 0) {
                found_something = true;
                points.push_back(cv::Point(c + 1, r));
                prev_col = c + 1;
                break;
            }
        }

        if (!found_something) {
            break;
        }
    }
    cv::Mat dexter_line = cv::Mat(black_white_image.rows, black_white_image.cols, CV_8UC1, cv::Scalar(0));
    fitAndDrawLine(dexter_line, points);
    dexter_line = dexter_line & black_white_image;
    findShoulderPoint(dexter_line, dexter_schoulder_cv_pt);
}

void Biotracking::calculateSinisterLine(cv::Mat& black_white_image)
{
    std::vector<cv::Point> points;
    int prev_col = -1;
    int printed = false;
    int moments_center_x = static_cast<int>(moments_center.x);
    int max_allowed_distance = 10;
    for(int r = black_white_image.rows - rows_rollator_offset; r > 0; r--) {
        uchar black_white = black_white_image.ptr<uchar>(r)[moments_center_x];
        if (black_white == 0) { continue; }

        bool found_something = false;

        int begin_col, end_col;
        if (prev_col == -1) {
            begin_col = moments_center_x;
            end_col = black_white_image.cols;
        } else {
            begin_col = std::max(0, prev_col - max_allowed_distance);
            end_col = std::min(prev_col + max_allowed_distance, black_white_image.cols);
        }
        for(int c = begin_col; c < end_col; c++) {
            uchar black_white_right = black_white_image.ptr<uchar>(r)[c];
            if (black_white_right == 0) {
                found_something = true;
                points.push_back(cv::Point(c - 1, r));
                prev_col = c - 1;
                break;
            }
        }

        if (!found_something) {
            break;
        }
    }
    cv::Mat sinister_line = cv::Mat(black_white_image.rows, black_white_image.cols, CV_8UC1, cv::Scalar(0));
    fitAndDrawLine(sinister_line, points);
    sinister_line = sinister_line & black_white_image;
    findShoulderPoint(sinister_line, sinister_schoulder_cv_pt);
}

void Biotracking::findShoulderPoint(cv::Mat& black_white_image, cv::Point& shoulder_pt)
{
    int prev_r = -1;
    int prev_col = -1;
    for(int r = black_white_image.rows - rows_rollator_offset; r > 0; r--) {
        for(int c = 0; c < black_white_image.cols; c++) {
            uchar black_white = black_white_image.ptr<uchar>(r)[c];
            if (black_white > 0) {
                if (prev_r != -1 && std::abs(prev_r - r) > 10 && prev_r < moments_center.y) {
                    shoulder_pt = cv::Point(prev_col, prev_r);
                    return;
                }
                prev_r = r;
                prev_col = c;
                break;
            }
        }
    }
    if (prev_col != -1 && prev_r != -1) {
        shoulder_pt = cv::Point(prev_col, prev_r);
    }
}

bool Biotracking::calculateAvgImageCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    isCalculateAvgSrvCalled = true;
    return true;
}

float Biotracking::distance(int x1, int y1, int x2, int y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) * 1.0);
}