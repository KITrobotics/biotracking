#include <biotracking/biotracking.h>
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>

Biotracking::Biotracking(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer), it_(nh)
{
    nh_.param("topic_point_cloud", topic_point_cloud, std::string("/camera/depth_registered/points"));
    nh_.param("camera_info_topic", camera_info_topic, std::string("camera_info"));
    nh_.param("from_pc2_depth_image_topic", from_pc2_depth_image_topic, std::string("/biotracking/from_pc2_depth_image"));
    nh_.param("cv_fs_image_id", cv_fs_image_id, std::string("avg_image"));
    nh_.param("depth_image_sub_topic", depth_image_sub_topic, std::string("/biotracking/from_pc2_depth_image"));
    nh_.param("image_cols", C, 640);
    nh_.param("image_rows", R, 480);
    nh_.param("num_images_for_background", num_images_for_background, 3);
    nh_.param("person_distance", person_distance, 1.);
    nh_.param("min_distance_near_camera", min_distance_near_camera, 0.4);
    nh_.param("bad_point", bad_point, 9.9);
    nh_.param("rows_rollator_offset", rows_rollator_offset, 140);
    
    std::string avg_image_path_param;
    nh_.param("avg_image_path", avg_image_path_param, std::string("background_image/avg_image.yml"));
    biotracking_path = ros::package::getPath("biotracking");
    avg_image_path = biotracking_path + "/" + avg_image_path_param;
    
    depth_image_sub_ = it_.subscribe(depth_image_sub_topic, 1, &Biotracking::imageCb, this);
    sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, &Biotracking::cameraInfoCb, this);
    
    calculateAvgService_ = nh_.advertiseService("calculateAvg", &Biotracking::calculateAvgImageCb, this);
    subtract_image_pub_ = it_.advertise("/biotracking/subtract_image", 1);
    working_image_pub_ = it_.advertise("/biotracking/working_image", 1);
    avg_image_pub_ = it_.advertise("/biotracking/avg_image", 1);
    erosion_image_pub_ = it_.advertise("/biotracking/erosion_image", 1);
    from_pc2_depth_image_pub_ = it_.advertise(from_pc2_depth_image_topic, 1);
    biofeedback_pub_ = nh_.advertise<biotracking::BioFeedbackMsg>("/biotracking/biofeedback", 10);
    
    remainedImagesToCalcAvg = num_images_for_background;
    avg_image.create(R,C,CV_32FC1);
    isCalculateAvgSrvCalled = isAvgCalculated = hasCameraInfo = has_avg_image = false;
    
    kalman_left_shoulder = new KalmanFilter();
    kalman_right_shoulder = new KalmanFilter();
}

// void Biotracking::publishBiofeedbackMsg()
// {
//   biotracking::BioFeedbackMsg biofeedback_msg;
//   biofeedback_msg.header = 
//   biofeedback_msg.hips_left_pt = 
//   biofeedback_msg.hips_right_pt =
//   biofeedback_msg.shoulder_left_pt = 
//   biofeedback_msg.shoulder_right_pt = 
//   biofeedback_pub.publish(biofeedback_msg);
// }
// void Biotracking::updateKalman(float x, float y, int& shoulder_x, int& shoulder_y)
// {
//     if (x == -1 && y == -1) { return; }
//     
//     std::vector<double> in, out;
//     in.push_back(x); in.push_back(y);
//     
//     if (kalman_left_shoulder->isInitializated()) {
//         kalman_left_shoulder->update(in, out);
//         shoulder_x = out[0];
//         shoulder_y = out[1];
//     } else {
//         for (int i = 0; i < 4; i++) { in.push_back(0.0); }
//         kalman_left_shoulder->configure(in);
//     }
// }

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
    int erosion_size = 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size) );
    
    erode(workingImg, erosion_dst, element);
    cv::Mat subtract_dst = workingImg - erosion_dst;
    
    sensor_msgs::ImagePtr msg_to_pub;
    
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "32FC1", avg_image).toImageMsg();
    avg_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", workingImg).toImageMsg();
    working_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", subtract_dst).toImageMsg();
    subtract_image_pub_.publish(msg_to_pub);
    msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "8UC1", erosion_dst).toImageMsg();
    erosion_image_pub_.publish(msg_to_pub);
}

bool Biotracking::calculateAvgImageCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    isCalculateAvgSrvCalled = true;
    return true;
}