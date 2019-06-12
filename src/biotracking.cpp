
#include <biotracking/biotracking.h>


Biotracking::Biotracking(ros::NodeHandle nh) : nh_(nh), tfListener(tfBuffer)
{
    nh_.param("topic_point_cloud", topic_point_cloud, std::string("/camera/depth_registered/points"));
    nh_.param("topic_image_rgb", topic_image_rgb, std::string("/camera/rgb/image_rect_color"));
    
    nh_.param("lower_limit", lower_limit, 0.0);
    nh_.param("upper_limit", upper_limit, 0.5);
    
    pcl_cloud_publisher = nh_.advertise<PointCloud>("pcl_point_cloud", 10);
    pc2_subscriber = nh_.subscribe<sensor_msgs::PointCloud2>(topic_point_cloud, 1, &Biotracking::processPointCloud2, this);
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
