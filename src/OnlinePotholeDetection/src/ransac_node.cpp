#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <dynamic_reconfigure/server.h>
#include </home/larde/Documents/GitHub/Rebar-segmentation-Ransac/devel/include/OnlinePotholeDetection/MyParamsConfig.h>
// #include <MyParamsConfig.h>

class RansacNode
{
public:
    RansacNode() : nh()
    {
        inlier_pub = nh.advertise<sensor_msgs::PointCloud2>("/inlier_points", 1);
        outlier_pub = nh.advertise<sensor_msgs::PointCloud2>("/outlier_points", 1);
        label_pub = nh.advertise<sensor_msgs::Image>("/label", 1);

        pointcloud_sub = nh.subscribe("/camera/depth/color/points", 1, &RansacNode::pointcloud_callback, this);
        camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1, &RansacNode::camera_info_callback, this);
    }

    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        ROS_INFO("ransac_threshold: %lf", ransac_threshold);
        seg.setDistanceThreshold(ransac_threshold);
        seg.setMaxIterations(500);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Extract inliers and outliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*inlier_cloud);
        extract.setNegative(true);
        extract.filter(*outlier_cloud);

        // Convert point clouds back to sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 inlier_msg;
        pcl::toROSMsg(*inlier_cloud, inlier_msg);
        inlier_msg.header.frame_id =  (*input).header.frame_id;
        inlier_msg.header.stamp = ros::Time::now();
        inlier_pub.publish(inlier_msg);

        sensor_msgs::PointCloud2 outlier_msg;
        pcl::toROSMsg(*outlier_cloud, outlier_msg);
        outlier_msg.header.frame_id = (*input).header.frame_id;  
        // input->header.frame_id;
        outlier_msg.header.stamp = ros::Time::now();
        outlier_pub.publish(outlier_msg);

        project_2D(outlier_cloud);
    }

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        // Extract intrinsic camera parameters
        fx = msg->K[0];  // Focal length in x direction
        fy = msg->K[4];  // Focal length in y direction
        cx = msg->K[2];  // Optical center x coordinate
        cy = msg->K[5];  // Optical center y coordinate
        img_height = msg->height;
        img_width  = msg->width;
        img_header = msg->header;

        //ROS_INFO("height, width: %i, %i", img_height, img_width);
        //ROS_INFO("height, width: %i, %i", msg->height, msg->width);

        //ROS_INFO("Intrinsic Parameters:");
        //ROS_INFO("  fx: %f", fx);
        //ROS_INFO("  fy: %f", fy);
        //ROS_INFO("  cx: %f", cx);
        //ROS_INFO("  cy: %f", cy);
    }

    void project_2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // Project points to image plane
        // Assuming pinhole camera parameters
        sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image());
        image_msg->header = img_header;
        image_msg->height = img_height;
        image_msg->width = img_width;
        image_msg->encoding = "rgb8";
        image_msg->is_bigendian = false;
        image_msg->step = img_width * 3;
        image_msg->data.resize(img_height * img_width * 3);
        // Iterate over all points in the point cloud
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];

            // Project point to image plane
            int u = static_cast<int>((fx * point.x / point.z) + cx);
            int v = static_cast<int>((fy * point.y / point.z) + cy);

            // Check if the point is within the image boundaries
            if (u >= 0 && u < image_msg->width && v >= 0 && v < image_msg->height)
            {
                // Calculate pixel index
                size_t pixel_idx = (v * image_msg->width + u) * 3;
                // Set pixel values
                image_msg->data[pixel_idx] = 255;
                image_msg->data[pixel_idx + 1] = 255;
                image_msg->data[pixel_idx + 2] = 255;
            }
        }

        if (image_msg->data.size() != img_height*img_width*3)
            ROS_ERROR("data size: %lu (should be: %i)", image_msg->data.size(), img_height*img_width*3);
        // Publish the image message
        label_pub.publish(image_msg);
    }

    void callback(my_dyn_rec::MyParamsConfig &config, uint32_t level)
    {
        // ROS_INFO("New values: [%d] - [%s]", config.int_param, config.str_param.c_str());
        ransac_threshold = config.int_param/(double)1000;
    }

private:
    double fx;
    double fy;
    double cx;
    double cy;
    double ransac_threshold = 0.05;
    unsigned int img_height;
    unsigned int img_width;
    std_msgs::Header img_header;
    ros::Publisher inlier_pub;
    ros::Publisher outlier_pub;
    ros::Publisher label_pub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber camera_info_sub;
    ros::NodeHandle nh;
};

// RansacNode* callback_object;

// void callback_wrapper()
// {
//     //ROS_INFO("Callback wrapper");
//     callback_object -> callback();
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_points_node");
    RansacNode rn;
    // callback_object = &rn;

    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig> server;
    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;

    f = boost::bind(&RansacNode::callback, &rn, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
