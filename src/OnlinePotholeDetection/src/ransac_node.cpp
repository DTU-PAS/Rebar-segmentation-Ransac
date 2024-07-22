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

// Include dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <OnlinePotholeDetection/Ransac_node_ParamsConfig.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ximgproc.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Custom code
#include <rebar_seg.h>

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
        // camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &RansacNode::camera_info_callback, this);
    }

    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &input)
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

        // Dot product of each outlier point and the plane normal
        // If the dot product is less than or equal to 0, the point is kept
        // That means the point is on the side of the plane where the normal is pointing

        std::vector<double> a{coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};
        double dot_product = 0;
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> kept_points;
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> non_kept_points;
        for (size_t i = 0; i < outlier_cloud->points.size(); ++i)
        {
            pcl::PointXYZ point = outlier_cloud->points[i];
            std::vector<double> b{point.x, point.y, point.z, 1};
            dot_product = std::inner_product(std::begin(a), std::end(a), std::begin(b), 0.0);
            if (dot_product <= 0)
            {
                kept_points.push_back(point);
            }
            else
            {
                non_kept_points.push_back(point);
            }
        }
        // Add non-kept points to inlier_cloud
        inlier_cloud->points.insert(inlier_cloud->points.end(), non_kept_points.begin(), non_kept_points.end());
        inlier_cloud->width = inlier_cloud->points.size();

        // Update outlier_cloud with kept_points
        outlier_cloud->points = kept_points;
        outlier_cloud->width = kept_points.size();

        // Convert point clouds back to sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 inlier_msg;
        pcl::toROSMsg(*inlier_cloud, inlier_msg);
        inlier_msg.header.frame_id = (*input).header.frame_id;
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

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        // Extract intrinsic camera parameters
        fx = msg->K[0]; // Focal length in x direction
        fy = msg->K[4]; // Focal length in y direction
        cx = msg->K[2]; // Optical center x coordinate
        cy = msg->K[5]; // Optical center y coordinate
        img_height = msg->height;
        img_width = msg->width;
        img_header = msg->header;

        // ROS_INFO("height, width: %i, %i", img_height, img_width);
        // ROS_INFO("height, width: %i, %i", msg->height, msg->width);

        // ROS_INFO("Intrinsic Parameters:");
        // ROS_INFO("  fx: %f", fx);
        // ROS_INFO("  fy: %f", fy);
        // ROS_INFO("  cx: %f", cx);
        // ROS_INFO("  cy: %f", cy);
    }

    void project_2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // Project points to image plane
        // Assuming pinhole camera parameters
        sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image());
        image_msg->header = img_header;
        image_msg->height = img_height;
        // ROS_INFO("Image height %d", img_height);
        image_msg->width = img_width;
        // ROS_INFO("Image width %d", img_width);
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
        // Show the image

        cv::Mat img(img_height, img_width, CV_8UC3, image_msg->data.data());

        if (show_orig_image)
        {
            cv::imshow("Original Ransac Image", img);
            cv::waitKey(1);
        }

        cv::Mat gray_orig;
        //gray_orig = img.clone();
        // Convert the image to grayscale
        // ROS_INFO("Converting image to grayscale");
        cv::cvtColor(img, gray_orig, cv::COLOR_RGB2GRAY);

        // Cluster the image and remove small clusters
        cv::Mat labels, stats, centroids;
        // ROS_INFO("Clustering image");
        cv::connectedComponentsWithStats(gray_orig, labels, stats, centroids);
        for (int i = 1; i < stats.rows; ++i)
        {
            if (stats.at<int>(i, cv::CC_STAT_AREA) < min_cluster_size)
            {
                cv::Mat mask = labels == i;
                gray_orig.setTo(0, mask);
                img.setTo(cv::Scalar(0, 0, 0), mask);
            }
        }

        float vertical_angle, horizontal_angle;

        // ROS_INFO("Finding rotation");
        std::pair<double, double> angles = find_rotation(gray_orig, show_angles);

        // ROS_INFO("Rotating image");
        cv::Mat rotated_image = rotate_image("Image", gray_orig, angles.second, show_rotated_image);
        cv::Mat thresholded_image;
        // ROS_INFO("Thresholding image");
        cv::threshold(rotated_image, thresholded_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // Apply opencv ximgproc thinning algorithm
        cv::Mat thinned_image;
        // ROS_INFO("Thinning image");
        cv::ximgproc::thinning(rotated_image, thinned_image);

        // Call the split_horizontal_and_vertical function
        // ROS_INFO("Splitting image");
        auto resulting_split = split_horizontal_and_vertical(thinned_image, 10, show_split_image);

        // Extract the results
        cv::Mat pruned_vertical = resulting_split.first;
        cv::Mat pruned_horizontal = resulting_split.second;

        // ROS_INFO("Reconstructing skeleton");
        cv::Mat reconstructed_vertical_skeleton = reconstruct_skeleton("Vertical", pruned_vertical, thinned_image, 3, 10, show_reconstructed_image);
        cv::Mat reconstructed_horizontal_skeleton = reconstruct_skeleton("Horizontal", pruned_horizontal, thinned_image, 3, 10, show_reconstructed_image);

        // ROS_INFO("Removing small blobs");
        cv::Mat clean_vertical = remove_small_blobs("Vertical", reconstructed_vertical_skeleton, 70, show_image_without_blobs);
        cv::Mat clean_horizontal = remove_small_blobs("Horizontal", reconstructed_horizontal_skeleton, 70, show_image_without_blobs);

        // ROS_INFO("Reverse vertical image rotating image");
        cv::Mat back_rotated_image_vertical = rotate_image("Vertical - reverse rotation", clean_vertical, -angles.second, show_rotated_image);
        cv::Mat thresholded_image_vertical;
        cv::threshold(back_rotated_image_vertical, thresholded_image_vertical, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::Mat vertical;
        cv::ximgproc::thinning(thresholded_image_vertical, vertical);

        // ROS_INFO("Reverse horizontal image rotating image");
        cv::Mat back_rotated_image_horizontal = rotate_image("Horizontal - reverse rotation", clean_horizontal, -angles.second, show_rotated_image);
        cv::Mat thresholded_image_horizontal;
        cv::threshold(back_rotated_image_horizontal, thresholded_image_horizontal, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::Mat horizontal;
        cv::ximgproc::thinning(thresholded_image_horizontal, horizontal);

        // ROS_INFO("Clustering vertical and horizontal skeletons");
        cluster_info result_clustering_vertical = cluster_skeleton("Vertical", vertical, show_clusters);
        cluster_info result_clustering_horizontal = cluster_skeleton("Horizontal", horizontal, show_clusters);

        cv::Mat vertical_clustered = result_clustering_vertical.img;
        cv::Mat horizontal_clustered = result_clustering_horizontal.img;

        cv::Mat vertical_labels = result_clustering_vertical.labels;
        cv::Mat horizontal_labels = result_clustering_horizontal.labels;

        int num_labels_vertical = result_clustering_vertical.num_clusters;
        int num_labels_horizontal = result_clustering_horizontal.num_clusters;

        // // // ROS_INFO("Finding area of interest");
        // ROS_INFO("Shape of gray_orig: %i, %i", gray_orig.rows, gray_orig.cols);
        // ROS_INFO("Shape of img: %i, %i", img.rows, img.cols);
        // ROS_INFO("Shape of vertical_labels: %i, %i", vertical_labels.rows, vertical_labels.cols);
        // ROS_INFO("Shape of horizontal_labels: %i, %i", horizontal_labels.rows, horizontal_labels.cols);

        auto result_vertical = find_area_of_interest("Vertical", vertical_labels, num_labels_vertical, gray_orig, img, show_roi);
        auto result_horizontal = find_area_of_interest("Horizontal", horizontal_labels, num_labels_horizontal, gray_orig, img, show_roi);

        // Extract results
        std::vector<std::pair<cv::Point, cv::Point>> closest_pixels_vertical = std::get<0>(result_vertical);
        std::vector<std::pair<cv::Point, cv::Point>> bboxs_vertical = std::get<1>(result_vertical);

        std::vector<std::pair<cv::Point, cv::Point>> closest_pixels_horizontal = std::get<0>(result_horizontal);
        std::vector<std::pair<cv::Point, cv::Point>> bboxs_horizontal = std::get<1>(result_horizontal);

        // Draw bboxs on the image
        if (show_final_image)
        {
            for (size_t i = 0; i < bboxs_vertical.size(); ++i)
            {
                cv::rectangle(img, bboxs_vertical[i].first, bboxs_vertical[i].second, cv::Scalar(0, 255, 0), 2);
            }
            for (size_t i = 0; i < bboxs_horizontal.size(); ++i)
            {
                cv::rectangle(img, bboxs_horizontal[i].first, bboxs_horizontal[i].second, cv::Scalar(0, 255, 255), 2);
            }

            // Draw a line between the closest pixels
            for (size_t i = 0; i < closest_pixels_vertical.size(); ++i)
            {
                //ROS_INFO("Vertical: (%i, %i) - (%i, %i)", closest_pixels_vertical[i].first.x, closest_pixels_vertical[i].first.y, closest_pixels_vertical[i].second.x, closest_pixels_vertical[i].second.y);
                cv::line(img, closest_pixels_vertical[i].first, closest_pixels_vertical[i].second, cv::Scalar(0, 0, 255), 2);
            }
            for (size_t i = 0; i < closest_pixels_horizontal.size(); ++i)
            {
                //ROS_INFO("Horizontal: (%i, %i) - (%i, %i)", closest_pixels_horizontal[i].first.x, closest_pixels_horizontal[i].first.y, closest_pixels_horizontal[i].second.x, closest_pixels_horizontal[i].second.y);
                cv::line(img, closest_pixels_horizontal[i].first, closest_pixels_horizontal[i].second, cv::Scalar(255, 0, 0), 2);
            }
            
            // std::cout << std::endl;

            // Show the image
            cv::namedWindow("Projected_Image");
            cv::imshow("Projected_Image", img);
            cv::waitKey(1);
        }

        if (image_msg->data.size() != img_height * img_width * 3)
            ROS_ERROR("data size: %lu (should be: %i)", image_msg->data.size(), img_height * img_width * 3);
        // Publish the image message
        label_pub.publish(image_msg);
    }

    void callback(OnlinePotholeDetection::Ransac_node_ParamsConfig &config, uint32_t level)
    {
        // ROS_INFO("New values: [%d] - [%s]", config.int_param, config.str_param.c_str());
        ransac_threshold = config.ransac_threshold / (double)1000;
        min_cluster_size = config.min_cluster_size;

        // Flags for showing images
        show_orig_image = config.show_orig_image;
        show_rotated_image = config.show_rotated_image;
        show_split_image = config.show_split_image;
        show_reconstructed_image = config.show_reconstructed_image;
        show_image_without_blobs = config.show_image_without_blobs;
        show_clusters = config.show_clusters;
        show_roi = config.show_roi;
        show_final_image = config.show_final_image;
        show_angles = config.show_angles;
    }

private:
    double fx;
    double fy;
    double cx;
    double cy;
    double ransac_threshold = 0.03;
    int min_cluster_size = 350;
    unsigned int img_height;
    unsigned int img_width;
    std_msgs::Header img_header;
    ros::Publisher inlier_pub;
    ros::Publisher outlier_pub;
    ros::Publisher label_pub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber camera_info_sub;
    ros::NodeHandle nh;

    // Flags for showing images
    bool show_orig_image = false;
    bool show_rotated_image = false;
    bool show_split_image = false;
    bool show_reconstructed_image = false;
    bool show_image_without_blobs = false;
    bool show_clusters = false;
    bool show_roi = true;
    bool show_final_image = true;
    bool show_angles = false;
};

// RansacNode* callback_object;

// void callback_wrapper()
// {
//     //ROS_INFO("Callback wrapper");
//     callback_object -> callback();
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ransac_points_node");
    RansacNode rn;
    // callback_object = &rn;

    dynamic_reconfigure::Server<OnlinePotholeDetection::Ransac_node_ParamsConfig> server;
    dynamic_reconfigure::Server<OnlinePotholeDetection::Ransac_node_ParamsConfig>::CallbackType f;

    f = boost::bind(&RansacNode::callback, &rn, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
