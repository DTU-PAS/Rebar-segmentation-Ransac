// Include C++ headers
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

// Include ROS
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

// Include dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <rebarsegmenation/Ransac_node_ParamsConfig.h>

// Include opencv2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// Include PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>


// RobetArme service calls
#include <atomic>
#include <functional>
#include <memory>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include <boost/bind.hpp>
#include <ros/callback_queue.h>

// Custom code
#include <image_processing.h>
#include <ransac_node.h>
#include <publish.h>
#include <utils.h>


bool aligned_depth = false;

class RansacNode
{
public:
    RansacNode(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void dynamic_reconfigure_callback(
        rebarsegmenation::Ransac_node_ParamsConfig &config, uint32_t level);

private:
    double fx;
    double fy;
    double cx;
    double cy;
    cv::Mat K;
    cv::Mat K_inv;

    std::vector<double> normal_vector = {0.0, 0.0, 0.0}; // To store the normal vector

    unsigned int img_height;
    unsigned int img_width;
    std_msgs::Header img_header;
    double distance_from_rebar_to_background;
    double distance_from_camera_to_rebar;

    // Publishers
    ros::Publisher inlier_pub;
    ros::Publisher outlier_pub;
    ros::Publisher label_pub;
    ros::Publisher ball_pub;
    ros::Publisher str_pub;
    // ros::Publisher centerline_pub_vertical;
    // ros::Publisher centerline_pub_horizontal;
    ros::Publisher line_pub;
    ros::Publisher point_pub;
    ros::Publisher vertical_point_pub;
    ros::Publisher horizontal_point_pub;

    // Subscribers
    ros::Subscriber pointcloud_sub;
    ros::Subscriber camera_info_sub;
    ros::Subscriber rgb_sub;
    ros::Subscriber depth_sub;

    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::pair<double, double> angles;

    cv::Mat rgb_image;
    cv::Mat depth_image;
    cv::Mat depth_image_rgb;
    cv::Mat depth_image_rotated;
    cv::Mat main_img;
    cv::Mat main_img_rgb;
    cv::Mat gray_orig;
    cv::Mat thinned_image;
    std::pair<cv::Mat, cv::Mat> resulting_split;
    cv::Mat vertical;
    cv::Mat horizontal;

    frame_AOI_info frames_history_vertical;
    frame_AOI_info frames_history_horizontal;

    // Dynamic reconfigure variables
    double ransac_threshold_1 = 0.02;
    double ransac_threshold_2 = 0.02;

    int min_cluster_size = 150;
    double required_confidence = 0.7;

    // Flags for showing images
    bool show_orig_image = false;
    bool show_rotated_image = false;
    bool show_split_image = false;
    bool show_clusters = false;
    bool show_roi = false;
    bool show_final_image = true;
    bool show_angles = false;

    int exit_counter = 0;

    int frame_count = 0;
    double fps = 0.0;
    std::chrono::high_resolution_clock::time_point start =
        std::chrono::high_resolution_clock::now();

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_buffer_;
    int buffer_size_ = 3;

    // void
    // dynamic_reconfigure_callback(rebarsegmenation::Ransac_node_ParamsConfig
    // &config, uint32_t level);
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &input);
    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg);
    void rgb_callback(const sensor_msgs::ImageConstPtr &msg);
    void depth_callback(const sensor_msgs::ImageConstPtr &msg);

    void project_2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void detect_AOI();
    void verticalProcess();
    void horizontalProcess();

    bool enable_detection = true;
    bool camera_info_received = false;

    ros::CallbackQueue independent_service_queue_;

    // Spins independent_service_queue in a separate thread
    ros::AsyncSpinner async_spinner_;

    // Service servers to power this node on and off
    ros::ServiceServer enable_service_;
    ros::ServiceServer disable_pause_service_;
    ros::ServiceServer disable_stop_service_;
    ros::ServiceServer shutdown_service_;
    ros::ServiceServer kill_service_;

    // Service client for task planner to join when this node's execution is
    // over
    ros::ServiceClient trigger_join_service_client_;

    // A timer that checks whether it's time to modify the state of the
    // functionality that this node provides
    ros::Timer clock_;

    // Available states
    enum States_
    {
        PLAY = 0,
        PAUSE = 1,
        STOP = 2,
        SHUTDOWN = 3,
        KILL = 4
    };
    std::vector<std::vector<bool>> transition_table_;

    enum States_ state_t0_init_ = STOP;
    enum States_ state_t1_init_ = STOP;
    std::atomic<States_> state_t0_;
    std::atomic<States_> state_t1_;
    std::atomic<bool> is_node_alive_;

    // In case of shutdown or kill: the call to notification of the task
    // planner must occur inside the clock callback instead of inside the
    // handleRequest{Kill,Shutdown}; otherwise ROS forces a 5 sec block.
    // If then the call to notification happens inside the callback, the
    // threads running commit*Suicide run on their own, and perhaps kill or
    // shutdown happens before the task planner has been notified. This variable
    // mitigates this.
    bool is_task_planner_notified_;

    // Set this at the end
    bool has_functionality_been_delivered_successfully_;

    // Does this node need to notify the task planner that its functionality
    // has been served? Or is this node running solo?
    bool task_planner_notify_end_;

    // Maps a thread's given name to its native_handle()
    std::unordered_map<std::string, pthread_t> pthread_map_;

    /***************************************************************************
     * @brief Executes periodically. Reads the execution status flags, which are
     * modified from the service server calls coming from the task planner
     * that manages this pkg.
     */
    void clockCallback(const ros::TimerEvent &te);

    /***************************************************************************
     * @brief The final act: calls ros::shutdown()
     */
    void commitNodeSuicide();

    /***************************************************************************
     * @brief The final act: calls system("kill self pid")
     */
    void commitProcessSuicide();

    /***************************************************************************
     * @brief Wraps the functionality that this package provides for use with
     * a task planner
     */
    void entrypointWrapper();

    /***************************************************************************
     * @brief Welding: the functionality that the package offers
     */
    void executeWelding();

    /***************************************************************************
     * @brief Service servers to start, pause, and stop the functionality that
     * this node offers, and kill the node altogether
     */
    bool handleRequestDisablePause(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res);
    bool handleRequestDisableStop(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res);
    bool handleRequestEnable(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res);
    bool handleRequestShutdown(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response &res);
    bool handleRequestKill(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res);

    /***************************************************************************
     * @brief Notifies the task planner that manages this pkg that this node has
     * finished its work (or that it was stopped/killed in the process---and its
     * work was forced to finish).
     */
    bool taskPlannerNotifyEnd();

    /***************************************************************************
     * @brief If the entrypoint of your node is not a periodic callback then you
     * may spin up a thread.
     */
    void threadStart(const std::string &tname);
    void threadStop(const std::string &tname);
};

void RansacNode::dynamic_reconfigure_callback(rebarsegmenation::Ransac_node_ParamsConfig &config, uint32_t level)
{
    // ransac_threshold = config.ransac_threshold / (double)1000;
    ransac_threshold_1 = config.ransac_threshold_1 / (double)1000;
    ransac_threshold_2 = config.ransac_threshold_2 / (double)1000;
    min_cluster_size = config.min_cluster_size;

    // Flags for showing images
    show_rotated_image = config.show_rotated_image;
    show_split_image = config.show_split_image;
    show_clusters = config.show_clusters;
    show_roi = config.show_roi;
    show_angles = config.show_angles;
    required_confidence = config.required_confidence / (double)HISTORY;
}

void RansacNode::pointcloud_callback(
    const sensor_msgs::PointCloud2ConstPtr &input)
{
    if (!enable_detection || !camera_info_received) {
        return;
    }

    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    // Store the cloud in buffer
    cloud_buffer_.push_back(cloud);
    if (cloud_buffer_.size() > buffer_size_) {
        cloud_buffer_.erase(cloud_buffer_.begin()); // Keep buffer size fixed
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &cloud : cloud_buffer_) {
        *merged_cloud += *cloud; // Concatenation
    }

    // project_2D(cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_threshold_1);
    seg.setMaxIterations(500);
    seg.setInputCloud(merged_cloud);
    seg.segment(*inliers, *coefficients);

    // Extract inliers and outliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_cloud_out(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(merged_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inlier_cloud);
    extract.setNegative(true);
    extract.filter(*outlier_cloud);

    // Dot product of each outlier point and the plane normal
    // If the dot product is less than or equal to 0, the point is kept
    // That means the point is on the side of the plane where the normal is
    // pointing

    // Also calculate the average distance of the kept points to the plane

    std::vector<double> a{coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};
    double dot_product = 0;
    double total_distance_to_background = 0.0;
    double total_distance_to_camera = 0.0;
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>
        kept_points;
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>
        non_kept_points;
    for (size_t i = 0; i < outlier_cloud->points.size(); ++i) {
        pcl::PointXYZRGB point = outlier_cloud->points[i];
        std::vector<double> b{point.x, point.y, point.z, 1};
        dot_product = std::inner_product(std::begin(a), std::end(a), std::begin(b), 0.0);
        // if (dot_product <= 0 && dot_product >= -0.2)
        if (dot_product >= -0.2 && dot_product <= -0.15) {
            kept_points.push_back(point);
            double distance_to_background =
                std::abs(dot_product) /
                std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            total_distance_to_background += distance_to_background;

            double distance_to_camera =
                std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            total_distance_to_camera += distance_to_camera;

            normal_vector[0] += a[0];
            normal_vector[1] += a[1];
            normal_vector[2] += a[2];
        } 
        else 
        {
            non_kept_points.push_back(point);
        }
    }

    double norm = std::sqrt(normal_vector[0] * normal_vector[0] +
                            normal_vector[1] * normal_vector[1] +
                            normal_vector[2] * normal_vector[2]);

    if (norm != 0) {
        normal_vector[0] /= norm;
        normal_vector[1] /= norm;
        normal_vector[2] /= norm;
    }

    if (!kept_points.empty()) {
        distance_from_rebar_to_background =
            total_distance_to_background / kept_points.size();
        distance_from_camera_to_rebar =
            total_distance_to_camera / kept_points.size();
    }

    // Add non-kept points to inlier_cloud
    inlier_cloud->points.insert(inlier_cloud->points.end(),
                                non_kept_points.begin(), non_kept_points.end());
    inlier_cloud->width = inlier_cloud->points.size();
    inlier_cloud->height = 1; // Since it's a 1D point cloud
    inlier_cloud->is_dense = true;

    // Update outlier_cloud with kept_points
    outlier_cloud_out->points = kept_points;
    outlier_cloud_out->width = kept_points.size();
    outlier_cloud_out->height = 1; // Since it's a 1D point cloud
    outlier_cloud_out->is_dense = true;

    if (outlier_cloud_out->points.empty()) {
        ROS_ERROR("Outlier cloud is empty after updating with kept points");

        pcl::ModelCoefficients::Ptr coefficients_new(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_new(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg_new;
        seg_new.setOptimizeCoefficients(true);
        seg_new.setModelType(pcl::SACMODEL_PLANE);
        seg_new.setMethodType(pcl::SAC_RANSAC);
        seg_new.setDistanceThreshold(ransac_threshold_2);
        seg_new.setMaxIterations(500);
        seg_new.setInputCloud(outlier_cloud);
        seg_new.segment(*inliers_new, *coefficients_new);

        // Extract inliers and outliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud_new(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_cloud_new(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_cloud_out_new( new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract_new;
        extract_new.setInputCloud(outlier_cloud);
        extract_new.setIndices(inliers_new);
        extract_new.setNegative(false);
        extract_new.filter(*inlier_cloud_new);
        extract_new.setNegative(true);
        extract_new.filter(*outlier_cloud_new);

        outlier_cloud_out = outlier_cloud_new;

        std::vector<double> a_new{coefficients_new->values[0], coefficients_new->values[1], coefficients_new->values[2], coefficients_new->values[3]};
        double dot_product_new = 0;
        // double total_distance_to_background = 0.0;
        // double total_distance_to_camera = 0.0;
        std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>kept_points_new;
        std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>non_kept_points_new;
        for (size_t i = 0; i < outlier_cloud_new->points.size(); ++i)
        {
            pcl::PointXYZRGB point_new = outlier_cloud_new->points[i];
            std::vector<double> b_new{point_new.x, point_new.y, point_new.z, 1};
            dot_product_new = std::inner_product(std::begin(a_new), std::end(a_new), std::begin(b_new), 0.0);
            // if (dot_product <= 0 && dot_product >= -0.2)
            if (dot_product_new >= -0.2 && dot_product_new <= -0.07) {
                kept_points_new.push_back(point_new);
                // double distance_to_background = std::abs(dot_product) /
                // std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
                // total_distance_to_background += distance_to_background;

                // double distance_to_camera = std::sqrt(point.x * point.x + point.y *
                // point.y + point.z * point.z); total_distance_to_camera +=
                // distance_to_camera;
            } else {
                non_kept_points_new.push_back(point_new);
            }
        }

        // if (!kept_points.empty())
        // {
        //     distance_from_rebar_to_background = total_distance_to_background /
        //     kept_points.size(); distance_from_camera_to_rebar =
        //     total_distance_to_camera / kept_points.size();
        // }

        // if (outlier_cloud_out->points.empty()) {
        //     ROS_ERROR("Outlier cloud is empty after running ransac again");
        //     return;
        // }

        // // Update outlier_cloud with kept_points
        outlier_cloud_out->points = kept_points_new;
        outlier_cloud_out->width = kept_points_new.size();
        outlier_cloud_out->height = 1; // Since it's a 1D point cloud
        outlier_cloud_out->is_dense = true;
    }

    // Convert point clouds back to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 inlier_msg;
    pcl::toROSMsg(*inlier_cloud, inlier_msg);
    inlier_msg.header.frame_id = (*input).header.frame_id;
    inlier_msg.header.stamp = ros::Time::now();
    inlier_pub.publish(inlier_msg);

    sensor_msgs::PointCloud2 outlier_msg;
    pcl::toROSMsg(*outlier_cloud_out, outlier_msg);
    outlier_msg.header.frame_id = (*input).header.frame_id;
    outlier_msg.header.stamp = ros::Time::now();
    outlier_pub.publish(outlier_msg);

    project_2D(outlier_cloud_out);
}

// // TODO: The out of memory error is caused by not receiving the camera info
// message first and thereby setting important variables
void RansacNode::camera_info_callback(
    const sensor_msgs::CameraInfoConstPtr &msg)
{
    if (!enable_detection || camera_info_received) {
        return;
    }
    // Extract intrinsic camera parameters
    fx = msg->K[0]; // Focal length in x direction
    fy = msg->K[4]; // Focal length in y direction
    cx = msg->K[2]; // Optical center x coordinate
    cy = msg->K[5]; // Optical center y coordinate
    img_height = msg->height;
    img_width = msg->width;
    img_header = msg->header;

    K = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    K_inv = K.inv();
    camera_info_received = true;
}

void RansacNode::rgb_callback(const sensor_msgs::ImageConstPtr &rgb_msg)
{
    if (!enable_detection || !camera_info_received) {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        rgb_image = cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void RansacNode::depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    if (!enable_detection || !camera_info_received) {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat depth_image_color;
    cv::Mat depth_image_shifted;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_image = cv_ptr->image;
        if (depth_image.empty()) {
            ROS_ERROR("Depth image is empty");
            return;
        }
        
    } catch (cv_bridge::Exception &e) { 
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    // cv::cvtColor(depth_image, depth_image_color, cv::COLOR_GRAY2BGR);
    // // Shift depth image to the right by 10 pixels
    // cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, 18, 0, 1, 0);
    // cv::warpAffine(depth_image, depth_image_shifted, M, depth_image.size());
    // depth_image = depth_image_shifted;
}

void RansacNode::project_2D(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Project points to image plane
    // Assuming pinhole camera parameters
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image());
    sensor_msgs::ImagePtr image_msg_rgb(new sensor_msgs::Image());

    image_msg->header = img_header;
    image_msg->height = img_height;
    image_msg->width = img_width;
    image_msg->encoding = "rgb8";
    image_msg->is_bigendian = false;
    image_msg->step = img_width * 3;
    image_msg->data.resize(img_height * img_width * 3);

    image_msg_rgb->header = img_header;
    image_msg_rgb->height = img_height;
    image_msg_rgb->width = img_width;
    image_msg_rgb->encoding = "rgb8";
    image_msg_rgb->is_bigendian = false;
    image_msg_rgb->step = img_width * 3;
    image_msg_rgb->data.resize(img_height * img_width * 3);

    // Iterate over all points in the point cloud
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        pcl::PointXYZRGB point = cloud->points[i];

        // Project point to image plane
        int u = static_cast<int>((fx * point.x / point.z) + cx);
        int v = static_cast<int>((fy * point.y / point.z) + cy);
        // Check if the point is within the image boundaries
        if (u >= 0 && u < image_msg->width && v >= 0 && v < image_msg->height) {
            // Calculate pixel index
            size_t pixel_idx = (v * image_msg->width + u) * 3;
            // Set pixel values
            image_msg_rgb->data[pixel_idx] = point.b;
            image_msg_rgb->data[pixel_idx + 1] = point.g;
            image_msg_rgb->data[pixel_idx + 2] = point.r;

            image_msg->data[pixel_idx] = 255;
            image_msg->data[pixel_idx + 1] = 255;
            image_msg->data[pixel_idx + 2] = 255;
        }
    }

    cv::Mat img(img_height, img_width, CV_8UC3, image_msg->data.data());
    cv::Mat img_rgb(img_height, img_width, CV_8UC3, image_msg_rgb->data.data());

    main_img = img;
    main_img_rgb = img_rgb;

    detect_AOI();

    if (image_msg->data.size() != img_height * img_width * 3) {
        ROS_ERROR("data size: %lu (should be: %i)", image_msg->data.size(),
                  img_height * img_width * 3);
    }
    // Publish the image message
    label_pub.publish(image_msg);
}

void RansacNode::detect_AOI()
{
    if (main_img.empty()) {
        ROS_ERROR("Main image is empty");
        return;
    }

    if (depth_image.empty()) {
        ROS_ERROR("Depth image is empty");
        return;
    }
    //  Convert the image to grayscale
    cv::cvtColor(main_img, gray_orig, cv::COLOR_RGB2GRAY);

    if (gray_orig.empty()) {
        ROS_ERROR("Grayscale image is empty");
        return;
    }

    // Dialate image to remove small holes
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::dilate(gray_orig, gray_orig, kernel);

    // Erode image to remove small blobs
    cv::erode(gray_orig, gray_orig, kernel);

    // Cluster the image and remove small clusters
    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(gray_orig, labels, stats, centroids);
    for (int i = 1; i < stats.rows; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) < min_cluster_size) {
            cv::Mat mask = labels == i;
            gray_orig.setTo(0, mask);
        }
    }

    angles = find_rotation(gray_orig, show_angles);

    cv::Mat rotated_image =
        rotate_image("Image", gray_orig, angles.second, show_rotated_image);
    depth_image_rotated = rotate_image("Depth Image", depth_image, angles.second,
                                       show_rotated_image);

    // Check if the rotated image is empty
    if (rotated_image.empty()) {
        ROS_ERROR("Rotated image is empty");
        return;
    }

    cv::cvtColor(depth_image_rotated, depth_image_rgb, cv::COLOR_GRAY2BGR);

    cv::Mat thresholded_image;
    cv::threshold(rotated_image, thresholded_image, 0, 255,
                  cv::THRESH_BINARY | cv::THRESH_OTSU);

    resulting_split = split_horizontal_and_vertical(rotated_image, angles, 60,
                                                    show_split_image);


    verticalProcess();
    horizontalProcess();
    cv::imshow("Main Image", main_img_rgb);
    cv::imshow("Depth Image", depth_image_rgb);
    cv::waitKey(1);

    // exit_counter++;
    // if (exit_counter >= 50)
    // {
    //     // cv::destroyAllWindows();
    //     enable_detection = false;
    //     state_t1_.store(PAUSE);
    //     exit_counter = 0;
    // }
}

void RansacNode::verticalProcess()
{
    frames_history_vertical.ns = "Vertical";
    // dialate the image to remove small holes
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(resulting_split.first, resulting_split.first, kernel);
    // erode the image to remove small blobs
    cv::erode(resulting_split.first, resulting_split.first, kernel);
    // cv::ximgproc::thinning(resulting_split.first, frames_history_vertical.skeleton, cv::ximgproc::THINNING_GUOHALL);
    std::vector<skeleton_info> skel_info;
    skel_info = detect_width(frames_history_vertical, resulting_split.first);

    for(size_t i = 0; i < skel_info.size(); i++)
    {
        for (size_t j = 0; j < skel_info[i].points.size(); j++)
        {
            cv::circle(main_img_rgb, skel_info[i].points[j], 1, cv::Scalar(0, 0, 255), -1);
        }
    }

    for (size_t i = 0; i < skel_info.size(); i++) {
        std::vector<cv::Point> points;
        for (size_t j = 0; j < skel_info[i].points.size(); j++) {
            points.push_back(skel_info[i].points[j]);
            cv::circle(main_img_rgb, skel_info[i].points[j], 1, cv::Scalar(0, 255, 0), -1);
            cv::circle(depth_image_rgb, skel_info[i].points[j], 1, cv::Scalar(0, 255, 0), -1);
        }

        // Convert to 3D points
        std::vector<cv::Point3f> points_3f_list;
        for (size_t j = 0; j < points.size(); j++) {
            cv::Point3f point_3f;
            double Z = find_depth(depth_image_rotated, points[j].x, points[j].y);
            if (std::isnan(Z) || Z <= 0) {
                continue; // Ignore invalid depth values
            }
            cv::Point3f point = pixel_to_camera(K, points[j].x, points[j].y, Z);
            points_3f_list.push_back(point);
        }

        if (points_3f_list.empty()) {
            ROS_WARN("No valid 3D points to publish");
            continue;
        }

        std::pair<cv::Point3f, cv::Point3f> line_points = fitLineRANSAC(points_3f_list);

        publish_line(frames_history_vertical, line_points, cv::Scalar(0, 0, 255), line_pub);

        
    }
}

void RansacNode::horizontalProcess()
{
    frames_history_horizontal.ns = "Horizontal";
    // dialate the image to remove small holes
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(resulting_split.second, resulting_split.second, kernel);

    // erode the image to remove small blobs
    cv::erode(resulting_split.second, resulting_split.second, kernel);
    // cv::ximgproc::thinning(resulting_split.second, frames_history_horizontal.skeleton, cv::ximgproc::THINNING_GUOHALL);
    std::vector<skeleton_info> points;

    std::vector<skeleton_info> skel_info;
    skel_info = detect_width(frames_history_horizontal, resulting_split.second);

    // Inside your processing loop:
    for (size_t i = 0; i < skel_info.size(); i++) {
        std::vector<cv::Point> points;
        for (size_t j = 0; j < skel_info[i].points.size(); j++) {
            points.push_back(skel_info[i].points[j]);
            cv::circle(main_img_rgb, skel_info[i].points[j], 1, cv::Scalar(0, 255, 0), -1);
            cv::circle(depth_image_rgb, skel_info[i].points[j], 1, cv::Scalar(0, 255, 0), -1);
        }

        // Convert to 3D points
        std::vector<cv::Point3f> points_3f_list;
        for (size_t j = 0; j < points.size(); j++) {
            cv::Point3f point_3f;
            double Z = find_depth(depth_image_rotated, points[j].x, points[j].y);
            if (std::isnan(Z) || Z <= 0) {
                continue; // Ignore invalid depth values
            }
            cv::Point3f point = pixel_to_camera(K, points[j].x, points[j].y, Z);
            points_3f_list.push_back(point);
        }

        if (points_3f_list.empty()) {
            ROS_WARN("No valid 3D points to publish");
            continue;
        }

        std::pair<cv::Point3f, cv::Point3f> line_points = fitLineRANSAC(points_3f_list);

        publish_line(frames_history_horizontal, line_points, cv::Scalar(255, 0, 255), line_pub);
    }
}

/*******************************************************************************
 * @brief Executes periodically.
 */
void RansacNode::clockCallback(const ros::TimerEvent &te)
{
    // ---------------------------------------------------------------------------
    if (state_t0_.load() != state_t1_.load()) {
        // Transition to state if transition is allowed
        if (transition_table_[state_t0_.load()][state_t1_.load()] == true) {
            ROS_INFO("EXECUTING TRANSITION: %d -> %d", state_t0_.load(),
                     state_t1_.load());

            // Store new state to old
            state_t0_.store(state_t1_.load());

            // -------------------------------------
            if (state_t1_.load() == PLAY) {
                ROS_INFO("Triggering execution PLAY");
                threadStart("shotcreting_thread");
            }
            // -------------------------------------
            if (state_t1_.load() == PAUSE) {
                ROS_INFO("Triggering execution PAUSE");
                threadStop("shotcreting_thread");
            }
            // -------------------------------------
            if (state_t1_.load() == STOP) {
                ROS_INFO("Triggering execution STOP");
                threadStop("shotcreting_thread");

                // Notify that welding has ended unsuccessfully first (otherwise the
                // task planner that called this package cannot join it after it was
                // stopped, which means that it will block)
                if (task_planner_notify_end_ == true)
                    taskPlannerNotifyEnd();
            }
            // -------------------------------------
            if (state_t1_.load() == SHUTDOWN) {
                // Handled in own service callback. Otherwise functions called in this
                // scope might block SHUTDOWN

                // Notify that welding has ended unsuccessfully first (otherwise the
                // task planner that called this package cannot join it after it was
                // stopped, which means that it will block)
                if (task_planner_notify_end_ == true)
                    is_task_planner_notified_ = taskPlannerNotifyEnd();
            }
            // -------------------------------------
            if (state_t1_.load() == KILL) {
                // Handled in own service callback. Otherwise functions called in this
                // scope might block KILL

                // Notify that welding has ended unsuccessfully first (otherwise the
                // task planner that called this package cannot join it after it was
                // stopped, which means that it will block)
                if (task_planner_notify_end_ == true)
                    is_task_planner_notified_ = taskPlannerNotifyEnd();
            }
        } else {
            ROS_INFO("TRANSITION: %d -> %d NOT ALLOWED", state_t0_.load(),
                     state_t1_.load());

            // Store old state to new
            state_t1_.store(state_t0_.load());
        }
    }
    // ---------------------------------------------------------------------------
}

/*******************************************************************************
 * @brief The final act: calls ros::shutdown()
 */
void RansacNode::commitNodeSuicide()
{
    if (task_planner_notify_end_ == true) {
        while (is_task_planner_notified_ == false)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ROS_INFO("Triggering node SHUTDOWN");
    ros::shutdown();
}

/*******************************************************************************
 * @brief The final act: calls system("kill self pid")
 */
void RansacNode::commitProcessSuicide()
{
    if (task_planner_notify_end_ == true) {
        while (is_task_planner_notified_ == false)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ROS_INFO("Triggering node KILL");

    pid_t pid = getpid();
    std::string s = "kill -9 " + std::to_string(pid);
    int ret = system(s.c_str());
}

/*******************************************************************************
 * @brief Service server to pause the functionality that this node offers
 */
bool RansacNode::handleRequestDisablePause(std_srvs::Trigger::Request &req,
                                           std_srvs::Trigger::Response &res)
{
    ROS_INFO("Requesting execution PAUSE");
    state_t1_.store(PAUSE);

    res.message = "SUCCESS";
    res.success = true;
    return res.success;
}

/*******************************************************************************
 * @brief Service server to stop the functionality that this node offers
 * (pause + return to task planner)
 */
bool RansacNode::handleRequestDisableStop(std_srvs::Trigger::Request &req,
                                          std_srvs::Trigger::Response &res)
{
    ROS_INFO("Requesting execution STOP");
    state_t1_.store(STOP);

    res.message = "SUCCESS";
    res.success = true;
    return res.success;
}

/*******************************************************************************
 * @brief Service server to start the functionality that this node offers
 */
bool RansacNode::handleRequestEnable(std_srvs::Trigger::Request &req,
                                     std_srvs::Trigger::Response &res)
{
    ROS_INFO("Requesting execution PLAY");
    state_t1_.store(PLAY);

    res.message = "SUCCESS";
    res.success = true;
    return res.success;
}

/*******************************************************************************
 * @brief Service server to kill the process that runs the node
 */
bool RansacNode::handleRequestKill(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res)
{
    ROS_WARN("Requesting node KILL");

    state_t1_.store(KILL);
    is_node_alive_.store(false);

    // Spin a new thread so that the service has time to respond back
    std::thread t = std::thread(&RansacNode::commitProcessSuicide, this);
    t.detach();

    res.message = "SUCCESS";
    res.success = true;
    return res.success;
}

/*******************************************************************************
 * @brief Service server to shutdown the node altogether
 * https://answers.ros.org/question/294069/shutdown-a-node-from-another-node-in-ros-cpp/
 */
bool RansacNode::handleRequestShutdown(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
    ROS_WARN("Requesting node SHUTDOWN");

    state_t1_.store(SHUTDOWN);
    is_node_alive_.store(SHUTDOWN);

    // Spin a new thread so that the service has time to respond back
    std::thread t = std::thread(&RansacNode::commitNodeSuicide, this);
    t.detach();

    res.message = "SUCCESS";
    res.success = true;
    return res.success;
}

/*******************************************************************************
 * @brief Notifies the task planner that manages this pkg that this node has
 * finished its work (or that it was stopped/shut down in the process---and its
 * work was forced to finish).
 */
bool RansacNode::taskPlannerNotifyEnd()
{
    while (!ros::service::waitForService(
        trigger_join_service_client_.getService(), ros::Duration(1.0))) {
        if (!ros::ok()) {
            ROS_ERROR("Client of service %s interrupted while waiting for service to "
                      "appear",
                      trigger_join_service_client_.getService().c_str());
            return false;
        }
        ROS_WARN("Waiting for service %s to appear...",
                 trigger_join_service_client_.getService().c_str());
    }
    // service appeared

    // Craft request
    std_srvs::SetBool srv;
    srv.request.data = has_functionality_been_delivered_successfully_;

    // Notify the task planner that this package ended work
    // and continue with execution
    while (!trigger_join_service_client_.call(srv)) {
        ROS_ERROR("Failed to call join service");
        ros::Duration(0.5).sleep();
    }

    if (!ros::ok()) {
        ROS_ERROR("Program canceled");
        return false;
    }

    ROS_INFO("Will now join main task planner caller ...");
    return true;
}

/*******************************************************************************
 * @brief If the entrypoint of your node is not a periodic callback then you
 * may spin up a thread
 */
void RansacNode::threadStart(const std::string &tname)
{
    enable_detection = true;
    ROS_INFO("Thread %s is alive", tname.c_str());
}

/*******************************************************************************
 * @brief
 * https://github.com/bo-yang/terminate_cpp_thread/blob/master/kill_cpp_thread.cc
 */
void RansacNode::threadStop(const std::string &tname)
{
    enable_detection = false;
    ROS_INFO("Thread %s is rightfully dead", tname.c_str());
    // }
}

RansacNode::RansacNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh(nh), pnh(pnh), state_t0_(state_t0_init_), state_t1_(state_t1_init_),
      is_node_alive_(true), is_task_planner_notified_(false),
      has_functionality_been_delivered_successfully_(false),
      async_spinner_(1, &independent_service_queue_), it(nh)
{
    inlier_pub = nh.advertise<sensor_msgs::PointCloud2>("/inlier_points", 10);
    outlier_pub = nh.advertise<sensor_msgs::PointCloud2>("/outlier_points", 10);
    label_pub = nh.advertise<sensor_msgs::Image>("/label", 10);
    ball_pub = nh.advertise<visualization_msgs::Marker>("/ball", 10);
    str_pub = nh.advertise<std_msgs::String>("/damage_string", 10);
    vertical_point_pub = nh.advertise<sensor_msgs::PointCloud2>("/to_weld_vertical", 10);
    horizontal_point_pub = nh.advertise<sensor_msgs::PointCloud2>("/to_weld_horizontal", 10);
    // centerline_pub_vertical = nh.advertise<sensor_msgs::PointCloud2>("/centerline_vertical", 10);
    // centerline_pub_horizontal = nh.advertise<sensor_msgs::PointCloud2>("/centerline_horizontal", 10);

    // Line publishers
    line_pub = nh.advertise<visualization_msgs::Marker>("/line", 10);

    pointcloud_sub = nh.subscribe("/camera/depth/color/points", 1,
                                  &RansacNode::pointcloud_callback, this);

    camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1,
                                   &RansacNode::camera_info_callback, this);

    rgb_sub = nh.subscribe("/camera/color/image_raw", 1,
                           &RansacNode::rgb_callback, this);
    depth_sub = nh.subscribe("/camera/depth/image_rect_raw", 1,
                             &RansacNode::depth_callback, this);

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    // Task planner's requirements
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //
    // Call this service to start functionality
    enable_service_ =
        nh.advertiseService(ros::this_node::getName() + "/execution/enable",
                            &RansacNode::handleRequestEnable, this);

    // Call this service to pause functionality
    disable_pause_service_ = nh.advertiseService(
        ros::this_node::getName() + "/execution/disable/pause",
        &RansacNode::handleRequestDisablePause, this);

    // Call this service to stop functionality
    disable_stop_service_ =
        nh.advertiseService(ros::this_node::getName() + "/execution/disable/stop",
                            &RansacNode::handleRequestDisableStop, this);

    // The details of the shutdown service. It runs in a separate thread so that
    // it can override normal execution
    ros::AdvertiseServiceOptions shutdown_service_options =
        ros::AdvertiseServiceOptions::create<std_srvs::Trigger>(
            ros::this_node::getName() + "/execution/halt/shutdown_node",
            boost::bind(&RansacNode::handleRequestShutdown, this, _1, _2),
            ros::VoidPtr(), &independent_service_queue_);

    // The details of the kill service. It runs in a separate thread so that
    // it can override normal execution
    ros::AdvertiseServiceOptions kill_service_options =
        ros::AdvertiseServiceOptions::create<std_srvs::Trigger>(
            ros::this_node::getName() + "/execution/halt/kill",
            boost::bind(&RansacNode::handleRequestKill, this, _1, _2),
            ros::VoidPtr(), &independent_service_queue_);

    // Call this service to shutdown this node
    shutdown_service_ = nh.advertiseService(shutdown_service_options);

    // Call this service to kill the process that runs this node
    kill_service_ = nh.advertiseService(kill_service_options);

    // This node may now wait for service requests to its {shutdown,kill} service
    async_spinner_.start();

    // This service should be called by code in this node in order to signify
    // the end of execution of this node to the task planner that enabled it
    trigger_join_service_client_ = nh.serviceClient<std_srvs::SetBool>(
        ros::this_node::getName() + "/execution/join");

    // A timer that checks whether it's time to modify the state of the
    // functionality that this node provides
    clock_ = nh.createTimer(ros::Duration(1), &RansacNode::clockCallback, this);

    // ---------------------------------------------------------------------------
    // Does this node need to notify the task planner that its functionality
    // has been served? Or is this node running solo?
    if (!pnh.getParam("task_planner_notify_end", task_planner_notify_end_)) {
        ROS_WARN("No task_planner_notify_end param found; resorting to default");
        task_planner_notify_end_ = false;
    }

    // ---------------------------------------------------------------------------
    // Transitions between states
    // There are 5 available states in States_
    size_t w = 5;
    size_t h = w;

    for (unsigned int i = 0; i < w; i++) {
        std::vector<bool> tmp;
        for (unsigned int j = 0; j < h; j++)
            tmp.push_back(false);

        transition_table_.push_back(tmp);
    }

    // transition_table_[PLAY][PLAY]    = false;
    transition_table_[PLAY][PAUSE] = true;
    transition_table_[PLAY][STOP] = true;
    transition_table_[PLAY][SHUTDOWN] = true;
    transition_table_[PLAY][KILL] = true;
    //----------------------------------------------------
    transition_table_[PAUSE][PLAY] = true;
    // transition_table_[PAUSE][PAUSE]  = false;
    transition_table_[PAUSE][STOP] = true;
    transition_table_[PAUSE][SHUTDOWN] = true;
    transition_table_[PAUSE][KILL] = true;
    //-----------------------------------------------------
    transition_table_[STOP][PLAY] = true;
    // transition_table_[STOP][PAUSE]   = false;
    // transition_table_[STOP][STOP]    = false;
    transition_table_[STOP][SHUTDOWN] = true;
    transition_table_[STOP][KILL] = true;
    //-----------------------------------------------------
    // transition_table_[SHUTDOWN][*]   = false;
    //-----------------------------------------------------
    // transition_table_[KILL][*]       = false;
    //-----------------------------------------------------
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ransac_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    RansacNode rn(nh, nh_private);
    // callback_object = &rn;

    dynamic_reconfigure::Server<rebarsegmenation::Ransac_node_ParamsConfig>
        server;
    dynamic_reconfigure::Server<
        rebarsegmenation::Ransac_node_ParamsConfig>::CallbackType f;

    f = boost::bind(&RansacNode::dynamic_reconfigure_callback, &rn, _1, _2);
    server.setCallback(f);
    ros::spin();

    return 0;
}

// gdb
// /home/larde/Documents/GitHub/Rebar-segmentation-Ransac/devel/lib/rebarsegmenation/ransac_node
// run __name : = ransac_node __log : = / home / larde /.ros / log / b1301476 -
// d72a - 11ef - 920d - 350a44b5be26 / ransac_node - 3.log bt