#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <thread>
#include <sstream>
#include <iomanip>
#include <chrono>

// Include dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <rebarsegmenation/Ransac_node_ParamsConfig.h>

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

// RobetArme service calls
#include <atomic>
#include <functional>
#include <unordered_map>
#include <memory>
#include <thread>
#include <sys/types.h>
#include <unistd.h>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"

#include <image_transport/image_transport.h>

bool aligned_depth = false;

double round(double var, int precision = 2)
{
    // if precision = 3 then
    // 37.66666 * 10^3 =37666.66
    // 37666.66 + .5 =37667.1    for rounding off value
    // then type cast to <int> so value is 37667
    // then divided by 10^3 so the value converted into 37.667
    if (precision < 0)
        precision = 0;
    double value = (var >= 0) ? (int)(var * pow(10, precision) + .5) : (int)(var * pow(10, precision) - .5);
    return value / pow(10, precision);
}

class RansacNode
{
public:
    RansacNode(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void dynamic_reconfigure_callback(rebarsegmenation::Ransac_node_ParamsConfig &config, uint32_t level);
    // void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &input);
    // void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg);
    // void rgb_callback(const sensor_msgs::ImageConstPtr &msg);
    // void depth_callback(const sensor_msgs::ImageConstPtr &msg);
    // void project_2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    // void detect_AOI();
    // void verticalProcess();
    // void horizontalProcess();
    // void publish_AOI_to_3d(frame_AOI_info frame_history);
    // void publish_centerline_to_3d(frame_AOI_info frame_history);

private:
    double fx;
    double fy;
    double cx;
    double cy;
    cv::Mat K;
    cv::Mat K_inv;

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
    ros::Publisher centerline_pub_vertical;
    ros::Publisher centerline_pub_horizontal;

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
    cv::Mat gray_orig;
    cv::Mat thinned_image;
    std::pair<cv::Mat, cv::Mat> resulting_split;
    cv::Mat vertical;
    cv::Mat horizontal;

    frame_AOI_info frames_history_vertical;
    frame_AOI_info frames_history_horizontal;

    // Dynamic reconfigure variables
    double ransac_threshold = 0.02;
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
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // void dynamic_reconfigure_callback(rebarsegmenation::Ransac_node_ParamsConfig &config, uint32_t level);
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &input);
    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg);
    void rgb_callback(const sensor_msgs::ImageConstPtr &msg);
    void depth_callback(const sensor_msgs::ImageConstPtr &msg);
    void project_2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void detect_AOI();
    void verticalProcess();
    void horizontalProcess();
    void publish_AOI_to_3d(frame_AOI_info frame_history);
    void publish_centerline_to_3d(frame_AOI_info frame_history);

    bool enable_detection = false;
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
    bool handleRequestDisablePause(
        std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool handleRequestDisableStop(
        std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool handleRequestEnable(
        std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool handleRequestShutdown(
        std_srvs::Trigger::Request &req,
        std_srvs::Trigger::Response &res);
    bool handleRequestKill(
        std_srvs::Trigger::Request &req,
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
    ransac_threshold = config.ransac_threshold / (double)1000;
    min_cluster_size = config.min_cluster_size;

    // Flags for showing images
    show_rotated_image = config.show_rotated_image;
    show_split_image = config.show_split_image;
    show_clusters = config.show_clusters;
    show_roi = config.show_roi;
    show_angles = config.show_angles;
    required_confidence = config.required_confidence / (double)HISTORY;
}

void RansacNode::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    if (!enable_detection || !camera_info_received)
    {
        return;
    }
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

    // Also calculate the average distance of the kept points to the plane

    std::vector<double> a{
        coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};
    double dot_product = 0;
    double total_distance_to_background = 0.0;
    double total_distance_to_camera = 0.0;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> kept_points;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> non_kept_points;
    for (size_t i = 0; i < outlier_cloud->points.size(); ++i)
    {
        pcl::PointXYZ point = outlier_cloud->points[i];
        std::vector<double> b{point.x, point.y, point.z, 1};
        dot_product = std::inner_product(std::begin(a), std::end(a), std::begin(b), 0.0);
        if (dot_product <= 0 && dot_product >= -0.2)
        {
            kept_points.push_back(point);
            double distance_to_background = std::abs(dot_product) / std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            total_distance_to_background += distance_to_background;

            double distance_to_camera = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            total_distance_to_camera += distance_to_camera;
        }
        else
        {
            non_kept_points.push_back(point);
        }
    }

    if (!kept_points.empty())
    {
        distance_from_rebar_to_background = total_distance_to_background / kept_points.size();
        distance_from_camera_to_rebar = total_distance_to_camera / kept_points.size();
    }

    // Add non-kept points to inlier_cloud
    inlier_cloud->points.insert(inlier_cloud->points.end(), non_kept_points.begin(), non_kept_points.end());
    inlier_cloud->width = inlier_cloud->points.size();
    inlier_cloud->height = 1; // Since it's a 1D point cloud
    inlier_cloud->is_dense = true;

    if (inlier_cloud->points.empty())
    {
        ROS_ERROR("Inlier cloud is empty after adding non-kept points");
        return;
    }

    // Update outlier_cloud with kept_points
    outlier_cloud->points = kept_points;
    outlier_cloud->width = kept_points.size();
    outlier_cloud->height = 1; // Since it's a 1D point cloud
    outlier_cloud->is_dense = true;

    if (outlier_cloud->points.empty())
    {
        ROS_ERROR("Outlier cloud is empty after updating with kept points");
        return;
    }

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

// // TODO: The out of memory error is caused by not receiving the camera info message first and thereby setting important variables
void RansacNode::camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    if (!enable_detection || camera_info_received)
    {
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
    if (!enable_detection || !camera_info_received)
    {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        rgb_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void RansacNode::depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    if (!enable_detection || !camera_info_received)
    {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_image = cv_ptr->image;
        cv::Mat depth_image_color;
        cv::Mat depth_image_shifted;
        if (depth_image.empty())
        {
            ROS_ERROR("Depth image is empty");
            return;
        }
        cv::cvtColor(depth_image, depth_image_color, cv::COLOR_GRAY2BGR);
        // Shift depth image to the right by 10 pixels
        cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, 18, 0, 1, 0);
        cv::warpAffine(depth_image, depth_image_shifted, M, depth_image.size());
        depth_image = depth_image_shifted;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void RansacNode::project_2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

    cv::Mat img(img_height, img_width, CV_8UC3, image_msg->data.data());

    main_img = img;

    // Check if image is empty or not
    if (img.empty())
    {
        ROS_ERROR("Image is empty");
        return;
    }
    detect_AOI();

    if (image_msg->data.size() != img_height * img_width * 3)
    {
        ROS_ERROR("data size: %lu (should be: %i)", image_msg->data.size(), img_height * img_width * 3);
    }
    // Publish the image message
    label_pub.publish(image_msg);
}

void RansacNode::detect_AOI()
{
    if (main_img.empty())
    {
        ROS_ERROR("Main image is empty");
        return;
    }

    if(depth_image.empty())
    {
        ROS_ERROR("Depth image is empty");
        return;
    }
    //  Convert the image to grayscale
    cv::cvtColor(main_img, gray_orig, cv::COLOR_RGB2GRAY);

    if (gray_orig.empty())
    {
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
    for (int i = 1; i < stats.rows; ++i)
    {
        if (stats.at<int>(i, cv::CC_STAT_AREA) < min_cluster_size)
        {
            cv::Mat mask = labels == i;
            gray_orig.setTo(0, mask);
        }
    }

    angles = find_rotation(gray_orig, show_angles);

    cv::Mat rotated_image = rotate_image("Image", gray_orig, angles.second, show_rotated_image);
    depth_image_rotated = rotate_image("Depth Image", depth_image, angles.second, show_rotated_image);

    // Check if the rotated image is empty
    if (rotated_image.empty())
    {
        ROS_ERROR("Rotated image is empty");
        return;
    }

    cv::cvtColor(depth_image_rotated, depth_image_rgb, cv::COLOR_GRAY2BGR);

    cv::Mat thresholded_image;
    cv::threshold(rotated_image, thresholded_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    resulting_split = split_horizontal_and_vertical(rotated_image, angles, 60, show_split_image);

    // resulting_split.first = rotate_image("Vertical", resulting_split.first, -angles.second, show_rotated_image);
    // resulting_split.second = rotate_image("Horizontal", resulting_split.second, -angles.second, show_rotated_image);

    // cv::threshold(resulting_split.first, resulting_split.first, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // cv::threshold(resulting_split.second, resulting_split.second, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    verticalProcess();
    horizontalProcess();

    // std::thread vertical_thread(&RansacNode::verticalProcess, this);
    // std::thread horizontal_thread(&RansacNode::horizontalProcess, this);

    // vertical_thread.join();
    // horizontal_thread.join();

    // Print and visualize the new or updated AOIs.
    // The id are saved in frames_history_vertical.nr_of_new_AOI

    cv::cvtColor(rotated_image, rotated_image, cv::COLOR_GRAY2BGR);

    for (const auto &aoi : frames_history_vertical.aoiList)
    {
        if (aoi.confidence >= required_confidence)
        {
            cv::putText(rotated_image, std::to_string(aoi.id) + "V", (aoi.bounding_box.first + cv::Point(-10, -10)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 0, 255), 2);
            cv::line(rotated_image, aoi.closest_pixels_pair.first, aoi.closest_pixels_pair.second, cv::Scalar(0, 0, 255), 2);
            cv::rectangle(rotated_image, aoi.bounding_box.first, aoi.bounding_box.second, cv::Scalar(0, 255, 0), 2);
            cv::putText(rotated_image, std::to_string(aoi.confidence), (aoi.bounding_box.first + cv::Point(25, +25)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
        }
    }
    for (const auto &aoi : frames_history_horizontal.aoiList)
    {

        if (aoi.confidence >= required_confidence)
        {
            cv::putText(rotated_image, std::to_string(aoi.id) + "H", (aoi.bounding_box.first + cv::Point(-15, -15)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 0, 255), 1);
            cv::line(rotated_image, aoi.closest_pixels_pair.first, aoi.closest_pixels_pair.second, cv::Scalar(255, 0, 0), 2);
            cv::rectangle(rotated_image, aoi.bounding_box.first, aoi.bounding_box.second, cv::Scalar(255, 255, 0), 2);
            cv::putText(rotated_image, std::to_string(aoi.confidence), (aoi.bounding_box.first + cv::Point(0, -5)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }
    }

    cv::Mat back_rotated_image = rotate_image("Vertical - reverse rotation", rotated_image, -angles.second, show_rotated_image);

    putText(back_rotated_image, "Min. confidence: " + std::to_string(required_confidence), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    frame_count++;
    if (frame_count == 1.0)
    {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        fps = 1.0 / elapsed.count(); // FPS is frames divided by time
        start = end;                 // Reset the start time
        frame_count = 0;             // Reset the frame counter
    }

    // Display FPS on the image using cv::putText
    std::ostringstream fps_stream;
    fps_stream << "FPS: " << std::fixed << std::setprecision(0) << fps;
    std::string fps_text = fps_stream.str();
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.5;
    int thickness = 1;
    cv::Point text_origin(10, 50); // Position of the text

    // Put the FPS text on the frame
    cv::putText(back_rotated_image, fps_text, text_origin, font_face, font_scale, cv::Scalar(0, 255, 0), thickness);

    cv::imshow("Projected_Image", back_rotated_image);
    cv::waitKey(1);

    // exit_counter++;
    // if (exit_counter == 50)
    // {
    //     exit(0);
    // }
}

void RansacNode::verticalProcess()
{
    cv::ximgproc::thinning(resulting_split.first, frames_history_vertical.skeleton);

    frames_history_vertical.ns = "Vertical";
    detectInterruptions(frames_history_vertical, resulting_split.first, 70, 20, show_roi, show_clusters);
    frames_history_vertical.calculateConfidence();
    publish_AOI_to_3d(frames_history_vertical);
    publish_centerline_to_3d(frames_history_vertical);
}

void RansacNode::horizontalProcess()
{
    cv::ximgproc::thinning(resulting_split.second, frames_history_horizontal.skeleton);

    frames_history_horizontal.ns = "Horizontal";
    detectInterruptions(frames_history_horizontal, resulting_split.second, 70, 10, show_roi, show_clusters);
    frames_history_horizontal.calculateConfidence();
    publish_AOI_to_3d(frames_history_horizontal);
    publish_centerline_to_3d(frames_history_horizontal);
}

void RansacNode::publish_AOI_to_3d(frame_AOI_info frame_history)
{
    // deleteMarkers(ball_pub);
    for (int k = 0; k < frame_history.clusters.size(); ++k)
    {
        std::vector<PointPair> pairs;

        // Calculate distances between all pairs of points
        for (size_t i = 0; i < frame_history.clusters[k].midpoints.size(); ++i)
        {
            for (size_t j = i + 1; j < frame_history.clusters[k].midpoints.size(); ++j)
            {
                double dist = euclideanDistance(frame_history.clusters[k].midpoints[i], frame_history.clusters[k].midpoints[j]);
                pairs.push_back(PointPair(frame_history.clusters[k].midpoints[i], frame_history.clusters[k].midpoints[j], dist));
            }
        }
        // Sort the pairs by distance
        std::sort(pairs.begin(), pairs.end());

        // The closest pair is the first in the sorted list
        const PointPair &closestPair = pairs.front();
        const PointPair &furthestPair = pairs.back();

        // Calculate closes point pair into 3d
        if (frame_history.ns == "Vertical")
        {
            double Z1 = find_depth(depth_image_rotated, closestPair.p1.x - 20, closestPair.p1.y) - distance_from_rebar_to_background;
            double Z2 = find_depth(depth_image_rotated, closestPair.p2.x + 20, closestPair.p2.y) - distance_from_rebar_to_background;
            double Z3 = find_depth(depth_image_rotated, furthestPair.p1.x - 20, furthestPair.p1.y) - distance_from_rebar_to_background;
            double Z4 = find_depth(depth_image_rotated, furthestPair.p2.x + 20, furthestPair.p2.y) - distance_from_rebar_to_background;

            cv::Point3f point1 = pixel_to_camera(K, closestPair.p1.x, closestPair.p1.y, Z1);
            cv::Point3f point2 = pixel_to_camera(K, closestPair.p2.x, closestPair.p2.y, Z2);
            cv::Point3f point3 = pixel_to_camera(K, furthestPair.p1.x, furthestPair.p1.y, Z3);
            cv::Point3f point4 = pixel_to_camera(K, furthestPair.p2.x, furthestPair.p2.y, Z4);

            double dist = std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
            std::cout << "Diameter: " << dist << std::endl;

            publish_ball(point3, 0.005, k, frame_history.ns + " - Endpoint3", ball_pub, {1, 0, 1, 1});
            publish_ball(point4, 0.005, k, frame_history.ns + " - Endpoint4", ball_pub, {1, 0, 1, 1});
        }
        else if (frame_history.ns == "Horizontal")
        {
            double Z1 = find_depth(depth_image_rotated, closestPair.p1.x, closestPair.p1.y - 20) - distance_from_rebar_to_background;
            double Z2 = find_depth(depth_image_rotated, closestPair.p2.x, closestPair.p2.y + 20) - distance_from_rebar_to_background;
            double Z3 = find_depth(depth_image_rotated, furthestPair.p1.x, furthestPair.p1.y - 20) - distance_from_rebar_to_background;
            double Z4 = find_depth(depth_image_rotated, furthestPair.p2.x, furthestPair.p2.y + 20) - distance_from_rebar_to_background;

            cv::Point3f point1 = pixel_to_camera(K, closestPair.p1.x, closestPair.p1.y, Z1);
            cv::Point3f point2 = pixel_to_camera(K, closestPair.p2.x, closestPair.p2.y, Z2);
            cv::Point3f point3 = pixel_to_camera(K, furthestPair.p1.x, furthestPair.p1.y, Z3);
            cv::Point3f point4 = pixel_to_camera(K, furthestPair.p2.x, furthestPair.p2.y, Z4);

            double dist = std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
            std::cout << "Diameter: " << dist << std::endl;

            publish_ball(point3, 0.005, k, frame_history.ns + " - Endpoint3", ball_pub, {1, 1, 0, 1});
            publish_ball(point4, 0.005, k, frame_history.ns + " - Endpoint4", ball_pub, {1, 1, 0, 1});
        }
    }
    for (size_t i = 0; i < frame_history.aoiList.size(); ++i)
    {

        if (frame_history.aoiList[i].confidence >= required_confidence)
        {

            std::vector<cv::Point3f> vertical_3d_coordinates;
            std::vector<cv::Point3f> horizontal_3d_coordinates;
            if (frame_history.ns == "Vertical")
            {

                double Z1 = find_depth(depth_image_rotated, frame_history.aoiList[i].closest_pixels_pair.first.x, frame_history.aoiList[i].closest_pixels_pair.first.y - 20);
                double Z2 = find_depth(depth_image_rotated, frame_history.aoiList[i].closest_pixels_pair.second.x, frame_history.aoiList[i].closest_pixels_pair.second.y + 20);

                cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].closest_pixels_pair.first.x, frame_history.aoiList[i].closest_pixels_pair.first.y - 20), 1, cv::Scalar(0, 255, 0), 2);
                cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].closest_pixels_pair.second.x, frame_history.aoiList[i].closest_pixels_pair.second.y + 20), 1, cv::Scalar(0, 255, 0), 2);

                cv::Point3f point1 = pixel_to_camera(K, frame_history.aoiList[i].closest_pixels_pair.first.x, frame_history.aoiList[i].closest_pixels_pair.first.y, Z1);
                cv::Point3f point2 = pixel_to_camera(K, frame_history.aoiList[i].closest_pixels_pair.second.x, frame_history.aoiList[i].closest_pixels_pair.second.y, Z2);

                publish_ball(point1, 0.005, 0, frame_history.ns, ball_pub, {1, 0, 0, 1});
                publish_ball(point2, 0.005, 1, frame_history.ns, ball_pub, {1, 1, 0, 0});

                std::ostringstream damage_stream;

                // Set the output to fixed-point notation and round to four decimal places
                damage_stream << std::fixed << std::setprecision(4)
                              << frame_history.aoiList[i].id << "V,"
                              << point1.x << "," << point1.y << "," << point1.z << ","
                              << point2.x << "," << point2.y << "," << point2.z;

                // Convert the stream to a string
                std::string damage = damage_stream.str();
                std_msgs::String msg;
                msg.data = damage;

                str_pub.publish(msg);

                double distance = std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
            }
            else if (frame_history.ns == "Horizontal")
            {

                double Z1 = find_depth(depth_image_rotated, frame_history.aoiList[i].closest_pixels_pair.first.x - 20, frame_history.aoiList[i].closest_pixels_pair.first.y);
                double Z2 = find_depth(depth_image_rotated, frame_history.aoiList[i].closest_pixels_pair.second.x + 20, frame_history.aoiList[i].closest_pixels_pair.second.y);

                cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].closest_pixels_pair.first.x - 20, frame_history.aoiList[i].closest_pixels_pair.first.y), 1, cv::Scalar(255, 0, 0), 2);
                cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].closest_pixels_pair.second.x + 20, frame_history.aoiList[i].closest_pixels_pair.second.y), 1, cv::Scalar(255, 0, 0), 2);

                cv::Point3f point1 = pixel_to_camera(K, frame_history.aoiList[i].closest_pixels_pair.first.x, frame_history.aoiList[i].closest_pixels_pair.first.y, Z1);
                cv::Point3f point2 = pixel_to_camera(K, frame_history.aoiList[i].closest_pixels_pair.second.x, frame_history.aoiList[i].closest_pixels_pair.second.y, Z2);

                publish_ball(point1, 0.005, 2, frame_history.ns, ball_pub, {1, 1, 0, 1});
                publish_ball(point2, 0.005, 3, frame_history.ns, ball_pub, {1, 0, 1, 1});

                // Create an ostringstream for building the string with rounded values
                std::ostringstream damage_stream;

                // Set the output to fixed-point notation and round to four decimal places
                damage_stream << std::fixed << std::setprecision(4)
                              << frame_history.aoiList[i].id << "H,"
                              << point1.x << "," << point1.y << "," << point1.z << ","
                              << point2.x << "," << point2.y << "," << point2.z;

                // Convert the stream to a string
                std::string damage = damage_stream.str();
                std_msgs::String msg;
                msg.data = damage;

                str_pub.publish(msg);

                // publish_string(frame_history.aoiList[i].id, point1, point2);

                double distance = std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
            }
            cv::imshow("Depth Image", depth_image_rgb);
            cv::waitKey(1);
        }
    }
}

void RansacNode::publish_centerline_to_3d(frame_AOI_info frame_history)
{

    // cv::imshow("Overlay", overlay);
    // cv::waitKey(1);

    int count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr centerline_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pcl_point;
    // lopp through all white pixels in the skeleton image and publish them as balls

    if (frame_history.ns == "Vertical")
    {
        for (int i = 0; i < frame_history.skeleton.rows; ++i)
        {
            for (int j = 0; j < frame_history.skeleton.cols; ++j)
            {
                if (frame_history.skeleton.at<uchar>(i, j) == 255)
                {
                    double Z = find_depth(depth_image_rotated, j, i);
                    cv::Point3f point = pixel_to_camera(K, j, i, Z);
                    // create a new pointcloud with the centerline points
                    pcl_point.x = point.x;
                    pcl_point.y = point.y;
                    pcl_point.z = point.z;
                    centerline_cloud->points.push_back(pcl_point);
                    centerline_cloud->height = 1;
                    centerline_cloud->width = centerline_cloud->points.size();
                    centerline_cloud->is_dense = true;

                    // count++;
                }
            }
        }
        // publish the pointcloud
        sensor_msgs::PointCloud2 centerline_msg;
        pcl::toROSMsg(*centerline_cloud, centerline_msg);
        centerline_msg.header.frame_id = "camera_color_optical_frame";
        centerline_msg.header.stamp = ros::Time::now();
        centerline_pub_vertical.publish(centerline_msg);
    }
    else if (frame_history.ns == "Horizontal")
    {
        for (int i = 0; i < frame_history.skeleton.rows; ++i)
        {
            for (int j = 0; j < frame_history.skeleton.cols; ++j)
            {
                if (frame_history.skeleton.at<uchar>(i, j) == 255)
                {
                    double Z = find_depth(depth_image_rotated, j, i);
                    cv::Point3f point = pixel_to_camera(K, j, i, Z);
                    // create a new pointcloud with the centerline points
                    pcl_point.x = point.x;
                    pcl_point.y = point.y;
                    pcl_point.z = point.z;
                    centerline_cloud->points.push_back(pcl_point);
                    centerline_cloud->height = 1;
                    centerline_cloud->width = centerline_cloud->points.size();
                    centerline_cloud->is_dense = true;

                    // count++;
                }
            }
        }
        // publish the pointcloud
        sensor_msgs::PointCloud2 centerline_msg;
        pcl::toROSMsg(*centerline_cloud, centerline_msg);
        centerline_msg.header.frame_id = "camera_color_optical_frame";
        centerline_msg.header.stamp = ros::Time::now();
        centerline_pub_horizontal.publish(centerline_msg);
    }
}

/*******************************************************************************
 * @brief Executes periodically.
 */
void RansacNode::clockCallback(const ros::TimerEvent &te)
{
    // ---------------------------------------------------------------------------
    if (state_t0_.load() != state_t1_.load())
    {
        // Transition to state if transition is allowed
        if (transition_table_[state_t0_.load()][state_t1_.load()] == true)
        {
            ROS_INFO("EXECUTING TRANSITION: %d -> %d",
                     state_t0_.load(), state_t1_.load());

            // Store new state to old
            state_t0_.store(state_t1_.load());

            // -------------------------------------
            if (state_t1_.load() == PLAY)
            {
                ROS_INFO("Triggering execution PLAY");
                threadStart("shotcreting_thread");
            }
            // -------------------------------------
            if (state_t1_.load() == PAUSE)
            {
                ROS_INFO("Triggering execution PAUSE");
                threadStop("shotcreting_thread");
            }
            // -------------------------------------
            if (state_t1_.load() == STOP)
            {
                ROS_INFO("Triggering execution STOP");
                threadStop("shotcreting_thread");

                // Notify that welding has ended unsuccessfully first (otherwise the
                // task planner that called this package cannot join it after it was
                // stopped, which means that it will block)
                if (task_planner_notify_end_ == true)
                    taskPlannerNotifyEnd();
            }
            // -------------------------------------
            if (state_t1_.load() == SHUTDOWN)
            {
                // Handled in own service callback. Otherwise functions called in this
                // scope might block SHUTDOWN

                // Notify that welding has ended unsuccessfully first (otherwise the
                // task planner that called this package cannot join it after it was
                // stopped, which means that it will block)
                if (task_planner_notify_end_ == true)
                    is_task_planner_notified_ = taskPlannerNotifyEnd();
            }
            // -------------------------------------
            if (state_t1_.load() == KILL)
            {
                // Handled in own service callback. Otherwise functions called in this
                // scope might block KILL

                // Notify that welding has ended unsuccessfully first (otherwise the
                // task planner that called this package cannot join it after it was
                // stopped, which means that it will block)
                if (task_planner_notify_end_ == true)
                    is_task_planner_notified_ = taskPlannerNotifyEnd();
            }
        }
        else
        {
            ROS_INFO("TRANSITION: %d -> %d NOT ALLOWED",
                     state_t0_.load(), state_t1_.load());

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
    if (task_planner_notify_end_ == true)
    {
        while (is_task_planner_notified_ == false)
            sleep(0.1);
    }

    ROS_INFO("Triggering node SHUTDOWN");
    ros::shutdown();
}

/*******************************************************************************
 * @brief The final act: calls system("kill self pid")
 */
void RansacNode::commitProcessSuicide()
{
    if (task_planner_notify_end_ == true)
    {
        while (is_task_planner_notified_ == false)
            sleep(0.1);
    }

    ROS_INFO("Triggering node KILL");

    pid_t pid = getpid();
    std::string s = "kill -9 " + std::to_string(pid);
    int ret = system(s.c_str());
}

/*******************************************************************************
 * @brief Wraps the functionality that this package provides for use with
 * a task planner
 */
// void RansacNode::entrypointWrapper()
// {
//     executeWelding();

//     // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//     // The functionality has been delivered successfully, or maybe it hasn't
//     has_functionality_been_delivered_successfully_ = true;

//     // Execution has ended; return to initial state
//     state_t1_.store(state_t1_init_);
//     // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// }

/*******************************************************************************
 * @brief Welding: the functionality that the package offers
 */
// void RansacNode::executeWelding()
// {
//     ROS_INFO("STARTING TO WELD ...");

//     // It will take 5 sec to complete welding
//     ros::Time deadline = ros::Time::now() + ros::Duration(5);

//     while (ros::Time::now() < deadline)
//     {
//         ROS_INFO("Welding ... (Could be publishing feedback now)");

//         sleep(1);
//     }

//     ROS_INFO("DONE welding.");
// }

/*******************************************************************************
 * @brief Service server to pause the functionality that this node offers
 */
bool RansacNode::handleRequestDisablePause(
    std_srvs::Trigger::Request &req,
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
bool RansacNode::handleRequestDisableStop(
    std_srvs::Trigger::Request &req,
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
bool RansacNode::handleRequestEnable(
    std_srvs::Trigger::Request &req,
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
bool RansacNode::handleRequestKill(
    std_srvs::Trigger::Request &req,
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
bool RansacNode::handleRequestShutdown(
    std_srvs::Trigger::Request &req,
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
        trigger_join_service_client_.getService(), ros::Duration(1.0)))
    {
        if (!ros::ok())
        {
            ROS_ERROR("Client of service %s interrupted while waiting for service to appear",
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
    while (!trigger_join_service_client_.call(srv))
    {
        ROS_ERROR("Failed to call join service");
        ros::Duration(0.5).sleep();
    }

    if (!ros::ok())
    {
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
    // std::thread t = std::thread(&RansacNode::entrypointWrapper, this);
    // pthread_map_[tname] = t.native_handle();
    // t.detach();
    enable_detection = true;
    ROS_INFO("Thread %s is alive", tname.c_str());
}

/*******************************************************************************
 * @brief https://github.com/bo-yang/terminate_cpp_thread/blob/master/kill_cpp_thread.cc
 */
void RansacNode::threadStop(const std::string &tname)
{
    // std::unordered_map<std::string, pthread_t>::const_iterator it =
    //     pthread_map_.find(tname);

    // if (it != pthread_map_.end())
    // {
    //     pthread_cancel(it->second);
    //     pthread_map_.erase(tname);
        enable_detection = false;
        ROS_INFO("Thread %s is rightfully dead", tname.c_str());
    // }
}

RansacNode::RansacNode(ros::NodeHandle nh, ros::NodeHandle pnh) : 
    nh(nh),
    pnh(pnh),
    state_t0_(state_t0_init_),
    state_t1_(state_t1_init_),
    is_node_alive_(true),
    is_task_planner_notified_(false),
    has_functionality_been_delivered_successfully_(false),
    async_spinner_(1, &independent_service_queue_),
    it(nh)
{
    inlier_pub = nh.advertise<sensor_msgs::PointCloud2>("/inlier_points", 10);
    outlier_pub = nh.advertise<sensor_msgs::PointCloud2>("/outlier_points", 10);
    label_pub = nh.advertise<sensor_msgs::Image>("/label", 10);
    ball_pub = nh.advertise<visualization_msgs::Marker>("/ball", 10);
    str_pub = nh.advertise<std_msgs::String>("/damage_string", 10);
    centerline_pub_vertical = nh.advertise<sensor_msgs::PointCloud2>("/centerline_vertical", 10);
    centerline_pub_horizontal = nh.advertise<sensor_msgs::PointCloud2>("/centerline_horizontal", 10);

    pointcloud_sub = nh.subscribe("/camera/depth/color/points", 1, &RansacNode::pointcloud_callback, this);

    camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1, &RansacNode::camera_info_callback, this);

    rgb_sub = nh.subscribe("/camera/color/image_raw", 1, &RansacNode::rgb_callback, this);
    depth_sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &RansacNode::depth_callback, this);

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    // Task planner's requirements
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //
    // Call this service to start functionality
    enable_service_ =
        nh.advertiseService(ros::this_node::getName() + "/execution/enable",
                             &RansacNode::handleRequestEnable, this);

    // Call this service to pause functionality
    disable_pause_service_ =
        nh.advertiseService(ros::this_node::getName() + "/execution/disable/pause",
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
            ros::VoidPtr(),
            &independent_service_queue_);

    // The details of the kill service. It runs in a separate thread so that
    // it can override normal execution
    ros::AdvertiseServiceOptions kill_service_options =
        ros::AdvertiseServiceOptions::create<std_srvs::Trigger>(
            ros::this_node::getName() + "/execution/halt/kill",
            boost::bind(&RansacNode::handleRequestKill, this, _1, _2),
            ros::VoidPtr(),
            &independent_service_queue_);

    // Call this service to shutdown this node
    shutdown_service_ = nh.advertiseService(shutdown_service_options);

    // Call this service to kill the process that runs this node
    kill_service_ = nh.advertiseService(kill_service_options);

    // This node may now wait for service requests to its {shutdown,kill} service
    async_spinner_.start();

    // This service should be called by code in this node in order to signify
    // the end of execution of this node to the task planner that enabled it
    trigger_join_service_client_ =
        nh.serviceClient<std_srvs::SetBool>(
            ros::this_node::getName() + "/execution/join");

    // A timer that checks whether it's time to modify the state of the
    // functionality that this node provides
    clock_ = nh.createTimer(ros::Duration(1), &RansacNode::clockCallback, this);

    // ---------------------------------------------------------------------------
    // Does this node need to notify the task planner that its functionality
    // has been served? Or is this node running solo?
    if (!pnh.getParam("task_planner_notify_end", task_planner_notify_end_))
    {
        ROS_WARN("No task_planner_notify_end param found; resorting to default");
        task_planner_notify_end_ = false;
    }

    // ---------------------------------------------------------------------------
    // Transitions between states
    // There are 5 available states in States_
    size_t w = 5;
    size_t h = w;

    for (unsigned int i = 0; i < w; i++)
    {
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
    // ros::master::V_TopicInfo master_topics;
    // ros::master::getTopics(master_topics);
    // for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
    // {
    //     const ros::master::TopicInfo &info = *it;
    //     if (info.name == "/camera/aligned_depth_to_color/camera_info")
    //     {
    //         ROS_INFO("Aligned depth to color camera info topic found");
    //         aligned_depth = true;
    //         break;
    //     }
    // }
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    RansacNode rn(nh, nh_private);
    // callback_object = &rn;

    dynamic_reconfigure::Server<rebarsegmenation::Ransac_node_ParamsConfig> server;
    dynamic_reconfigure::Server<rebarsegmenation::Ransac_node_ParamsConfig>::CallbackType f;

    f = boost::bind(&RansacNode::dynamic_reconfigure_callback, &rn, _1, _2);
    server.setCallback(f);
    ros::spin();

    return 0;
}

// gdb /home/larde/Documents/GitHub/Rebar-segmentation-Ransac/devel/lib/rebarsegmenation/ransac_node
// run __name : = ransac_node __log : = / home / larde /.ros / log / b1301476 - d72a - 11ef - 920d - 350a44b5be26 / ransac_node - 3.log
// bt