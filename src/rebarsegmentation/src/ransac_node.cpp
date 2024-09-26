#include <pcl/sample_consensus/sac_model_plane.h>
#include <image_transport/image_transport.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/version.hpp>
#include <pcl/ModelCoefficients.h>
#include "std_msgs/msg/string.hpp"
#include <opencv2/core/types.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/ximgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"
#include <pcl/pcl_config.h>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <memory>

// Custom code
#include <rebarsegmentation/rebar_seg.h>

using std::placeholders::_1;

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

class RansacNode : public rclcpp::Node
{
public:
  RansacNode() : Node("ransac_node"), logger_(this->get_logger())
  {
    RCLCPP_INFO(this->get_logger(), "Initializing RansacNode...");
    RCLCPP_INFO(this->get_logger(), "PCL Version: %d", PCL_VERSION);
    RCLCPP_INFO(this->get_logger(), "OpenCV Version: %s", CV_VERSION);
    if (__cplusplus == 202101L)
      RCLCPP_INFO(this->get_logger(), "C++23");
    else if (__cplusplus == 202002L)
      RCLCPP_INFO(this->get_logger(), "C++20");
    else if (__cplusplus == 201703L)
      RCLCPP_INFO(this->get_logger(), "C++17");
    else if (__cplusplus == 201402L)
      RCLCPP_INFO(this->get_logger(), "C++14");
    else if (__cplusplus == 201103L)
      RCLCPP_INFO(this->get_logger(), "C++11");
    else if (__cplusplus == 199711L)
      RCLCPP_INFO(this->get_logger(), "C++98");
    else
      RCLCPP_INFO(this->get_logger(), "pre-standard C++: %ld", __cplusplus);

    // Declare parameters with default values
    this->declare_parameter<double>("required_confidence", 0.7);
    this->declare_parameter<double>("ransac_threshold", 0.02);
    this->declare_parameter<int>("min_cluster_size", 150);

    // Get parameter values
    this->get_parameter("required_confidence", required_confidence);
    this->get_parameter("ransac_threshold", ransac_threshold);
    this->get_parameter("min_cluster_size", min_cluster_size);

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    outlier_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("outlier_topic", 100);
    inlier_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("inlier_topic", 100);
    ball_pub = this->create_publisher<visualization_msgs::msg::Marker>("ball_topic", 100);
    label_pub = this->create_publisher<sensor_msgs::msg::Image>("label_topic", 100);
    str_pub = this->create_publisher<std_msgs::msg::String>("str_topic", 100);

    pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/camera/depth/color/points", qos, std::bind(&RansacNode::pointcloud_callback, this, _1));
    camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera/depth/camera_info", qos, std::bind(&RansacNode::camera_info_callback, this, std::placeholders::_1));
    // camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera/aligned_depth_to_color/camera_info", qos, std::bind(&RansacNode::camera_info_callback, this, std::placeholders::_1));

    rgb_image_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", qos, std::bind(&RansacNode::rgb_image_callback, this, _1));
    depth_image_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/depth/image_rect_raw", qos, std::bind(&RansacNode::depth_image_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "RansacNode initialized successfully.");
  }

private:
  // Logger
  rclcpp::Logger logger_; 

  // Publishers and subscribers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ball_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr outlier_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inlier_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr label_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub;

  // Frame history
  frame_AOI_info frames_history_vertical;
  frame_AOI_info frames_history_horizontal;
  
  // Camera parameters
  double fx;
  double fy;
  double cx;
  double cy;
  cv::Mat Km;
  cv::Mat K_inv;
  unsigned int img_height;
  unsigned int img_width;
  std_msgs::msg::Header img_header;

  // Distances
  double distance_from_rebar_to_background;
  double distance_from_camera_to_rebar;

  // Angles for the vertical and horizontal rebar
  std::pair<double, double> angles;

  // Images
  cv::Mat rgb_image;
  cv::Mat depth_image;
  cv::Mat depth_image_rgb;
  cv::Mat main_img;
  cv::Mat main_img_rgb;
  cv::Mat gray_orig;
  cv::Mat thinned_image;
  std::pair<cv::Mat, cv::Mat> resulting_split;
  cv::Mat vertical;
  cv::Mat horizontal;

  // Dynamic reconfigure variables
  double ransac_threshold;
  int min_cluster_size;
  double required_confidence;

  // Exit after counter
  int exit_counter = 0;

  // Frame counter
  int frame_count = 0;
  double fps = 0.0;
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

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

      RCLCPP_ERROR(this->get_logger(), "Inlier cloud is empty after adding non-kept points");
      return;
    }

    // Update outlier_cloud with kept_points
    outlier_cloud->points = kept_points;
    outlier_cloud->width = kept_points.size();
    outlier_cloud->height = 1; // Since it's a 1D point cloud
    outlier_cloud->is_dense = true;

    if (outlier_cloud->points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Outlier cloud is empty after updating with kept points");
      return;
    }

    // Convert point clouds back to sensor_msgs::PointCloud2
    // Create a new empty point cloud
    sensor_msgs::msg::PointCloud2 inlier_msg;
    pcl::toROSMsg(*inlier_cloud, inlier_msg);
    inlier_msg.header.frame_id = msg->header.frame_id;
    inlier_msg.header.stamp = rclcpp::Clock().now();
    inlier_pub->publish(inlier_msg);

    sensor_msgs::msg::PointCloud2 outlier_msg;
    pcl::toROSMsg(*outlier_cloud, outlier_msg);
    outlier_msg.header.frame_id = msg->header.frame_id;
    outlier_msg.header.stamp = rclcpp::Clock().now();
    outlier_pub->publish(outlier_msg);

    project_2D(outlier_cloud);
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Extract intrinsic camera parameters
    fx = msg->k[0]; // Focal length in x direction
    fy = msg->k[4]; // Focal length in y direction
    cx = msg->k[2]; // Optical center x coordinate
    cy = msg->k[5]; // Optical center y coordinate
    img_height = msg->height;
    img_width = msg->width;
    img_header = msg->header;

    // // print camera parameters
    // RCLCPP_INFO(this->get_logger(), "fx: %f", fx);
    // RCLCPP_INFO(this->get_logger(), "fy: %f", fy);
    // RCLCPP_INFO(this->get_logger(), "cx: %f", cx);
    // RCLCPP_INFO(this->get_logger(), "cy: %f", cy);
    // RCLCPP_INFO(this->get_logger(), "img_height: %d", img_height);
    // RCLCPP_INFO(this->get_logger(), "img_width: %d", img_width);

    // Calculate the camera matrix
    Km = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Calculate the inverse of the camera matrix
    K_inv = Km.inv();
  }

  void rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    rgb_image = cv_ptr->image;
  }

  void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    depth_image = cv_ptr->image;
    cv::cvtColor(depth_image, depth_image_rgb, cv::COLOR_GRAY2BGR);
  }
  
  void project_2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // Project points to image plane
    // Assuming pinhole camera parameters
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
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
      if (u >= 0 && u < static_cast<int>(image_msg->width) && v >= 0 && v < static_cast<int>(image_msg->height))
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
    cv::cvtColor(main_img, main_img_rgb, cv::COLOR_BGR2RGB);

    // Check if image is empty or not
    if (img.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Image is empty");
      return;
    }

    if (image_msg->data.size() != img_height * img_width * 3)
    {
      RCLCPP_ERROR(this->get_logger(), "data size: %lu (should be: %i)", image_msg->data.size(), img_height * img_width * 3);
    }
    // Publish the image message
    label_pub->publish(*image_msg);

    detect_AOI();
  }

  void detect_AOI()
  {
    //  Convert the image to grayscale
    cv::cvtColor(main_img, gray_orig, cv::COLOR_RGB2GRAY);

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

    angles = find_rotation(gray_orig);

    cv::Mat rotated_image = rotate_image("Image", gray_orig, angles.second);
    cv::Mat thresholded_image;
    cv::threshold(rotated_image, thresholded_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    resulting_split = split_horizontal_and_vertical(rotated_image, angles, 60);

    verticalProcess();
    horizontalProcess();

    cv::cvtColor(rotated_image, rotated_image, cv::COLOR_GRAY2BGR);

    for (const auto &aoi : frames_history_vertical.aoiList)
    {
      if (aoi.confidence >= required_confidence)
      {
        cv::putText(rotated_image, std::to_string(aoi.id) + "V", (aoi.bounding_box.first + cv::Point(-10, -10)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 0, 255), 1);
        cv::line(rotated_image, aoi.closest_pixels_pair.first, aoi.closest_pixels_pair.second, cv::Scalar(0, 0, 255), 2);
        cv::rectangle(rotated_image, aoi.bounding_box.first, aoi.bounding_box.second, cv::Scalar(0, 255, 0), 2);
        cv::putText(rotated_image, std::to_string(aoi.confidence), (aoi.bounding_box.first + cv::Point(25, +25)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
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

    cv::Mat back_rotated_image = rotate_image("Vertical - reverse rotation", rotated_image, -angles.second);

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

    publish_to_3d(frames_history_vertical);
    publish_to_3d(frames_history_horizontal);

    // exit_counter++;
    // if (exit_counter == 50)
    // {
    //   exit(0);
    // }
  }

  void verticalProcess()
  {
    cv::Mat pruned_vertical = resulting_split.first;
    frames_history_vertical.ns = "Vertical";
    detectInterruptions(frames_history_vertical, pruned_vertical, 70, 20);
    frames_history_vertical.calculateConfidence();
  }

  void horizontalProcess()
  {
    cv::Mat pruned_horizontal = resulting_split.second;
    frames_history_horizontal.ns = "Horizontal";
    detectInterruptions(frames_history_horizontal, pruned_horizontal, 70, 10);
    frames_history_horizontal.calculateConfidence();
  }

  void publish_to_3d(frame_AOI_info frame_history)
  {
    // deleteMarkers(ball_pub);
    for (size_t i = 0; i < frame_history.aoiList.size(); ++i)
    {
      if (frame_history.aoiList[i].confidence >= required_confidence)
      {
        std::vector<cv::Point3f> vertical_3d_coordinates;
        std::vector<cv::Point3f> horizontal_3d_coordinates;
        if (frame_history.ns == "Vertical")
        {
          double Z1 = find_depth(depth_image, frame_history.aoiList[i].points[2].x + 20, frame_history.aoiList[i].points[2].y) - distance_from_rebar_to_background;
          double Z2 = find_depth(depth_image, frame_history.aoiList[i].points[3].x + 20, frame_history.aoiList[i].points[3].y) - distance_from_rebar_to_background;

          cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].points[2].x + 20, frame_history.aoiList[i].points[2].y), 1, cv::Scalar(0, 255, 0), 2);
          cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].points[3].x + 20, frame_history.aoiList[i].points[3].y), 1, cv::Scalar(0, 255, 0), 2);

          // Also draw on the orignal image
          cv::circle(main_img_rgb, cv::Point(frame_history.aoiList[i].points[2].x, frame_history.aoiList[i].points[2].y), 1, cv::Scalar(0, 255, 0), 2);
          cv::circle(main_img_rgb, cv::Point(frame_history.aoiList[i].points[3].x, frame_history.aoiList[i].points[3].y), 1, cv::Scalar(0, 255, 0), 2);

          cv::Point3f point1 = pixel_to_camera(K_inv, frame_history.aoiList[i].points[2].x + 17, frame_history.aoiList[i].points[2].y, Z1);
          cv::Point3f point2 = pixel_to_camera(K_inv, frame_history.aoiList[i].points[3].x + 17, frame_history.aoiList[i].points[3].y, Z2);

          // Print the 3D coordinates of the rebar
          RCLCPP_INFO(this->get_logger(), "Vertical Rebar %d:\t\t (%f, %f, %f) and (%f, %f, %f)", frame_history.aoiList[i].id, point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);

          publish_ball(point1, 0.01, 0, frame_history.ns, ball_pub, {1, 0, 0, 1});
          publish_ball(point2, 0.01, 1, frame_history.ns, ball_pub, {1, 1, 0, 1});

          std::ostringstream damage_stream;

          damage_stream << std::fixed << std::setprecision(4)
                        << frame_history.aoiList[i].id << "V,"
                        << point1.x << "," << point1.y << "," << point1.z << ","
                        << point2.x << "," << point2.y << "," << point2.z;

          std::string damage = damage_stream.str();
          auto msg = std::make_shared<std_msgs::msg::String>();
          msg->data = damage;

          str_pub->publish(*msg);

          // double distance = std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
        }
        else if (frame_history.ns == "Horizontal")
        {
          double Z1 = find_depth(depth_image, frame_history.aoiList[i].points[0].x, frame_history.aoiList[i].points[0].y + 20) - distance_from_rebar_to_background;
          double Z2 = find_depth(depth_image, frame_history.aoiList[i].points[1].x, frame_history.aoiList[i].points[1].y + 20) - distance_from_rebar_to_background;

          cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].points[0].x, frame_history.aoiList[i].points[0].y + 20), 1, cv::Scalar(255, 0, 0), 2);
          cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].points[1].x, frame_history.aoiList[i].points[1].y + 20), 1, cv::Scalar(255, 0, 0), 2);

          // Also draw on the orignal image
          cv::circle(main_img_rgb, cv::Point(frame_history.aoiList[i].points[0].x, frame_history.aoiList[i].points[0].y), 1, cv::Scalar(255, 0, 0), 2);
          cv::circle(main_img_rgb, cv::Point(frame_history.aoiList[i].points[1].x, frame_history.aoiList[i].points[1].y), 1, cv::Scalar(255, 0, 0), 2);

          cv::Point3f point1 = pixel_to_camera(K_inv, frame_history.aoiList[i].points[0].x + 17, frame_history.aoiList[i].points[0].y, Z1);
          cv::Point3f point2 = pixel_to_camera(K_inv, frame_history.aoiList[i].points[1].x + 17, frame_history.aoiList[i].points[1].y, Z2);

          // Print the 3D coordinates of the rebar
          RCLCPP_INFO(this->get_logger(), "Horizontal Rebar %d:\t (%f, %f, %f) and (%f, %f, %f)", frame_history.aoiList[i].id, point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);

          publish_ball(point1, 0.01, 3, frame_history.ns, ball_pub, {1, 0, 1, 1});
          publish_ball(point2, 0.01, 4, frame_history.ns, ball_pub, {1, 0, 1, 1});

          std::ostringstream damage_stream;

          damage_stream << std::fixed << std::setprecision(4)
                        << frame_history.aoiList[i].id << "H,"
                        << point1.x << "," << point1.y << "," << point1.z << ","
                        << point2.x << "," << point2.y << "," << point2.z;

          std::string damage = damage_stream.str();
          auto msg = std::make_shared<std_msgs::msg::String>();
          msg->data = damage;

          str_pub->publish(*msg);

          // double distance = std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
        }
        cv::imshow("Depth_Image", depth_image_rgb);
        cv::imshow("RGB_Image", main_img_rgb);
        cv::waitKey(1);
        

      }
    }
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RansacNode>());

  rclcpp::shutdown();
  return 0;
}

