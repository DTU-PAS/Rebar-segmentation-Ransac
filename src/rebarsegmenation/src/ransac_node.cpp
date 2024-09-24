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
    RansacNode() : nh()
    {
        std::cout << "PCL Version: " << PCL_VERSION << std::endl;
        std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
        if (__cplusplus == 202101L)
            std::cout << "C++23";
        else if (__cplusplus == 202002L)
            std::cout << "C++20";
        else if (__cplusplus == 201703L)
            std::cout << "C++17";
        else if (__cplusplus == 201402L)
            std::cout << "C++14";
        else if (__cplusplus == 201103L)
            std::cout << "C++11";
        else if (__cplusplus == 199711L)
            std::cout << "C++98";
        else
            std::cout << "pre-standard C++." << __cplusplus;
        std::cout << "\n";
        inlier_pub = nh.advertise<sensor_msgs::PointCloud2>("/inlier_points", 1);
        outlier_pub = nh.advertise<sensor_msgs::PointCloud2>("/outlier_points", 1);
        label_pub = nh.advertise<sensor_msgs::Image>("/label", 1);
        ball_pub = nh.advertise<visualization_msgs::Marker>("/ball", 1);
        str_pub = nh.advertise<std_msgs::String>("/damage_string", 1000);

        pointcloud_sub = nh.subscribe("/camera/depth/color/points", 1, &RansacNode::pointcloud_callback, this);
        std::string camera_info_topic;
        if (aligned_depth)
        {
            camera_info_topic = "/camera/aligned_depth_to_color/camera_info";
            // camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &RansacNode::camera_info_callback, this);
        }
        else
        {
            camera_info_topic = "/camera/depth/camera_info";
            // camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1, &RansacNode::camera_info_callback, this);
        }
        // camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1, &RansacNode::camera_info_callback, this);
        // camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info", 1, &RansacNode::camera_info_callback, this);
        camera_info_topic = "/camera/depth/camera_info";
        ROS_INFO("Camera info topic: %s", camera_info_topic.c_str());
        camera_info_sub = nh.subscribe(camera_info_topic, 1, &RansacNode::camera_info_callback, this);

        rgb_sub = nh.subscribe("/camera/color/image_raw", 1, &RansacNode::rgb_callback, this);
        depth_sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &RansacNode::depth_callback, this);

        // pointcloud_sub = nh.subscribe("/stereo/points2", 1, &RansacNode::pointcloud_callback, this);
        // // camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1, &RansacNode::camera_info_callback, this);
        // camera_info_sub = nh.subscribe("/stereo/left/camera_info", 1, &RansacNode::camera_info_callback, this);
        // rgb_sub = nh.subscribe("/stereo/left/image_rect_color", 1, &RansacNode::rgb_callback, this);
        // depth_sub = nh.subscribe("/stereo/depth", 1, &RansacNode::depth_callback, this);
    }

    void dynamic_reconfigure_callback(rebarsegmenation::Ransac_node_ParamsConfig &config, uint32_t level)
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

    // TODO: The out of memory error is caused by not receiving the camera info message first and thereby setting important variables
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
    }

    void rgb_callback(const sensor_msgs::ImageConstPtr &rgb_msg)
    {
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

    void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_image = cv_ptr->image;
            cv::cvtColor(depth_image, depth_image_rgb, cv::COLOR_GRAY2BGR);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
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

        angles = find_rotation(gray_orig, show_angles);

        cv::Mat rotated_image = rotate_image("Image", gray_orig, angles.second, show_rotated_image);
        cv::Mat thresholded_image;
        cv::threshold(rotated_image, thresholded_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        resulting_split = split_horizontal_and_vertical(rotated_image, angles, 60, show_split_image);

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
            start = end;                  // Reset the start time
            frame_count = 0;              // Reset the frame counter
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

        exit_counter++;
        if (exit_counter == 50)
        {
            exit(0);
        }
    }

    void verticalProcess()
    {
        cv::Mat pruned_vertical = resulting_split.first;
        frames_history_vertical.ns = "Vertical";
        detectInterruptions(frames_history_vertical, pruned_vertical, 70, 20, show_roi, show_clusters);
        frames_history_vertical.calculateConfidence();
    }

    void horizontalProcess()
    {
        cv::Mat pruned_horizontal = resulting_split.second;
        frames_history_horizontal.ns = "Horizontal";
        detectInterruptions(frames_history_horizontal, pruned_horizontal, 70, 10, show_roi, show_clusters);
        frames_history_horizontal.calculateConfidence();
    }

    void publish_to_3d(frame_AOI_info frame_history)
    {
        // deleteMarkers(ball_pub);
        for (size_t i = 0; i < frame_history.aoiList.size(); ++i)
        {
            if (frame_history.aoiList[i].confidence >= required_confidence)
            {
                cv::Mat K;

                K = (cv::Mat_<double>(3, 3) << 431.6277160644531, 0.0, 428.92486572265625, 0.0, 431.6277160644531, 233.90313720703125, 0.0, 0.0, 1.0);

                std::vector<cv::Point3f> vertical_3d_coordinates;
                std::vector<cv::Point3f> horizontal_3d_coordinates;
                if (frame_history.ns == "Vertical")
                {
                    // for (size_t j = 0; j < frame_history.aoiList[i].points.size(); ++j)
                    // {
                    //     cv::circle(depth_image_rgb, frame_history.aoiList[i].points[j] - cv::Point(15, 0), 1, cv::Scalar(255, 0, 0), 3);
                    // }
                    double Z1 = find_depth(depth_image, frame_history.aoiList[i].points[2].x, frame_history.aoiList[i].points[2].y) - distance_from_rebar_to_background;
                    double Z2 = find_depth(depth_image, frame_history.aoiList[i].points[3].x, frame_history.aoiList[i].points[3].y) - distance_from_rebar_to_background;
                    
                    cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].points[2].x, frame_history.aoiList[i].points[2].y), 1, cv::Scalar(0, 255, 0), 2);
                    cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].points[3].x, frame_history.aoiList[i].points[3].y), 1, cv::Scalar(0, 255, 0), 2);

                    cv::Point3f point1 = pixel_to_camera(K, frame_history.aoiList[i].points[2].x, frame_history.aoiList[i].points[2].y, Z1);
                    cv::Point3f point2 = pixel_to_camera(K, frame_history.aoiList[i].points[3].x, frame_history.aoiList[i].points[3].y, Z2);

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
                    double Z1 = find_depth(depth_image, frame_history.aoiList[i].points[0].x, frame_history.aoiList[i].points[0].y + 20) - distance_from_rebar_to_background;
                    double Z2 = find_depth(depth_image, frame_history.aoiList[i].points[1].x, frame_history.aoiList[i].points[1].y + 20) - distance_from_rebar_to_background;

                    cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].points[0].x, frame_history.aoiList[i].points[0].y + 20), 1, cv::Scalar(255, 0, 0), 2);
                    cv::circle(depth_image_rgb, cv::Point(frame_history.aoiList[i].points[1].x, frame_history.aoiList[i].points[1].y + 20), 1, cv::Scalar(255, 0, 0), 2);

                    cv::Point3f point1 = pixel_to_camera(K, frame_history.aoiList[i].points[0].x, frame_history.aoiList[i].points[0].y, Z1);
                    cv::Point3f point2 = pixel_to_camera(K, frame_history.aoiList[i].points[1].x, frame_history.aoiList[i].points[1].y, Z2);

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
            }
        }
    }

private:
    double fx;
    double fy;
    double cx;
    double cy;
    cv::Mat Km;
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

    // Subscribers
    ros::Subscriber pointcloud_sub;
    ros::Subscriber camera_info_sub;
    ros::Subscriber rgb_sub;
    ros::Subscriber depth_sub;
    ros::NodeHandle nh;

    std::pair<double, double> angles;

    cv::Mat rgb_image;
    cv::Mat depth_image;
    cv::Mat depth_image_rgb;
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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ransac_node");
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
    {
        const ros::master::TopicInfo &info = *it;
        if (info.name == "/camera/aligned_depth_to_color/camera_info")
        {
            ROS_INFO("Aligned depth to color camera info topic found");
            aligned_depth = true;
            break;
        }
    }
    RansacNode rn;
    // callback_object = &rn;

    // If topic aligned_depth_to_color/camera_info is available, use it set flag aligned_depth to true

    dynamic_reconfigure::Server<rebarsegmenation::Ransac_node_ParamsConfig> server;
    dynamic_reconfigure::Server<rebarsegmenation::Ransac_node_ParamsConfig>::CallbackType f;

    f = boost::bind(&RansacNode::dynamic_reconfigure_callback, &rn, _1, _2);
    server.setCallback(f);

    ros::spin();


    return 0;
}
