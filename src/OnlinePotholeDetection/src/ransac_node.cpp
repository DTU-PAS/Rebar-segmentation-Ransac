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
#include <AOITracker.h>

bool aligned_depth = false;

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
        // rgb_sub = nh.subscribe("/camera/color/image_raw", 1, &RansacNode::rgb_callback, this);
        // depth_sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &RansacNode::depth_callback, this);

        // tracker.initialize(5);
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
            cv::Mat depth_image = cv_ptr->image;
            // ROS_INFO("Depth image: %i, %i", depth_image.rows, depth_image.cols);
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

        cv::Mat img(img_height, img_width, CV_8UC3, image_msg->data.data());

        main_img = img;

        detect_AOI();

        if (image_msg->data.size() != img_height * img_width * 3)
            ROS_ERROR("data size: %lu (should be: %i)", image_msg->data.size(), img_height * img_width * 3);
        // Publish the image message
        label_pub.publish(image_msg);
    }

    void detect_AOI()
    {
        cv::Mat img = main_img.clone();
        if (show_orig_image)
        {
            cv::imshow("Original Ransac Image", img);
            cv::waitKey(1);
        }

        cv::Mat gray_orig;
        //  Convert the image to grayscale
        cv::cvtColor(img, gray_orig, cv::COLOR_RGB2GRAY);

        if (!aligned_depth)
        {
            // Dialate image to remove small holes
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::dilate(gray_orig, gray_orig, kernel);

            // Erode image to remove small blobs
            cv::erode(gray_orig, gray_orig, kernel);
        }

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
                img_small_blobs_removed = img.setTo(cv::Scalar(0, 0, 0), mask);
            }
        }
        float vertical_angle, horizontal_angle;

        std::pair<double, double> angles = find_rotation(gray_orig, show_angles);

        cv::Mat rotated_image = rotate_image("Image", gray_orig, angles.second, show_rotated_image);
        cv::Mat thresholded_image;
        cv::threshold(rotated_image, thresholded_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // Apply opencv ximgproc thinning algorithm
        cv::Mat thinned_image;
        cv::ximgproc::thinning(rotated_image, thinned_image);

        // Call the split_horizontal_and_vertical function
        auto resulting_split = split_horizontal_and_vertical(thinned_image, 10, show_split_image);

        // Extract the results
        cv::Mat pruned_vertical = resulting_split.first;
        cv::Mat pruned_horizontal = resulting_split.second;

        cv::Mat reconstructed_vertical_skeleton = reconstruct_skeleton("Vertical", pruned_vertical, thinned_image, 3, 10, show_reconstructed_image);
        cv::Mat reconstructed_horizontal_skeleton = reconstruct_skeleton("Horizontal", pruned_horizontal, thinned_image, 3, 10, show_reconstructed_image);

        cv::Mat clean_vertical = remove_small_blobs("Vertical", reconstructed_vertical_skeleton, 70, show_image_without_blobs);
        cv::Mat clean_horizontal = remove_small_blobs("Horizontal", reconstructed_horizontal_skeleton, 70, show_image_without_blobs);

        cv::Mat back_rotated_image_vertical = rotate_image("Vertical - reverse rotation", clean_vertical, -angles.second, show_rotated_image);
        cv::Mat thresholded_image_vertical;
        cv::threshold(back_rotated_image_vertical, thresholded_image_vertical, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::Mat vertical;
        cv::ximgproc::thinning(thresholded_image_vertical, vertical);

        cv::Mat back_rotated_image_horizontal = rotate_image("Horizontal - reverse rotation", clean_horizontal, -angles.second, show_rotated_image);
        cv::Mat thresholded_image_horizontal;
        cv::threshold(back_rotated_image_horizontal, thresholded_image_horizontal, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::Mat horizontal;
        cv::ximgproc::thinning(thresholded_image_horizontal, horizontal);

        cluster_info result_clustering_vertical = cluster("Vertical", vertical, show_clusters);
        cluster_info result_clustering_horizontal = cluster("Horizontal", horizontal, show_clusters);

        cv::Mat vertical_labels = result_clustering_vertical.labels;
        cv::Mat horizontal_labels = result_clustering_horizontal.labels;

        int num_labels_vertical = result_clustering_vertical.num_clusters;
        int num_labels_horizontal = result_clustering_horizontal.num_clusters;

        auto result_vertical = find_area_of_interest("Vertical", vertical_labels, num_labels_vertical, gray_orig, img, show_roi);
        auto result_horizontal = find_area_of_interest("Horizontal", horizontal_labels, num_labels_horizontal, gray_orig, img, show_roi);

        // for (size_t i = 0; i < result_vertical.aoiList.size(); ++i)
        // {
        //     const auto &pixels = result_vertical.aoiList[i].closest_pixels_pair;
        //     cv::line(img, pixels.first, pixels.second, cv::Scalar(0, 0, 255), 2);
        //     cv::rectangle(img, result_vertical.aoiList[i].bounding_box.first, result_vertical.aoiList[i].bounding_box.second, cv::Scalar(0, 255, 0), 2);
        //     cv::putText(img, std::to_string(result_vertical.aoiList[i].id), cv::Point(result_vertical.aoiList[i].bounding_box.first.x - 10, result_vertical.aoiList[i].bounding_box.first.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(156, 0, 255), 2);
        // }
        // for (size_t i = 0; i < result_horizontal.aoiList.size(); ++i)
        // {
        //     const auto &pixels = result_horizontal.aoiList[i].closest_pixels_pair;
        //     cv::line(img, pixels.first, pixels.second, cv::Scalar(255, 0, 0), 2);
        //     cv::rectangle(img, result_horizontal.aoiList[i].bounding_box.first, result_horizontal.aoiList[i].bounding_box.second, cv::Scalar(0, 255, 255), 2);
        //     cv::putText(img, std::to_string(result_horizontal.aoiList[i].id), cv::Point(result_horizontal.aoiList[i].bounding_box.first.x, result_horizontal.aoiList[i].bounding_box.first.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 165), 2);
        // }

        // tracker.addFrame(result_vertical);
        // tracker.addFrame(result_horizontal);

        calculate_confidence("vertical", result_vertical, frames_vertical);
        calculate_confidence("horizontal", result_horizontal, frames_horizontal);

        // cv::imshow("Projected_Image", img);
        // cv::waitKey(0);
    }

    void calculate_confidence(const std::string &name, frame_AOI_info frame, std::vector<frame_AOI_info> &frames)
    {
        int N = 10;                // Number of frames to consider for confidence score
        float IoUThreshold = 0.8f; // IoU threshold for matching

        // Simulate adding frames with AOI information

        if (frames.size() == N)
        {
            frames.erase(frames.begin());
        }
        frames.push_back(frame);

        // Match AOIs and compute confidence scores

        matchAOIsAndComputeConfidence(name, frames, N, IoUThreshold, aoiHistories);

        // for (const auto &frame : frames)
        // {
        drawAOIs(img_small_blobs_removed, frames[frames.size() -1], aoiHistories, N);
        // }

        cv::imshow("AOIs with Confidence and ID", img_small_blobs_removed);
        cv::waitKey(0);
    }

    void dynamic_reconfigure_callback(OnlinePotholeDetection::Ransac_node_ParamsConfig &config, uint32_t level)
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
    ros::Subscriber rgb_sub;
    ros::Subscriber depth_sub;
    ros::NodeHandle nh;

    cv::Mat rgb_image;
    cv::Mat depth_image;
    cv::Mat main_img;
    cv::Mat img_small_blobs_removed;

    // AOITracker tracker;

    std::vector<frame_AOI_info> frames_vertical;
    std::vector<frame_AOI_info> frames_horizontal;
    std::unordered_map<int, AOIHistory> aoiHistories;

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
//     callback_object -> dynamic_reconfigure_callback();
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ransac_points_node");
    RansacNode rn;
    // callback_object = &rn;

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    // If topic aligned_depth_to_color/camera_info is available, use it set flag aligned_depth to true
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

    // // Simulate adding frames with AOIs
    // frame_AOI_info frame1 = {{{cv::Point(10, 10), cv::Point(50, 50)}}, {{cv::Point(10, 10), cv::Point(50, 50)}, {cv::Point(60, 60), cv::Point(100, 100)}}};
    // frame_AOI_info frame2 = {{{cv::Point(12, 12), cv::Point(52, 52)}}, {{cv::Point(12, 12), cv::Point(52, 52)}, {cv::Point(62, 62), cv::Point(102, 102)}}};

    // tracker.addFrame(frame1);
    // tracker.addFrame(frame2);

    // // Get and print tracked AOIs for the current frame
    // auto trackedAOIs = tracker.getCurrentTrackedAOIs();
    // for (const auto &aoi : trackedAOIs)
    // {
    //     std::cout << "Tracked AOI ID: " << aoi.id << ", bounding_box: (" << aoi.bounding_box.first.x << ", " << aoi.bounding_box.first.y << ") - ("
    //               << aoi.bounding_box.second.x << ", " << aoi.bounding_box.second.y << "), Confidence: " << aoi.confidence << std::endl;
    // }

    dynamic_reconfigure::Server<OnlinePotholeDetection::Ransac_node_ParamsConfig> server;
    dynamic_reconfigure::Server<OnlinePotholeDetection::Ransac_node_ParamsConfig>::CallbackType f;

    f = boost::bind(&RansacNode::dynamic_reconfigure_callback, &rn, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
