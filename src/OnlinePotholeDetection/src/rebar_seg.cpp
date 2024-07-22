#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <rebar_seg.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <random>

std::pair<double, double> find_rotation(cv::Mat &image, bool debug_level)
{
    // Detect lines using Hough Line Transform
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(image, lines, 1, CV_PI / 180, 50, 10, 10);

    // Classify lines based on their angle
    std::vector<double> angles;
    for (size_t i = 0; i < lines.size(); ++i)
    {
        cv::Vec4i l = lines[i];
        double angle = std::atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
        angles.push_back(angle);
    }

    // Determine horizontal-like and vertical-like angles
    std::vector<double> horizontal_angles;
    std::vector<double> vertical_angles;
    for (size_t i = 0; i < angles.size(); ++i)
    {
        if (-45 <= angles[i] && angles[i] <= 45)
        {
            horizontal_angles.push_back(angles[i]);
        }
        else if ((angles[i] > 45 && angles[i] <= 135) || (angles[i] < -45 && angles[i] >= -135))
        {
            vertical_angles.push_back(angles[i]);
        }
    }

    // Compute the averages
    double average_horizontal_angle = 0;
    if (!horizontal_angles.empty())
    {
        average_horizontal_angle = std::accumulate(horizontal_angles.begin(), horizontal_angles.end(), 0.0) / horizontal_angles.size();
    }

    double average_vertical_angle = 0;
    if (!vertical_angles.empty())
    {
        average_vertical_angle = std::accumulate(vertical_angles.begin(), vertical_angles.end(), 0.0) / vertical_angles.size();
    }

    if (debug_level)
    {
        ROS_INFO("Vertical angle: %f", average_vertical_angle);
        ROS_INFO("Horizontal angle: %f", average_horizontal_angle);
    }

    return std::make_pair(average_vertical_angle, average_horizontal_angle);
}

cv::Mat rotate_image(const std::string &name, const cv::Mat &image, double angle, bool debug_level)
{
    int h = image.rows;
    int w = image.cols;

    cv::Point2f center = cv::Point2f(w / 2.0, h / 2.0);

    // Get the rotation matrix
    cv::Mat M = cv::getRotationMatrix2D(center, angle, 1);

    // Perform the affine transformation (rotation)
    cv::Mat rotated;
    cv::warpAffine(image, rotated, M, cv::Size(w, h));

    if (debug_level)
    {
        cv::imshow("Original " + name, image);
        cv::waitKey(1);
        cv::imshow("Rotated " + name, rotated);
        cv::waitKey(1);
    }

    return rotated;
}

std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &skeleton, int left_right_num, bool debug_level)
{
    int height = skeleton.rows;
    int width = skeleton.cols;

    cv::Mat pruned_vertical = skeleton.clone();
    cv::Mat pruned_horizontal = skeleton.clone();

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (skeleton.at<uchar>(y, x) == 255)
            {
                // Check next lines in the vertical direction
                for (int i = 1; i < left_right_num; ++i)
                {
                    if (y + i < height && skeleton.at<uchar>(y + i, x) == 255)
                    {
                        pruned_horizontal.at<uchar>(y, x) = 0;
                        break;
                    }
                    if (y - i >= 0 && skeleton.at<uchar>(y - i, x) == 255)
                    {
                        pruned_horizontal.at<uchar>(y, x) = 0;
                        break;
                    }
                }
                // Check next lines in the horizontal direction
                for (int i = 1; i < left_right_num; ++i)
                {
                    if (x + i < width && skeleton.at<uchar>(y, x + i) == 255)
                    {
                        pruned_vertical.at<uchar>(y, x) = 0;
                        break;
                    }
                    if (x - i >= 0 && skeleton.at<uchar>(y, x - i) == 255)
                    {
                        pruned_vertical.at<uchar>(y, x) = 0;
                        break;
                    }
                }
            }
        }
    }
    if (debug_level)
    {
        std::cout << "Vertical Shape: " << pruned_vertical.size() << std::endl;
        cv::imshow("pruned_vertical", pruned_vertical);
        cv::waitKey(1);

        std::cout << "Horizontal Shape: " << pruned_horizontal.size() << std::endl;
        cv::imshow("pruned_horizontal", pruned_horizontal);
        cv::waitKey(1);
    }

    return std::make_pair(pruned_vertical, pruned_horizontal);
}

cv::Mat reconstruct_skeleton(const std::string &name, const cv::Mat &src, const cv::Mat &orig, int ksize, int iterations, int debug_level)
{
    cv::Mat this_iteration = src.clone();
    cv::Mat last_iteration = src.clone();

    for (int i = 0; i < iterations; ++i)
    {
        // Dilate the image
        cv::Mat kernel = cv::Mat::ones(ksize, ksize, CV_8U);
        cv::dilate(last_iteration, this_iteration, kernel, cv::Point(-1, -1), 1);

        // Show the image at this iteration
        if (debug_level > 1)
        {
            cv::imshow("reconstruction " + name + " - dilated", this_iteration);
            cv::waitKey(1);
        }

        // Mask the dilated image with the original image
        this_iteration &= orig;

        // Show the image at this iteration
        if (debug_level > 1)
        {
            cv::imshow("reconstruction " + name + " - masked", this_iteration);
            cv::waitKey(1);
        }

        if (cv::countNonZero(this_iteration != last_iteration) == 0)
        {
            // Convergence
            break;
        }

        last_iteration = this_iteration.clone();
    }

    // Display the input and pruned output if debug_level > 0
    if (debug_level > 0)
    {
        cv::imshow("reconstruction input " + name + " - skeleton", src);
        cv::waitKey(1);
        cv::imshow("reconstruction output " + name + " - pruned", this_iteration);
        cv::waitKey(1);
    }

    return this_iteration; // Note: Pruning functionality is not implemented
}

cv::Mat remove_small_blobs(const std::string &name, const cv::Mat &img, int min_blob_size, bool debug_level)
{
    // Perform connected components analysis
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(img, labels, stats, centroids, 8);

    // Create a mask to filter out small components
    cv::Mat large_blobs_mask = cv::Mat::zeros(img.size(), CV_8U);

    // Find components that are larger than the minimum blob size
    for (int i = 1; i < num_labels; ++i)
    { // Start from 1 to skip the background
        if (stats.at<int>(i, cv::CC_STAT_AREA) >= min_blob_size)
        {
            large_blobs_mask.setTo(255, labels == i);
        }
    }

    // Apply the mask to the input image to retain only large components
    cv::Mat output;
    cv::bitwise_and(img, img, output, large_blobs_mask);

    if (debug_level)
    {
        cv::imshow("Input " + name, img);
        cv::waitKey(1);
        cv::imshow("Small blobs removed output " + name, output);
        cv::waitKey(1);
    }

    return output;
}

cv::Vec3b random_color()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 255);
    return cv::Vec3b(dis(gen), dis(gen), dis(gen));
}

cluster_info cluster_skeleton(const std::string &name, const cv::Mat &skeleton, bool debug_level)
{
    int height = skeleton.rows;
    int width = skeleton.cols;

    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(skeleton, labels, stats, centroids, 8);

    cv::Mat cluster_img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    std::vector<cv::Vec3b> colors(num_labels);
    for (int i = 0; i < num_labels; ++i)
    {
        colors[i] = random_color();
    }

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (skeleton.at<uchar>(y, x) == 255)
            {
                cluster_img.at<cv::Vec3b>(y, x) = colors[labels.at<int>(y, x)];
            }
        }
    }

    if (debug_level)
    {
        cv::imshow("Clustered " + name, cluster_img);
        cv::waitKey(1);
    }

    return cluster_info{cluster_img, num_labels - 1, labels, stats, centroids};
}

// Helper function to clamp a value within a range
int clamp(int value, int min_val, int max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

std::tuple<std::vector<std::pair<cv::Point, cv::Point>>, std::vector<std::pair<cv::Point, cv::Point>>>
find_area_of_interest(const std::string &name, const cv::Mat &labels, int num_labels, const cv::Mat &gray_orig, const cv::Mat &img, bool debug_level = 0)
{
    // int WIDTH = gray_orig.cols;
    // int HEIGHT = gray_orig.rows;

    int WIDTH = gray_orig.cols;
    int HEIGHT = gray_orig.rows;

    std::vector<std::pair<cv::Point, cv::Point>> closest_pixels_skeleton;
    std::vector<std::pair<cv::Point, cv::Point>> bboxs_skeleton;

    std::vector<std::pair<cv::Point, cv::Point>> closest_pixels_orig;
    std::vector<std::pair<cv::Point, cv::Point>> bboxs_orig;

    // // Show the input labels
    // if (debug_level)
    // {
    //     cv::Mat label_image = cv::Mat::zeros(gray_orig.size(), CV_8U);
    //     for (int i = 1; i <= num_labels; ++i)
    //     {
    //         label_image.setTo(255, labels == i);
    //     }
    //     cv::imshow(name + " label", label_image);
    //     cv::waitKey(1);
    // }

    cv::Mat damaged_area_orig_size;
    damaged_area_orig_size = cv::Mat::zeros(gray_orig.size(), CV_8U);

    // // Find the closest pixel between the vertical clusters
    for (int i = 1; i <= num_labels; ++i)
    {
        for (int j = i + 1; j <= num_labels; ++j)
        {
            cv::Mat cluster1, cluster2;
            cv::findNonZero(labels == i, cluster1);
            cv::findNonZero(labels == j, cluster2);

            double min_distance = 1000;
            std::pair<cv::Point, cv::Point> closest_pixel_pair;
            for (cv::MatIterator_<cv::Point> it1 = cluster1.begin<cv::Point>(), end1 = cluster1.end<cv::Point>(); it1 != end1; ++it1)
            {
                for (cv::MatIterator_<cv::Point> it2 = cluster2.begin<cv::Point>(), end2 = cluster2.end<cv::Point>(); it2 != end2; ++it2)
                {
                    double distance = cv::norm(*it1 - *it2);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        closest_pixel_pair = std::make_pair(*it1, *it2);
                    }
                }
            }

            if (min_distance < 100)
            {
                // ROS_INFO("Closest pixel distance: %f", min_distance);
                // Visualize the damaged area in the original image
                // Draw a bounding box around the damaged area
                // Make the bounding box bigger by 10 pixels in each direction
                cv::Point pt1 = closest_pixel_pair.first, pt2 = closest_pixel_pair.second;
                int x1 = clamp(pt1.x - 10, 0, WIDTH - 1);
                int y1 = clamp(pt1.y - 10, 0, HEIGHT - 1);
                int x2 = clamp(pt2.x + 10, 0, WIDTH - 1);
                int y2 = clamp(pt2.y + 10, 0, HEIGHT - 1);

                cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
                cv::Rect img_bounds(0, 0, WIDTH, HEIGHT);
                rect = rect & img_bounds; // Intersection of rect with image bounds

                cv::Mat damaged_area = cv::Mat::zeros(rect.size(), gray_orig.type());

                // cv::Mat damaged_area = cv::Mat::zeros(gray_orig.size(), CV_8U);
                // Extract the damaged area from the original image
                // if (rect_width > 0 && rect_height > 0)
                if (x1 < x2 && y1 < y2)
                {
                    gray_orig(rect).copyTo(damaged_area);
                    damaged_area.copyTo(damaged_area_orig_size(cv::Rect(x1, y1, damaged_area.cols, damaged_area.rows)));

                    closest_pixels_skeleton.push_back(closest_pixel_pair);
                    bboxs_skeleton.push_back(std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2)));
                    // cv::imshow("Damaged area " + name, damaged_area_orig_size);
                    // cv::waitKey(1);
                }
                else
                {
                    // std::cerr << "Invalid rectangle dimensions: (" << x1 << ", " << y1 << ") to (" << x2 << ", " << y2 << ")" << std::endl;
                    //std::cerr << std::endl;
                }

                // SHOw the damaged area
                // if (debug_level)
                // {
                // }

                // Insert the damaged area into the original image in the correct location

                // Find connected components
                cv::Mat labels_damaged, stats, centroids;
                int num_components = cv::connectedComponentsWithStats(damaged_area_orig_size, labels_damaged, stats, centroids, 8);
                // cluster_info clustering = cluster_skeleton("Vertical", damaged_area_orig_size, 1);

                // int num_components = clustering.num_clusters;
                // cv::Mat labels_damaged = clustering.labels;
                int height = damaged_area_orig_size.rows;
                int width = damaged_area_orig_size.cols;
                cv::Mat cluster_img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

                std::vector<cv::Vec3b> colors(num_components);
                for (int i = 0; i < num_components; ++i)
                {
                    colors[i] = random_color();
                }

                for (int y = 0; y < height; ++y)
                {
                    for (int x = 0; x < width; ++x)
                    {
                        if (damaged_area_orig_size.at<uchar>(y, x) == 255)
                        {
                            cluster_img.at<cv::Vec3b>(y, x) = colors[labels_damaged.at<int>(y, x)];
                        }
                    }
                }

                // Find closest pixels between clusters within the damaged area
                for (int k = 1; k < num_components; ++k)
                {
                    for (int l = k + 1; l < num_components; ++l)
                    {
                        cv::Mat cluster1_damaged, cluster2_damaged;
                        cv::findNonZero(labels_damaged == k, cluster1_damaged);
                        cv::findNonZero(labels_damaged == l, cluster2_damaged);

                        double min_distance_inner = 1000;
                        std::pair<cv::Point, cv::Point> closest_pixel_pair_inner;

                        for (cv::MatIterator_<cv::Point> it1 = cluster1_damaged.begin<cv::Point>(), end1 = cluster1_damaged.end<cv::Point>(); it1 != end1; ++it1)
                        {
                            for (cv::MatIterator_<cv::Point> it2 = cluster2_damaged.begin<cv::Point>(), end2 = cluster2_damaged.end<cv::Point>(); it2 != end2; ++it2)
                            {
                                double distance_inner = cv::norm(*it1 - *it2);
                                if (distance_inner < min_distance_inner)
                                {
                                    min_distance_inner = distance_inner;
                                    closest_pixel_pair_inner = std::make_pair(*it1, *it2);
                                }
                            }
                        }

                        if (closest_pixel_pair_inner.first != cv::Point() && closest_pixel_pair_inner.second != cv::Point())
                        {
                            if (min_distance_inner < 100)
                            {
                                double distance = cv::norm(closest_pixel_pair_inner.first - closest_pixel_pair_inner.second);
                                //ROS_INFO("Closest pixel distance inner: %f", distance);
                                closest_pixels_orig.push_back(closest_pixel_pair_inner);
                                bboxs_orig.push_back(std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2)));
                            }
                        }
                    }
                }
            }
        }
    }

    return std::make_tuple(closest_pixels_orig, bboxs_orig);
}