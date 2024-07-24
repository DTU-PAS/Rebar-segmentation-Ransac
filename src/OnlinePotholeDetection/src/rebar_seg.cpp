#include <rebar_seg.h>

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

cluster_info cluster(const std::string &name, const cv::Mat &img, bool debug_level)
{
    int height = img.rows;
    int width = img.cols;

    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(img, labels, stats, centroids, 8);

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
            if (img.at<uchar>(y, x) == 255)
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

double euclideanDistance(cv::Point2d pt1, cv::Point2d pt2)
{
    return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

frame_AOI_info find_area_of_interest(const std::string &name, const cv::Mat &labels, int num_labels, const cv::Mat &gray_orig, const cv::Mat &img, bool debug_level = 0)
{
    // int WIDTH = gray_orig.cols;
    // int HEIGHT = gray_orig.rows;

    int WIDTH = gray_orig.cols;
    int HEIGHT = gray_orig.rows;

    // std::vector<std::pair<cv::Point, cv::Point>> closest_pixels_skeleton;
    // std::vector<std::pair<cv::Point, cv::Point>> bboxs_skeleton;

    // std::vector<std::pair<cv::Point, cv::Point>> closest_pixels_orig;
    std::vector<std::pair<cv::Point, cv::Point>> bboxs_orig;

    // Vector to store the pairs
    std::vector<std::pair<int, int>> clusterPairs;
    std::vector<std::pair<cv::Point, cv::Point>> closestPixels;

    frame_AOI_info frame_aoi;

    cv::Mat damaged_area_orig_size;
    damaged_area_orig_size = cv::Mat::zeros(gray_orig.size(), CV_8U);

    cv::Mat clustered_image;

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

                // Extract the damaged area from the original image
                if (x1 < x2 && y1 < y2)
                {
                    gray_orig(rect).copyTo(damaged_area);
                    damaged_area.copyTo(damaged_area_orig_size(cv::Rect(x1, y1, damaged_area.cols, damaged_area.rows)));

                    // closest_pixels_skeleton.push_back(closest_pixel_pair);
                    // bboxs_skeleton.push_back(std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2)));
                }

                // Find connected components
                // cv::Mat labels_damaged, stats, centroids;
                // int num_components = cv::connectedComponentsWithStats(damaged_area_orig_size, labels_damaged, stats, centroids, 8);

                cluster_info result = cluster(name, damaged_area_orig_size, 0);

                clustered_image = result.img;

                cv::Mat labels_damaged = result.labels;
                int num_components = result.num_clusters + 1;
                cv::Mat centroids = result.centroids;

                // pair clusters based on the distance of their centroids
                if (num_components < 2)
                {
                    std::cerr << "Not enough clusters to pair." << std::endl;
                    return frame_aoi;
                }

                std::vector<bool> paired(num_components, false);

                // Function to find all pixels belonging to a cluster
                auto getClusterPixels = [&](int label)
                {
                    std::vector<cv::Point> pixels;
                    for (int y = 0; y < labels_damaged.rows; ++y)
                    {
                        for (int x = 0; x < labels_damaged.cols; ++x)
                        {
                            if (labels_damaged.at<int>(y, x) == label)
                            {
                                pixels.push_back(cv::Point(x, y));
                            }
                        }
                    }
                    return pixels;
                };

                // Pair clusters until one is left
                while (true)
                {
                    double closestDistance = std::numeric_limits<double>::max();
                    int closestIdx1 = -1;
                    int closestIdx2 = -1;

                    // Find the closest pair of unpaired clusters
                    for (int i = 1; i < num_components; ++i)
                    {
                        if (paired[i])
                            continue;
                        for (int j = i + 1; j < num_components; ++j)
                        {
                            if (paired[j])
                                continue;

                            cv::Point2d pt1(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
                            cv::Point2d pt2(centroids.at<double>(j, 0), centroids.at<double>(j, 1));
                            double distance = euclideanDistance(pt1, pt2);

                            if (distance < closestDistance)
                            {
                                closestDistance = distance;
                                closestIdx1 = i;
                                closestIdx2 = j;
                            }
                        }
                    }

                    // If there are no more pairs to make, break the loop
                    if (closestIdx1 == -1 || closestIdx2 == -1)
                        break;

                    // Pair the closest clusters
                    paired[closestIdx1] = true;
                    paired[closestIdx2] = true;
                    clusterPairs.push_back(std::make_pair(closestIdx1, closestIdx2));

                    // Get pixels for each cluster
                    std::vector<cv::Point> pixels1 = getClusterPixels(closestIdx1);
                    std::vector<cv::Point> pixels2 = getClusterPixels(closestIdx2);

                    // Find the closest pixels between the two clusters
                    double minPixelDistance = std::numeric_limits<double>::max();
                    cv::Point closestPixel1, closestPixel2;
                    for (const auto &p1 : pixels1)
                    {
                        for (const auto &p2 : pixels2)
                        {
                            double pixelDistance = euclideanDistance(p1, p2);
                            if (pixelDistance < minPixelDistance)
                            {
                                minPixelDistance = pixelDistance;
                                closestPixel1 = p1;
                                closestPixel2 = p2;
                            }
                        }
                    }

                    // Save the closest pixels
                    // closestPixels.push_back(std::make_pair(closestPixel1, closestPixel2));
                    // bboxs_orig.push_back(std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2)));
                    AOI aoi;
                    aoi.closest_pixels_pair = std::make_pair(closestPixel1, closestPixel2);
                    aoi.bbox = std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2));
                    aoi.id = -1;
                    frame_aoi.aoiList.push_back(aoi);
                }

                if (debug_level)
                {
                    // print text onto the image with cluster id
                    for (int i = 1; i < num_components; ++i)
                    {
                        cv::putText(clustered_image, std::to_string(i), cv::Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                    }

                    // Find the unpaired cluster
                    int unpairedCluster = -1;
                    for (int i = 1; i < num_components; ++i)
                    {
                        if (!paired[i])
                        {
                            unpairedCluster = i;
                            break;
                        }
                    }

                    // Output the pairs and their closest pixels
                    for (size_t i = 0; i < clusterPairs.size(); ++i)
                    {
                        const auto &pair = clusterPairs[i];
                        const auto &pixels = closestPixels[i];
                        std::cout << name << " Cluster " << pair.first << " is paired with Cluster " << pair.second << std::endl;
                        std::cout << name << " Closest pixels: (" << pixels.first.x << ", " << pixels.first.y << ") and ("
                                  << pixels.second.x << ", " << pixels.second.y << ")" << std::endl;
                    }

                    // Output the unpaired cluster
                    if (unpairedCluster != -1)
                    {
                        std::cout << name << " Unpaired Cluster: " << unpairedCluster << std::endl;
                    }
                    cv::imshow("Clustered2 " + name, clustered_image);
                    cv::waitKey(0);
                    std::cout << std::endl;
                }
            }
        }
    }

    return frame_aoi;
}

cv::Point3f pixel_to_camera(int u, int v, float Z, const cv::Mat &K_inv)
{
    cv::Mat pixel_coords = (cv::Mat_<double>(3, 1) << u, v, 1);
    cv::Mat camera_coords = K_inv * pixel_coords * Z;
    return cv::Point3f(camera_coords.at<double>(0, 0), camera_coords.at<double>(1, 0), camera_coords.at<double>(2, 0));
}

// Function to plot camera coordinates (placeholder, implement as needed)
void plot_camera_coordinates(const std::vector<std::vector<cv::Point3f>> &camera_cluster_coordinates, int param)
{
    // Implement your plotting logic here
}

// Function that calculates and returns 3d coordinates of the white pixels in the image
std::vector<std::vector<cv::Point3f>> get_3d_coordinates(const cv::Mat &img, const cv::Mat &depth_image, const cv::Mat &labels, double Z)
{

    cv::Mat K = (cv::Mat_<double>(3, 3) << 465.33203125, 0, 353.9921875, 0, 465.33203125, 251.28125, 0, 0, 1);
    cv::Mat K_inv = K.inv();

    // cv::Mat img;         // Load your image here
    // cv::Mat depth_image; // Load your depth image here

    std::vector<cv::Point> white_pixels;
    cv::findNonZero(img == 255, white_pixels);

    std::vector<std::vector<cv::Point>> pixel_coordinates;

    // cv::Mat labels;   // Load your vertical labels here

    int max_labels = *std::max_element(labels.begin<int>(), labels.end<int>());

    for (int i = 1; i <= max_labels; ++i)
    {
        std::vector<cv::Point> cluster;
        cv::findNonZero(labels == i, cluster);
        pixel_coordinates.push_back(cluster);
    }

    std::vector<std::vector<cv::Point3f>> camera_cluster_coordinates;

    for (const auto &cluster : pixel_coordinates)
    {
        std::vector<cv::Point3f> camera_coordinates;
        for (const auto &pt : cluster)
        {
            int v = pt.y, u = pt.x;
            float Z = depth_image.at<uint16_t>(v, u) * 0.001f;
            ROS_INFO("Depth: %f", Z);
            camera_coordinates.push_back(pixel_to_camera(u, v, Z, K_inv));
        }
        camera_cluster_coordinates.push_back(camera_coordinates);
    }

    // plot_camera_coordinates(camera_cluster_coordinates, 0);

    return camera_cluster_coordinates;
}