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
        average_horizontal_angle = std::accumulate(horizontal_angles.begin(), horizontal_angles.end(), 0.0) /
                                   horizontal_angles.size();
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

/*std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &skeleton, int left_right_num, bool debug_level)
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
        cv::imshow("pruned_vertical", pruned_vertical);
        cv::waitKey(1);

        cv::imshow("pruned_horizontal", pruned_horizontal);
        cv::waitKey(1);
    }

    return std::make_pair(pruned_vertical, pruned_horizontal);
}*/

std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &image, int left_right_num, bool debug_level)
{

    // Step 2: Detect Horizontal Lines
    cv::Mat horizontalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 1));
    cv::Mat horizontalLines;
    cv::morphologyEx(image, horizontalLines, cv::MORPH_OPEN, horizontalKernel);

    // Step 3: Detect Vertical Lines
    cv::Mat verticalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 25));
    cv::Mat verticalLines;
    cv::morphologyEx(image, verticalLines, cv::MORPH_OPEN, verticalKernel);

    if (debug_level)
    {
        cv::imshow("Detected Horizontal Lines", horizontalLines);
        cv::imshow("Detected Vertical Lines", verticalLines);
        cv::waitKey(1);
    }

    return std::make_pair(verticalLines, horizontalLines);
}

cv::Mat reconstruct_skeleton(const std::string &name, const cv::Mat &src, const cv::Mat &orig, int ksize,
                             int iterations, int debug_level)
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
    {
        // Start from 1 to skip the background
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

float computeIoU(const AOI &a, const AOI &b)
{
    int x1 = std::max(a.bounding_box.first.x, b.bounding_box.first.x);
    int y1 = std::max(a.bounding_box.first.y, b.bounding_box.first.y);
    int x2 = std::min(a.bounding_box.second.x, b.bounding_box.second.x);
    int y2 = std::min(a.bounding_box.second.y, b.bounding_box.second.y);

    int intersectionArea = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int unionArea = a.area() + b.area() - intersectionArea;

    return static_cast<float>(intersectionArea) / unionArea;
}

void computeAOI(frame_AOI_info &frame_history, AOI current_aoi, int highestId = 0)
{
    bool matched = false;
    if (frame_history.aoiList.size() <= 0)
    {
        current_aoi.id = 1;
        current_aoi.matchCount = 1;

        frame_history.addAOI(current_aoi);
        frame_history.nr_of_new_AOIs.push_back(current_aoi.id);
    }
    else
    {
        for (auto &aoi : frame_history.aoiList)
        {
            if (computeIoU(aoi, current_aoi) > 0.2)
            {
                aoi.closest_pixels_pair = current_aoi.closest_pixels_pair;
                aoi.bounding_box = current_aoi.bounding_box;
                aoi.matchCount += 2;
                if (aoi.matchCount > HISTORY)
                {
                    aoi.matchCount = HISTORY;
                }
                matched = true;
                frame_history.nr_of_new_AOIs.push_back(aoi.id);
                break; // Exit the loop since we found a match
            }
        }

        if (!matched)
        {
            current_aoi.id = highestId + 1;
            current_aoi.matchCount = 1;
            frame_history.addAOI(current_aoi);
            frame_history.nr_of_new_AOIs.push_back(current_aoi.id);
        }
    }
}

/*void find_area_of_interest(const std::string &name, const cv::Mat &labels, int num_labels, const cv::Mat &gray_orig, frame_AOI_info &frame_history, bool debug_level = 0)
{
    int WIDTH = gray_orig.cols;
    int HEIGHT = gray_orig.rows;

    std::vector<std::pair<cv::Point, cv::Point>> bounding_boxs_orig;

    // Vector to store the pairs
    std::vector<std::pair<int, int>> clusterPairs;
    std::vector<std::pair<cv::Point, cv::Point>> closestPixels;

    cv::Mat damaged_area_orig_size;
    damaged_area_orig_size = cv::Mat::zeros(gray_orig.size(), CV_8U);

    // find the highest id sofar
    int highestId = 0;
    for (auto &aoi : frame_history.aoiList)
    {
        if (aoi.id > highestId)
        {
            highestId = aoi.id;
        }
        if (aoi.matchCount > 0)
        {
            aoi.matchCount--;
        }
    }
    frame_history.nr_of_new_AOIs.clear();
    // Find the closest pixel between the clusters
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
            // This if statement ensures that only damages are marked and not just the distance between two rebars

            if (min_distance < 100)
            {
                // Make the bounding box bigger by 10 pixels in each direction
                cv::Point pt1 = closest_pixel_pair.first, pt2 = closest_pixel_pair.second;
                int x1 = clamp(pt1.x - 15, 0, WIDTH - 1);
                int y1 = clamp(pt1.y - 15, 0, HEIGHT - 1);
                int x2 = clamp(pt2.x + 15, 0, WIDTH - 1);
                int y2 = clamp(pt2.y + 15, 0, HEIGHT - 1);

                if (x1 > x2)
                    std::swap(x1, x2);
                if (y1 > y2)
                    std::swap(y1, y2);

                cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
                cv::Rect img_bounds(0, 0, WIDTH, HEIGHT);
                rect = rect & img_bounds; // Intersection of rect with image bounds

                cv::Mat damaged_area = cv::Mat::zeros(rect.size(), CV_8U);

                // Extract the damaged area from the original image
                if (x1 <= x2 && y1 <= y2)
                {
                    gray_orig(rect).copyTo(damaged_area);
                    damaged_area.copyTo(damaged_area_orig_size(cv::Rect(x1, y1, damaged_area.cols, damaged_area.rows)));
                }
                else
                {
                    gray_orig(rect).copyTo(damaged_area);
                    damaged_area.copyTo(damaged_area_orig_size(cv::Rect(x1, y1, damaged_area.cols, damaged_area.rows)));
                }

                // Within the bounding box, cluster the damaged area. We can then pair the clusters based on the distance of their centroids
                // and ultimately find the closest pixels between the clusters.
                // The closes pixels can then be used to calcuate the length of the damage and show a line between the two points.
                cluster_info result = cluster("FAOI: " + name, damaged_area_orig_size, 0);

                cv::Mat clustered_image = result.img;
                cv::Mat labels_damaged = result.labels;
                int num_components = result.num_clusters + 1;
                cv::Mat centroids = result.centroids;

                cv::rectangle(clustered_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255), 2);

                cv::imshow("Clustered 2 " + name, clustered_image);
                cv::waitKey(1);

                // Pair clusters based on the distance of their centroids
                // The clustering output the clusters in wrong order so it can happen, that the line are drawn between the wrong clusters.
                if (num_components < 2)
                {
                    // Not enough clusters to pair
                    return;
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
                // The closest pair of clusters is paired and the closest pixels between the two clusters are found
                // The clusters shouldn't be paired too far away from each other

                while (true)
                {
                    double closestDistance = std::numeric_limits<double>::max();
                    int closestIdx1 = -1;
                    int closestIdx2 = -1;

                    // Find the closest pair of unpaired clusters
                    for (int n = 1; n < num_components; ++n)
                    {
                        if (paired[n])
                            continue;
                        for (int k = n + 1; k < num_components; ++k)
                        {
                            if (paired[j])
                                continue;

                            cv::Point2d pt1_pair(centroids.at<double>(n, 0), centroids.at<double>(n, 1));
                            cv::Point2d pt2_pair(centroids.at<double>(k, 0), centroids.at<double>(k, 1));
                            double distance = euclideanDistance(pt1_pair, pt2_pair);

                            if (distance < closestDistance)
                            {
                                closestDistance = distance;
                                closestIdx1 = n;
                                closestIdx2 = k;
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
                            // if (euclideanDistance(p1, p2) < 100)
                            // {

                            double pixelDistance = euclideanDistance(p1, p2);
                            if (pixelDistance < minPixelDistance)
                            {
                                minPixelDistance = pixelDistance;
                                closestPixel1 = p1;
                                closestPixel2 = p2;
                            }
                            // }
                        }
                    }

                    AOI current_aoi;
                    current_aoi.closest_pixels_pair = std::make_pair(closestPixel1, closestPixel2);
                    current_aoi.bounding_box = std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2));
                    computeAOI(frame_history, current_aoi, highestId);
                }
            }
        }
    }
}*/

void detectInterruptions(frame_AOI_info &frame_history, const cv::Mat &lineImage, const std::string &lineType, double maxDistance, bool debug_level)
{
    cluster_info result = cluster("FAOI: " + lineType, lineImage, debug_level);

    int WIDTH = lineImage.cols;
    int HEIGHT = lineImage.rows;

    cv::Mat clustered_image = result.img;
    cv::Mat labels_damaged = result.labels;
    int nLabels = result.num_clusters + 1;
    cv::Mat centroids = result.centroids;
    cv::Mat stats = result.stats;

    std::vector<std::pair<cv::Point, cv::Point>> bounding_boxs_orig;

    // Vector to store the pairs
    std::vector<std::pair<int, int>> clusterPairs;
    std::vector<std::pair<cv::Point, cv::Point>> closestPixels;

    cv::Mat damaged_area_orig_size;
    damaged_area_orig_size = cv::Mat::zeros(lineImage.size(), CV_8U);

    // find the highest id sofar
    for (auto &aoi : frame_history.aoiList)
    {
        // if (aoi.id > highestId)
        // {
        //     highestId = aoi.id;
        // }
        if (aoi.matchCount > 0)
        {
            aoi.matchCount--;
        }
    }
    frame_history.nr_of_new_AOIs.clear();

    if (nLabels > 2)
    { // nLabels includes the background as one of the labels
        // std::cout << lineType << " Line interruptions detected! Number of clusters: " << (nLabels - 1) << std::endl;

        // Mark the clusters on the image
        cv::Mat outputImage;
        cv::cvtColor(lineImage, outputImage, cv::COLOR_GRAY2BGR);

        std::vector<Cluster> clusters;

        for (int i = 1; i < nLabels; ++i)
        {
            int x = stats.at<int>(i, cv::CC_STAT_LEFT);
            int y = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

            Cluster cluster = {x, x + width, y, y + height, cv::Point(x + width / 2, y + height / 2)};
            clusters.push_back(cluster);
        }

        // Function to calculate the horizontal or vertical distance between edges
        auto distance = [&](const Cluster &a, const Cluster &b) -> double
        {
            if (lineType == "Horizontal")
            {
                return std::abs(a.rightEdge - b.leftEdge);
            }
            else if (lineType == "Vertical")
            {
                return std::abs(a.bottomEdge - b.topEdge);
            }
            return std::numeric_limits<double>::max();
        };

        // for(int i = 0; i < clusters.size(); ++i)
        // {
        //     // Show the edges of each cluster with a circle
        //     cv::circle(outputImage, cv::Point(clusters[i].leftEdge, clusters[i].topEdge), 5, cv::Scalar(0, 255, 0), 2);
        //     cv::circle(outputImage, cv::Point(clusters[i].rightEdge, clusters[i].bottomEdge), 5, cv::Scalar(0, 0, 255), 2);
        // }

        std::vector<bool> pairedLeft(clusters.size(), false);
        std::vector<bool> pairedRight(clusters.size(), false);

        // Pair clusters based on their proximity and draw lines between closest pairs
        while (std::count(pairedLeft.begin(), pairedLeft.end(), false) > 1 ||
               std::count(pairedRight.begin(), pairedRight.end(), false) > 1)
        {
            double minDistance = std::numeric_limits<double>::max();
            int minIndex1 = -1;
            int minIndex2 = -1;
            bool isLeftEdge = false;

            for (int i = 0; i < clusters.size(); ++i)
            {
                if (pairedRight[i])
                    continue;
                for (int j = 0; j < clusters.size(); ++j)
                {
                    if (i == j || pairedLeft[j])
                        continue;
                    double dist = distance(clusters[i], clusters[j]);
                    if (dist < minDistance && dist <= maxDistance)
                    {
                        minDistance = dist;
                        minIndex1 = i;
                        minIndex2 = j;
                        isLeftEdge = true;
                    }
                }
            }

            cv::Point pt1 = cv::Point(clusters[minIndex1].rightEdge, (clusters[minIndex1].topEdge + clusters[minIndex1].bottomEdge) / 2);
            cv::Point pt2 = cv::Point(clusters[minIndex2].leftEdge, (clusters[minIndex2].topEdge + clusters[minIndex2].bottomEdge) / 2);
            cv::Point pt3 = cv::Point((clusters[minIndex1].leftEdge + clusters[minIndex1].rightEdge) / 2, clusters[minIndex1].bottomEdge);
            cv::Point pt4 = cv::Point((clusters[minIndex2].leftEdge + clusters[minIndex2].rightEdge) / 2, clusters[minIndex2].topEdge);

            int highestId = 0;
            for (auto &aoi : frame_history.aoiList)
            {
                if (aoi.id > highestId)
                {
                    highestId = aoi.id;
                }
            }

            if (minIndex1 != -1 && minIndex2 != -1)
            {
                if (lineType == "Horizontal")
                {
                    // The are most likely two rebars in the picture. 
                    // Without this if statement the program would draw a line between the two rebars
                    if (!(euclideanDistance(pt1, pt2) > 100))
                    {
                        int x1 = clamp(pt1.x - 10, 0, WIDTH - 1);
                        int y1 = clamp(pt1.y - 10, 0, HEIGHT - 1);
                        int x2 = clamp(pt2.x + 10, 0, WIDTH - 1);
                        int y2 = clamp(pt2.y + 10, 0, HEIGHT - 1);

                        cv::line(outputImage, pt1, pt2, cv::Scalar(0, 0, 255), 2);
                        cv::rectangle(outputImage, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255), 2);

                        AOI current_aoi;
                        current_aoi.closest_pixels_pair = std::make_pair(pt1, pt2);
                        current_aoi.bounding_box = std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2));
                        computeAOI(frame_history, current_aoi, highestId);
                    }
                }
                else if (lineType == "Vertical")
                {
                    if (!(euclideanDistance(pt3, pt4) > 100))
                    {

                        int x1 = clamp(pt3.x - 10, 0, WIDTH - 1);
                        int y1 = clamp(pt3.y - 10, 0, HEIGHT - 1);
                        int x2 = clamp(pt4.x + 10, 0, WIDTH - 1);
                        int y2 = clamp(pt4.y + 10, 0, HEIGHT - 1);

                        cv::line(outputImage, pt3, pt4, cv::Scalar(0, 0, 255), 2);
                        cv::rectangle(outputImage, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 255), 2);

                        AOI current_aoi;
                        current_aoi.closest_pixels_pair = std::make_pair(pt3, pt4);
                        current_aoi.bounding_box = std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2));
                        computeAOI(frame_history, current_aoi, highestId);
                    }
                }
                pairedRight[minIndex1] = true;
                pairedLeft[minIndex2] = true;
            }
            else
            {
                break;
            }
        }

        // Show the result
        cv::imshow(lineType + " Lines with Interruptions", outputImage);
        cv::waitKey(1);
    }
    else
    {
        // std::cout << "No interruptions detected in " << lineType << " lines." << std::endl;
    }
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
std::vector<std::vector<cv::Point3f>> get_3d_coordinates(const cv::Mat &img, const cv::Mat &depth_image,
                                                         const cv::Mat &labels, double Z)
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
