#include <image_processing.h>
#include <ransac_node.h>
#include <publish.h>
#include <utils.h>

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

    if (image.empty())
    {
        ROS_ERROR("Input image for rotation is empty");
        return cv::Mat();
    }
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
        // cv::imshow("Original " + name, image);
        // cv::waitKey(1);
        // cv::imshow("Rotated " + name, rotated);
        // cv::waitKey(1);
    }

    return rotated;
}

// Rotate a 2d point around the origin
cv::Point rotate_point(const std::string &name, const cv::Point point, cv::Point2f center, double angle, bool debug_level)
{
    // Get the rotation matrix
    cv::Mat M = cv::getRotationMatrix2D(center, angle, 1);

    // Rotate the point
    double x = point.x * M.at<double>(0, 0) + point.y * M.at<double>(0, 1) + M.at<double>(0, 2);
    double y = point.x * M.at<double>(1, 0) + point.y * M.at<double>(1, 1) + M.at<double>(1, 2);

    if (debug_level)
    {
        ROS_INFO("Original Point: (%d, %d)", point.x, point.y);
        ROS_INFO("Rotated Point: (%f, %f)", x, y);
    }

    return cv::Point(x, y);
}

std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &image, std::pair<double, double> angles, int kernelsize, bool debug_level)
{

    // Step 2: Detect Horizontal Lines
    cv::Mat horizontalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelsize, 1));
    cv::Mat horizontalLines;
    cv::morphologyEx(image, horizontalLines, cv::MORPH_OPEN, horizontalKernel);

    // Step 3: Detect Vertical Lines
    cv::Mat verticalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, kernelsize));
    cv::Mat verticalLines;
    cv::morphologyEx(image, verticalLines, cv::MORPH_OPEN, verticalKernel);

    if (debug_level)
    {
        // cv::imshow("Detected Horizontal Lines", horizontalLines);
        // cv::imshow("Detected Vertical Lines", verticalLines);
        // cv::waitKey(1);
    }

    return std::make_pair(verticalLines, horizontalLines);
}

/*std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &image, std::pair<double, double> angles, int left_right_num, bool debug_level)
{

    // Create a horizontal kernel
    cv::Mat horizontalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 1));

    // Create a larger square kernel that can accommodate the rotation
    int kernelSize = std::max(horizontalKernel.cols, horizontalKernel.rows) * 2;
    cv::Mat largeKernel = cv::Mat::zeros(kernelSize, kernelSize, CV_8UC1);

    // Place the horizontal kernel at the center of the larger kernel
    horizontalKernel.copyTo(largeKernel(cv::Rect((kernelSize - horizontalKernel.cols) / 2, (kernelSize - horizontalKernel.rows) / 2, horizontalKernel.cols, horizontalKernel.rows)));

    // Rotate the large kernel by the known angle
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(kernelSize / 2, kernelSize / 2), -angles.second, 1.0);
    cv::Mat rotatedKernel;
    cv::warpAffine(largeKernel, rotatedKernel, rotationMatrix, largeKernel.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));

    // Crop the rotated kernel to remove extra black areas
    cv::Rect boundingBox = cv::boundingRect(rotatedKernel);
    rotatedKernel = rotatedKernel(boundingBox);
    std::cout << "Rotated Kernel: " << rotatedKernel << std::endl;

    // Apply morphology operation to detect lines with the rotated kernel
    cv::Mat detectedLines;
    cv::morphologyEx(image, detectedLines, cv::MORPH_OPEN, rotatedKernel);

    cv::imshow("Detected Horizontal Lines", detectedLines);
    cv::waitKey(1);

    // // Create a horizontal kernel
    // cv::Mat horizontalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 1));
    // cv::Mat verticalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 25));
    // // Print the kernel
    // std::cout << "Horizontal Kernel: " << horizontalKernel << std::endl;
    // std::cout << "Vertical Kernel: " << verticalKernel << std::endl;

    return std::make_pair(detectedLines, detectedLines);
}*/

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
        // cv::imshow("Clustered " + name, cluster_img);
        // cv::waitKey(1);
    }

    return cluster_info{cluster_img, num_labels - 1, labels, stats, centroids};
}

void computeAOI(frame_AOI_info &frame_history, AOI current_aoi /*, std::vector<cv::Point> points*/, int highestId = 0)
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
                // aoi.points = points;
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

void detectInterruptions(frame_AOI_info &frame_history, const cv::Mat &lineImage, double maxDistance, int paddding, bool debug_level, bool show_clusters)
{
    cluster_info result = cluster("FAOI: " + frame_history.ns, lineImage, 0);

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

    for (auto &aoi : frame_history.aoiList)
    {
        if (aoi.matchCount > 0)
        {
            aoi.matchCount--;
        }
    }
    frame_history.nr_of_new_AOIs.clear();

    if (nLabels > 2)
    {

        std::vector<Cluster> clusters;

        // Calculate the oriented bounding box of each cluster
        int width = labels_damaged.cols;
        int height = labels_damaged.rows;

        for (int i = 1; i < nLabels; ++i)
        {
            std::vector<cv::Point> points;
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    if (labels_damaged.at<int>(y, x) == i)
                    {
                        points.push_back(cv::Point(x, y));
                    }
                }
            }

            if (points.size() > 0)
            {
                cv::RotatedRect rotatedRect = cv::minAreaRect(points);
                cv::Point2f vertices[4];
                rotatedRect.points(vertices);

                Cluster cluster;
                cluster.center = rotatedRect.center;
                cluster.centroid = cv::Point((vertices[0].x + vertices[2].x) / 2, (vertices[0].y + vertices[2].y) / 2);

                // Calculate the midpoints of each line segment
                std::vector<cv::Point2f> midpoints(4);
                for (int j = 0; j < 4; ++j)
                {
                    midpoints[j] = cv::Point2f((vertices[j].x + vertices[(j + 1) % 4].x) / 2,
                                               (vertices[j].y + vertices[(j + 1) % 4].y) / 2);
                }

                // Store the midpoints in the cluster if needed
                cluster.midpoints = midpoints;

                clusters.push_back(cluster);
                frame_history.clusters = clusters;

                // Visualize the oriented bounding box
                // for (int j = 0; j < 4; ++j)
                // {
                //     cv::line(clustered_image, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255, 255, 255), 2);
                // }

                // // Visualize the midpoints
                // for (const auto &midpoint : midpoints)
                // {
                //     cv::circle(clustered_image, midpoint, 3, cv::Scalar(0, 0, 255), -1);
                // }
            }
        }

        // Find the edges of the clusters that are closest to each other stop using the centroid and use the midpoints instead
        for (int i = 0; i < clusters.size(); ++i)
        {
            for (int j = i + 1; j < clusters.size(); ++j)
            {
                double minDistance = std::numeric_limits<double>::max();
                cv::Point2f closestPoint1, closestPoint2;
                for (const auto &point1 : clusters[i].midpoints)
                {
                    for (const auto &point2 : clusters[j].midpoints)
                    {
                        double distance = euclideanDistance(point1, point2);
                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            closestPoint1 = point1;
                            closestPoint2 = point2;
                        }
                    }
                }

                if (minDistance < maxDistance)
                {
                    clusterPairs.push_back(std::make_pair(i, j));
                    closestPixels.push_back(std::make_pair(cv::Point(closestPoint1.x, closestPoint1.y), cv::Point(closestPoint2.x, closestPoint2.y)));
                }
            }
        }

        // Visualize the closest points
        // for (const auto &pair : closestPixels)
        // {
        //     cv::line(clustered_image, pair.first, pair.second, cv::Scalar(0, 0, 255), 2);
        // }

        int highestId = 0;
        for (auto &aoi : frame_history.aoiList)
        {
            if (aoi.id > highestId)
            {
                highestId = aoi.id;
            }
        }

        AOI current_aoi;
        // Loop through the closest pixels and create an AOI for each pair
        for (int i = 0; i < closestPixels.size(); ++i)
        {
            cv::Point pt1 = closestPixels[i].first;
            cv::Point pt2 = closestPixels[i].second;

            int x1 = clamp(pt1.x - paddding, 0, WIDTH - 1);
            int y1 = clamp(pt1.y - paddding, 0, HEIGHT - 1);
            int x2 = clamp(pt2.x + paddding, 0, WIDTH - 1);
            int y2 = clamp(pt2.y + paddding, 0, HEIGHT - 1);

            cv::rectangle(clustered_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255), 2);

            current_aoi.closest_pixels_pair = std::make_pair(pt1, pt2);
            current_aoi.bounding_box = std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2));
            // std::vector<cv::Point2f> points = clusters[clusterPairs[i].first].midpoints;
            computeAOI(frame_history, current_aoi /*, points*/, highestId);
        }
    }
}

double find_depth(const cv::Mat &depth_image, int u, int v)
{
    std::vector<float> Z_values;
    for (int i = -1; i < 2; ++i)
    {
        for (int j = -1; j < 2; ++j)
        {
            if (v + i >= 0 && v + i < depth_image.rows && u + j >= 0 && u + j < depth_image.cols)
            {
                Z_values.push_back(depth_image.at<float>(v + i, u + j) * 0.001f);
            }
        }
    }

    if (Z_values.size() > 0)
    {
        return std::accumulate(Z_values.begin(), Z_values.end(), 0.0) / Z_values.size();
    }

    return 0.0;
}



// This function is used to detect the width of the rebar in the image using the image processing techniques
// When the rebar suddenly changes its width, it is marked as a defect
std::vector<skeleton_info> detect_width(frame_AOI_info &frame_history, cv::Mat &binary)
{
    // cv::imshow("Binary", binary);
    // cv::waitKey(1);
    // 4. Distance transform on this component
    cv::Mat dist;
    cv::distanceTransform(binary, dist, cv::DIST_L2, 3);

    // cv::imshow("Binary", binary);
    // cv::waitKey(1);

    cluster_info result = cluster("FAOI: " + frame_history.ns, binary, 0);

    // The 0-th component is background. So real components are [1..nComponents-1].
    std::cout << "Found " << (result.num_clusters) << " separate rebar component(s)\n";

    std::vector<skeleton_info> skeleton_info_list;
    for (int c = 1; c < result.num_clusters +1; c++) 
    {
        // 3. Create a mask for component c
        // Using a comparison to build a mask: (labels == c) → 255, else 0
        cv::Mat componentMask = (result.labels == c);
        componentMask.convertTo(componentMask, CV_8UC1, 255); // convert bool to 0/255
        
        // Optional: Clean up small noise inside the mask if needed
        // e.g., morphological opening, etc.
        
        cv::Mat skel;
        skeleton_info skel_info;
        skel_info.skeleton = skel;
        skel_info.binary = componentMask;
        
        cv::ximgproc::thinning(componentMask, skel, cv::ximgproc::THINNING_GUOHALL);


        // cv::imshow("Skeleton", skel);
        // cv::waitKey(0);

        // 6. Collect skeleton points + width
        std::vector<SkelPoint> skelPoints;
        for (int y = 0; y < skel.rows; y++) {
            const uchar *skelRow = skel.ptr<uchar>(y);
            for (int x = 0; x < skel.cols; x++) {
                if (skelRow[x] == 255) {
                    float d = dist.at<float>(y, x);
                    float width = 2.0f * d;
                    skelPoints.push_back(SkelPoint((float)x, (float)y, width));
                }
            }
        }

        // print width of each skeleton point
        // for (auto &pt : skelPoints) {
        //     std::cout << "Skeleton point at " << std::to_string(c) << frame_history.ns << " (" << pt.x << ", " << pt.y << ") has width " << pt.width << std::endl;
        // }
        // std::cout << "\n";

        cv::Mat colorVis;
        cv::cvtColor(binary, colorVis, cv::COLOR_GRAY2BGR); // skeleton is 0/255
        for (size_t i = 0; i < skelPoints.size(); ++i) {
            if (skelPoints[i].width > 20.0f) {
                cv::Point pt(skelPoints[i].x, skelPoints[i].y);
                skel_info.points.push_back(pt);
                cv::circle(colorVis, pt , 3, cv::Scalar(0, 0, 255), -1); // mark in red
            }
        }

        // // Normalize distance transform for display
        cv::Mat distVis;
        cv::normalize(dist, distVis, 0, 1.0, cv::NORM_MINMAX);

        for (auto &pt : skelPoints) {
            cv::circle(colorVis, cv::Point((int)pt.x, (int)pt.y), 1, cv::Scalar(255, 0, 0), -1);
        }

        // cv::imshow("Binary Input", binary);
        cv::imshow("Width change" + frame_history.ns, colorVis);
        // cv::imshow("Distance Transform", distVis);
        cv::waitKey(1);
        skeleton_info_list.push_back(skel_info);
    }
    return skeleton_info_list;
}