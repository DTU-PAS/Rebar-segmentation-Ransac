#include <rebarsegmentation/rebar_seg.h>


std::pair<double, double> find_rotation(cv::Mat &image)
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

    return std::make_pair(average_vertical_angle, average_horizontal_angle);
}

cv::Mat rotate_image(const std::string &name, const cv::Mat &image, double angle)
{
    int h = image.rows;
    int w = image.cols;

    cv::Point2f center = cv::Point2f(w / 2.0, h / 2.0);

    // Get the rotation matrix
    cv::Mat M = cv::getRotationMatrix2D(center, angle, 1);

    // Perform the affine transformation (rotation)
    cv::Mat rotated;
    cv::warpAffine(image, rotated, M, cv::Size(w, h));

    return rotated;
}

// Rotate a 2d point around the origin
cv::Point rotate_point(const std::string &name, const cv::Point point, cv::Point2f center, double angle)
{
    // Get the rotation matrix
    cv::Mat M = cv::getRotationMatrix2D(center, angle, 1);

    // Rotate the point 
    double x = point.x * M.at<double>(0, 0) + point.y * M.at<double>(0, 1) + M.at<double>(0, 2);
    double y = point.x * M.at<double>(1, 0) + point.y * M.at<double>(1, 1) + M.at<double>(1, 2);

    return cv::Point(x, y);
}

std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &image, std::pair<double, double> angles, int kernelsize)
{

    // Step 2: Detect Horizontal Lines
    cv::Mat horizontalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelsize, 1));
    cv::Mat horizontalLines;
    cv::morphologyEx(image, horizontalLines, cv::MORPH_OPEN, horizontalKernel);

    // Step 3: Detect Vertical Lines
    cv::Mat verticalKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, kernelsize));
    cv::Mat verticalLines;
    cv::morphologyEx(image, verticalLines, cv::MORPH_OPEN, verticalKernel);

    return std::make_pair(verticalLines, horizontalLines);
}

cv::Vec3b random_color()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 255);
    return cv::Vec3b(dis(gen), dis(gen), dis(gen));
}

cluster_info cluster(const std::string &name, const cv::Mat &img)
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

void computeAOI(frame_AOI_info &frame_history, AOI current_aoi, std::vector<cv::Point> points, int highestId = 0)
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
                aoi.points = points;
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

void detectInterruptions(frame_AOI_info &frame_history, const cv::Mat &lineImage, double maxDistance, int paddding)
{
    cluster_info result = cluster("FAOI: " + frame_history.ns, lineImage);

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
            if (frame_history.ns == "Horizontal")
            {
                return std::abs(a.rightEdge - b.leftEdge);
            }
            else if (frame_history.ns == "Vertical")
            {
                return std::abs(a.bottomEdge - b.topEdge);
            }
            return std::numeric_limits<double>::max();
        };

        std::vector<bool> pairedLeft(clusters.size(), false);
        std::vector<bool> pairedRight(clusters.size(), false);

        // Pair clusters based on their proximity and draw lines between closest pairs
        while (std::count(pairedLeft.begin(), pairedLeft.end(), false) > 1 || std::count(pairedRight.begin(), pairedRight.end(), false) > 1)
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
        
            std::vector<cv::Point> points = {pt1, pt2, pt3, pt4};

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
                if (frame_history.ns == "Horizontal")
                {
                    // The are most likely two rebars in the picture.
                    // Without this if statement the program would draw a line between the two rebars
                    if (!(euclideanDistance(pt1, pt2) > 100))
                    {
                        int x1 = clamp(pt1.x - paddding, 0, WIDTH - 1);
                        int y1 = clamp(pt1.y - paddding, 0, HEIGHT - 1);
                        int x2 = clamp(pt2.x + paddding, 0, WIDTH - 1);
                        int y2 = clamp(pt2.y + paddding, 0, HEIGHT - 1);

                        AOI current_aoi;
                        current_aoi.closest_pixels_pair = std::make_pair(pt1, pt2);
                        current_aoi.bounding_box = std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2));
                        computeAOI(frame_history, current_aoi, points, highestId);
                    }
                }
                else if (frame_history.ns == "Vertical")
                {
                    if (!(euclideanDistance(pt3, pt4) > 100))
                    {

                        int x1 = clamp(pt3.x - paddding, 0, WIDTH - 1);
                        int y1 = clamp(pt3.y - paddding, 0, HEIGHT - 1);
                        int x2 = clamp(pt4.x + paddding, 0, WIDTH - 1);
                        int y2 = clamp(pt4.y + paddding, 0, HEIGHT - 1);

                        AOI current_aoi;
                        current_aoi.closest_pixels_pair = std::make_pair(pt3, pt4);
                        current_aoi.bounding_box = std::make_pair(cv::Point(x1, y1), cv::Point(x2, y2));
                        computeAOI(frame_history, current_aoi, points, highestId);
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
    }
    else
    {
    }
}

cv::Point3f pixel_to_camera(cv::Mat K, int u, int v, float Z)
{
    cv::Mat K_inv = K.inv();
    cv::Mat pixel_coords = (cv::Mat_<double>(3, 1) << u, v, 1);
    cv::Mat camera_coords = K_inv * pixel_coords * Z;


    return cv::Point3f(camera_coords.at<double>(0, 0), camera_coords.at<double>(1, 0), camera_coords.at<double>(2, 0));
}

void deleteMarkers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub)
{
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "camera_color_optical_frame";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    pub->publish(marker);
}

void publish_ball(cv::Point3f &coord, float size, int ID, const std::string &ns, ros::Publisher &pub, const std::vector<int> &color)
{
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "camera_color_optical_frame";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.lifetime = rclcpp::Duration(1s);

    marker.type = visualization_msgs::msg::Marker::SPHERE;

    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    marker.color.a = color[0] / 255.0;
    marker.color.r = color[1] / 255.0;
    marker.color.g = color[2] / 255.0;
    marker.color.b = color[3] / 255.0;

    marker.id = ID;
    marker.pose.position.x = coord.x;
    marker.pose.position.y = coord.y;
    marker.pose.position.z = coord.z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    pub->publish(marker);
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