#include "utils.h"


double round(double var, int precision)
{
    if (precision < 0)
        precision = 0;
    double value = (var >= 0) ? (int)(var * pow(10, precision) + .5)
                              : (int)(var * pow(10, precision) - .5);
    return value / pow(10, precision);
}

cv::Vec3b random_color()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 255);
    return cv::Vec3b(dis(gen), dis(gen), dis(gen));
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

std::pair<cv::Point3f, cv::Point3f> fitLineRANSAC(const std::vector<cv::Point3f> &points_3f_list)
{
    if (points_3f_list.size() < 2) {
        std::cerr << "Not enough points to fit a line." << std::endl;
        return std::make_pair(cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    }

    // Container for the output line
    cv::Vec6f line; // (vx, vy, vz, x0, y0, z0)

    // Fit line using RANSAC
    cv::fitLine(points_3f_list, line, cv::DIST_L2, 0, 0.01, 0.01);

    // Extract line components
    cv::Point3f point_on_line(line[3], line[4], line[5]); // A point on the line
    cv::Point3f direction(line[0], line[1], line[2]);     // Direction vector

    // Compute min/max projections along the fitted line
    float t_min = std::numeric_limits<float>::max();
    float t_max = std::numeric_limits<float>::lowest();

    for (const auto &p : points_3f_list) {
        float t = (p - point_on_line).dot(direction); // Projection onto the line
        t_min = std::min(t_min, t);
        t_max = std::max(t_max, t);
    }

    // std::cout << "Point on line: " << point_on_line << std::endl;
    // std::cout << "Direction: " << direction << std::endl;

    // Define line endpoints for visualization (extend along direction)
    float line_length = 5.0; // Adjust for visualization
    // cv::Point3f start_point = point_on_line - direction * line_length;
    // cv::Point3f end_point = point_on_line + direction * line_length;

    // Compute actual start and end points of the line segment
    cv::Point3f start_point = point_on_line + direction * t_min;
    cv::Point3f end_point = point_on_line + direction * t_max;

    return std::make_pair(start_point, end_point);

    // // Create Marker message
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "camera_depth_optical_frame"; // Change to your desired frame
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "fitted_line";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.action = visualization_msgs::Marker::ADD;

    // // Line width and color
    // marker.scale.x = 0.005; // Thickness of the line
    // marker.color.r = 1.0;  // Red color
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // marker.color.a = 1.0; // Fully opaque

    // // Add start and end points
    // geometry_msgs::Point p1, p2;
    // p1.x = start_point.x;
    // p1.y = start_point.y;
    // p1.z = start_point.z;

    // p2.x = end_point.x;
    // p2.y = end_point.y;
    // p2.z = end_point.z;

    // marker.points.push_back(p1);
    // marker.points.push_back(p2);

    // // Publish marker
    // marker_pub.publish(marker);
}

