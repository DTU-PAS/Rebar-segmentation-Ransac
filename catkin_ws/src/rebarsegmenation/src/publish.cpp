#include "publish.h"

cv::Point3f pixel_to_camera(cv::Mat K, int u, int v, float Z)
{
    cv::Mat K_inv = K.inv();
    cv::Mat pixel_coords = (cv::Mat_<double>(3, 1) << u, v, 1);
    cv::Mat camera_coords = K_inv * pixel_coords * Z;

    return cv::Point3f(camera_coords.at<double>(0, 0), camera_coords.at<double>(1, 0), camera_coords.at<double>(2, 0));
}

void deleteMarkers(ros::Publisher &pub)
{
    visualization_msgs::Marker marker;
    // marker.header.frame_id = "camera_color_optical_frame";
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.action = visualization_msgs::Marker::DELETEALL;
    pub.publish(marker);
}

void publish_ball(cv::Point3f &coord, float size, int ID, const std::string &ns, ros::Publisher &pub, const std::vector<int> &color)
{
    visualization_msgs::Marker marker;
    // marker.header.frame_id = "camera_color_optical_frame";
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.lifetime = ros::Duration(1);

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    marker.color.a = color[0];
    marker.color.r = color[1];
    marker.color.g = color[2];
    marker.color.b = color[3];

    marker.id = ID;
    marker.pose.position.x = coord.x;
    marker.pose.position.y = coord.y;
    marker.pose.position.z = coord.z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    pub.publish(marker);
}

void publish_line(frame_AOI_info &frame_history, std::pair<cv::Point3f, cv::Point3f> line_points, cv::Scalar color, ros::Publisher &pub)
{
    visualization_msgs::Marker marker;
    // marker.header.frame_id = "camera_color_optical_frame";
    marker.header.frame_id = "camera_depth_optical_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = frame_history.ns;
    marker.lifetime = ros::Duration(1);

    marker.type = visualization_msgs::Marker::LINE_STRIP;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.001;

    marker.color.a = 1;
    marker.color.r = color[2] / 255.0;
    marker.color.g = color[1] / 255.0;
    marker.color.b = color[0] / 255.0;

    marker.id = 0;

    geometry_msgs::Point p1, p2;
    p1.x = line_points.first.x;
    p1.y = line_points.first.y;
    p1.z = line_points.first.z;

    p2.x = line_points.second.x;
    p2.y = line_points.second.y;
    p2.z = line_points.second.z;

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    pub.publish(marker);
}