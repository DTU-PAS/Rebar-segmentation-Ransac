#pragma once
#define HEADER_H

#include <image_processing.h>
#include <ransac_node.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

struct Colors
{
    const float RED[4] = {1, 1, 0, 0};
    const float GREEN[4] = {1, 0, 1, 0};
    const float BLUE[4] = {1, 0, 0, 1};
    const float YELLOW[4] = {1, 1, 1, 0};
    const float ORANGE[4] = {1, 1, 0.5f, 0};
    const float CYAN[4] = {1, 0, 1, 1};
    const float MAGENTA[4] = {1, 1, 0, 1};
    const float WHITE[4] = {1, 1, 1, 1};
    const float BLACK[4] = {1, 0, 0, 0};
};

cv::Point3f pixel_to_camera(cv::Mat K, int u, int v, float Z);
void deleteMarkers(ros::Publisher &pub);
void publish_ball(cv::Point3f &coord, float size, int ID, const std::string &ns, ros::Publisher &pub, const std::vector<int> &color);
void publish_AOI_to_3d(frame_AOI_info frame_history);
void publish_centerline_to_3d(frame_AOI_info frame_history);
void publish_line(frame_AOI_info &frame_history, std::pair<cv::Point3f, cv::Point3f> line_points, cv::Scalar color, ros::Publisher &pub);