#pragma once
#define HEADER_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
// #include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <image_processing.h>

double round(double var, int precision = 2);
cv::Vec3b random_color();
int clamp(int value, int min_val, int max_val);
double euclideanDistance(cv::Point2d pt1, cv::Point2d pt2);
float computeIoU(const AOI &a, const AOI &b);
std::pair<cv::Point3f, cv::Point3f> fitLineRANSAC(const std::vector<cv::Point3f> &points_3f_list);