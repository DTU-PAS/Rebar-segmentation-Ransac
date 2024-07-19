#pragma once
#define HEADER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void segment_skeleton(const cv::Mat &skel_img, std::vector<std::vector<cv::Point>> &objects);

void segment_sort(const cv::Mat &skel_img, const std::vector<std::vector<cv::Point>> &objects, std::vector<std::vector<cv::Point>> &secondary_objects);

cv::Mat image_subtract(const cv::Mat &img1, const cv::Mat &img2);

cv::Mat iterative_prune(const cv::Mat &img, int iterations);

std::tuple<cv::Mat, cv::Mat, std::vector<std::vector<cv::Point>>> prune(const cv::Mat &skel_img, int size = 0, const cv::Mat &mask = cv::Mat());