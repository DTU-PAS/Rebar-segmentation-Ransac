#pragma once
#define HEADER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <AOITracker.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <random>

struct cluster_info
{   
    cv::Mat img;
    int num_clusters;
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
};


std::pair<double, double> find_rotation(cv::Mat &image, bool debug_level);
cv::Mat rotate_image(const std::string &name, const cv::Mat &image, double angle, bool debug_level);
std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &skeleton, int left_right_num, bool debug_level);
cv::Mat reconstruct_skeleton(const std::string &name, const cv::Mat &src, const cv::Mat &orig, int ksize, int iterations, int debug_level);
cv::Mat remove_small_blobs(const std::string &name, const cv::Mat &img, int min_blob_size, bool debug_level);
cluster_info cluster(const std::string &name, const cv::Mat &img, bool debug_level);
frame_AOI_info find_area_of_interest(const std::string &name, const cv::Mat &labels, int num_labels, const cv::Mat &gray_orig, const cv::Mat &img, bool debug_level);
std::vector<std::vector<cv::Point3f>> get_3d_coordinates(const cv::Mat &img, const cv::Mat &depth_image, const cv::Mat &labels, const cv::Mat &K_inv, double Z);