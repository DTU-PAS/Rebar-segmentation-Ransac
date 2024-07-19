#pragma once
#define HEADER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
cluster_info cluster_skeleton(const std::string &name, const cv::Mat &skeleton, bool debug_level);
std::tuple<std::vector<std::pair<cv::Point, cv::Point>>, std::vector<std::pair<cv::Point, cv::Point>>>
find_area_of_interest(const std::string &name, const cv::Mat &labels, int num_labels, const cv::Mat &gray_orig, const cv::Mat &img, bool debug_level);