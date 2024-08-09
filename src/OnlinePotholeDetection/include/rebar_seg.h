#pragma once
#define HEADER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <random>
#include <deque>
#include <limits>
#include <algorithm>
#include <visualization_msgs/Marker.h>

#define HISTORY 25

struct cluster_info
{
    cv::Mat img;
    int num_clusters;
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
};

struct Cluster
{
    int leftEdge;
    int rightEdge;
    int topEdge;
    int bottomEdge;
    cv::Point centroid;
};

struct AOI
{
    int id;                                              // AOI ID
    int matchCount;                                      // Number of times this AOI has been matched
    float confidence;                                    // Confidence of the AOI
    std::pair<cv::Point, cv::Point> closest_pixels_pair; // Closest pixels to the AOI
    std::pair<cv::Point, cv::Point> bounding_box;        // Bounding box defined by top-left and bottom-right points

    int area() const
    {
        int width = bounding_box.second.x - bounding_box.first.x;
        int height = bounding_box.second.y - bounding_box.first.y;
        return width * height;
    }

    // Method to check if a point is within the AOI
    bool contains(const cv::Point &point) const
    {
        return (point.x >= bounding_box.first.x && point.x <= bounding_box.second.x &&
                point.y >= bounding_box.first.y && point.y <= bounding_box.second.y);
    }
};

struct frame_AOI_info
{
    std::vector<AOI> aoiList;
    std::vector<int> nr_of_new_AOIs;
    std::string ns;

    // Method to add an AOI to the list
    void addAOI(const AOI &aoi)
    {
        // Check the length of the list
        // Keep the list length to N
        if (aoiList.size() >= HISTORY)
        {
            aoiList.erase(aoiList.begin());
        }
        aoiList.push_back(aoi);
    }

    // Loop through the AOI list and compare each element to every other element using IoU

    void calculateConfidence()
    {
        for (auto &aoi : aoiList)
        {
            aoi.confidence = (float)aoi.matchCount / HISTORY;
        }
    }
};

// struct Colors
// {
//     const float RED[4] = {1, 1, 0, 0};
//     const float GREEN[4] = {1, 0, 1, 0};
//     const float BLUE[4] = {1, 0, 0, 1};
//     const float YELLOW[4] = {1, 1, 1, 0};
//     const float ORANGE[4] = {1, 1, 0.5f, 0};
//     const float CYAN[4] = {1, 0, 1, 1};
//     const float MAGENTA[4] = {1, 1, 0, 1};
//     const float WHITE[4] = {1, 1, 1, 1};
//     const float BLACK[4] = {1, 0, 0, 0};
// };

std::pair<double, double> find_rotation(cv::Mat &image, bool debug_level);
cv::Mat rotate_image(const std::string &name, const cv::Mat &image, double angle, bool debug_level);
std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &image, int left_right_num, bool debug_level);
cluster_info cluster(const std::string &name, const cv::Mat &img, bool debug_level);
void detectInterruptions(frame_AOI_info &frame_history, const cv::Mat &lineImage, const std::string &lineType, double maxDistance, bool debug_level, bool show_clusters);
cv::Point3f pixel_to_camera(int u, int v, float Z);
std::vector<std::vector<cv::Point3f>> get_3d_coordinates(const cv::Mat &img, const cv::Mat &depth_image, const cv::Mat &labels, const cv::Mat &K_inv, double Z);
void deleteMarkers(ros::Publisher &pub);
void publish_ball(cv::Point3f &coord, float size, int ID, const std::string &ns, ros::Publisher &pub,
                                                          const std::vector<int> &color);