#pragma once
#define HEADER_H

#include <ransac_node.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <random>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#define HISTORY 15

extern bool aligned_depth;

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
    cv::Point centroid;
    cv::Point2f center;
    std::vector<cv::Point2f> midpoints;
};


#define HISTORY 15

struct AOI
{
    int id;                                              // AOI ID
    int matchCount;                                      // Number of times this AOI has been matched
    float confidence;                                    // Confidence of the AOI
    std::pair<cv::Point, cv::Point> closest_pixels_pair; // Closest pixels to the AOI
    std::pair<cv::Point, cv::Point> bounding_box;        // Bounding box defined by top-left and bottom-right points
    // std::vector<cv::Point2f> points;                       // Pixels within the AOI
    
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
        cv::Mat skeleton;
        std::vector<Cluster> clusters;
        
        // Method to add an AOI to the list
        void addAOI(const AOI &aoi)
        {
            // Check the length of the list
            // Keep the list length to N
            if (aoiList.size() >= HISTORY) {
                aoiList.erase(aoiList.begin());
            }
            aoiList.push_back(aoi);
        }
        
        // Loop through the AOI list and compare each element to every other element using IoU
        
        void calculateConfidence()
        {
            for (auto &aoi : aoiList) {
                aoi.confidence = (float)aoi.matchCount / HISTORY;
            }
        }
    };
    
    struct PointPair
    {
        cv::Point2f p1, p2;
        double distance;
        
        PointPair(const cv::Point2f &point1, const cv::Point2f &point2, double dist)
        : p1(point1), p2(point2), distance(dist) {}
        
        // Sort pairs by distance (ascending order)
        bool operator<(const PointPair &other) const
        {
            return distance < other.distance;
        }
    };
    
    struct SkelPoint
    {
        float x;
        float y;
        float width;

        // Constructor that matches the usage in push_back
        SkelPoint(float x_, float y_, float w_)
        : x(x_), y(y_), width(w_) {}
    };
    
    struct skeleton_info
    {
        cv::Mat binary;
        cv::Mat skeleton;
        std::vector<cv::Point> points;
    
    };
    
    std::pair<double, double> find_rotation(cv::Mat &image, bool debug_level);
    cv::Mat rotate_image(const std::string &name, const cv::Mat &image, double angle, bool debug_level);
    cv::Point rotate_point(const std::string &name, const cv::Point point, cv::Point2f center, double angle, bool debug_level);
    // std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &image, int left_right_num, bool debug_level);
    std::pair<cv::Mat, cv::Mat> split_horizontal_and_vertical(const cv::Mat &image, std::pair<double, double> angles, int kernelsize, bool debug_level);
    void detectInterruptions(frame_AOI_info &frame_history, const cv::Mat &lineImage, double maxDistance, int padding, bool debug_level, bool show_clusters);
double find_depth(const cv::Mat &depth_image, int u, int v);
std::vector<skeleton_info> detect_width(frame_AOI_info &frame_history, cv::Mat &binary);