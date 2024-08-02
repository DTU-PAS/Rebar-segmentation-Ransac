#ifndef AOI_TRACKER_H
#define AOI_TRACKER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <vector>

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

    // Method to serialize AOI (for saving/transmitting)
    std::string serialize() const
    {
        return "Closes Pixels if ID: " + std::to_string(id) + "\n" +
               std::to_string(closest_pixels_pair.first.x) + "," +
               std::to_string(closest_pixels_pair.first.y) + "," +
               std::to_string(closest_pixels_pair.second.x) + "," +
               std::to_string(closest_pixels_pair.second.y) + "\n" +
               "BBox ID: " + std::to_string(id) + "\n" +
               std::to_string(bounding_box.first.x) + "," +
               std::to_string(bounding_box.first.y) + "," +
               std::to_string(bounding_box.second.x) + "," +
               std::to_string(bounding_box.second.y);
    }
};

struct frame_AOI_info
{
    std::vector<AOI> aoiList;
    std::vector nr_of_new_AOIs;

    // Method to add an AOI to the list
    void addAOI(const AOI &aoi)
    {
        // Check the length of the list
        // Keep the list length to N
        if (aoiList.size() >= 10)
        {
            aoiList.erase(aoiList.begin());
        }
        aoiList.push_back(aoi);
    }

    void updateAOI(const AOI &aoi)
    {
        for (auto &aoi_ : aoiList)
        {
            if (aoi_.id == aoi.id)
            {
                aoi_ = aoi;
                return;
            }
        }
    }

    // Method to serialize frame AOI info
    std::string serialize() const
    {
        std::string serialized = "";
        for (const auto &aoi : aoiList)
        {
            if (!serialized.empty())
            {
                serialized += ";";
            }
            serialized += aoi.serialize();
        }
        return serialized;
    }
};

struct AOIHistory
{
    frame_AOI_info frames_tracked;
};

float computeIoU(const AOI &a, const AOI &b);

void matchAOIsAndComputeConfidence(const std::string &name,
                                   std::vector<frame_AOI_info> &frames,
                                   int N,
                                   float IoUThreshold);

void drawAOIs(cv::Mat &image, const frame_AOI_info &frame, int N);

// struct TrackedAOI
// {
//     int id;
//     std::pair<cv::Point, cv::Point> bounding_box;
//     int matchCount;
//     float confidence;
// };

// class AOITracker
// {
// private:
//     std::deque<frame_AOI_info> frames;
//     size_t maxFrames;
//     mutable int nextId; // Make nextId mutable to allow modification in const method

//     // Helper method to calculate Intersection over Union (IoU) between two bounding boxes
//     float IoU(const std::pair<cv::Point, cv::Point> &bbox1, const std::pair<cv::Point, cv::Point> &bbox2) const;

// public:
//     // Constructor to initialize AOI tracker with a maximum number of frames
//     AOITracker();
//     void initialize(int maxFrames);

//     // Method to add a new frame's AOIs to the tracker
//     void addFrame(const frame_AOI_info &newFrame);

//     // Method to track AOIs between consecutive frames
//     std::vector<TrackedAOI> trackAOIs(const frame_AOI_info &oldFrame, const frame_AOI_info &newFrame) const;

//     // Method to get tracked AOIs for the current frame
//     std::vector<TrackedAOI> getCurrentTrackedAOIs() const;
// };

#endif // AOI_TRACKER_H
