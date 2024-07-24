#ifndef AOI_TRACKER_H
#define AOI_TRACKER_H

#include <opencv2/opencv.hpp>
#include <deque>
#include <vector>

struct AOI
{
    std::pair < cv::Point, cv::Point > closest_pixels_pair; // Closest pixels to the AOI
    std::pair<cv::Point, cv::Point> bbox; // Bounding box defined by top-left and bottom-right points
    int id;                               // Unique identifier for each AOI
};

// Structure to store frame AOI information
struct frame_AOI_info
{
    std::vector<AOI> aoiList;
};

struct TrackedAOI
{
    int id;
    std::pair<cv::Point, cv::Point> bbox;
    float confidence;
};

class AOITracker
{
private:
    std::deque<frame_AOI_info> frames;
    size_t maxFrames;
    mutable int nextId; // Make nextId mutable to allow modification in const method

    // Helper method to calculate Intersection over Union (IoU) between two bounding boxes
    float IoU(const std::pair<cv::Point, cv::Point> &bbox1, const std::pair<cv::Point, cv::Point> &bbox2) const;

public:
    // Constructor to initialize AOI tracker with a maximum number of frames
    AOITracker();
    void initialize(int maxFrames);

    // Method to add a new frame's AOIs to the tracker
    void addFrame(const frame_AOI_info &newFrame);

    // Method to track AOIs between consecutive frames
    std::vector<TrackedAOI> trackAOIs(const frame_AOI_info &oldFrame, const frame_AOI_info &newFrame) const;

    // Method to get tracked AOIs for the current frame
    std::vector<TrackedAOI> getCurrentTrackedAOIs() const;
};

#endif // AOI_TRACKER_H
