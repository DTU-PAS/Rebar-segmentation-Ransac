#include "AOITracker.h"

// Constructor
// AOITracker::AOITracker(size_t maxFrames) : maxFrames(maxFrames), nextId(1) {}
AOITracker::AOITracker() : maxFrames(0), nextId(1) {}

void AOITracker::initialize(int maxFrames)
{
    this->maxFrames = maxFrames;
}

float AOITracker::IoU(const std::pair<cv::Point, cv::Point> &bbox1, const std::pair<cv::Point, cv::Point> &bbox2) const
{
    int x1 = std::max(bbox1.first.x, bbox2.first.x);
    int y1 = std::max(bbox1.first.y, bbox2.first.y);
    int x2 = std::min(bbox1.second.x, bbox2.second.x);
    int y2 = std::min(bbox1.second.y, bbox2.second.y);

    int width = std::max(0, x2 - x1);
    int height = std::max(0, y2 - y1);

    int intersectionArea = width * height;

    int bbox1Area = (bbox1.second.x - bbox1.first.x) * (bbox1.second.y - bbox1.first.y);
    int bbox2Area = (bbox2.second.x - bbox2.first.x) * (bbox2.second.y - bbox2.first.y);

    int unionArea = bbox1Area + bbox2Area - intersectionArea;

    // // Debug prints
    // std::cout << "BBox1: (" << bbox1.first.x << ", " << bbox1.first.y << ") - ("
    //           << bbox1.second.x << ", " << bbox1.second.y << ")\n";
    // std::cout << "BBox2: (" << bbox2.first.x << ", " << bbox2.first.y << ") - ("
    //           << bbox2.second.x << ", " << bbox2.second.y << ")\n";
    // std::cout << "Intersection: Width=" << width << ", Height=" << height << ", Area=" << intersectionArea << "\n";
    // std::cout << "BBox1 Area=" << bbox1Area << ", BBox2 Area=" << bbox2Area << "\n";
    // std::cout << "Union Area=" << unionArea << "\n";

    if (unionArea == 0)
    {
        return 0;
    }

    return static_cast<float>(intersectionArea) / unionArea;
}

void AOITracker::addFrame(const frame_AOI_info &newFrame)
{
    if (frames.size() == maxFrames)
    {
        frames.pop_front();
    }
    frames.push_back(newFrame);
}

std::vector<TrackedAOI> AOITracker::trackAOIs(const frame_AOI_info &oldFrame, const frame_AOI_info &newFrame) const
{
    const float IoUThreshold = 0.5;
    std::vector<TrackedAOI> trackedAOIs;

    for (const auto &newBbox : newFrame.bboxs)
    {
        bool matched = false;

        for (const auto &oldBbox : oldFrame.bboxs)
        {
            std::cout << "IoU: " << IoU(newBbox, oldBbox) << "\n";
            if (IoU(newBbox, oldBbox) > IoUThreshold)
            {
                matched = true;
                trackedAOIs.push_back({nextId++, newBbox, 1.0f});
                break;
            }
        }

        if (!matched)
        {
            trackedAOIs.push_back({nextId++, newBbox, 1.0f});
        }
    }

    return trackedAOIs;
}

std::vector<TrackedAOI> AOITracker::getCurrentTrackedAOIs() const
{
    if (frames.size() < 2)
    {
        return {};
    }

    return trackAOIs(frames[frames.size() - 2], frames.back());
}
