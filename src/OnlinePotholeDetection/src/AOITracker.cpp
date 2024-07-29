#include "AOITracker.h"

// // Constructor
// // AOITracker::AOITracker(size_t maxFrames) : maxFrames(maxFrames), nextId(1) {}
// AOITracker::AOITracker() : maxFrames(0), nextId(1) {}

// void AOITracker::initialize(int maxFrames)
// {
//     this->maxFrames = maxFrames;
// }

// float AOITracker::IoU(const std::pair<cv::Point, cv::Point> &bbox1, const std::pair<cv::Point, cv::Point> &bbox2) const
// {
//     int x1 = std::max(bbox1.first.x, bbox2.first.x);
//     int y1 = std::max(bbox1.first.y, bbox2.first.y);
//     int x2 = std::min(bbox1.second.x, bbox2.second.x);
//     int y2 = std::min(bbox1.second.y, bbox2.second.y);

//     int width = std::max(0, x2 - x1);
//     int height = std::max(0, y2 - y1);

//     int intersectionArea = width * height;

//     int bbox1Area = (bbox1.second.x - bbox1.first.x) * (bbox1.second.y - bbox1.first.y);
//     int bbox2Area = (bbox2.second.x - bbox2.first.x) * (bbox2.second.y - bbox2.first.y);

//     int unionArea = bbox1Area + bbox2Area - intersectionArea;

//     // // Debug prints
//     // std::cout << "BBox1: (" << bbox1.first.x << ", " << bbox1.first.y << ") - ("
//     //           << bbox1.second.x << ", " << bbox1.second.y << ")\n";
//     // std::cout << "BBox2: (" << bbox2.first.x << ", " << bbox2.first.y << ") - ("
//     //           << bbox2.second.x << ", " << bbox2.second.y << ")\n";
//     // std::cout << "Intersection: Width=" << width << ", Height=" << height << ", Area=" << intersectionArea << "\n";
//     // std::cout << "BBox1 Area=" << bbox1Area << ", BBox2 Area=" << bbox2Area << "\n";
//     // std::cout << "Union Area=" << unionArea << "\n";

//     if (unionArea == 0)
//     {
//         return 0;
//     }

//     return static_cast<float>(intersectionArea) / unionArea;
// }

// void AOITracker::addFrame(const frame_AOI_info &newFrame)
// {
//     if (frames.size() == maxFrames)
//     {
//         frames.pop_front();
//     }
//     frames.push_back(newFrame);
// }

// std::vector<TrackedAOI> AOITracker::trackAOIs(const frame_AOI_info &oldFrame, const frame_AOI_info &newFrame) const
// {
//     const float IoUThreshold = 0.5;
//     std::vector<TrackedAOI> trackedAOIs;

//     std::unordered_map<int, TrackedAOI> oldTrackedAOIs;
//     for (const auto &oldAOI : oldFrame.aoiList)
//     {
//         oldTrackedAOIs[oldAOI.id] = {oldAOI.id, oldAOI.bbox, 0.0f, {}};
//     }

//     for (const auto &newBbox : newFrame.bboxs)
//     {
//         bool matched = false;

//         for (const auto &oldBbox : oldFrame.bboxs)
//         {
//             std::cout << "IoU: " << IoU(newBbox, oldBbox) << "\n";
//             if (IoU(newBbox, oldBbox) > IoUThreshold)
//             {
//                 matched = true;
//                 trackedAOIs.push_back({nextId++, newBbox, 1.0f});
//                 break;
//             }
//         }

//         if (!matched)
//         {
//             trackedAOIs.push_back({nextId++, newBbox, 1.0f});
//         }
//     }

//     return trackedAOIs;
// }

// std::vector<TrackedAOI> AOITracker::getCurrentTrackedAOIs() const
// {
//     if (frames.size() < 2)
//     {
//         return {};
//     }

//     return trackAOIs(frames[frames.size() - 2], frames.back());
// }

// Function to compute IoU (Intersection over Union) between two AOIs
float computeIoU(const AOI &a, const AOI &b)
{
    int x1 = std::max(a.bounding_box.first.x, b.bounding_box.first.x);
    int y1 = std::max(a.bounding_box.first.y, b.bounding_box.first.y);
    int x2 = std::min(a.bounding_box.second.x, b.bounding_box.second.x);
    int y2 = std::min(a.bounding_box.second.y, b.bounding_box.second.y);

    int intersectionArea = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int unionArea = a.area() + b.area() - intersectionArea;

    return static_cast<float>(intersectionArea) / unionArea;
}

void matchAOIsAndComputeConfidence(const std::string &name,
                                   std::vector<frame_AOI_info> &frames,
                                   int N,
                                   float IoUThreshold,
                                   std::unordered_map<int, AOIHistory> &aoiHistories)
{

    int nextAOIId = 0;

    // Iterate over all frames
    for (int i = 0; i < frames.size(); ++i)
    {
        for (auto &aoi : frames[i].aoiList)
        {
            bool matched = false;

            // Check the AOI against previous N frames
            for (int j = std::max(0, i - N); j < i; ++j)
            {
                for (const auto &prevAoi : frames[j].aoiList)
                {
                    if (computeIoU(aoi, prevAoi) >= IoUThreshold)
                    {
                        aoi.id = prevAoi.id;
                        aoiHistories[aoi.id].matches++;
                        matched = true;
                        break;
                    }
                }
                if (matched)
                    break;
            }

            if (!matched)
            {
                aoi.id = nextAOIId++;
                aoiHistories[aoi.id] = {aoi.id, 0, 0};
            }

            aoiHistories[aoi.id].frames_tracked++;
        }
    }

    // Calculate confidence scores and store them
    for (auto &frame : frames)
    {
        for (auto &aoi : frame.aoiList)
        {
            auto &history = aoiHistories[aoi.id];
            aoiHistories[aoi.id].frames_tracked++;
            history.frames_tracked++;
            history.matches = std::min(history.frames_tracked, N);
        }
    }
}

void drawAOIs(cv::Mat &image, const frame_AOI_info &frame, const std::unordered_map<int, AOIHistory> &aoiHistories, int N)
{
    
    for (const auto &aoi : frame.aoiList)
    {
        const auto &history = aoiHistories.at(aoi.id);
        float confidence = static_cast<float>(history.matches) / std::min(N, history.frames_tracked);

        // Draw bounding box
        cv::rectangle(image, aoi.bounding_box.first, aoi.bounding_box.second, cv::Scalar(0, 255, 0), 2);

        // Annotate with ID and confidence score
        std::string label = "ID: " + std::to_string(aoi.id) + " Conf: " + std::to_string(confidence);
        int baseLine = 0;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::rectangle(image, cv::Point(aoi.bounding_box.first.x, aoi.bounding_box.first.y - labelSize.height - baseLine),
                      cv::Point(aoi.bounding_box.first.x + labelSize.width, aoi.bounding_box.first.y),
                      cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(image, label, cv::Point(aoi.bounding_box.first.x, aoi.bounding_box.first.y - baseLine),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}