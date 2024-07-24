#ifndef MULTI_MARKER_POSE_ESTIMATOR_H
#define MULTI_MARKER_POSE_ESTIMATOR_H

#include "simple_tag.h"
#include <vector>
#include <optional>

struct MarkerPair {
    apriltag_t marker1;
    apriltag_t marker2;
    Pose3D average_pose;
};

class MultiMarkerPoseEstimator {
public:
    MultiMarkerPoseEstimator() = default;
    std::optional<MarkerPair> detectAndEstimatePair(cv::Mat& frame, cv::Mat& output_frame);

    DetectApriltag detector;

private:
    Pose3D calculateAveragePose(const Pose3D& pose1, const Pose3D& pose2);
};

#endif // MULTI_MARKER_POSE_ESTIMATOR_H
