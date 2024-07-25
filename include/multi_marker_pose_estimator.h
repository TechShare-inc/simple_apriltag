#ifndef MULTI_MARKER_POSE_ESTIMATOR_H
#define MULTI_MARKER_POSE_ESTIMATOR_H

#include "simple_tag.h"
#include "pose_utils.h"
#include <vector>
#include <optional>

typedef struct {
    std::pair<uint16_t, double> tag1_id_size;
    std::pair<uint16_t, double> tag2_id_size;
    double tag2_y_from_tag1; // Y座標：左方向
    double tag2_z_from_tag1; // Z座標：上方向
} tag_pair_t;

typedef struct {
    apriltag_t marker1;
    apriltag_t marker2;
    Pose3D average_pose;
} marker_pair_t;

class MultiMarkerPoseEstimator {
public:
    MultiMarkerPoseEstimator() = default;
    std::optional<marker_pair_t> detectAndEstimatePair(cv::Mat& frame, cv::Mat& output_frame, const tag_pair_t& pair_config);

    bool validateRelativePose(const Pose3D& pose1, const Pose3D& pose2, const tag_pair_t& pair_config, double threshold_percentage);

    DetectApriltag detector;

private:
    Pose3D calculateAveragePose(const Pose3D& pose1, const Pose3D& pose2);
};

#endif // MULTI_MARKER_POSE_ESTIMATOR_H
