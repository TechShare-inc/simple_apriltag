#ifndef MULTI_MARKER_POSE_ESTIMATOR_H
#define MULTI_MARKER_POSE_ESTIMATOR_H

#include "simple_tag.h"
#include "pose_utils.h"
#include <vector>
#include <optional>
#include <memory>

struct tag_info_t {
    uint16_t id;
    uint8_t marker_flag;  // 0: 検出失敗または統合失敗, 1: 検出成功または統合成功
    double size;
    Pose3D pose;

    bool operator==(const tag_info_t& other) const {
        return id == other.id;
    }
};

struct tag_offset_t {
    double tag2_y_from_tag1; // Y座標：左方向
    double tag2_z_from_tag1; // Z座標：上方向
};

struct tag_node_t {
    std::optional<tag_info_t> tag_info;
    std::optional<tag_offset_t> tag_offset;
    std::unique_ptr<tag_node_t> left_child;
    std::unique_ptr<tag_node_t> right_child;
};

class MultiMarkerPoseEstimator {
public:
    MultiMarkerPoseEstimator() = default;
    tag_info_t detectAndEstimate(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root);
    DetectApriltag detector;

private:
    std::vector<tag_info_t> collectTagsAndDetect(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root);
    tag_info_t processNode(const tag_node_t& node, std::vector<tag_info_t>& tag_info_list);
    bool validateAndEstimatePair(tag_info_t& combined_tag, const tag_info_t& tag1, const tag_info_t& tag2, const tag_offset_t& offset, double threshold_percentage);
    Pose3D calculateAveragePose(const Pose3D& pose1, const Pose3D& pose2);
};

#endif // MULTI_MARKER_POSE_ESTIMATOR_H
