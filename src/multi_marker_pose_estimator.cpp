#include "multi_marker_pose_estimator.h"
#include "pose_utils.h"
#include <iostream>
#include <cmath>

std::optional<marker_pair_t> MultiMarkerPoseEstimator::detectAndEstimatePair(cv::Mat& frame, cv::Mat& output_frame, const tag_pair_t& pair_config) {
    std::vector<std::pair<int, double>> tag_id_size_pairs = {
        {pair_config.tag1_id, pair_config.tag1_size},
        {pair_config.tag2_id, pair_config.tag2_size}
    };
    
    std::vector<apriltag_t> tags = detector.detect_multiple_apriltags(frame, output_frame, tag_id_size_pairs);

    apriltag_t* tag1 = nullptr;
    apriltag_t* tag2 = nullptr;

    for (auto& tag : tags) {
        if (tag.apriltag_id == pair_config.tag1_id) {
            tag1 = &tag;
        } else if (tag.apriltag_id == pair_config.tag2_id) {
            tag2 = &tag;
        }
    }

    if (!tag1 || !tag2) {
        std::cerr << "Could not find both tags." << std::endl;
        return std::nullopt;
    }

    Pose3D pose1 = detector.convertTo3DPose(tag1->pose);
    Pose3D pose2 = detector.convertTo3DPose(tag2->pose);

    Pose3D average_pose = calculateAveragePose(pose1, pose2);

    marker_pair_t pair = { *tag1, *tag2, average_pose };
    return pair;
}

Pose3D MultiMarkerPoseEstimator::calculateAveragePose(const Pose3D& pose1, const Pose3D& pose2) {
    Pose3D average_pose;

    average_pose.x = (pose1.x + pose2.x) / 2.0;
    average_pose.y = (pose1.y + pose2.y) / 2.0;
    average_pose.z = (pose1.z + pose2.z) / 2.0;

    average_pose.roll = (pose1.roll + pose2.roll) / 2.0;
    average_pose.pitch = (pose1.pitch + pose2.pitch) / 2.0;
    average_pose.yaw = (pose1.yaw + pose2.yaw) / 2.0;

    return average_pose;
}

bool MultiMarkerPoseEstimator::validateRelativePose(const Pose3D& pose1, const Pose3D& pose2, const tag_pair_t& pair_config, double threshold_percentage) {
    tf2::Transform transform1 = createTransform(pose1.x, pose1.y, pose1.z, pose1.roll, pose1.pitch, pose1.yaw);
    tf2::Transform transform2 = createTransform(pose2.x, pose2.y, pose2.z, pose2.roll, pose2.pitch, pose2.yaw);

    tf2::Transform relative_transform = transform1.inverse() * transform2;
    
    // Extract translation components
    tf2::Vector3 translation = relative_transform.getOrigin();
    double x = translation.x();
    double y = translation.y();
    double z = translation.z();

    double max_tag_size = std::max(pair_config.tag1_size, pair_config.tag2_size);

    double y_error = std::abs(y - pair_config.tag2_y_from_tag1);
    double z_error = std::abs(z - pair_config.tag2_z_from_tag1);
    
    double y_error_percentage = y_error / max_tag_size * 100.0;
    double z_error_percentage = z_error / max_tag_size * 100.0;

    std::cout << "Relative Pose Error: Y=" << y_error_percentage << "%, Z=" << z_error_percentage << "%" << std::endl;

    return (y_error_percentage <= threshold_percentage && z_error_percentage <= threshold_percentage);
}
