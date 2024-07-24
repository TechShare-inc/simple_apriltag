#include "multi_marker_pose_estimator.h"
#include <iostream>
#include <cmath>

std::optional<MarkerPair> MultiMarkerPoseEstimator::detectAndEstimatePair(cv::Mat& frame, cv::Mat& output_frame) {
    std::vector<apriltag_t> tags = detector.detect_multiple_apriltags(frame, output_frame);

    if (tags.size() < 2) {
        std::cerr << "Less than 2 tags detected." << std::endl;
        return std::nullopt;
    }

    apriltag_t tag1 = tags[0];
    apriltag_t tag2 = tags[1];

    Pose3D pose1 = detector.convertTo3DPose(tag1.pose);
    Pose3D pose2 = detector.convertTo3DPose(tag2.pose);

    Pose3D average_pose = calculateAveragePose(pose1, pose2);

    MarkerPair pair = { tag1, tag2, average_pose };
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
