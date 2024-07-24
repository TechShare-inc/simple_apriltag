#include <iostream>
#include <opencv2/opencv.hpp>
#include "multi_marker_pose_estimator.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return -1;
    }
    std::string image_path = argv[1];

    cam_info_t cam_info = {826.1, 826.1, 640, 360};
    double THRESHOLD_PERCENTAGE = 10.0;

    MultiMarkerPoseEstimator pose_estimator;
    pose_estimator.detector.setCamInfo(cam_info);

    cv::Mat frame = cv::imread(image_path);
    if (frame.empty()) {
        std::cerr << "Cannot open image: " << image_path << std::endl;
        return -1;
    }

    cv::Mat output_frame;
    frame.copyTo(output_frame); // Ensure output_frame is initialized properly

    tag_pair_t pair_config = {301, 302, 0.15, 0.0, 0.15, 0.2};  // タグID 301と302のペアの設定、タグサイズを追加

    auto marker_pair_opt = pose_estimator.detectAndEstimatePair(frame, output_frame, pair_config);
    if (marker_pair_opt) {
        const marker_pair_t& marker_pair = marker_pair_opt.value();
        std::cout << "Detected Marker 1 ID: " << marker_pair.marker1.apriltag_id << std::endl;
        Pose3D pose1 = pose_estimator.detector.convertTo3DPose(marker_pair.marker1.pose);
        std::cout << "Marker 1 Pose: x=" << pose1.x
                  << ", y=" << pose1.y
                  << ", z=" << pose1.z
                  << ", roll=" << pose1.roll
                  << ", pitch=" << pose1.pitch
                  << ", yaw=" << pose1.yaw << std::endl;

        std::cout << "Detected Marker 2 ID: " << marker_pair.marker2.apriltag_id << std::endl;
        Pose3D pose2 = pose_estimator.detector.convertTo3DPose(marker_pair.marker2.pose);
        std::cout << "Marker 2 Pose: x=" << pose2.x
                  << ", y=" << pose2.y
                  << ", z=" << pose2.z
                  << ", roll=" << pose2.roll
                  << ", pitch=" << pose2.pitch
                  << ", yaw=" << pose2.yaw << std::endl;

        std::cout << "Average Pose: x=" << marker_pair.average_pose.x
                  << ", y=" << marker_pair.average_pose.y
                  << ", z=" << marker_pair.average_pose.z
                  << ", roll=" << marker_pair.average_pose.roll
                  << ", pitch=" << marker_pair.average_pose.pitch
                  << ", yaw=" << marker_pair.average_pose.yaw << std::endl;

        bool is_valid = pose_estimator.validateRelativePose(pose1, pose2, pair_config, THRESHOLD_PERCENTAGE);
        if (is_valid) {
            std::cout << "The relative pose is valid." << std::endl;
        } else {
            std::cerr << "The relative pose is not valid." << std::endl;
        }
    } else {
        std::cerr << "Could not detect a pair of markers." << std::endl;
    }

    if (!output_frame.empty()) {
        cv::imshow("Detected Markers", output_frame);
        cv::waitKey(0);
    } else {
        std::cerr << "Output frame is empty, cannot display." << std::endl;
    }

    return 0;
}
