#include "multi_marker_pose_estimator.h"
#include "pose_utils.h"
#include <iostream>
#include <cmath>
#include <algorithm> // std::remove

tag_info_t MultiMarkerPoseEstimator::detectAndEstimate(cv::Mat& frame, cv::Mat& output_frame, const tag_tournament_node_t& tournament_root) {
    std::vector<std::pair<uint16_t, double>> tag_id_size_pairs;

    // トーナメントノードを走査し、一番端（子を持たないノード）のタグIDとサイズを収集
    std::function<void(const tag_tournament_node_t&)> collect_tags = [&](const tag_tournament_node_t& node) {
        if (!node.left_child && !node.right_child) {
            tag_id_size_pairs.push_back(node.pair_config.tag1_id_size);
            tag_id_size_pairs.push_back(node.pair_config.tag2_id_size);
        } else {
            if (node.left_child) collect_tags(*node.left_child);
            if (node.right_child) collect_tags(*node.right_child);
        }
    };
    collect_tags(tournament_root);

    std::vector<apriltag_t> tags = detector.detect_multiple_apriltags(frame, output_frame, tag_id_size_pairs);
    std::vector<tag_info_t> tag_info_list;

    for (auto& tag : tags) {
        tag_info_t tag_info = { tag.apriltag_id, 1, tag.size, detector.convertTo3DPose(tag.pose) };
        tag_info_list.push_back(tag_info);
    }

    // デバッグ出力: 初期のタグ情報リスト
    std::cout << "Initial tag_info_list:" << std::endl;
    for (const auto& tag_info : tag_info_list) {
        std::cout << "Tag ID: " << tag_info.id << ", Size: " << tag_info.size << ", Pose: (" << tag_info.pose.x << ", " << tag_info.pose.y << ", " << tag_info.pose.z << ")" << std::endl;
    }

    // トーナメント形式でペアを統合
    std::function<bool(const tag_tournament_node_t&, tag_info_t&)> process_node = [&](const tag_tournament_node_t& node, tag_info_t& combined_tag) {
        // まず子ノードを持つかどうかを検証する
        if (node.left_child) {
            tag_info_t left_combined = {0, 0, 0.0, {}};
            if (process_node(*node.left_child, left_combined) && left_combined.marker_flag == 1) {
                tag_info_list.push_back(left_combined);
            }
        }

        if (node.right_child) {
            tag_info_t right_combined = {0, 0, 0.0, {}};
            if (process_node(*node.right_child, right_combined) && right_combined.marker_flag == 1) {
                tag_info_list.push_back(right_combined);
            }
        }

        // 子ノードを持たない場合、タグIDを探す
        if (!node.left_child && !node.right_child) {
            auto tag1_it = std::find_if(tag_info_list.begin(), tag_info_list.end(), [&](const tag_info_t& tag) { return tag.id == node.pair_config.tag1_id_size.first; });
            auto tag2_it = std::find_if(tag_info_list.begin(), tag_info_list.end(), [&](const tag_info_t& tag) { return tag.id == node.pair_config.tag2_id_size.first; });

            if (tag1_it != tag_info_list.end() && tag2_it != tag_info_list.end()) {
                if (validateAndEstimatePair(combined_tag, *tag1_it, *tag2_it, node.pair_config, 10.0)) {
                    // 統合された元のタグをリストから削除
                    tag_info_list.erase(tag1_it);
                    tag_info_list.erase(std::remove(tag_info_list.begin(), tag_info_list.end(), *tag2_it), tag_info_list.end());
                    tag_info_list.push_back(combined_tag);
                    combined_tag.marker_flag = 1;
                    return true;
                }
            }
        }

        combined_tag.marker_flag = 0;
        return false;
    };

    tag_info_t final_combined_tag = {0, 0, 0.0, {}};
    process_node(tournament_root, final_combined_tag);

    // デバッグ出力: 統合後のタグ情報リスト
    std::cout << "Final tag_info_list:" << std::endl;
    for (const auto& tag_info : tag_info_list) {
        std::cout << "Tag ID: " << tag_info.id << ", Size: " << tag_info.size << ", Pose: (" << tag_info.pose.x << ", " << tag_info.pose.y << ", " << tag_info.pose.z << "), Flag: " << static_cast<int>(tag_info.marker_flag) << std::endl;
    }

    return final_combined_tag;
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

bool MultiMarkerPoseEstimator::validateAndEstimatePair(tag_info_t& combined_tag, const tag_info_t& tag1, const tag_info_t& tag2, const tag_pair_config_t& pair_config, double threshold_percentage) {
    tf2::Transform transform1 = createTransform(tag1.pose.x, tag1.pose.y, tag1.pose.z, tag1.pose.roll, tag1.pose.pitch, tag1.pose.yaw);
    tf2::Transform transform2 = createTransform(tag2.pose.x, tag2.pose.y, tag2.pose.z, tag2.pose.roll, tag2.pose.pitch, tag2.pose.yaw);

    tf2::Transform relative_transform = transform1.inverse() * transform2;

    // Extract translation components
    tf2::Vector3 translation = relative_transform.getOrigin();
    double y = translation.y();
    double z = translation.z();

    double max_tag_size = std::max(pair_config.tag1_id_size.second, pair_config.tag2_id_size.second);

    double y_error = std::abs(y - pair_config.tag2_y_from_tag1);
    double z_error = std::abs(z - pair_config.tag2_z_from_tag1);

    double y_error_percentage = y_error / max_tag_size * 100.0;
    double z_error_percentage = z_error / max_tag_size * 100.0;

    std::cout << "Relative Pose Error: Y=" << y_error_percentage << "%, Z=" << z_error_percentage << "%" << std::endl;

    if (y_error_percentage <= threshold_percentage && z_error_percentage <= threshold_percentage) {
        combined_tag.id = tag1.id + tag2.id;
        combined_tag.size = tag1.size + tag2.size;
        combined_tag.pose = calculateAveragePose(tag1.pose, tag2.pose);
        combined_tag.marker_flag = 1; // 統合成功
        return true;
    }

    combined_tag.marker_flag = 0; // 統合失敗
    return false;
}
