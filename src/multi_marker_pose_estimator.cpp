#include "multi_marker_pose_estimator.h"
#include "pose_utils.h"
#include <iostream>
#include <cmath>
#include <algorithm> // std::remove


tag_node_t MultiMarkerPoseEstimator::createTripletTagNode(uint16_t root_id, double root_size, uint16_t left_id, uint16_t right_id) {
    double tag_size_half = root_size / 2;
    double offset_y = tag_size_half * 10 / 8;
    double offset_z = offset_y * 1.5;

    return tag_node_t{
        std::nullopt,
        tag_offset_t{0.0, -offset_z},
        std::make_unique<tag_node_t>(tag_node_t{
            tag_info_t{root_id, 1, root_size, {}},
            std::nullopt,
            nullptr,
            nullptr
        }),
        std::make_unique<tag_node_t>(tag_node_t{
            std::nullopt,
            tag_offset_t{-offset_y, 0.0},
            std::make_unique<tag_node_t>(tag_node_t{
                tag_info_t{left_id, 1, tag_size_half, {}},
                std::nullopt,
                nullptr,
                nullptr
            }),
            std::make_unique<tag_node_t>(tag_node_t{
                tag_info_t{right_id, 1, tag_size_half, {}},
                std::nullopt,
                nullptr,
                nullptr
            })
        })
    };
}

std::vector<tag_info_t> MultiMarkerPoseEstimator::collectTagsAndDetect(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root) {
    std::vector<std::pair<uint16_t, double>> tag_id_size_pairs;

    // トーナメントノードを走査し、タグIDとサイズを収集
    std::function<void(const tag_node_t&)> collect_tags = [&](const tag_node_t& node) {
        if (node.tag_info) {
            tag_id_size_pairs.push_back({node.tag_info->id, node.tag_info->size});
        } else {
            if (node.left_child) collect_tags(*node.left_child);
            if (node.right_child) collect_tags(*node.right_child);
        }
    };
    collect_tags(root);

    std::vector<apriltag_t> tags = detector.detect_multiple_apriltags(frame, output_frame, tag_id_size_pairs);
    std::vector<tag_info_t> tag_info_list;

    for (auto& tag : tags) {
        tag_info_t tag_info = { tag.apriltag_id, 1, tag.size, detector.convertTo3DPose(tag.pose) };
        tag_info_list.push_back(tag_info);
    }

    // デバッグ出力: 初期のタグ情報リスト
    std::cout << "Initial tag_info_list:" << std::endl;
    for (const auto& tag_info : tag_info_list) {
        std::cout << "Tag ID: " << tag_info.id << ", Size: " << tag_info.size 
                  << ", Pose: (" << tag_info.pose.x << ", " << tag_info.pose.y 
                  << ", " << tag_info.pose.z << ", roll=" << tag_info.pose.roll 
                  << ", pitch=" << tag_info.pose.pitch << ", yaw=" << tag_info.pose.yaw << ")" << std::endl;
    }

    return tag_info_list;
}

tag_info_t MultiMarkerPoseEstimator::moveTagInfo(const tag_info_t& tag, double y_offset, double z_offset) {
    tag_info_t moved_tag = tag;

    // tf2::Transformを用いて現在のposeから移動量を適用
    tf2::Transform transform = createTransform(tag.pose.x, tag.pose.y, tag.pose.z, tag.pose.roll, tag.pose.pitch, tag.pose.yaw);
    tf2::Vector3 translation = transform.getOrigin();

    // y, zの移動量を適用
    translation.setY(translation.y() + y_offset);
    translation.setZ(translation.z() + z_offset);

    // 更新されたposeをmoved_tagに設定
    moved_tag.pose.y = translation.y();
    moved_tag.pose.z = translation.z();

    return moved_tag;
}

tag_info_t MultiMarkerPoseEstimator::processNode(const tag_node_t& node, std::vector<tag_info_t>& tag_info_list) {
    tag_info_t combined_tag = {0, 0, 0.0, {}};

    if (node.left_child && node.right_child) {
        // 子ノードを持つ場合
        tag_info_t left_tag = processNode(*node.left_child, tag_info_list);
        tag_info_t right_tag = processNode(*node.right_child, tag_info_list);

        if (left_tag.marker_flag == 1 && right_tag.marker_flag == 1) {
            if (validateAndEstimatePair(combined_tag, left_tag, right_tag, *node.tag_offset, 10.0)) {
                combined_tag.marker_flag = 1;
            } else {
                // SIZEの小さくない方を選択
                combined_tag = (left_tag.size >= right_tag.size) ? left_tag : right_tag;
                // y, z 平行移動を適用
                if (combined_tag.id == left_tag.id) {
                    combined_tag = moveTagInfo(left_tag, node.tag_offset->tag2_y_from_tag1 / 2, node.tag_offset->tag2_z_from_tag1 / 2);
                } else {
                    combined_tag = moveTagInfo(right_tag, -node.tag_offset->tag2_y_from_tag1 / 2, -node.tag_offset->tag2_z_from_tag1 / 2);
                }
                combined_tag.marker_flag = 1;
            }
        } else if (left_tag.marker_flag == 1) {
            combined_tag = moveTagInfo(left_tag, node.tag_offset->tag2_y_from_tag1, node.tag_offset->tag2_z_from_tag1);
            combined_tag.marker_flag = 1;
        } else if (right_tag.marker_flag == 1) {
            combined_tag = moveTagInfo(right_tag, -node.tag_offset->tag2_y_from_tag1, -node.tag_offset->tag2_z_from_tag1);
            combined_tag.marker_flag = 1;
        }
    } else if (!node.left_child && !node.right_child) {
        // 子ノードを持たない場合、タグIDを探す
        auto tag_it = std::find_if(tag_info_list.begin(), tag_info_list.end(), [&](const tag_info_t& tag) { return tag.id == node.tag_info->id; });
        if (tag_it != tag_info_list.end()) {
            combined_tag = *tag_it;
            tag_info_list.erase(tag_it);
            combined_tag.marker_flag = 1;
        } else {
            combined_tag.marker_flag = 0;
        }
    }

    return combined_tag;
}

tag_info_t MultiMarkerPoseEstimator::detectAndEstimate(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root) {
    std::vector<tag_info_t> tag_info_list = collectTagsAndDetect(frame, output_frame, root);

    tag_info_t final_combined_tag = processNode(root, tag_info_list);

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

bool MultiMarkerPoseEstimator::validateAndEstimatePair(tag_info_t& combined_tag, const tag_info_t& tag1, const tag_info_t& tag2, const tag_offset_t& offset, double threshold_percentage) {
    // std::cout << "Initial Tag1 Pose: x=" << tag1.pose.x << ", y=" << tag1.pose.y << ", z=" << tag1.pose.z
    //           << ", roll=" << tag1.pose.roll << ", pitch=" << tag1.pose.pitch << ", yaw=" << tag1.pose.yaw << std::endl;
    // std::cout << "Initial Tag2 Pose: x=" << tag2.pose.x << ", y=" << tag2.pose.y << ", z=" << tag2.pose.z
    //           << ", roll=" << tag2.pose.roll << ", pitch=" << tag2.pose.pitch << ", yaw=" << tag2.pose.yaw << std::endl;
    tf2::Transform transform1 = createTransform(tag1.pose.x, tag1.pose.y, tag1.pose.z, tag1.pose.roll, tag1.pose.pitch, tag1.pose.yaw);
    tf2::Transform transform2 = createTransform(tag2.pose.x, tag2.pose.y, tag2.pose.z, tag2.pose.roll, tag2.pose.pitch, tag2.pose.yaw);

    tf2::Transform relative_transform = transform1.inverse() * transform2;

    // Extract translation components
    tf2::Vector3 translation = relative_transform.getOrigin();
    double x = translation.x();
    double y = translation.y();
    double z = translation.z();

    // Extract rotation components
    double roll, pitch, yaw;
    relative_transform.getBasis().getRPY(roll, pitch, yaw);

    double max_tag_size = std::max(tag1.size, tag2.size);
    double max_dist = std::max(tag1.pose.x, tag2.pose.x);

    // Calculate errors
    double x_error = std::abs(x);
    double y_error = std::abs(y - offset.tag2_y_from_tag1);
    double z_error = std::abs(z - offset.tag2_z_from_tag1);
    double roll_error = std::abs(roll);
    double pitch_error = std::abs(pitch);
    double yaw_error = std::abs(yaw);

    double x_error_percentage = x_error / max_dist * 100.0;
    double y_error_percentage = y_error / max_tag_size * 100.0;
    double z_error_percentage = z_error / max_tag_size * 100.0;
    double roll_error_percentage = roll_error / 3.14 * 100.0;
    double pitch_error_percentage = pitch_error / 3.14 * 100.0;
    double yaw_error_percentage = yaw_error / 3.14 * 100.0;

    std::cout << "Relative Pose Error: X=" << x_error_percentage << "%, Y=" << y_error_percentage << "%, Z=" << z_error_percentage << "%" << std::endl;
    std::cout << "Relative Rotation Error: Roll=" << roll_error_percentage << "%, Pitch=" << pitch_error_percentage << "%, Yaw=" << yaw_error_percentage << "%" << std::endl;

    if (x_error_percentage <= threshold_percentage &&
        y_error_percentage <= threshold_percentage &&
        z_error_percentage <= threshold_percentage &&
        roll_error_percentage <= threshold_percentage &&
        pitch_error_percentage <= threshold_percentage &&
        yaw_error_percentage <= threshold_percentage) {
        combined_tag.id = tag1.id + tag2.id;
        combined_tag.size = tag1.size + tag2.size;
        combined_tag.pose = calculateAveragePose(tag1.pose, tag2.pose);
        combined_tag.marker_flag = 1; // 統合成功
        return true;
    }

    combined_tag.marker_flag = 0; // 統合失敗
    return false;
}
