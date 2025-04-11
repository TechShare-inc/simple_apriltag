#include "multi_marker_pose_estimator.h"
#include "pose_utils.h"
#include <iostream>
#include <cmath>
#include <algorithm>

// ヘルパー：オフセットをスケールする (例えば半分のオフセットを適用する際)
static tag_offset_t scaleOffset(const tag_offset_t& offset, double factor) {
    return tag_offset_t{offset.dx * factor, offset.dy * factor, offset.dz * factor, offset.dyaw * factor};
}

tag_node_t MultiMarkerPoseEstimator::createTriplet3DTagNode(uint16_t parent_tag_id, double parent_tag_size, uint16_t left_id, uint16_t right_id) {
    double child_size = parent_tag_size / 2;                       // 子タグは親タグの半分
    double half_child_x = child_size / 4;                          // 子タグのベッパリ高さの半分
    double child_with_border_size = child_size * 10 / 8;           // 子タグ（白い部分込み）の大きさ
    double offset_child_x = child_with_border_size * 1.732 / 4;    // 左の子タグの中心からみた、右の子タグの中心(x)
    double offset_child_y = child_with_border_size * 3 / 4;        // 左の子タグの中心からみた、右の子タグの中心(y)
    double half_full_tag_height = child_with_border_size * 1.5;    // 全体タグの白い部分込みの高さの半分
    double child_arg = 3.1415 * 60 / 180;                          // 子タグ同士のなす角

    return tag_node_t{
        std::nullopt,
        // 上の親タグの中心からみた、下２つの子タグの中心の相対位置
        tag_offset_t{-half_child_x, 0.0, -half_full_tag_height, 0.0},
        std::make_unique<tag_node_t>(tag_node_t{
            tag_info_t{parent_tag_id, 1, parent_tag_size, {}},
            std::nullopt,
            nullptr,
            nullptr
        }),
        std::make_unique<tag_node_t>(tag_node_t{
            std::nullopt,
            // 左の子タグの中心からみた、右の子タグの中心
            tag_offset_t{offset_child_x, -offset_child_y, 0.0, child_arg},
            std::make_unique<tag_node_t>(tag_node_t{
                tag_info_t{left_id, 1, child_size, {}},
                std::nullopt,
                nullptr,
                nullptr
            }),
            std::make_unique<tag_node_t>(tag_node_t{
                tag_info_t{right_id, 1, child_size, {}},
                std::nullopt,
                nullptr,
                nullptr
            })
        })
    };
}

tag_node_t MultiMarkerPoseEstimator::createTripletTagNode(uint16_t parent_tag_id, double parent_tag_size, uint16_t left_id, uint16_t right_id) {
    double child_size = parent_tag_size / 2;                       // 子タグは親タグの半分
    double child_with_border_size = child_size * 10 / 8;           // 子タグ（白い部分込み）の大きさ
    double half_full_tag_height = child_with_border_size * 1.5;    // 全体タグの白い部分込みの高さの半分

    return tag_node_t{
        std::nullopt,
        // 下２つの子タグの中心と、上の親タグの中心との相対位置
        tag_offset_t{0.0, 0.0, -half_full_tag_height, 0.0},
        std::make_unique<tag_node_t>(tag_node_t{
            tag_info_t{parent_tag_id, 1, parent_tag_size, {}},
            std::nullopt,
            nullptr,
            nullptr
        }),
        std::make_unique<tag_node_t>(tag_node_t{
            std::nullopt,
            // 左の子タグの中心からみた、右の子タグの中心
            tag_offset_t{0.0, -child_with_border_size, 0.0, 0.0},
            std::make_unique<tag_node_t>(tag_node_t{
                tag_info_t{left_id, 1, child_size, {}},
                std::nullopt,
                nullptr,
                nullptr
            }),
            std::make_unique<tag_node_t>(tag_node_t{
                tag_info_t{right_id, 1, child_size, {}},
                std::nullopt,
                nullptr,
                nullptr
            })
        })
    };
}

tag_node_t MultiMarkerPoseEstimator::createQuattroPlusNode(uint16_t parent_tag_id, double parent_tag_size, 
                                                             uint16_t ll_id, uint16_t left_id, 
                                                             uint16_t right_id, uint16_t rr_id) {
    // 親タグサイズに対して子タグは半分の大きさとなる
    double child_tag_size = parent_tag_size / 2; // 子タグ本体の大きさ

    // 子タグの白い部分を含めたサイズ
    double child_full_size = child_tag_size * 10 / 8;

    // 親タグの白い部分を含めたサイズ（全体タグのサイズ感に用いる）
    double parent_full_size = parent_tag_size * 10 / 8;

    // 全体（フル）タグの高さは、ここでは子タグの白い部分込みのサイズに基づき、その半分となる
    double half_full_tag_height = child_full_size * 1.5;

    return tag_node_t{
        std::nullopt,
        // 上部に配置される4つの子タグの中心と、下部の親タグ中心との相対位置（X, Y, Z, yaw）
        tag_offset_t{0.0, 0.0, half_full_tag_height, 0.0},
        std::make_unique<tag_node_t>(tag_node_t{
            // 親タグの情報
            tag_info_t{parent_tag_id, 1, parent_tag_size, {}},
            std::nullopt,
            nullptr,
            nullptr
        }),
        std::make_unique<tag_node_t>(tag_node_t{
            std::nullopt,
            // 左側子タググループ：
            // 親タグの白い部分込みのサイズを用いて左右グループの相対的な位置を決定
            tag_offset_t{0.0, -parent_full_size, 0.0, 0.0},
            std::make_unique<tag_node_t>(tag_node_t{
                std::nullopt,
                // 左側子タググループ内の、右側から見たオフセット（子タグの白い部分込み）
                tag_offset_t{0.0, child_full_size, 0.0, 0.0},
                std::make_unique<tag_node_t>(tag_node_t{
                    tag_info_t{left_id, 1, child_tag_size, {}},
                    std::nullopt,
                    nullptr,
                    nullptr
                }),
                std::make_unique<tag_node_t>(tag_node_t{
                    tag_info_t{ll_id, 1, child_tag_size, {}},
                    std::nullopt,
                    nullptr,
                    nullptr
                })
            }),
            std::make_unique<tag_node_t>(tag_node_t{
                std::nullopt,
                // 右側子タググループ内の、左側から見たオフセット（符号反転）
                tag_offset_t{0.0, -child_full_size, 0.0, 0.0},
                std::make_unique<tag_node_t>(tag_node_t{
                    tag_info_t{right_id, 1, child_tag_size, {}},
                    std::nullopt,
                    nullptr,
                    nullptr
                }),
                std::make_unique<tag_node_t>(tag_node_t{
                    tag_info_t{rr_id, 1, child_tag_size, {}},
                    std::nullopt,
                    nullptr,
                    nullptr
                })
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

    // // デバッグ出力: 初期のタグ情報リスト
    // std::cout << "Initial tag_info_list:" << std::endl;
    // for (const auto& tag_info : tag_info_list) {
    //     std::cout << "Tag ID: " << tag_info.id << ", Size: " << tag_info.size 
    //               << ", Pose: (" << tag_info.pose.x << ", " << tag_info.pose.y 
    //               << ", " << tag_info.pose.z << ", roll=" << tag_info.pose.roll 
    //               << ", pitch=" << tag_info.pose.pitch << ", yaw=" << tag_info.pose.yaw << ")" << std::endl;
    // }

    return tag_info_list;
}

// シンプルな3D変換の適用：roll, pitchはそのままで、x,y座標は現在のyawに基づく平行移動、zとyawは単純加算
tag_info_t MultiMarkerPoseEstimator::moveTagInfo(const tag_info_t& tag, const tag_offset_t& offset) {
    tag_info_t moved_tag = tag;
    
    double cos_yaw = cos(tag.pose.yaw);
    double sin_yaw = sin(tag.pose.yaw);
    moved_tag.pose.x = tag.pose.x + offset.dx * cos_yaw - offset.dy * sin_yaw;
    moved_tag.pose.y = tag.pose.y + offset.dx * sin_yaw + offset.dy * cos_yaw;
    moved_tag.pose.z = tag.pose.z + offset.dz;
    moved_tag.pose.yaw = tag.pose.yaw + offset.dyaw;
    
    return moved_tag;
}

tag_info_t MultiMarkerPoseEstimator::processNode(const tag_node_t& node, std::vector<tag_info_t>& tag_info_list) {
    tag_info_t combined_tag = {0, 0, 0.0, {}};

    if (node.left_child && node.right_child) {
        // 子ノードを持つ場合
        tag_info_t left_tag = processNode(*node.left_child, tag_info_list);
        tag_info_t right_tag = processNode(*node.right_child, tag_info_list);

        if (left_tag.marker_flag == 1 && right_tag.marker_flag == 1) {
            // 統合の際に、２つのタグがともに見つかっていれば、統合をチャレンジし、失敗したら、タグサイズの大きい方を採用する
            if (validateAndEstimatePair(combined_tag, left_tag, right_tag, *node.tag_offset, 30.0)) {
                combined_tag.marker_flag = 1;
            } else {
                combined_tag = (left_tag.size >= right_tag.size) ? left_tag : right_tag;
                // オフセットの半分を適用して調整
                if (combined_tag.id == left_tag.id) {
                    tag_offset_t halfOffset = scaleOffset(*node.tag_offset, 0.5);
                    combined_tag = moveTagInfo(left_tag, halfOffset);
                } else {
                    tag_offset_t halfOffset = scaleOffset(*node.tag_offset, 0.5);
                    // 反対方向の場合、符号を反転
                    halfOffset.dx = -halfOffset.dx;
                    halfOffset.dy = -halfOffset.dy;
                    halfOffset.dz = -halfOffset.dz;
                    halfOffset.dyaw = -halfOffset.dyaw;
                    combined_tag = moveTagInfo(right_tag, halfOffset);
                }
                combined_tag.marker_flag = 1;
            }
        } else if (left_tag.marker_flag == 1) {
            combined_tag = moveTagInfo(left_tag, *node.tag_offset);
            combined_tag.marker_flag = 1;
        } else if (right_tag.marker_flag == 1) {
            tag_offset_t negOffset = *node.tag_offset;
            negOffset.dx = -negOffset.dx;
            negOffset.dy = -negOffset.dy;
            negOffset.dz = -negOffset.dz;
            negOffset.dyaw = -negOffset.dyaw;
            combined_tag = moveTagInfo(right_tag, negOffset);
            combined_tag.marker_flag = 1;
        }
    } else if (!node.left_child && !node.right_child) {
        // 葉ノードの場合、対応するタグIDを探す
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

    // // デバッグ出力: 統合後のタグ情報リスト
    // std::cout << "Final tag_info_list:" << std::endl;
    // for (const auto& tag_info : tag_info_list) {
    //     std::cout << "Tag ID: " << tag_info.id << ", Size: " << tag_info.size << ", Pose: (" << tag_info.pose.x << ", " << tag_info.pose.y << ", " << tag_info.pose.z << "), Flag: " << static_cast<int>(tag_info.marker_flag) << std::endl;
    // }

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

bool MultiMarkerPoseEstimator::validateAndEstimatePair(tag_info_t& combined_tag, 
                                                       const tag_info_t& tag1, 
                                                       const tag_info_t& tag2, 
                                                       const tag_offset_t& offset, 
                                                       double threshold_percentage) {
    // 入力タグの初期Pose表示（デバッグ用）
    std::cout << "Initial Tag1 Pose: x=" << tag1.pose.x << ", y=" << tag1.pose.y << ", z=" << tag1.pose.z
              << ", roll=" << tag1.pose.roll << ", pitch=" << tag1.pose.pitch << ", yaw=" << tag1.pose.yaw << std::endl;
    std::cout << "Initial Tag2 Pose: x=" << tag2.pose.x << ", y=" << tag2.pose.y << ", z=" << tag2.pose.z
              << ", roll=" << tag2.pose.roll << ", pitch=" << tag2.pose.pitch << ", yaw=" << tag2.pose.yaw << std::endl;

    // 各タグのPoseからTFを作成する
    tf2::Transform transform1 = createTransform(tag1.pose.x, tag1.pose.y, tag1.pose.z, 
                                                  tag1.pose.roll, tag1.pose.pitch, tag1.pose.yaw);
    tf2::Transform transform2 = createTransform(tag2.pose.x, tag2.pose.y, tag2.pose.z, 
                                                  tag2.pose.roll, tag2.pose.pitch, tag2.pose.yaw);

    // measured_transform： tag1 から tag2 への相対変換
    tf2::Transform measured_transform = transform1.inverse() * transform2;

    // 期待される変化量 offset（translation: dx,dy,dz と yaw成分: dyaw、rollとpitchは 0 とする）
    tf2::Transform expected_transform = createTransform(offset.dx, offset.dy, offset.dz, 0.0, 0.0, offset.dyaw);

    // error_transform = expected_transform⁻¹ * measured_transform
    // このエラーがアイデンティティ（ほぼゼロ変化）に近いほど、期待値と実際の変換が合致している
    tf2::Transform error_transform = expected_transform.inverse() * measured_transform;

    // エラーの平行移動成分
    tf2::Vector3 error_translation = error_transform.getOrigin();
    double error_x = fabs(error_translation.x());
    double error_y = fabs(error_translation.y());
    double error_z = fabs(error_translation.z());

    // エラーの回転成分（RPY 表現で取得）
    double error_roll, error_pitch, error_yaw;
    error_transform.getBasis().getRPY(error_roll, error_pitch, error_yaw);

    // 位置成分は、タグサイズなどのスケールで正規化できると考え、
    // ここでは大きい方のタグサイズを基準にパーセンテージを計算
    double reference_size = std::max(tag1.size, tag2.size);
    double x_error_percentage = (error_x / reference_size) * 100.0;
    double y_error_percentage = (error_y / reference_size) * 100.0;
    double z_error_percentage = (error_z / reference_size) * 100.0;
    // 回転誤差はπ(≈3.14)を基準とする（角度はラジアン）
    double roll_error_percentage = (fabs(error_roll) / 3.14) * 100.0;
    double pitch_error_percentage = (fabs(error_pitch) / 3.14) * 100.0;
    double yaw_error_percentage = (fabs(error_yaw) / 3.14) * 100.0;

    std::cout << "Error Translation: X=" << error_x << ", Y=" << error_y << ", Z=" << error_z << std::endl;
    std::cout << "Error Rotation: Roll=" << error_roll << ", Pitch=" << error_pitch << ", Yaw=" << error_yaw << std::endl;
    std::cout << "Error Percentages: X=" << x_error_percentage << "%, Y=" << y_error_percentage 
              << "%, Z=" << z_error_percentage << "%" << std::endl;
    std::cout << "Rotation Error Percentages: Roll=" << roll_error_percentage << "%, Pitch=" 
              << pitch_error_percentage << "%, Yaw=" << yaw_error_percentage << "%" << std::endl;

    // すべての誤差が指定の閾値（％）以内であればタグの統合を行う
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
