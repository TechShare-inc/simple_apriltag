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
            quot_tag_info_t{parent_tag_id, 1, parent_tag_size, {}},
            std::nullopt,
            nullptr,
            nullptr
        }),
        std::make_unique<tag_node_t>(tag_node_t{
            std::nullopt,
            // 左の子タグの中心からみた、右の子タグの中心
            tag_offset_t{offset_child_x, -offset_child_y, 0.0, child_arg},
            std::make_unique<tag_node_t>(tag_node_t{
                quot_tag_info_t{left_id, 1, child_size, {}},
                std::nullopt,
                nullptr,
                nullptr
            }),
            std::make_unique<tag_node_t>(tag_node_t{
                quot_tag_info_t{right_id, 1, child_size, {}},
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
            quot_tag_info_t{parent_tag_id, 1, parent_tag_size, {}},
            std::nullopt,
            nullptr,
            nullptr
        }),
        std::make_unique<tag_node_t>(tag_node_t{
            std::nullopt,
            // 左の子タグの中心からみた、右の子タグの中心
            tag_offset_t{0.0, -child_with_border_size, 0.0, 0.0},
            std::make_unique<tag_node_t>(tag_node_t{
                quot_tag_info_t{left_id, 1, child_size, {}},
                std::nullopt,
                nullptr,
                nullptr
            }),
            std::make_unique<tag_node_t>(tag_node_t{
                quot_tag_info_t{right_id, 1, child_size, {}},
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
            quot_tag_info_t{parent_tag_id, 1, parent_tag_size, {}},
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
                    quot_tag_info_t{left_id, 1, child_tag_size, {}},
                    std::nullopt,
                    nullptr,
                    nullptr
                }),
                std::make_unique<tag_node_t>(tag_node_t{
                    quot_tag_info_t{ll_id, 1, child_tag_size, {}},
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
                    quot_tag_info_t{right_id, 1, child_tag_size, {}},
                    std::nullopt,
                    nullptr,
                    nullptr
                }),
                std::make_unique<tag_node_t>(tag_node_t{
                    quot_tag_info_t{rr_id, 1, child_tag_size, {}},
                    std::nullopt,
                    nullptr,
                    nullptr
                })
            })
        })
    };
}

std::vector<quot_tag_info_t> MultiMarkerPoseEstimator::collectTagsAndDetect(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root) {
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
    std::vector<quot_tag_info_t> tag_info_list;

    for (auto& tag : tags) {
        quot_tag_info_t tag_info = { tag.apriltag_id, 1, tag.size, detector.convertToQuat3DPose(tag.pose) };
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
quot_tag_info_t MultiMarkerPoseEstimator::moveTagInfo(const quot_tag_info_t& tag, const tag_offset_t& offset) {
    quot_tag_info_t moved_tag = tag;

    // 現在のクォータニオンから RPY を取得
    tf2::Quaternion q(tag.pose.qx, tag.pose.qy, tag.pose.qz, tag.pose.qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 現在の yaw に基づいて x, y 座標の平行移動を計算
    double cos_yaw = cos(pitch);
    double sin_yaw = sin(pitch);
    double new_x = tag.pose.x + offset.dx * cos_yaw - offset.dy * sin_yaw;
    double new_y = tag.pose.y + offset.dx * sin_yaw + offset.dy * cos_yaw;
    double new_z = tag.pose.z + offset.dz;

    // yaw のみ単純加算（roll, pitch はそのまま）
    double new_pitch = pitch - offset.dyaw;

    // 更新後の RPY から新しいクォータニオンを生成
    tf2::Quaternion new_q;
    new_q.setRPY(roll, new_pitch, yaw);

    // 新たな平行移動後の位置と回転（クォータニオン）を格納
    moved_tag.pose.x = new_x;
    moved_tag.pose.y = new_y;
    moved_tag.pose.z = new_z;
    moved_tag.pose.qw = new_q.getW();
    moved_tag.pose.qx = new_q.getX();
    moved_tag.pose.qy = new_q.getY();
    moved_tag.pose.qz = new_q.getZ();

    return moved_tag;
}

quot_tag_info_t MultiMarkerPoseEstimator::processNode(const tag_node_t& node, std::vector<quot_tag_info_t>& tag_info_list) {
    quot_tag_info_t combined_tag = {0, 0, 0.0, {}};

    if (node.left_child && node.right_child) {
        // 子ノードを持つ場合
        quot_tag_info_t left_tag = processNode(*node.left_child, tag_info_list);
        quot_tag_info_t right_tag = processNode(*node.right_child, tag_info_list);

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
        auto tag_it = std::find_if(tag_info_list.begin(), tag_info_list.end(), [&](const quot_tag_info_t& tag) { return tag.id == node.tag_info->id; });
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
    // タグ検出および推定を行い、QuotPose3D を持つタグ情報リストを取得
    std::vector<quot_tag_info_t> tag_info_list = collectTagsAndDetect(frame, output_frame, root);

    // 複数タグの統合処理（内部的にはクォータニオン表現のPoseを扱う）
    quot_tag_info_t final_combined_tag = processNode(root, tag_info_list);

    // --- QuatPose3Dから Pose3D (RPY) への変換 ---
    Pose3D rpy_pose;
    // 位置はそのままコピー
    rpy_pose.x = final_combined_tag.pose.x;
    rpy_pose.y = final_combined_tag.pose.y;
    rpy_pose.z = final_combined_tag.pose.z;

    // QuatPose3D のクォータニオン成分から tf2::Quaternion を生成し RPY に変換
    tf2::Quaternion q(final_combined_tag.pose.qx, 
                      final_combined_tag.pose.qy, 
                      final_combined_tag.pose.qz, 
                      final_combined_tag.pose.qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    rpy_pose.roll = roll;
    rpy_pose.pitch = pitch;
    rpy_pose.yaw = yaw;

    // 最終的な tag_info_t に変換して返す
    tag_info_t output_tag;
    output_tag.id = final_combined_tag.id;
    output_tag.marker_flag = final_combined_tag.marker_flag;
    output_tag.size = final_combined_tag.size;
    output_tag.pose = rpy_pose;

    return output_tag;
}

QuatPose3D MultiMarkerPoseEstimator::calculateAveragePose(const QuatPose3D& pose1, const QuatPose3D& pose2) {
    QuatPose3D average_pose;

    // 位置は単純に平均
    average_pose.x = (pose1.x + pose2.x) / 2.0;
    average_pose.y = (pose1.y + pose2.y) / 2.0;
    average_pose.z = (pose1.z + pose2.z) / 2.0;

    // それぞれのクォータニオンから RPY への変換
    tf2::Quaternion q1(pose1.qx, pose1.qy, pose1.qz, pose1.qw);
    double roll1, pitch1, yaw1;
    tf2::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);

    tf2::Quaternion q2(pose2.qx, pose2.qy, pose2.qz, pose2.qw);
    double roll2, pitch2, yaw2;
    tf2::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);

    // RPY の平均を計算（角度は単純平均）
    double avg_roll = (roll1 + roll2) / 2.0;
    double avg_pitch = (pitch1 + pitch2) / 2.0;
    double avg_yaw = (yaw1 + yaw2) / 2.0;

    // 平均 RPY からクォータニオンを生成
    tf2::Quaternion q_avg;
    q_avg.setRPY(avg_roll, avg_pitch, avg_yaw);

    // 結果のクォータニオン成分を average_pose に設定
    average_pose.qw = q_avg.getW();
    average_pose.qx = q_avg.getX();
    average_pose.qy = q_avg.getY();
    average_pose.qz = q_avg.getZ();

    return average_pose;
}

bool MultiMarkerPoseEstimator::validateAndEstimatePair(quot_tag_info_t& combined_tag, 
                                                       const quot_tag_info_t& tag1, 
                                                       const quot_tag_info_t& tag2, 
                                                       const tag_offset_t& offset, 
                                                       double threshold_percentage) {
    // --- デバッグ用：入力タグの初期 Pose 表示 ---
    // クォータニオンから RPY への変換のため、各タグのクォータニオンを生成する
    tf2::Quaternion q1(tag1.pose.qx, tag1.pose.qy, tag1.pose.qz, tag1.pose.qw);
    double tag1_roll, tag1_pitch, tag1_yaw;
    tf2::Matrix3x3(q1).getRPY(tag1_roll, tag1_pitch, tag1_yaw);

    tf2::Quaternion q2(tag2.pose.qx, tag2.pose.qy, tag2.pose.qz, tag2.pose.qw);
    double tag2_roll, tag2_pitch, tag2_yaw;
    tf2::Matrix3x3(q2).getRPY(tag2_roll, tag2_pitch, tag2_yaw);

    std::cout << "Initial Tag1 Pose: x=" << tag1.pose.x 
              << ", y=" << tag1.pose.y 
              << ", z=" << tag1.pose.z 
              << ", roll=" << tag1_roll 
              << ", pitch=" << tag1_pitch 
              << ", yaw=" << tag1_yaw << std::endl;
    std::cout << "Initial Tag2 Pose: x=" << tag2.pose.x 
              << ", y=" << tag2.pose.y 
              << ", z=" << tag2.pose.z 
              << ", roll=" << tag2_roll 
              << ", pitch=" << tag2_pitch 
              << ", yaw=" << tag2_yaw << std::endl;

    // --- 各タグの Pose から TF 変換行列を生成 ---
    tf2::Transform transform1;
    transform1.setOrigin(tf2::Vector3(tag1.pose.x, tag1.pose.y, tag1.pose.z));
    transform1.setRotation(q1);

    tf2::Transform transform2;
    transform2.setOrigin(tf2::Vector3(tag2.pose.x, tag2.pose.y, tag2.pose.z));
    transform2.setRotation(q2);

    // measured_transform： tag1 から tag2 への相対変換
    tf2::Transform measured_transform = transform1.inverse() * transform2;

    // 期待される変化量 offset（translation: dx,dy,dz と yaw 成分: dyaw、roll と pitch は 0 とする）
    // offset は RPY 表現になっているので、ここでは yaw のみを利用してクォータニオンに変換
    tf2::Quaternion expected_q;
    expected_q.setRPY(0.0, -offset.dyaw, 0.0);  // robot座標系的な見方でみたdyawは、tag座標系でy軸に当たる
    tf2::Transform expected_transform;
    expected_transform.setOrigin(tf2::Vector3(offset.dx, offset.dy, offset.dz));
    expected_transform.setRotation(expected_q);

    // error_transform = expected_transform⁻¹ * measured_transform
    tf2::Transform error_transform = expected_transform.inverse() * measured_transform;

    // --- 誤差の算出 ---
    // 平行移動成分のエラー
    tf2::Vector3 error_translation = error_transform.getOrigin();
    double error_x = fabs(error_translation.x());
    double error_y = fabs(error_translation.y());
    double error_z = fabs(error_translation.z());

    // 回転成分は RPY 表現で取得
    double error_roll, error_pitch, error_yaw;
    error_transform.getBasis().getRPY(error_roll, error_pitch, error_yaw);

    // タグサイズ（大きい方のタグサイズ）で正規化してパーセンテージを計算
    double reference_size = std::max(tag1.size, tag2.size);
    double x_error_percentage = (error_x / reference_size) * 100.0;
    double y_error_percentage = (error_y / reference_size) * 100.0;
    double z_error_percentage = (error_z / reference_size) * 100.0;
    // 回転誤差は π (約3.14) を基準に（角度はラジアン）
    double roll_error_percentage = (fabs(error_roll) / 3.14) * 100.0;
    double pitch_error_percentage = (fabs(error_pitch) / 3.14) * 100.0;
    double yaw_error_percentage = (fabs(error_yaw) / 3.14) * 100.0;

    std::cout << "Error Translation: X=" << error_x 
              << ", Y=" << error_y 
              << ", Z=" << error_z << std::endl;
    std::cout << "Error Rotation: Roll=" << error_roll 
              << ", Pitch=" << error_pitch 
              << ", Yaw=" << error_yaw << std::endl;
    std::cout << "Error Percentages: X=" << x_error_percentage 
              << "%, Y=" << y_error_percentage 
              << "%, Z=" << z_error_percentage << "%" << std::endl;
    std::cout << "Rotation Error Percentages: Roll=" << roll_error_percentage 
              << "%, Pitch=" << pitch_error_percentage 
              << "%, Yaw=" << yaw_error_percentage << "%" << std::endl;

    // --- 統合条件の評価 ---
    if (x_error_percentage <= threshold_percentage &&
        y_error_percentage <= threshold_percentage &&
        z_error_percentage <= threshold_percentage &&
        roll_error_percentage <= threshold_percentage &&
        pitch_error_percentage <= threshold_percentage &&
        yaw_error_percentage <= threshold_percentage) {
        combined_tag.id = tag1.id + tag2.id;
        combined_tag.size = tag1.size + tag2.size;
        // calculateAveragePose は QuatPose3D の平均を算出する処理に修正済み
        combined_tag.pose = calculateAveragePose(tag1.pose, tag2.pose);
        combined_tag.marker_flag = 1; // 統合成功
        return true;
    }

    combined_tag.marker_flag = 0; // 統合失敗
    return false;
}
