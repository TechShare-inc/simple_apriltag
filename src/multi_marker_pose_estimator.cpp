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
        quot_tag_info_t tag_info = { tag.apriltag_id, 1, tag.size, convertToQuat3DPose(tag.pose) };
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

// 補助関数: -π～πの範囲の角度 a, b の平均を、境界補正を行って求める
static double averageAngle(double a, double b) {
    // まず角度差を計算し、[-π, π] の範囲に正規化する
    double diff = b - a;
    while (diff < -M_PI)
        diff += 2.0 * M_PI;
    while (diff > M_PI)
        diff -= 2.0 * M_PI;

    // 補正した角度差の半分を加算して平均値を得る
    double avg = a + diff / 2.0;

    // 平均値が再び [-π, π] を超えないように正規化する
    while (avg < -M_PI)
        avg += 2.0 * M_PI;
    while (avg > M_PI)
        avg -= 2.0 * M_PI;

    return avg;
}

QuatPose3D MultiMarkerPoseEstimator::calculateAveragePose(const QuatPose3D& pose1, const QuatPose3D& pose2) {
    QuatPose3D average_pose;

    // 位置は単純な平均を行う
    average_pose.x = (pose1.x + pose2.x) / 2.0;
    average_pose.y = (pose1.y + pose2.y) / 2.0;
    average_pose.z = (pose1.z + pose2.z) / 2.0;

    // クォータニオンからそれぞれ RPY に変換する
    tf2::Quaternion q1(pose1.qx, pose1.qy, pose1.qz, pose1.qw);
    double roll1, pitch1, yaw1;
    tf2::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);

    tf2::Quaternion q2(pose2.qx, pose2.qy, pose2.qz, pose2.qw);
    double roll2, pitch2, yaw2;
    tf2::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);

    // 各角度の平均を計算（境界近傍での値のずれを補正）
    double avg_roll  = averageAngle(roll1, roll2);
    double avg_pitch = averageAngle(pitch1, pitch2);
    double avg_yaw   = averageAngle(yaw1, yaw2);

    // 平均した RPY からクォータニオンを生成する
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
    // （デバッグ用に各タグのクォータニオンからRPYを取得して表示）
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

    // --- 期待される変化量 offset の適用 ---
    // 回転成分 offset.dyaw はタグ座標系での y 軸回りの回転として扱う
    // ここでは setRotation を用いて、軸‐角（axis: (0,1,0)）からクォータニオンを直接生成する
    tf2::Quaternion expected_q;
    expected_q.setRotation(tf2::Vector3(0, 0, 1), offset.dyaw);
    tf2::Transform expected_transform;
    expected_transform.setOrigin(tf2::Vector3(offset.dx, offset.dy, offset.dz));
    expected_transform.setRotation(expected_q);

    // --- error_transform の算出 ---
    // ここまではすべてクォータニオンおよび変換行列による計算で行い、RPY変換は使用していません。
    tf2::Transform error_transform = expected_transform.inverse() * measured_transform;

    // --- 誤差の算出 ---
    // 平行移動成分のエラー（直接取得）
    tf2::Vector3 error_translation = error_transform.getOrigin();
    double error_x = fabs(error_translation.x());
    double error_y = fabs(error_translation.y());
    double error_z = fabs(error_translation.z());

    // ここから回転誤差算出のために、error_transform の回転を RPY 表現に変換
    double error_roll, error_pitch, error_yaw;
    tf2::Matrix3x3(error_transform.getRotation()).getRPY(error_roll, error_pitch, error_yaw);

    // タグサイズ（大きい方のタグサイズ）で正規化し、パーセンテージとして計算
    double reference_size = std::max(tag1.size, tag2.size);
    double x_error_percentage = (error_x / reference_size) * 100.0;
    double y_error_percentage = (error_y / reference_size) * 100.0;
    double z_error_percentage = (error_z / reference_size) * 100.0;
    // 回転誤差は π (約3.14) を基準（角度はラジアン）
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

QuatPose3D MultiMarkerPoseEstimator::convertToQuat3DPose(const apriltag_pose_t& pose) {
    QuatPose3D quatPose3D;
    
    // 座標の変換
    quatPose3D.x = matd_get(pose.t, 2, 0);  // tagのz方向
    quatPose3D.y = -matd_get(pose.t, 0, 0); // tagのx方向
    quatPose3D.z = -matd_get(pose.t, 1, 0); // tagのy方向

    // apriltag_pose_t 内の回転行列の各要素を取得
    double r11 = matd_get(pose.R, 0, 0);
    double r12 = matd_get(pose.R, 0, 1);
    double r13 = matd_get(pose.R, 0, 2);
    double r21 = matd_get(pose.R, 1, 0);
    double r22 = matd_get(pose.R, 1, 1);
    double r23 = matd_get(pose.R, 1, 2);
    double r31 = matd_get(pose.R, 2, 0);
    double r32 = matd_get(pose.R, 2, 1);
    double r33 = matd_get(pose.R, 2, 2);

    // apriltag の回転行列を tf2::Matrix3x3 によって q_orig へ変換
    tf2::Matrix3x3 m(r11, r12, r13,
                     r21, r22, r23,
                     r31, r32, r33);
    tf2::Quaternion q_orig;
    m.getRotation(q_orig);

    // 固定の回転 q_fixed を生成
	// x,y,z in apriltag座標系 === -y,-z,x in robot_frame座標系
    tf2::Quaternion q_fixed(-0.5, 0.5, -0.5, 0.5);

    // 固定の回転 q_convert_robot_frame を生成
	// apriltagの座標系の取り方が、robot座標系っぽくなるように変換（マーカーを立てたときに、変換がアイデンティティになるように）
    tf2::Quaternion q_convert_robot_frame(0.5, -0.5, 0.5, 0.5);

    // 全体のクォータニオンは q_total = q_fixed * q_orig
    // ※ tf2 のクォータニオンの掛け算は、左側の回転が先に適用される順序です
    tf2::Quaternion q_total = q_fixed * q_orig * q_convert_robot_frame;
    q_total.normalize();

    // q_total の各成分を QuatPose3D に設定
    quatPose3D.qw = q_total.getW();
    quatPose3D.qx = q_total.getX();
    quatPose3D.qy = q_total.getY();
    quatPose3D.qz = q_total.getZ();

    return quatPose3D;
}