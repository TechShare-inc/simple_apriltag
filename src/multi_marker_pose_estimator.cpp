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
    double tag_thickness = 0.005;                                  // タグの厚み
    double child_size = parent_tag_size / 2;                       // 子タグは親タグの半分
    double half_child_x = child_size / 4;                          // 子タグのベッパリ高さの半分
    double offset_parent_x = half_child_x + tag_thickness * 1.732 /2;                       // 親タグからみた、子タグ中心(x)
    double child_with_border_size = child_size * 10 / 8;           // 子タグ（白い部分込み）の大きさ
    double offset_child_x = child_with_border_size * 1.732 / 4 + tag_thickness / 2;         // 左の子タグの中心からみた、右の子タグの中心(x)
    double offset_child_y = child_with_border_size * 3 / 4 + tag_thickness * 1.732 /2;      // 左の子タグの中心からみた、右の子タグの中心(y)
    double half_full_tag_height = child_with_border_size * 1.5;    // 全体タグの白い部分込みの高さの半分
    double child_arg = 3.1415 * 60 / 180;                          // 子タグ同士のなす角

    return tag_node_t{
        std::nullopt,
        // 上の親タグの中心からみた、下２つの子タグの中心の相対位置
        tag_offset_t{-offset_parent_x, 0.0, -half_full_tag_height, 0.0},
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
    //     tf2::Quaternion q(tag_info.pose.qx, tag_info.pose.qy, tag_info.pose.qz, tag_info.pose.qw);
    //     double tag_roll, tag_pitch, tag_yaw;
    //     tf2::Matrix3x3(q).getRPY(tag_roll, tag_pitch, tag_yaw);
    //     std::cout << "Tag ID: " << tag_info.id << ", Size: " << tag_info.size 
    //               << ", Pose: (" << tag_info.pose.x << ", " << tag_info.pose.y 
    //               << ", " << tag_info.pose.z << ", roll=" << tag_roll
    //               << ", pitch=" << tag_pitch << ", yaw=" << tag_yaw << ")" << std::endl;
    // }

    return tag_info_list;
}

// QuatPose3D → Pose3D(RPY) に変換するヘルパー
static Pose3D toPose3D(const QuatPose3D& qpose) {
    Pose3D p;
    p.x     = qpose.x;
    p.y     = qpose.y;
    p.z     = qpose.z;
    tf2::Quaternion q(qpose.qx, qpose.qy, qpose.qz, qpose.qw);
    tf2::Matrix3x3(q).getRPY(p.roll, p.pitch, p.yaw);
    return p;
}

// ２つの Pose3D 間で、許容誤差内かどうか判定する関数
static bool validatePairRPY(const Pose3D& p1, const Pose3D& p2,
                            double threshold_pct, double reference_size)
{
    auto diff = [](double a, double b) {
        double d = a - b;
        while (d < -M_PI) d += 2*M_PI;
        while (d >  M_PI) d -= 2*M_PI;
        return fabs(d);
    };

    //――― 1) 絶対誤差（距離 & 角度） ―――
    double dx   = fabs(p1.x     - p2.x);
    double dy   = fabs(p1.y     - p2.y);
    double dz   = fabs(p1.z     - p2.z);
    double drad = diff(p1.roll,  p2.roll);
    double dpid = diff(p1.pitch, p2.pitch);
    double dyaw = diff(p1.yaw,   p2.yaw);

    //――― 2) 誤差を % 表示に変換 ―――
    double x_pct    = dx   / reference_size * 100.0;
    double y_pct    = dy   / reference_size * 100.0;
    double z_pct    = dz   / reference_size * 100.0;
    double roll_pct = drad / M_PI           * 100.0;
    double pitch_pct= dpid / M_PI           * 100.0;
    double yaw_pct  = dyaw / M_PI           * 100.0;

    // //――― 3) ログ出力 ―――
    // std::cout << "[validatePairRPY] Translation deltas: "
    //           << "dx=" << dx << ", dy=" << dy << ", dz=" << dz << std::endl;
    // std::cout << "[validatePairRPY] Translation error %: "
    //           << "X=" << x_pct << "%, "
    //           << "Y=" << y_pct << "%, "
    //           << "Z=" << z_pct << "%" << std::endl;
    // std::cout << "[validatePairRPY] Rotation error %: "
    //           << "Roll="  << roll_pct  << "%, "
    //           << "Pitch=" << pitch_pct << "%, "
    //           << "Yaw="   << yaw_pct   << "%" << std::endl;

    //――― 4) 判定 ―――
    return (x_pct    <= threshold_pct &&
            y_pct    <= threshold_pct &&
            z_pct    <= threshold_pct &&
            roll_pct <= threshold_pct &&
            pitch_pct<= threshold_pct &&
            yaw_pct  <= threshold_pct);
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

// RPY平均を取って QuatPose3D に戻すヘルパー
static QuatPose3D averageQuatFromPose(const Pose3D& a, const Pose3D& b) {
    Pose3D avg;
    avg.x     = 0.5*(a.x     + b.x);
    avg.y     = 0.5*(a.y     + b.y);
    avg.z     = 0.5*(a.z     + b.z);
    avg.roll  = averageAngle(a.roll,  b.roll);
    avg.pitch = averageAngle(a.pitch, b.pitch);
    avg.yaw   = averageAngle(a.yaw,   b.yaw);
    tf2::Quaternion q; q.setRPY(avg.roll, avg.pitch, avg.yaw);
    return QuatPose3D{ avg.x, avg.y, avg.z, q.getW(), q.getX(), q.getY(), q.getZ() };
}

quot_tag_info_t MultiMarkerPoseEstimator::moveHalfTagInfo(
    const quot_tag_info_t& tag,
    const tag_offset_t& offset,
    bool inverse
) {
    // 元の Transform を一行で作成
    tf2::Transform original_tf{
      tf2::Quaternion{tag.pose.qx, tag.pose.qy, tag.pose.qz, tag.pose.qw},
      tf2::Vector3{tag.pose.x, tag.pose.y, tag.pose.z}
    };

    // オフセットを作る小さなラムダ（軸–角度回転＋並進をまとめる）
    auto makeOffset = [&](double factor){
      tf2::Quaternion q;
      q.setRotation(tf2::Vector3{0,0,1}, offset.dyaw * factor);
      return tf2::Transform{q, tf2::Vector3{
        offset.dx * factor,
        offset.dy * factor,
        offset.dz * factor
      }};
    };

    // 半分オフセットだけは常に使う
    tf2::Transform offHalf = makeOffset(0.5);

    // inverse フラグで式を切り替え
    tf2::Transform result_tf = original_tf
      * ( inverse
          ? makeOffset(1.0).inverse() * offHalf
          : offHalf
        );

    // --- 結果抽出（省略） ---
    tf2::Vector3 t = result_tf.getOrigin();
    tf2::Quaternion q_new = result_tf.getRotation();
    quot_tag_info_t moved_tag = tag;
    moved_tag.pose.x  = t.x();
    moved_tag.pose.y  = t.y();
    moved_tag.pose.z  = t.z();
    moved_tag.pose.qw = q_new.getW();
    moved_tag.pose.qx = q_new.getX();
    moved_tag.pose.qy = q_new.getY();
    moved_tag.pose.qz = q_new.getZ();

    return moved_tag;
}

quot_tag_info_t MultiMarkerPoseEstimator::processNode(const tag_node_t& node, std::vector<quot_tag_info_t>& tag_info_list) {
    quot_tag_info_t combined_tag = {0, 0, 0.0, {}};

    if (node.left_child && node.right_child) {
        // 子ノードを持つ場合
        quot_tag_info_t left_tag = processNode(*node.left_child, tag_info_list);
        quot_tag_info_t right_tag = processNode(*node.right_child, tag_info_list);

        // if (left_tag.marker_flag == 1) {
        //     // QuatPose3D を直接取り出してログ出力
        //     const auto& qp = left_tag.pose;
        //     std::cout << "[processNode] Left child: "
        //             << "x="  << qp.x  << ", y="  << qp.y  << ", z="  << qp.z
        //             << ", qw=" << qp.qw << ", qx=" << qp.qx
        //             << ", qy=" << qp.qy << ", qz=" << qp.qz 
        //             << std::endl;
        // }
        // if (right_tag.marker_flag == 1) {
        //     const auto& qp = right_tag.pose;
        //     std::cout << "[processNode] Right child: "
        //             << "x="  << qp.x  << ", y="  << qp.y  << ", z="  << qp.z
        //             << ", qw=" << qp.qw << ", qx=" << qp.qx
        //             << ", qy=" << qp.qy << ", qz=" << qp.qz 
        //             << std::endl;
        // }

        if (left_tag.marker_flag == 1 && right_tag.marker_flag == 1) {
            // ① 半オフセット適用
            auto left_moved  = moveHalfTagInfo(left_tag,  *node.tag_offset, /*inverse=*/false);
            auto right_moved = moveHalfTagInfo(right_tag, *node.tag_offset, /*inverse=*/true);

            // ② Pose3D(RPY) に変換
            Pose3D pL = toPose3D(left_moved.pose);
            Pose3D pR = toPose3D(right_moved.pose);

            // ③ RPYベースで組み合わせ可否判定（サイズは大きい方を基準に）
            double ref_size = std::max(left_tag.size, right_tag.size);
            if (validatePairRPY(pL, pR, /*threshold_pct=*/50.0, ref_size)) {
                // 成功したら平均RPYで合成
                combined_tag.id         = left_tag.id + right_tag.id;
                combined_tag.size       = (left_tag.size + right_tag.size) * 2;
                combined_tag.pose       = averageQuatFromPose(pL, pR);
                combined_tag.marker_flag = 1;
            } else {
                // フォールバック: 大きい方を半オフセット適用でそのまま使う
                if (left_tag.size >= right_tag.size) {
                    combined_tag = left_moved;
                } else {
                    combined_tag = right_moved;
                }
                combined_tag.marker_flag = 1;
            }
        } else if (left_tag.marker_flag == 1) {
            combined_tag = moveHalfTagInfo(left_tag, *node.tag_offset, /*inverse=*/false);
            combined_tag.marker_flag = 1;
        } else if (right_tag.marker_flag == 1) {
            combined_tag = moveHalfTagInfo(right_tag, *node.tag_offset, /*inverse=*/true);
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

    // // --- ログ出力 ---
    // std::cout << "[detectAndEstimate] Tag ID=" << output_tag.id
    //           << "  Pos=(x="   << rpy_pose.x   << ", y="   << rpy_pose.y   << ", z="   << rpy_pose.z   << ")"
    //           << "  RPY=(roll=" << rpy_pose.roll << ", pitch=" << rpy_pose.pitch << ", yaw=" << rpy_pose.yaw << ")"
    //           << std::endl;

    return output_tag;
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