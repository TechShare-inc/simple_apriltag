#ifndef MULTI_MARKER_POSE_ESTIMATOR_H
#define MULTI_MARKER_POSE_ESTIMATOR_H

#include "simple_tag.h"
#include "pose_utils.h"
#include <vector>
#include <optional>
#include <memory>
#include <cmath>

typedef struct {
    double x;     // X座標：前方向
    double y;     // Y座標：左方向
    double z;     // Z座標：上方向
    double qw;    // クォータニオンの実部
    double qx;    // クォータニオンのx成分
    double qy;    // クォータニオンのy成分
    double qz;    // クォータニオンのz成分
} QuatPose3D;

struct quot_tag_info_t {
    uint16_t id;
    uint8_t marker_flag;  // 0: 検出失敗または統合失敗, 1: 検出成功または統合成功
    double size;
    QuatPose3D pose;

    bool operator==(const tag_info_t& other) const {
        return id == other.id;
    }
};

// 変更後のオフセット構造体：x,y,zとyawのオフセットを保持（roll, pitchは固定）
struct tag_offset_t {
    double dx;     // X方向オフセット
    double dy;     // Y方向オフセット
    double dz;     // Z方向オフセット
    double dyaw;   // Yaw方向オフセット (roll, pitchは固定)
};

struct tag_node_t {
    std::optional<quot_tag_info_t> tag_info;
    std::optional<tag_offset_t> tag_offset;
    std::unique_ptr<tag_node_t> left_child;
    std::unique_ptr<tag_node_t> right_child;
};

class MultiMarkerPoseEstimator {
public:
    MultiMarkerPoseEstimator() = default;

    /*** @brief Triplet3DTagノード構造を作成します。
     * 
     * 構造は以下の通りです:
     *       +---------+
     *       |  Parent |
     *       +---------+
     *       |  L | R  |
     *       +---------+
     * Parentは親タグ、Lは左子タグ、Rは右子タグを示します。
     * 子タグ (L, R) は親タグ (Parent) の半分の大きさで、30°傾いています。
     * 
     * @param parent_tag_id 親タグのID
     * @param parent_tag_size 親タグのサイズ
     * @param left_id 左子タグのID
     * @param right_id 右子タグのID
     * @return TripletTagを表すtag_node_t構造体
     */
    tag_node_t createTriplet3DTagNode(uint16_t parent_tag_id, double parent_tag_size, uint16_t left_id, uint16_t right_id);

    /*** @brief TripletTagノード構造を作成します。
     * 
     * 構造は以下の通りです:
     *       +---------+
     *       |  Parent |
     *       +---------+
     *       |  L | R  |
     *       +---------+
     * Parentは親タグ、Lは左子タグ、Rは右子タグを示します。
     * 子タグ (L, R) は親タグ (Parent) の半分の大きさです。
     * 
     * @param parent_tag_id 親タグのID
     * @param parent_tag_size 親タグのサイズ
     * @param left_id 左子タグのID
     * @param right_id 右子タグのID
     * @return TripletTagを表すtag_node_t構造体
     */
    tag_node_t createTripletTagNode(uint16_t parent_tag_id, double parent_tag_size, uint16_t left_id, uint16_t right_id);

    /*** @brief QuattroPlusノード構造を作成します。
    * 
    * 構造は以下の通りです:
    *  +----+---------+----+
    *  | LL |  L | R  | RR |
    *  +----+---------+----+
    *       |   Root  |
    *       +---------+
    * Rootは親タグ、Lは左子タグ、Rは右子タグ、LLは左外側タグ、RRは右外側タグを示します。
    * 子タグ (LL, L, R, RR) はすべて親タグ (Root) の半分の大きさです。
    * 
    * @param parent_tag_id 親タグのID
    * @param parent_tag_size 親タグのサイズ
    * @param left_id 左子タグのID
    * @param right_id 右子タグのID
    * @param ll_id 左外側タグのID
    * @param rr_id 右外側タグのID
    * @return QuattroPlusを表すtag_node_t構造体
    */
    tag_node_t createQuattroPlusNode(uint16_t parent_tag_id, double parent_tag_size, uint16_t left_id, uint16_t right_id, uint16_t ll_id, uint16_t rr_id);

    tag_info_t detectAndEstimate(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root);
    DetectApriltag detector;

private:
    std::vector<quot_tag_info_t> collectTagsAndDetect(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root);
    quot_tag_info_t moveHalfTagInfo(const quot_tag_info_t& tag, const tag_offset_t& offset, const bool inverse = false);
    quot_tag_info_t processNode(const tag_node_t& node, std::vector<quot_tag_info_t>& tag_info_list);
    bool validateAndEstimatePair(quot_tag_info_t& combined_tag, const quot_tag_info_t& tag1, const quot_tag_info_t& tag2, const tag_offset_t& offset, double threshold);
    QuatPose3D calculateAveragePose(const QuatPose3D& pose1, const QuatPose3D& pose2);
    QuatPose3D convertToQuat3DPose(const apriltag_pose_t& pose);
};

#endif // MULTI_MARKER_POSE_ESTIMATOR_H
