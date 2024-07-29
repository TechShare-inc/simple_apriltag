#ifndef MULTI_MARKER_POSE_ESTIMATOR_H
#define MULTI_MARKER_POSE_ESTIMATOR_H

#include "simple_tag.h"
#include "pose_utils.h"
#include <vector>
#include <optional>
#include <memory>

struct tag_offset_t {
    double tag2_y_from_tag1; // Y座標：左方向
    double tag2_z_from_tag1; // Z座標：上方向
};

struct tag_node_t {
    std::optional<tag_info_t> tag_info;
    std::optional<tag_offset_t> tag_offset;
    std::unique_ptr<tag_node_t> left_child;
    std::unique_ptr<tag_node_t> right_child;
};

class MultiMarkerPoseEstimator {
public:
    MultiMarkerPoseEstimator() = default;
    
    /*** @brief TripletTagノード構造を作成します。
     * 
     * 構造は以下の通りです:
     *       +---------+
     *       |   Root  |
     *       +---------+
     *       |  L | R  |
     *       +---------+
     * Rootは親タグ、Lは左子タグ、Rは右子タグを示します。
     * 子タグ (L, R) は親タグ (Root) の半分の大きさです。
     * 
     * @param root_id 親タグのID
     * @param root_size 親タグのサイズ
     * @param left_id 左子タグのID
     * @param right_id 右子タグのID
     * @return TripletTagを表すtag_node_t構造体
     */
    tag_node_t createTripletTagNode(uint16_t root_id, double root_size, uint16_t left_id, uint16_t right_id);

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
    * @param root_id 親タグのID
    * @param root_size 親タグのサイズ
    * @param left_id 左子タグのID
    * @param right_id 右子タグのID
    * @param ll_id 左外側タグのID
    * @param rr_id 右外側タグのID
    * @return QuattroPlusを表すtag_node_t構造体
    */
    tag_node_t createQuattroPlusNode(uint16_t root_id, double root_size, uint16_t left_id, uint16_t right_id, uint16_t ll_id, uint16_t rr_id);

    tag_info_t detectAndEstimate(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root);
    DetectApriltag detector;

private:
    std::vector<tag_info_t> collectTagsAndDetect(cv::Mat& frame, cv::Mat& output_frame, const tag_node_t& root);
    tag_info_t moveTagInfo(const tag_info_t& tag, double y_offset, double z_offset);
    tag_info_t processNode(const tag_node_t& node, std::vector<tag_info_t>& tag_info_list);
    bool validateAndEstimatePair(tag_info_t& combined_tag, const tag_info_t& tag1, const tag_info_t& tag2, const tag_offset_t& offset, double threshold_percentage);
    Pose3D calculateAveragePose(const Pose3D& pose1, const Pose3D& pose2);
};

#endif // MULTI_MARKER_POSE_ESTIMATOR_H
