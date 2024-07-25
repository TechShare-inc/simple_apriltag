#include "simple_tag.h"

TagCalculate::TagCalculate(const cam_info_t& cameraInfo)
 : cam_info(cameraInfo) {}

void TagCalculate::tag_calculate(apriltag_t& data, apriltag_detection_t* det) {
    apriltag_detection_info_t info;
    info.det = det;
    info.tagsize = TAG_SIZE;
    info.fx = cam_info.CAM_FX;
    info.fy = cam_info.CAM_FY;
    info.cx = cam_info.CAM_CX;
    info.cy = cam_info.CAM_CY;

    apriltag_pose_t pose;
    double err = estimate_tag_pose(&info, &pose);

    data.marker_flag = 1;
    data.apriltag_id = det->id;
    data.size = TAG_SIZE;
    data.pose = pose;
}

void TagCalculate::tag_calculate(apriltag_t& data, apriltag_detection_t* det, double tag_size) {
    apriltag_detection_info_t info;
    info.det = det;
    info.tagsize = tag_size;
    info.fx = cam_info.CAM_FX;
    info.fy = cam_info.CAM_FY;
    info.cx = cam_info.CAM_CX;
    info.cy = cam_info.CAM_CY;

    apriltag_pose_t pose;
    double err = estimate_tag_pose(&info, &pose);

    data.marker_flag = 1;
    data.apriltag_id = det->id;
    data.size = tag_size;
    data.pose = pose;
}

Pose2D TagCalculate::convertTo2DPose(const apriltag_pose_t& pose) {
    Pose2D pose2D;
    pose2D.x = matd_get(pose.t, 0, 0);
    pose2D.y = matd_get(pose.t, 1, 0);
    pose2D.z = matd_get(pose.t, 2, 0);
    float ang_x = std::atan2(matd_get(pose.R, 2, 0), matd_get(pose.R, 0, 0));
    float ang_z = std::atan2(-matd_get(pose.R, 1, 2), matd_get(pose.R, 1, 1));
    pose2D.rotation = (ang_x + ang_z) / 2;
    return pose2D;
}

Pose3D TagCalculate::convertTo3DPose(const apriltag_pose_t& pose) {
    Pose3D pose3D;
    pose3D.x = matd_get(pose.t, 2, 0); // tagのz方向
    pose3D.y = - matd_get(pose.t, 0, 0); // tagのx方向
    pose3D.z = - matd_get(pose.t, 1, 0); // tagのy方向

    // robot座標系においてz-y-x順序になるように、tag座標系においてy-x-z順序で取得
    double pitch = atan2(-matd_get(pose.R, 2, 0), sqrt(matd_get(pose.R, 0, 0) * matd_get(pose.R, 0, 0) + matd_get(pose.R, 1, 0) * matd_get(pose.R, 1, 0)));
    double roll = atan2(matd_get(pose.R, 2, 1), matd_get(pose.R, 2, 2));
    double yaw = atan2(matd_get(pose.R, 1, 0), matd_get(pose.R, 0, 0));
    pose3D.roll = yaw; // tagのz軸周り
    pose3D.pitch = - roll; // tagのx軸周り
    pose3D.yaw = - pitch; // tagのy軸周り

    return pose3D;
}

DetectApriltag::DetectApriltag()
 : tf(tag36h11_create())
{
    apriltag_detector_add_family(td, tf);
}

DetectApriltag::DetectApriltag(const cam_info_t& cameraInfo)
 : tag_calculate(cameraInfo), tf(tag36h11_create())
{
    apriltag_detector_add_family(td, tf);
}

DetectApriltag::~DetectApriltag() {
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

void DetectApriltag::setCamInfo(const cam_info_t& cam_info){
  tag_calculate.cam_info = cam_info;
}

void DetectApriltag::setTagSize(const double& TAG_SIZE){
  tag_calculate.TAG_SIZE = TAG_SIZE;
}

bool DetectApriltag::detect_tag(cv::Mat& frame){
    // cv::Matのグレースケール
    cv::Mat gray;
    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // cv::Matのグレースケールをimage_u8_tに変換する
    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
    };

    // 検出を行う
    detections = apriltag_detector_detect(td, &im);

    if(zarray_size(detections) == 0){ //マーカー検出がなかった場合
      // printf("no marker! \n");
      return false;
    }

    return true;
}

void DetectApriltag::draw(cv::Mat& frame, cv::Point top_right, cv::Point top_left, cv::Point bottom_left, cv::Point bottom_right){
    // 各頂点をつなげる線を描画
    cv::line(frame, top_right, bottom_right, cv::Scalar(0, 255, 0), 2);
    cv::line(frame, bottom_right, bottom_left, cv::Scalar(0, 255, 0), 2);
    cv::line(frame, bottom_left, top_left, cv::Scalar(0, 255, 0), 2);
    cv::line(frame, top_left, top_right, cv::Scalar(0, 255, 0), 2);

    // 各頂点に小さな円を描画（オプション）
    cv::circle(frame, top_right, 3, cv::Scalar(0, 0, 255), -1);
}

apriltag_t DetectApriltag::detect_apriltag(cv::Mat& frame, cv::Mat& output_frame){
  apriltag_t data;
  
  if(!detect_tag(frame)){
    // マーカー検出がない場合
    data.marker_flag = 0;
    apriltag_detections_destroy(detections);
    return data;
  }

  // マーカー検出があったの場合: calculate tag-pose
  apriltag_detection_t *det;
  zarray_get(detections, 0, &det);
  tag_calculate.tag_calculate(data, det);

  // draw
  draw(output_frame,
   cv::Point(det->p[2][0], det->p[2][1]), // top right
   cv::Point(det->p[3][0], det->p[3][1]), // top left
   cv::Point(det->p[0][0], det->p[0][1]), // bottom left
   cv::Point(det->p[1][0], det->p[1][1])); // bottom right

  apriltag_detections_destroy(detections);

  return data;
}

apriltag_t DetectApriltag::detect_apriltag(cv::Mat& frame, cv::Mat& output_frame, int tag_id){
  apriltag_t data;
  
  if(!detect_tag(frame)){
    // マーカー検出がない場合
    data.marker_flag = 0;
    apriltag_detections_destroy(detections);
    return data;
  }

  // マーカーを検出した場合、tag_idを選ぶ
  apriltag_detection_t *det;
  bool tag_found = false;
  for(int i = 0; i < zarray_size(detections); i++) {
      zarray_get(detections, i, &det);
      // printf("i: %d, id: %d, p: %f\n", i, det->id, CAM_CX - det->p[0][0]);
      if(det->id == tag_id) {
          // printf("tag_id %d was found.\n", tag_id);
          tag_found = true;
          break; // 見つけた段階でfor文を抜けるので、detには期待するものが格納されている
      }
  }

  // tag_foundの結果を出力
  if(!tag_found) {
    // printf("tag_id %d was not found.\n", tag_id);
    return data;
  }

  // 特定のマーカーがあったの場合: calculate tag-pose
  tag_calculate.tag_calculate(data, det);

  // draw
  draw(output_frame,
   cv::Point(det->p[2][0], det->p[2][1]), // top right
   cv::Point(det->p[3][0], det->p[3][1]), // top left
   cv::Point(det->p[0][0], det->p[0][1]), // bottom left
   cv::Point(det->p[1][0], det->p[1][1])); // bottom right

  apriltag_detections_destroy(detections);

  return data;
}

std::vector<apriltag_t> DetectApriltag::detect_multiple_apriltags(cv::Mat& frame, cv::Mat& output_frame){
  std::vector<apriltag_t> tags_data;
  
  if(!detect_tag(frame)){
    // マーカー検出がない場合
    apriltag_detections_destroy(detections);
    return tags_data;
  }

  // マーカー検出があった場合: calculate tag-pose
  apriltag_detection_t *det;
  for(int i = 0; i < zarray_size(detections); i++) {
    zarray_get(detections, i, &det);
    apriltag_t data;
    tag_calculate.tag_calculate(data, det);
    tags_data.push_back(data);

    // draw
    draw(output_frame,
     cv::Point(det->p[2][0], det->p[2][1]), // top right
     cv::Point(det->p[3][0], det->p[3][1]), // top left
     cv::Point(det->p[0][0], det->p[0][1]), // bottom left
     cv::Point(det->p[1][0], det->p[1][1])); // bottom right
  }

  apriltag_detections_destroy(detections);

  return tags_data;
}

std::vector<apriltag_t> DetectApriltag::detect_multiple_apriltags(cv::Mat& frame, cv::Mat& output_frame, const std::vector<std::pair<uint16_t, double>>& tag_id_size_pairs){
  std::vector<apriltag_t> tags_data;

  if(!detect_tag(frame)){
    // マーカー検出がない場合
    apriltag_detections_destroy(detections);
    return tags_data;
  }

  // マーカー検出があった場合: calculate tag-pose
  apriltag_detection_t *det;
  for(int i = 0; i < zarray_size(detections); i++) {
    zarray_get(detections, i, &det);

    for(const auto& pair : tag_id_size_pairs) {
      if(det->id == pair.first) {
        apriltag_t data;
        tag_calculate.tag_calculate(data, det, pair.second);
        tags_data.push_back(data);

        // draw
        draw(output_frame,
          cv::Point(det->p[2][0], det->p[2][1]), // top right
          cv::Point(det->p[3][0], det->p[3][1]), // top left
          cv::Point(det->p[0][0], det->p[0][1]), // bottom left
          cv::Point(det->p[1][0], det->p[1][1])); // bottom right
      }
    }
  }

  apriltag_detections_destroy(detections);

  return tags_data;
}

Pose2D DetectApriltag::convertTo2DPose(const apriltag_pose_t& pose) {
    return tag_calculate.convertTo2DPose(pose);
}

Pose3D DetectApriltag::convertTo3DPose(const apriltag_pose_t& pose) {
    return tag_calculate.convertTo3DPose(pose);
}
