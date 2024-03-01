#include "simple_tag.h"

TagCalculate::TagCalculate(const cam_info_t& cameraInfo)
 : cam_info(cameraInfo) {}

void TagCalculate::tag_calculate(apriltag_t& data, apriltag_detection_t* det){
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

DetectApriltag::DetectApriltag(const cam_info_t& cameraInfo)
 : tag_calculate(cameraInfo), tf(tag36h11_create())
{
    apriltag_detector_add_family(td, tf);
}

DetectApriltag::~DetectApriltag() {
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
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
      printf("no marker! \n");
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

  //マーカー検出があったの場合: calculate tag-pose
  data.marker_flag = 1;
  data.apriltag_id = 0;

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

Pose2D DetectApriltag::convertTo2DPose(const apriltag_pose_t& pose) {
    return tag_calculate.convertTo2DPose(pose);
}
