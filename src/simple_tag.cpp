#include "simple_tag.h"

TagCalculate::TagCalculate(double tagSize, const cam_info_t& cameraInfo)
 : TAG_SIZE(tagSize), cam_info(cameraInfo) {}

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

DetectApriltag::DetectApriltag(double tagSize, const cam_info_t& cameraInfo)
 : tag_calculate(tagSize, cameraInfo), tf(tag36h11_create())
{
    apriltag_detector_add_family(td, tf);
}

DetectApriltag::~DetectApriltag() {
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
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

void DetectApriltag::draw(cv::Mat& frame, cv::Point upper_right, cv::Point lower_left){
  cv::circle(frame, upper_right, 3, cv::Scalar(0, 0, 255), -1);
  cv::rectangle(frame, lower_left, upper_right, cv::Scalar(0, 255, 0), 1);
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
  draw(output_frame, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[2][0], det->p[2][1]));

  apriltag_detections_destroy(detections);

  return data;
}
