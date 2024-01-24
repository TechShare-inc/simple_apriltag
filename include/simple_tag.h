#ifndef SIMPLE_TAG_H
#define SIMPLE_TAG_H

#include "opencv2/opencv.hpp"

#include "apriltag.h"
#include "tag36h11.h"

#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

#include "apriltag_pose.h" //pose estimation lib

typedef struct{
  uint8_t marker_flag;
  uint16_t apriltag_id;
  apriltag_pose_t pose;
}apriltag_t;

typedef struct{
  double CAM_FX; // [px]
  double CAM_FY; // [px]
  double CAM_CX; // [px]: half of camera width
  double CAM_CY; // [px]: half of camera height
}cam_info_t;

class TagCalculate{
private:
  const double TAG_SIZE; // [m]
  const cam_info_t cam_info;
public:
  TagCalculate(double tagSize, const cam_info_t& cameraInfo);
  void tag_calculate(apriltag_t& data, apriltag_detection_t* det);
};

class DetectApriltag{
private:
  TagCalculate tag_calculate;
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_family_t *tf = NULL;
  zarray_t *detections;
  bool detect_tag(cv::Mat& frame);
  void draw(cv::Mat& frame, cv::Point upper_right, cv::Point lower_left);
public:
  DetectApriltag(double tagSize, const cam_info_t& cameraInfo);
  ~DetectApriltag();
  DetectApriltag(const DetectApriltag&) = delete;
  DetectApriltag& operator=(const DetectApriltag&) = delete;
  apriltag_t detect_apriltag(cv::Mat& frame, cv::Mat& output_frame);
};

#endif  // SIMPLE_TAG_H