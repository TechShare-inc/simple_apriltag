#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "simple_tag.h"

int main() {
    // カメラパラメータを設定
    cam_info_t cam_info = {826.1, 826.1, 640, 360}; // fx, fy, cx, cy: go2 HD cam
    double tagSize = 0.162; // 実際のAprilTagのサイズ (メートル単位)

    // AprilTag検出器を初期化
    DetectApriltag detector(tagSize, cam_info);

    // GStreamerのパイプラインを定義
    std::string address = "230.1.1.1"; // マルチキャストアドレス
    std::string port = "1720"; // UDPポート
    std::string gst_cmd = "udpsrc address=" + address + " port=" + port + " multicast-iface=eth0 ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! appsink sync=false";

    // GStreamerパイプラインでVideoCaptureオブジェクトを開く
    cv::VideoCapture cap(gst_cmd);
    if (!cap.isOpened()) {
        std::cerr << "cannot open camera" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        // カメラからフレームをキャプチャ
        cap >> frame;
        if (frame.empty()) break;

        // AprilTagを検出し、結果を取得
        apriltag_t tag = detector.detect_apriltag(frame, frame);

        // 検出結果を画面に表示
        if (tag.marker_flag) {
            std::cout << "detected AprilTag ID: " << tag.apriltag_id << std::endl;
            // ここで2Dポーズ情報を取得して表示
            Pose2D pose2D = detector.convertTo2DPose(tag.pose);
            std::cout << "2D Pose: x=" << pose2D.x << ", y=" << pose2D.y << ", z=" << pose2D.z
                      << ", rotation=" << pose2D.rotation << " rad" << std::endl;
        }

        // 結果を表示
        cv::imshow("detect AprilTag", frame);

        // 'q' キーでループを抜ける
        if (cv::waitKey(1) == 'q') break;
    }

    return 0;
}
