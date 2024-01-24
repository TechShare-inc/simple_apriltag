#include <iostream>
#include "opencv2/opencv.hpp"
#include "simple_tag.h"

int main() {
    // カメラパラメータを設定 (これらは実際のカメラに合わせて調整する必要があります)
    cam_info_t cam_info = {480, 480, 960, 540}; // 例: fx, fy, cx, cy
    double tagSize = 0.162; // 実際のAprilTagのサイズ (メートル単位)

    // AprilTag検出器を初期化
    DetectApriltag detector(tagSize, cam_info);

    // カメラを開く
    cv::VideoCapture cap(0); // 0 は通常デフォルトのカメラを指します
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
            // ここで tag.pose を使用して、さらなる処理を行うことができます
        }

        // 結果を表示
        cv::imshow("detect AprilTag", frame);

        // 'q' キーでループを抜ける
        if (cv::waitKey(1) == 'q') break;
    }

    return 0;
}
