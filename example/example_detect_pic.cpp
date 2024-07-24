#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "simple_tag.h"

int main(int argc, char** argv) {
    // コマンドライン引数で画像ファイルパスを受け取る
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return -1;
    }
    std::string image_path = argv[1];

    // カメラパラメータを設定
    cam_info_t cam_info = {826.1, 826.1, 640, 360}; // fx, fy, cx, cy: go2 HD cam

    // AprilTag検出器を初期化
    DetectApriltag detector(cam_info);

    // 画像を読み込む
    cv::Mat frame = cv::imread(image_path);
    if (frame.empty()) {
        std::cerr << "cannot open image: " << image_path << std::endl;
        return -1;
    }

    // TAG SIZE
    detector.setTagSize(0.15); // [m]

    // 複数のAprilTagを検出し、結果を取得
    std::vector<apriltag_t> tags = detector.detect_multiple_apriltags(frame, frame);

    // 検出結果を画面に表示
    for (const auto& tag : tags) {
        if (tag.marker_flag) {
            std::cout << "detected AprilTag ID: " << tag.apriltag_id << std::endl;
            // ここで2Dポーズ情報を取得して表示
            Pose2D pose2D = detector.convertTo2DPose(tag.pose);
            std::cout << "2D Pose: x=" << pose2D.x << ", y=" << pose2D.y << ", z=" << pose2D.z
                      << ", rotation=" << pose2D.rotation << " rad" << std::endl;
        }
    }

    // 結果を表示
    cv::imshow("detect AprilTag", frame);
    cv::waitKey(0);

    return 0;
}
