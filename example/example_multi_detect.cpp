#include <iostream>
#include <opencv2/opencv.hpp>
#include "multi_marker_pose_estimator.h"

int main(int argc, char** argv) {
    // if (argc < 2) {
    //     std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
    //     return -1;
    // }
    // std::string image_path = argv[1];

    // cam_info_t cam_info = {528.433756558705, 528.433756558705, 320.5, 240.5}; // diffbot
    cam_info_t cam_info = {632.7, 630.6, 640, 480}; // cyborg incam
    double THRESHOLD_PERCENTAGE = 10.0;

    MultiMarkerPoseEstimator pose_estimator;
    pose_estimator.detector.setCamInfo(cam_info);

    // cv::Mat frame = cv::imread(image_path);
    // if (frame.empty()) {
    //     std::cerr << "Cannot open image: " << image_path << std::endl;
    //     return -1;
    // }

    // cv::Mat output_frame;
    // frame.copyTo(output_frame); // Ensure output_frame is initialized properly


    // GStreamerパイプラインでVideoCaptureオブジェクトを開く
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "cannot open camera" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        // カメラからフレームをキャプチャ
        cap >> frame;
        if (frame.empty()) break;
    cv::Mat output_frame;
    frame.copyTo(output_frame); // Ensure output_frame is initialized properly


    // トーナメント形式の構造を定義
    // tag_node_t root = {
    //     std::nullopt,
    //     tag_offset_t{0.0, -0.0735},
    //     std::make_unique<tag_node_t>(tag_node_t{
    //         tag_info_t{501, 1, 0.078, {}},
    //         std::nullopt,
    //         nullptr,
    //         nullptr
    //     }),
    //     std::make_unique<tag_node_t>(tag_node_t{
    //         std::nullopt,
    //         tag_offset_t{-0.0485, 0.0},
    //         std::make_unique<tag_node_t>(tag_node_t{
    //             tag_info_t{301, 1, 0.039, {}},
    //             std::nullopt,
    //             nullptr,
    //             nullptr
    //         }),
    //         std::make_unique<tag_node_t>(tag_node_t{
    //             tag_info_t{302, 1, 0.039, {}},
    //             std::nullopt,
    //             nullptr,
    //             nullptr
    //         })
    //     })
    // };

    // タグ情報を設定
    uint16_t root_id = 521;
    double root_size = 0.088;
    uint16_t left_id = 522;
    uint16_t right_id = 523;
    tag_node_t root = pose_estimator.createTripletTagNode(root_id, root_size, left_id, right_id);

    // // タグ情報を設定
    // uint16_t root_id = 401;
    // double root_size = 0.0348;
    // uint16_t ll_id = 402;
    // uint16_t l_id = 403;
    // uint16_t r_id = 404;
    // uint16_t rr_id = 405;
    // tag_node_t root = pose_estimator.createQuattroPlusNode(root_id, root_size, ll_id, l_id, r_id, rr_id);

    tag_info_t detected_tag = pose_estimator.detectAndEstimate(frame, output_frame, root);

    if (detected_tag.marker_flag == 1) {
        std::cout << "Detected Tag ID: " << detected_tag.id << std::endl;
        std::cout << "Tag Size: " << detected_tag.size << std::endl;
        std::cout << "Tag Pose: x=" << detected_tag.pose.x
                  << ", y=" << detected_tag.pose.y
                  << ", z=" << detected_tag.pose.z
                  << ", roll=" << detected_tag.pose.roll
                  << ", pitch=" << detected_tag.pose.pitch
                  << ", yaw=" << detected_tag.pose.yaw << std::endl;
    } else {
        std::cerr << "Could not detect or integrate tags." << std::endl;
    }

    if (!output_frame.empty()) {
        cv::imshow("Detected Markers", output_frame);
        // cv::waitKey(0);
        // 'q' キーでループを抜ける
        if (cv::waitKey(1) == 'q') break;
    } else {
        std::cerr << "Output frame is empty, cannot display." << std::endl;
    }

    }

    return 0;
}
