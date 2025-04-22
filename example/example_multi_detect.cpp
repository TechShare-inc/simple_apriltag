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
    // cam_info_t cam_info = {632.7, 630.6, 640, 480}; // cyborg incam
    // cam_info_t cam_info = {826.1, 826.1, 640, 360}; // fx, fy, cx, cy: go2 HD cam
    // cam_info_t cam_info = {401.74007, 536.59449, 315.97372, 226.14287}; // fx, fy, cx, cy: go2 cam
    // cam_info_t cam_info = {417.0, 556.4, 316.5, 234.7}; // fx, fy, cx, cy: go2 cam
    cam_info_t cam_info = {417.0, 556.4, 320.0, 240.0}; // fx, fy, cx, cy: go2 cam

// **** Calibrating ****
// mono pinhole calibration...
// D = [-0.35647140287582224, 0.09637954376727059, -0.004640572593814717, -0.001596285274610044, 0.0]
// K = [417.083644879942, 0.0, 316.4712869364082, 0.0, 556.3866886768242, 234.69357570297618, 0.0, 0.0, 1.0]
// R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
// P = [301.43511283447174, 0.0, 310.42562068496613, 0.0, 0.0, 515.7568168994495, 231.91562491921763, 0.0, 0.0, 0.0, 1.0, 0.0]
// None
// # oST version 5.0 parameters


    MultiMarkerPoseEstimator pose_estimator;
    pose_estimator.detector.setCamInfo(cam_info);

    // cv::VideoCapture cap(0);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <network-interface> (e.g., eth0, wlan0)" << std::endl;
        return -1;
    }
    std::string network_interface = argv[1]; // コマンドラインからのネットワークインターフェース
    // // GStreamerのパイプラインを定義
    // std::string address = "230.1.1.1"; // マルチキャストアドレス
    // std::string port = "1720"; // UDPポート
    // std::string gst_cmd = "udpsrc address=" + address + " port=" + port + " multicast-iface=" + network_interface + 
    //                       " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! appsink sync=false";
    const std::string gstrem_str = "udpsrc address=230.1.1.1 port=1720 multicast-iface=" + network_interface + " ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";
    cv::VideoCapture cap(gstrem_str);

    if (!cap.isOpened()) {
        std::cerr << "cannot open camera" << std::endl;
        return -1;
    }

    cv::Mat origin_frame, frame;
    while (true) {
        // カメラからフレームをキャプチャ
        cap >> origin_frame;
        cv::resize(origin_frame, frame, cv::Size(640, 480));
        if (frame.empty()) break;
    cv::Mat output_frame;
    frame.copyTo(output_frame); // Ensure output_frame is initialized properly

    // タグ情報を設定
    uint16_t root_id = 531;
    double root_size = 0.084;
    uint16_t left_id = 532;
    uint16_t right_id = 533;
    tag_node_t root = pose_estimator.createTriplet3DTagNode(root_id, root_size, left_id, right_id);

    // // タグ情報を設定
    // uint16_t root_id = 521;
    // double root_size = 0.088;
    // uint16_t left_id = 522;
    // uint16_t right_id = 523;
    // tag_node_t root = pose_estimator.createTripletTagNode(root_id, root_size, left_id, right_id);

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
        // std::cout << "Tag Size: " << detected_tag.size << std::endl;
        // std::cout << "Tag Pose: x=" << detected_tag.pose.x
        //           << ", y=" << detected_tag.pose.y
        //           << ", z=" << detected_tag.pose.z
        //           << ", roll=" << detected_tag.pose.roll
        //           << ", pitch=" << detected_tag.pose.pitch
        std::cout           << ", yaw=" << detected_tag.pose.yaw << std::endl;
    } else {
        std::cerr << "Could not detect or integrate tags." << std::endl;
    }

    if (!output_frame.empty()) {
        // コンソール出力に加えて映像上にオーバーレイ
        std::string info = "W:" + std::to_string(frame.cols)
                        + " H:" + std::to_string(frame.rows);
        cv::putText(output_frame, info,
                    cv::Point(10, 30),                // 描画位置
                    cv::FONT_HERSHEY_SIMPLEX,         // フォント
                    1.0,                              // スケール
                    cv::Scalar(0, 255, 0),            // 色（BGR）
                    2);                               // 線の太さ
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
