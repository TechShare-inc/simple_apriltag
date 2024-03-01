#include "rclcpp/rclcpp.hpp"
#include "apriltag_service/srv/detect_apriltag.hpp"
#include "opencv2/opencv.hpp"
#include "simple_tag.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

class AprilTagService : public rclcpp::Node
{
public:
    AprilTagService()
    : Node("apriltag_service"),
      cap("udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert ! appsink sync=false"),
      detector({826.1, 826.1, 640, 360}), // 必要な引数を渡して初期化
      running(true) // スレッド実行状態をtrueに設定
    {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera");
            rclcpp::shutdown();
        } else {
            // フレーム取得スレッドを起動
            frame_thread = std::thread(&AprilTagService::capture_frame, this);
        }

        service_ = this->create_service<apriltag_service::srv::DetectApriltag>(
            "detect_apriltag", std::bind(&AprilTagService::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~AprilTagService() {
        // スレッドの終了を指示し、終了を待つ
        running = false;
        if (frame_thread.joinable()) {
            frame_thread.join();
        }
    }

private:
    void capture_frame() {
        while (running) {
            cv::Mat frame_local;
            if (cap.read(frame_local)) {
                std::lock_guard<std::mutex> lock(frame_mutex);
                frame = frame_local;
                last_frame_time = std::chrono::steady_clock::now(); // 画像を取得した現在時刻を記録
            }
            // 適度にスリープしてリソースを圧迫しないようにする
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void handle_service_request(
        const std::shared_ptr<apriltag_service::srv::DetectApriltag::Request> request,
        std::shared_ptr<apriltag_service::srv::DetectApriltag::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service request received");

        auto now = std::chrono::steady_clock::now();
        cv::Mat frame_copy;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            auto time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_frame_time).count();

            if (frame.empty() || time_elapsed > 1) {
                response->result = 99; // フレームが古すぎる
                return;
            }
            frame.copyTo(frame_copy);
        }

        // リクエストからtag_sizeを取得して設定
        detector.setTagSize(request->tag_size);
        // AprilTag検出ロジック...
        apriltag_t tag = detector.detect_apriltag(frame, frame);
        if (tag.marker_flag) {
            response->result = 0; // 成功
            response->apriltag_id = tag.apriltag_id;
            Pose2D pose2D = detector.convertTo2DPose(tag.pose);
            response->x = pose2D.x;
            response->y = pose2D.y;
            response->z = pose2D.z;
            response->rotation = pose2D.rotation;
        } else {
            response->result = 1; // 検出失敗
            response->apriltag_id = -1;
            response->x = 0;
            response->y = 0;
            response->z = 0;
            response->rotation = 0;
        }
    }

    cv::VideoCapture cap;
    rclcpp::Service<apriltag_service::srv::DetectApriltag>::SharedPtr service_;
    DetectApriltag detector;  // AprilTag検出器
    std::thread frame_thread;
    std::mutex frame_mutex;
    cv::Mat frame;
    std::chrono::steady_clock::time_point last_frame_time; // 最後にフレームを取得した時刻
    std::atomic_bool running; // スレッド実行状態のフラグ
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagService>();
    RCLCPP_INFO(node->get_logger(), "AprilTag Service Ready.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
