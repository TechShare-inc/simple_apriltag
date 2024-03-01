#include "rclcpp/rclcpp.hpp"
#include "apriltag_service/srv/detect_apriltag.hpp"
#include "opencv2/opencv.hpp"
#include "simple_tag.h"

class AprilTagService : public rclcpp::Node
{
public:
    AprilTagService()
    : Node("apriltag_service"), 
      cap(0), // 0はデフォルトのカメラを指す
      detector({826.1, 826.1, 640, 360}) // 必要な引数を渡して初期化
    {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera");
            rclcpp::shutdown();
        }

        service_ = this->create_service<apriltag_service::srv::DetectApriltag>(
            "detect_apriltag", std::bind(&AprilTagService::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_service_request(
        const std::shared_ptr<apriltag_service::srv::DetectApriltag::Request> request,
        std::shared_ptr<apriltag_service::srv::DetectApriltag::Response> response)
    {
        cv::Mat frame;

        if (!cap.read(frame)) {
            response->result = 99; // カメラからのキャプチャ失敗
            response->apriltag_id = -1;
            response->x = 0;
            response->y = 0;
            response->z = 0;
            response->rotation = 0;
            return;
        }

        // リクエストからtag_sizeを取得して設定
        detector.setTagSize(request->tag_size);

        // ここでAprilTag検出ロジックを実装...
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
