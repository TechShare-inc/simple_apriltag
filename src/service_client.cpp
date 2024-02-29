#include "rclcpp/rclcpp.hpp"
#include "apriltag_service/srv/detect_apriltag.hpp"

using namespace std::chrono_literals; // この行を追加して、'1s' のような時間リテラルを使えるようにします。

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("apriltag_client_simple");
    auto client = node->create_client<apriltag_service::srv::DetectApriltag>("detect_apriltag");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service to appear...");
    }

    auto request = std::make_shared<apriltag_service::srv::DetectApriltag::Request>();
    // リクエストパラメータは、必要に応じてここで設定します。

    auto future_result = client->async_send_request(request);

    // レスポンスを待機
    if (rclcpp::spin_until_future_complete(node, future_result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future_result.get();
        if (response->result == 0) {
            RCLCPP_INFO(node->get_logger(), "AprilTag detected: ID %ld at [x: %f, y: %f, z: %f, rotation: %f]",
                        response->apriltag_id, response->x, response->y, response->z, response->rotation);
        } else if (response->result == 99) {
            RCLCPP_ERROR(node->get_logger(), "Camera capture failed.");
        } else {
            RCLCPP_WARN(node->get_logger(), "AprilTag detection failed.");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call failed.");
    }

    rclcpp::shutdown();
    return 0;
}
