#include "rclcpp/rclcpp.hpp"
#include "apriltag_service/srv/detect_apriltag.hpp"

using DetectApriltag = apriltag_service::srv::DetectApriltag;
rclcpp::Node::SharedPtr g_node = nullptr;

void apriltag_service_response_callback(
    rclcpp::Client<DetectApriltag>::SharedFuture future)
{
    auto response = future.get();
    if (response->result == 0) {
        RCLCPP_INFO(g_node->get_logger(), "AprilTag detected: ID %d at [x: %f, y: %f, z: %f, rotation: %f]",
                    response->apriltag_id, response->x, response->y, response->z, response->rotation);
    } else if (response->result == 99) {
        RCLCPP_ERROR(g_node->get_logger(), "Camera capture failed.");
    } else {
        RCLCPP_WARN(g_node->get_logger(), "AprilTag detection failed.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("apriltag_client");
    auto client = g_node->create_client<DetectApriltag>("detect_apriltag");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(g_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(g_node->get_logger(), "Waiting for service to appear...");
    }

    auto request = std::make_shared<DetectApriltag::Request>();
    // ここでリクエストパラメータを設定できます。今回のケースでは、リクエストの内容は特に指定されていません。

    auto future_result = client->async_send_request(request, apriltag_service_response_callback);
    
    // クライアントがサービスレスポンスを待機
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    return 0;
}
