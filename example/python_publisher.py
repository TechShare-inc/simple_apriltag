#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import tf_transformations as tft  # pip install tf-transformations

class TestTFPublisher(Node):
    def __init__(self):
        super().__init__('test_tf_publisher')
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_transforms()

    def publish_transforms(self):
        transforms = []

        # 1) TF1: 移動量＋固定の回転（world -> tag1_moving）
        tag1_moving_tf = TransformStamped()
        tag1_moving_tf.header.stamp = self.get_clock().now().to_msg()
        tag1_moving_tf.header.frame_id = "world"
        tag1_moving_tf.child_frame_id = "tag1_moving"  # 名前は任意。元の "tag1" と分けるために変更
        tag1_moving_tf.transform.translation.x = 0.297148
        tag1_moving_tf.transform.translation.y = 0.0509337
        tag1_moving_tf.transform.translation.z = -0.0121608
        # 固定の回転(z-y-x)
        q_fixed = tft.quaternion_from_euler(-math.pi/2, 0.0, -math.pi/2)
        self.get_logger().info(f"Quaternion values - x: {q_fixed[0]}, y: {q_fixed[1]}, z: {q_fixed[2]}, w: {q_fixed[3]}")
        # tag1_moving_tf.transform.rotation.x = q_fixed[0]
        # tag1_moving_tf.transform.rotation.y = q_fixed[1]
        # tag1_moving_tf.transform.rotation.z = q_fixed[2]
        # tag1_moving_tf.transform.rotation.w = q_fixed[3]
        tag1_moving_tf.transform.rotation.x = -0.5
        tag1_moving_tf.transform.rotation.y = 0.5
        tag1_moving_tf.transform.rotation.z = -0.5
        tag1_moving_tf.transform.rotation.w = 0.5
        transforms.append(tag1_moving_tf)

        # 2) TF2: 移動なし＋既存の回転（tag1_moving -> tag1）
        tag1_tf = TransformStamped()
        tag1_tf.header.stamp = self.get_clock().now().to_msg()
        tag1_tf.header.frame_id = "tag1_moving"
        tag1_tf.child_frame_id = "tag1"  # 名前は任意（元の tag1 と区別）
        tag1_tf.transform.translation.x = 0.0
        tag1_tf.transform.translation.y = 0.0
        tag1_tf.transform.translation.z = 0.0
        # 既存の回転: Euler(0.00495003, 0.406763, -0.0432819)
        q_orig = tft.quaternion_from_euler(0.00495003, 0.406763, -0.0432819)
        tag1_tf.transform.rotation.x = q_orig[0]
        tag1_tf.transform.rotation.y = q_orig[1]
        tag1_tf.transform.rotation.z = q_orig[2]
        tag1_tf.transform.rotation.w = q_orig[3]
        transforms.append(tag1_tf)

        # 3) TF2: 移動なし＋固定の回転（tag1 -> tag1_robot）
        tag1_robot_tf = TransformStamped()
        tag1_robot_tf.header.stamp = self.get_clock().now().to_msg()
        tag1_robot_tf.header.frame_id = "tag1"
        tag1_robot_tf.child_frame_id = "tag1_robot"  # 名前は任意（元の tag1 と区別）
        tag1_robot_tf.transform.translation.x = 0.0
        tag1_robot_tf.transform.translation.y = 0.0
        tag1_robot_tf.transform.translation.z = 0.0
        q_fixed = tft.quaternion_from_euler(0.0, -math.pi/2, math.pi)
        self.get_logger().info(f"Quaternion values - x: {q_fixed[0]}, y: {q_fixed[1]}, z: {q_fixed[2]}, w: {q_fixed[3]}")
        # tag1_moving_tf.transform.rotation.x = q_fixed[0]
        # tag1_moving_tf.transform.rotation.y = q_fixed[1]
        # tag1_moving_tf.transform.rotation.z = q_fixed[2]
        # tag1_moving_tf.transform.rotation.w = q_fixed[3]
        tag1_robot_tf.transform.rotation.x = q_orig[0]
        tag1_robot_tf.transform.rotation.y = q_orig[1]
        tag1_robot_tf.transform.rotation.z = q_orig[2]
        tag1_robot_tf.transform.rotation.w = q_orig[3]
        transforms.append(tag1_robot_tf)

        # # Tag1 の TF (world -> tag1)
        # tag1_tf = TransformStamped()
        # tag1_tf.header.stamp = self.get_clock().now().to_msg()
        # tag1_tf.header.frame_id = "world"
        # tag1_tf.child_frame_id = "tag1"
        # tag1_tf.transform.translation.x = 0.294068
        # tag1_tf.transform.translation.y = -0.00376796
        # tag1_tf.transform.translation.z = 0.0217497
        # # Euler角 (roll, pitch, yaw)
        # q1 = tft.quaternion_from_euler(-1.37314, 0.0815553, -2.02797)
        # tag1_tf.transform.rotation.x = q1[0]
        # tag1_tf.transform.rotation.y = q1[1]
        # tag1_tf.transform.rotation.z = q1[2]
        # tag1_tf.transform.rotation.w = q1[3]
        # transforms.append(tag1_tf)

        # Tag2 の TF (world -> tag2)
        tag2_tf = TransformStamped()
        tag2_tf.header.stamp = self.get_clock().now().to_msg()
        tag2_tf.header.frame_id = "world"
        tag2_tf.child_frame_id = "tag2"
        tag2_tf.transform.translation.x = 0.29749
        tag2_tf.transform.translation.y = -0.0594111
        tag2_tf.transform.translation.z = 0.0211601
        # Euler角 (roll, pitch, yaw)
        q2 = tft.quaternion_from_euler(-1.44934, -0.0847131, -1.15439)
        tag2_tf.transform.rotation.x = q2[0]
        tag2_tf.transform.rotation.y = q2[1]
        tag2_tf.transform.rotation.z = q2[2]
        tag2_tf.transform.rotation.w = q2[3]
        transforms.append(tag2_tf)



        # expected TF (tag1 -> expected) はそのまま
        expected_tf = TransformStamped()
        expected_tf.header.stamp = self.get_clock().now().to_msg()
        expected_tf.header.frame_id = "tag1"  # 注意: expected は tag1 を親フレームにしています
        expected_tf.child_frame_id = "expected"
        expected_tf.transform.translation.x = 0.0
        expected_tf.transform.translation.y = 0.0
        expected_tf.transform.translation.z = 0.0
        q_expected = tft.quaternion_from_euler(0.0, 0.0, math.pi/3)
        expected_tf.transform.rotation.x = q_expected[0]
        expected_tf.transform.rotation.y = q_expected[1]
        expected_tf.transform.rotation.z = q_expected[2]
        expected_tf.transform.rotation.w = q_expected[3]
        transforms.append(expected_tf)

        # すべてのTFを同時にパブリッシュ
        self._static_broadcaster.sendTransform(transforms)
        self.get_logger().info("Published static transforms for tag1_moving, tag1_static, tag2, and expected transform.")

def main(args=None):
    rclpy.init(args=args)
    node = TestTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
