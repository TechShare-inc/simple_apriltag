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

        # Tag1 の TF (world -> tag1)
        tag1_tf = TransformStamped()
        tag1_tf.header.stamp = self.get_clock().now().to_msg()
        tag1_tf.header.frame_id = "world"
        tag1_tf.child_frame_id = "tag1"
        tag1_tf.transform.translation.x = 0.243076
        tag1_tf.transform.translation.y = 0.0339853
        tag1_tf.transform.translation.z = 0.0656177
        q1 = tft.quaternion_from_euler(3.04817, 0.0849548, -0.220579)
        tag1_tf.transform.rotation.x = q1[0]
        tag1_tf.transform.rotation.y = q1[1]
        tag1_tf.transform.rotation.z = q1[2]
        tag1_tf.transform.rotation.w = q1[3]
        transforms.append(tag1_tf)

        # Tag2 の TF (world -> tag2)
        tag2_tf = TransformStamped()
        tag2_tf.header.stamp = self.get_clock().now().to_msg()
        tag2_tf.header.frame_id = "world"
        tag2_tf.child_frame_id = "tag2"
        tag2_tf.transform.translation.x = 0.243008
        tag2_tf.transform.translation.y = 0.0891314
        tag2_tf.transform.translation.z = 0.0611855
        q2 = tft.quaternion_from_euler(3.11143, 0.0505605, 0.46143)
        tag2_tf.transform.rotation.x = q2[0]
        tag2_tf.transform.rotation.y = q2[1]
        tag2_tf.transform.rotation.z = q2[2]
        tag2_tf.transform.rotation.w = q2[3]
        transforms.append(tag2_tf)

        # 期待される変換 (tag1 -> expected)
        expected_tf = TransformStamped()
        expected_tf.header.stamp = self.get_clock().now().to_msg()
        expected_tf.header.frame_id = "tag1"
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

        # パブリッシュ
        self._static_broadcaster.sendTransform(transforms)
        self.get_logger().info("Published static transforms for tag1, tag2, and expected transform.")


def main(args=None):
    rclpy.init(args=args)
    node = TestTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
