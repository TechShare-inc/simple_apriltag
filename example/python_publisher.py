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

        # 1) 左の子ノード１
        tf1 = TransformStamped()
        tf1.header.stamp = self.get_clock().now().to_msg()
        tf1.header.frame_id = "world"
        tf1.child_frame_id  = "child_502"
        tf1.transform.translation.x = 0.363555
        tf1.transform.translation.y = -0.0304098
        tf1.transform.translation.z = 0.0665764
        tf1.transform.rotation.w = 0.878308
        tf1.transform.rotation.x = -0.026533
        tf1.transform.rotation.y = -0.0556838
        tf1.transform.rotation.z = -0.4741
        transforms.append(tf1)

        # 2) 右の子ノード１
        tf2 = TransformStamped()
        tf2.header.stamp = self.get_clock().now().to_msg()
        tf2.header.frame_id = "world"
        tf2.child_frame_id  = "child_503"
        tf2.transform.translation.x = 0.345759
        tf2.transform.translation.y = -0.0760018
        tf2.transform.translation.z = 0.0695614
        tf2.transform.rotation.w = 0.997934
        tf2.transform.rotation.x = -0.0435647
        tf2.transform.rotation.y = 0.0137398
        tf2.transform.rotation.z = 0.0451731
        transforms.append(tf2)

        # 3) 左の子ノード２
        tf3 = TransformStamped()
        tf3.header.stamp = self.get_clock().now().to_msg()
        tf3.header.frame_id = "world"
        tf3.child_frame_id  = "child_501"
        tf3.transform.translation.x = 0.392542
        tf3.transform.translation.y = -0.05965
        tf3.transform.translation.z = 0.15616
        tf3.transform.rotation.w = 0.986253
        tf3.transform.rotation.x = -0.044739
        tf3.transform.rotation.y = -0.0562873
        tf3.transform.rotation.z = -0.148782
        transforms.append(tf3)

        # 4) 右の子ノード２
        tf4 = TransformStamped()
        tf4.header.stamp = self.get_clock().now().to_msg()
        tf4.header.frame_id = "world"
        tf4.child_frame_id  = "combined_502_503"
        tf4.transform.translation.x = 0.35424
        tf4.transform.translation.y = -0.0530383
        tf4.transform.translation.z = 0.0676427
        tf4.transform.rotation.w = 0.973647
        tf4.transform.rotation.x = -0.0426543
        tf4.transform.rotation.y = -0.0224405
        tf4.transform.rotation.z = -0.222909
        transforms.append(tf4)

        # 5) 最後の推定タグ位置 (world -> tag_estimated)
        estimated_tf = TransformStamped()
        estimated_tf.header.stamp = self.get_clock().now().to_msg()
        estimated_tf.header.frame_id    = "world"
        estimated_tf.child_frame_id     = "tag_estimated"
        estimated_tf.transform.translation.x = 0.374645
        estimated_tf.transform.translation.y = -0.0571119
        estimated_tf.transform.translation.z = 0.111785
        # RPY → クォータニオン
        q_est = tft.quaternion_from_euler(
            -0.0726931,  # roll
            -0.093709,   # pitch
            -0.371389    # yaw
        )
        estimated_tf.transform.rotation.x = q_est[0]
        estimated_tf.transform.rotation.y = q_est[1]
        estimated_tf.transform.rotation.z = q_est[2]
        estimated_tf.transform.rotation.w = q_est[3]
        transforms.append(estimated_tf)

        # すべてのTFを一斉配信
        self._static_broadcaster.sendTransform(transforms)
        self.get_logger().info("Published 4 initial child transforms.")

def main(args=None):
    rclpy.init(args=args)
    node = TestTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
