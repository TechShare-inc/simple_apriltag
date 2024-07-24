#include <iostream>
#include "pose_utils.h"

int main() {
    // テスト1: RPYからクオータニオンへの変換
    double roll = 0.1, pitch = 0.2, yaw = 0.3;
    tf2::Quaternion q = rpyToQuaternion(roll, pitch, yaw);
    std::cout << "Quaternion from RPY: x=" << q.x() << ", y=" << q.y() << ", z=" << q.z() << ", w=" << q.w() << std::endl;

    // テスト2: 座標とRPYからTransformの作成
    double x = 1.0, y = 2.0, z = 3.0;
    tf2::Transform transform = createTransform(x, y, z, roll, pitch, yaw);
    tf2::Vector3 origin = transform.getOrigin();
    tf2::Quaternion rotation = transform.getRotation();
    std::cout << "Transform: Origin(x=" << origin.x() << ", y=" << origin.y() << ", z=" << origin.z()
              << "), Rotation(x=" << rotation.x() << ", y=" << rotation.y() << ", z=" << rotation.z() << ", w=" << rotation.w() << ")" << std::endl;

    // テスト3: Transformの掛け算
    tf2::Transform t1 = createTransform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    tf2::Transform t2 = createTransform(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
    tf2::Transform result = multiplyTransforms(t1, t2);
    tf2::Vector3 result_origin = result.getOrigin();
    std::cout << "Result Transform: Origin(x=" << result_origin.x() << ", y=" << result_origin.y() << ", z=" << result_origin.z() << ")" << std::endl;

    return 0;
}
