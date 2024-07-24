#include "pose_utils.h"

tf2::Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

tf2::Transform createTransform(double x, double y, double z, double roll, double pitch, double yaw) {
    tf2::Quaternion q = rpyToQuaternion(roll, pitch, yaw);
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x, y, z));
    transform.setRotation(q);
    return transform;
}

tf2::Transform multiplyTransforms(const tf2::Transform& t1, const tf2::Transform& t2) {
    return t1 * t2;
}
