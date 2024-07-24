#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

#endif // POSE_UTILS_H
