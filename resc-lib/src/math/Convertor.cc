//
// Created by Zhaohong Liu on 24-10-15.
//

#include "Convertor.h"

void Convertor::q2EulerAngle(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    double sr_cp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cr_cp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sr_cp, cr_cp);

    double sin_p = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sin_p) >= 1)
        pitch = copysign(M_PI / 2, sin_p);  // pi/2
    else
        pitch = asin(sin_p);

    double sy_cp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cy_cp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(sy_cp, cy_cp);
}

void Convertor::q2EulerAngle(const Eigen::Quaterniond &q, Eigen::Vector3d &euler) {
    double roll, pitch, yaw;
    q2EulerAngle(q, roll, pitch, yaw);
    euler << roll, pitch, yaw;
}

void Convertor::euler2Quaternion(Eigen::Quaterniond &q, const Eigen::Vector3d &euler) {
    q = Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
}

Eigen::Quaterniond Convertor::euler2Quaternion(const Eigen::Vector3d &euler) {
    return Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
}