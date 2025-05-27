/**
 * Created by Zhaohong Liu on 2024/9/3.
 * Reference: https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/rate_control
 */

#ifndef RATECONTROL_H
#define RATECONTROL_H

#include <Eigen/Eigen>
#include <memory>
#include <iostream>

#include "Params.h"
#include "drone/DroneBase.h"

class RateControl {
private:
    /* MC Rate Control */
    Eigen::Matrix3d rate_p_gain_ = Eigen::Matrix3d::Identity();

    /* Mixer */
    Eigen::Matrix4d alloc_mat_ = Eigen::Matrix4d::Ones();
    Eigen::Matrix4d inv_alloc_mat_ = Eigen::Matrix4d::Ones();

    /* Motor Model */
    double motor_thrust_min_ = -1.0;
    double motor_thrust_max_ = -1.0;

    /* drone */
    std::shared_ptr<DroneBase> drone_ptr_;

public:
    explicit RateControl(std::shared_ptr<DroneBase> & drone);
    void setThrustRange();
    [[nodiscard]] Eigen::Vector3d update(const Eigen::Vector3d &body_rate,
                                         const Eigen::Vector3d &body_rate_setpoint,
                                         double collective_thrust) const;
    Eigen::Vector4d mixer(Eigen::Vector3d &torque_body, double thrust) const;
    void clampThrusts(Eigen::Vector4d& motor_thrusts) const;
    [[nodiscard]] Eigen::Matrix4d getAllocMat() const;
    void checkParams() const;

public:
    using Ptr = std::unique_ptr<RateControl>;
};



#endif //RATECONTROL_H
