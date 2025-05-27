//
// Created by Zhaohong Liu on 24-11-18.
//

#ifndef MOGENLIB_CONTROLALLOCATOR_H
#define MOGENLIB_CONTROLALLOCATOR_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <memory>
#include <cfloat>

#include "drone/DroneBase.h"
#include "Convertor.h"

static constexpr int NUM_ACTUATORS = 16;
static constexpr int NUM_AXES = 6;

class MixingOutput {
public:
    void setDrone(std::shared_ptr<DroneBase> & drone) {
        drone_ptr_ = drone;
        pwm_min_ = static_cast<float>(drone_ptr_->pwm_min_);
        pwm_max_ = static_cast<float>(drone_ptr_->pwm_max_);
        pwm_sum_half_ = (pwm_min_ + pwm_max_) / 2;
        pwm_diff_half_ = (pwm_max_ - pwm_min_) / 2;
    }
    void updateActuatorSetpointValues(const Eigen::Matrix<float, NUM_ACTUATORS, 1> & actuator_sp);
    void outputLimitCalcSingle();
    void reorderThrusts();
public:
    Eigen::Vector4d motor_thrusts_;
private:
    float pwm_min_, pwm_max_, pwm_sum_half_, pwm_diff_half_;
    Eigen::Matrix<float, NUM_ACTUATORS, 1> actuator_motors_;
    Eigen::Matrix<float, NUM_ACTUATORS, 1> actuator_outputs_;
    std::shared_ptr<DroneBase> drone_ptr_;
};

class ControlAllocator {
private:
    static constexpr float thrust_x_ = 0.0;
    static constexpr float thrust_y_ = 0.0;

    const Eigen::Vector3f axis_upward_ = Eigen::Vector3f(0, 0, -1.0);

    Eigen::Matrix<float, 4, 3> rotor_positions_;
    Eigen::Matrix<float, 4, 1> moment_constants_;
    Eigen::Matrix<float, 4, 1> thrust_coefficients_;

    Eigen::Matrix<float, NUM_AXES, NUM_ACTUATORS> effectiveness_;
    Eigen::Matrix<float, NUM_ACTUATORS, NUM_AXES> mix_;
    Eigen::Matrix<float, NUM_AXES, 1> control_sp_;
    Eigen::Matrix<float, NUM_ACTUATORS, 1> actuator_sp_;
    std::shared_ptr<DroneBase> drone_ptr_;

    MixingOutput mixing_output_;
public:
    explicit ControlAllocator(std::shared_ptr<DroneBase> & drone);
    void updateDroneParams();
    void updateEffectivenessMix();
    void normalizeMix();

    /**
     * @brief Set the control setpoint, torque and thrust
     * @param torque_sp
     * @param thrust_sp real thrust in newton, not normalized
     */
    void setControlSetpoint(const Eigen::Vector3d & torque_sp, const double & thrust_sp);
    void pseudoInverseAllocate();
    void clipActuatorSetpoint();

    [[nodiscard]] const Eigen::Matrix<float, NUM_AXES, NUM_ACTUATORS> & getEffectiveness() const { return effectiveness_; }
    [[nodiscard]] const Eigen::Matrix<float, NUM_ACTUATORS, NUM_AXES> & getMix() const { return mix_; }
    [[nodiscard]] const Eigen::Vector4d & getMotorThrusts() const { return mixing_output_.motor_thrusts_; }
};

#endif //MOGENLIB_CONTROLALLOCATOR_H
