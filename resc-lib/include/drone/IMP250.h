//
// Created by Zhaohong Liu on 24-10-28.
//

#ifndef MOGENLIB_IMP250_H
#define MOGENLIB_IMP250_H

#include "DroneBase.h"

class IMP250 : public DroneBase {
private:
    const double pwm_thrust_slope_ = 1.4987;
    const double pwm_thrust_intercept_ = -1823.063;
    // the original intercept is -1744.1436
    // however, for real drone, there is block for air flow, so the intercept should be lower
    // I use hovering throttle pwm to estimate the intercept
    // hovering throttle pwm is around 1480 to 1500, take 1490 as the middle value
    // given that mass is 1.64 kg for sure, assume the intercept is b
    // 1.4987 * pwm + b = 1640 / 4
    // b is -1823.063
    // be advised, this value should also be adjusted in roslaunch file, to get the correct mavros rescaled thrust
public:
    IMP250() {
        mass_ = 1.64;
        arm_x_ = 0.088;
        arm_y_front_ = 0.0875;
        arm_y_rear_ = 0.0875;

        ixx_ = 0.011;
        iyy_ = 0.01;
        izz_ = 0.0065;
        torque_constant_ = 0.012;
        thrust_coefficient_ = (pwm_thrust_slope_ * pwm_max_ + pwm_thrust_intercept_) * g_ / 1e3;

        mc_roll_rate_p_ = 0.15;
        mc_pitch_rate_p_ = 0.15;
        mc_yaw_rate_p_ = 0.2;

        ca_min_ = static_cast<float>((-pwm_thrust_intercept_ / pwm_thrust_slope_ - pwm_min_) / (pwm_max_ - pwm_min_));

        rotor_rise_time_ = 0.001;

        g_ = 9.7946;
    }

    double pwm2Thrust(double &pwm) override {
        return (pwm_thrust_slope_ * pwm + pwm_thrust_intercept_) * g_ / 1e3;
    }

    double rescaleThrust(const double &thrust) override {
        auto single_thrust = thrust / 4;
        auto pwm = (single_thrust / g_ * 1e3 - pwm_thrust_intercept_) / pwm_thrust_slope_;
        return (pwm - pwm_min_) / (pwm_max_ - pwm_min_);
    }
};

#endif //MOGENLIB_IMP250_H
