//
// Created by Zhaohong Liu on 24-9-10.
//

#ifndef MOGENLIB_PARAMS_H
#define MOGENLIB_PARAMS_H

class MogenParams {
public:
    /* RL env params */
    static constexpr double G = 9.8;
    static constexpr double RL_DT = 0.02;
    static constexpr double SIM_DT = 0.01;

    static constexpr double ROTOR_INPUT_SCALING = 1000;
    static constexpr double ROTOR_POSITION_ARMED = 100;
    static constexpr double ROTOR_VEL_SLOWDOWN_SIM = 10;
    static constexpr double ROTOR_RISE_TIME = 0.001;

    /* Dynamic and Kinematic Constraints */
    static constexpr double MAX_ROLL_RATE = 180 * M_PI / 180;
    static constexpr double MAX_PITCH_RATE = 180 * M_PI / 180;
    static constexpr double MAX_YAW_RATE = 60 * M_PI / 180;
    static constexpr double CT_G_MAX = 1.7;
    static constexpr double CT_G_MIN = 0.3;
};

#endif //MOGENLIB_PARAMS_H
