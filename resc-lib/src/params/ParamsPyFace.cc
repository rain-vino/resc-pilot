//
// Created by Zhaohong Liu on 24-9-17.
//

#include <pybind11/pybind11.h>

#include "Params.h"

PYBIND11_MODULE(params_cpp, m) {
    pybind11::class_<MogenParams>(m, "MogenParams")
        .def_readonly_static("G", &MogenParams::G)
        .def_readonly_static("RL_DT", &MogenParams::RL_DT)
        .def_readonly_static("SIM_DT", &MogenParams::SIM_DT)
        .def_readonly_static("CT_G_MIN", &MogenParams::CT_G_MIN)
        .def_readonly_static("CT_G_MAX", &MogenParams::CT_G_MAX)
        .def_readonly_static("ROTOR_INPUT_SCALING", &MogenParams::ROTOR_INPUT_SCALING)
        .def_readonly_static("ROTOR_POSITION_ARMED", &MogenParams::ROTOR_POSITION_ARMED)
        .def_readonly_static("ROTOR_VEL_SLOWDOWN_SIM", &MogenParams::ROTOR_VEL_SLOWDOWN_SIM)
        .def_readonly_static("ROTOR_RISE_TIME", &MogenParams::ROTOR_RISE_TIME);
}
