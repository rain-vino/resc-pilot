//
// Created by Zhaohong Liu on 24-9-11.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "QuadrotorDynamics.h"

PYBIND11_MODULE(quadrotor_dynamics_cpp, m) {
    pybind11::class_<QuadrotorDynamics>(m, "QuadrotorDynamics")
        .def(pybind11::init<>())
        .def("initDrone", &QuadrotorDynamics::initDrone,
             pybind11::arg("drone_name"))
        .def("run", &QuadrotorDynamics::run,
             pybind11::arg("state"),
             pybind11::arg("body_rate_setpoint"),
             pybind11::arg("motor_thrusts"),
             pybind11::arg("thrust_req"))
        .def("getFinalMotorThrusts", &QuadrotorDynamics::getFinalMotorThrusts)
        .def("resetDomainRandomization", &QuadrotorDynamics::resetDomainRandomization)
        .def("setDomainRandomizationFac", &QuadrotorDynamics::setDomainRandomizationFac,
             pybind11::arg("fac"))
        .def("setSysCtrlAlloc", &QuadrotorDynamics::setSysCtrlAlloc,
             pybind11::arg("sys_ctrl_alloc"))
        .def("resetCtrlAllocRange", &QuadrotorDynamics::resetCtrlAllocRange,
             pybind11::arg("ca_min"),
             pybind11::arg("ca_max"))
        .def_static("runKinematicUpdate", &QuadrotorDynamics::runKinematicUpdate,
             pybind11::arg("pos"),
             pybind11::arg("vel"),
             pybind11::arg("acc"),
             pybind11::arg("att"),
             pybind11::arg("action"));
}