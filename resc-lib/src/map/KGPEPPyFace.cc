//
// Created by Zhaohong Liu on 24-12-6.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>  // include this header to use Eigen matrix as argument

#include "K_GPEP.h"

PYBIND11_MODULE(kgpep_cpp, m) {
    pybind11::class_<K_GPEP>(m, "K_GPEP")
        .def(pybind11::init<>())
        .def("clearGlobalESDFBuffer", &K_GPEP::clearGlobalESDFBuffer)
        .def("set2DESDFMap", &K_GPEP::set2DESDFMap,
             pybind11::arg("sdf_map"),
             pybind11::arg("map_x"),
             pybind11::arg("map_y"))
        .def("getKinematicPseudoRaycast", &K_GPEP::getKinematicPseudoRaycast,
             pybind11::arg("pos"),
             pybind11::arg("vel"),
             pybind11::arg("ctrl_p"))
        .def("guidedPseudoRaycast", &K_GPEP::guidedPseudoRaycast,
             pybind11::arg("pos"),
             pybind11::arg("ctrl_p"),
             pybind11::arg("preset_radius") = -1)
        .def("getESDFPerception", &K_GPEP::getESDFPerception,
             pybind11::arg("pos"))
        .def("getDistance", &K_GPEP::getDistance,
             pybind11::arg("pos_3d"))
        .def("getDoublePseudoRaycast", &K_GPEP::getDoublePseudoRaycast,
             pybind11::arg("pos"),
             pybind11::arg("vel"),
             pybind11::arg("ctrl_p1"),
             pybind11::arg("ctrl_p2"))
        .def("setCollisionThreshold", &K_GPEP::setCollisionThreshold,
             pybind11::arg("threshold"))
        .def("clearGlobalESDFBuffer", &K_GPEP::clearGlobalESDFBuffer);
}