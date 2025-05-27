//
// Created by Zhaohong Liu on 24-9-5.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>  // include this header to use Eigen matrix as argument

#include "SignedDistanceField.h"

PYBIND11_MODULE(sdf_cpp, m) {
    pybind11::class_<SignedDistanceField>(m, "SignedDistanceField")
        .def(pybind11::init<>())
        .def("resetMapSize", &SignedDistanceField::resetMapSize,
             pybind11::arg("map_size"))
        .def("rayCastingEdge", &SignedDistanceField::rayCastingEdge,
             pybind11::arg("pos"),
             pybind11::arg("ctrl_p"),
             pybind11::arg("sdf_map"),
             pybind11::arg("preset_radius") = -1);
}
