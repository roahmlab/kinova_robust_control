#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "roahm_trajectories/TrajectoryPybindWrapper.hpp"

namespace py = pybind11;

using namespace Roahm;

PYBIND11_MODULE(roahm_trajectories_py, m) {
    m.doc() = "pybind11 roahm_trajectories_py plugin";

    py::class_<TrajectoryPybindWrapper>(m, "TrajectoryPybindWrapper")
        .def(py::init<>())
        .def("setup", &TrajectoryPybindWrapper::setup)
        .def("compute", &TrajectoryPybindWrapper::compute);
}