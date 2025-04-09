#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "trajectories/TrajectoryPybindWrapper.hpp"

namespace py = pybind11;

using namespace KinovaRobustControl;

PYBIND11_MODULE(trajectories_py, m) {
    m.doc() = "pybind11 trajectories_py plugin";

    py::class_<TrajectoryPybindWrapper>(m, "TrajectoryPybindWrapper")
        .def(py::init<>())
        .def("setup", &TrajectoryPybindWrapper::setup)
        .def("compute", &TrajectoryPybindWrapper::compute);
}