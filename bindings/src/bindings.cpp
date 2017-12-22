#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/LU>
#include <Eigen/StdVector>

#include "pose_estimation.h"
#include "local_ba.h"
#include "full_ba.h"

namespace py = pybind11;

PYBIND11_PLUGIN(urbg2o) {
    py::module m("urbg2o", "pybind11 opencv example plugin");

    m.def("poseOptimization", &poseOptimization, "pose-only bundle adjustment",
	py::arg("coords").noconvert(), py::arg("pose"));
    
    m.def("localBundleAdjustment", &localBundleAdjustment, "local bundle adjustment",
    py::arg("keyframes"), py::arg("fixedKeyframes"), py::arg("worldMapPoints"), py::arg("pointsRelation"));

      m.def("fullBundleAdjustment", &fullBundleAdjustment, "full bundle adjustment",
    py::arg("keyframes"), py::arg("mapPoints"), py::arg("pointsRelation"));

    return m.ptr();
}

