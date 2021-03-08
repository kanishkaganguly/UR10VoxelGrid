//
// Created by Kanishka Ganguly on 8/6/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#include "../include/pygazebo.hpp"

namespace py = pybind11;
/**
 * @brief This needs to be the same as the library name declared in CMakeLists.txt
 * Otherwise, Python import will throw an error about module not having "init" function.
 */
//@formatter:off
PYBIND11_MODULE(pygazebo_interface, m) {
	py::class_<GazeboInterface>(m, "GazeboInterface")
	        .def(py::init<>())
	        .def("Init", &GazeboInterface::Init, "Initialize Gazebo interface")
	        .def("InitSim", &GazeboInterface::InitSim, "Initialize simulator")
	        .def("StepSim", &GazeboInterface::StepSim, "Step simulator")
	        .def("PauseSim", &GazeboInterface::PauseSim, "Pause simulator")
	        .def("StartSim", &GazeboInterface::StartSim, "Start simulator")
			.def("ResetSim", &GazeboInterface::ResetSim, "Reset simulator");
}
//@formatter:on

int main(int argc, char *argv[]) {
	return 0;
}