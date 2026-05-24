#include "unitree_im6014/im6014_motor.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

PYBIND11_MODULE(pyim6014, m) {
  m.doc() = "Unitree IM6014 Motor Python Bindings (Output-end units)";

  py::class_<unitree::IM6014::IM6014State>(m, "IM6014State")
      .def(py::init<>())
      .def_readwrite("id", &unitree::IM6014::IM6014State::id)
      .def_readwrite("pos", &unitree::IM6014::IM6014State::pos) // rad (output)
      .def_readwrite("speed",
                     &unitree::IM6014::IM6014State::speed) // rad/s (output)
      .def_readwrite("torque",
                     &unitree::IM6014::IM6014State::torque) // Nm (output)
      .def_readwrite("valid", &unitree::IM6014::IM6014State::valid);

  py::class_<unitree::IM6014::IM6014Motor>(m, "IM6014Motor")
      .def(py::init<>())
      .def("init", &unitree::IM6014::IM6014Motor::init, py::arg("port"),
           py::arg("baudrate") = 4000000)
      .def("close", &unitree::IM6014::IM6014Motor::close)
      .def("send_cmd", &unitree::IM6014::IM6014Motor::send_cmd, py::arg("id"),
           py::arg("tor"), py::arg("spd"), py::arg("pos"), py::arg("kp"),
           py::arg("kd"), py::arg("status") = 1, py::arg("timeout") = 1,
           py::call_guard<py::gil_scoped_release>()) // 释放 GIL 支持多线程
      .def("recv_state", &unitree::IM6014::IM6014Motor::recv_state);
}