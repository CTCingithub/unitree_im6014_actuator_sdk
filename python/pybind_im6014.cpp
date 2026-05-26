#include "unitree_im6014/im6014_motor.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

PYBIND11_MODULE(_native, m) {
  m.doc() = "Unitree IM6014 Motor Python Bindings (Output-end units)";

  py::class_<unitree::IM6014::State>(m, "State")
      .def(py::init<>())
      .def_readwrite("id", &unitree::IM6014::State::id)
      .def_readwrite("pos", &unitree::IM6014::State::pos) // rad (output)
      .def_readwrite("speed",
                     &unitree::IM6014::State::speed) // rad/s (output)
      .def_readwrite("torque",
                     &unitree::IM6014::State::torque) // Nm (output)
      .def_readwrite("valid", &unitree::IM6014::State::valid);

  py::class_<unitree::IM6014::Motor>(m, "Motor")
      .def(py::init<>())
      .def("init", &unitree::IM6014::Motor::init, py::arg("port"),
           py::arg("baudrate") = 4000000)
      .def("close", &unitree::IM6014::Motor::close)
      .def("send_cmd", &unitree::IM6014::Motor::send_cmd, py::arg("id"),
           py::arg("tor"), py::arg("spd"), py::arg("pos"), py::arg("kp"),
           py::arg("kd"), py::arg("status") = 1, py::arg("timeout") = 0,
           py::call_guard<py::gil_scoped_release>()) // �ͷ� GIL ֧�ֶ��߳�
      .def("recv_state", &unitree::IM6014::Motor::recv_state,
           py::arg("id"), py::arg("state"),
           py::arg("timeout_ms") = 50);
}