#include "unitree_im6014/im6014_motor.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

PYBIND11_MODULE(_native, m) {
  m.doc() = "Unitree IM6014 Motor Python Bindings (Output-end units)";
  m.attr("WILDCARD_ID") = unitree::IM6014::WILDCARD_ID;

  py::class_<unitree::IM6014::State>(m, "State")
      .def(py::init<>())
      .def_readwrite("id", &unitree::IM6014::State::id)
      .def_readwrite("status", &unitree::IM6014::State::status)
      .def_readwrite("timeout", &unitree::IM6014::State::timeout)
      .def_readwrite("temp1", &unitree::IM6014::State::temp1)
      .def_readwrite("temp2", &unitree::IM6014::State::temp2)
      .def_readwrite("voltage", &unitree::IM6014::State::voltage)
      .def_readwrite("torque", &unitree::IM6014::State::torque)
      .def_readwrite("speed", &unitree::IM6014::State::speed)
      .def_readwrite("pos", &unitree::IM6014::State::pos)
      .def_readwrite("error", &unitree::IM6014::State::error)
      .def_readwrite("warning", &unitree::IM6014::State::warning)
      .def_readwrite("valid", &unitree::IM6014::State::valid)
      .def_readwrite("timestamp_us", &unitree::IM6014::State::timestamp_us);

  py::class_<unitree::IM6014::Motor>(m, "Motor")
      .def(py::init<>())
      .def("init", &unitree::IM6014::Motor::init, py::arg("port"),
           py::arg("baudrate") = 4000000)
      .def("close", &unitree::IM6014::Motor::close)
      .def("send_cmd", &unitree::IM6014::Motor::send_cmd, py::arg("id"),
           py::arg("tor"), py::arg("spd"), py::arg("pos"), py::arg("kp"),
           py::arg("kd"),
           py::arg("status") = 1, // MotorStatus::ENABLE
           py::arg("timeout") = 0, py::call_guard<py::gil_scoped_release>())
      .def("poll_states", &unitree::IM6014::Motor::poll_states,
           py::arg("timeout_ms") = 2)
      .def("get_state", &unitree::IM6014::Motor::get_state, py::arg("id"),
           py::arg("state"))
      .def("recv_state", &unitree::IM6014::Motor::recv_state, py::arg("id"),
           py::arg("state"), py::arg("timeout_ms") = 50);
}
