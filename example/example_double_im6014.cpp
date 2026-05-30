#include "unitree_im6014/im6014_motor.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <thread>

using namespace unitree::IM6014;

static std::atomic<bool> keep_running{true};

static void on_sigint(int) { keep_running.store(false); }

int main() {
  Motor motor;
  if (!motor.init("/dev/ttyUSB0", 6000000)) {
    std::cerr << "Failed to open serial port!" << std::endl;
    return -1;
  }

  std::signal(SIGINT, on_sigint);

  constexpr uint8_t id1 = 0, id2 = 1;
  constexpr float kp = 0.0f, kd = 2.5f;

  std::cout << "=== IM6014 Dual Motor Example (Output-end units) ==="
            << std::endl;

  State s1, s2;
  float t = 0.0f;
  while (keep_running.load()) {
    // Velocity control with sinusoidal trajectory (output-end units)
    motor.send_cmd(id1, 0.0f,
                   -0.5f * (1.0f - std::cos(3.0f * t)), 0.0f, kp, kd);
    motor.send_cmd(id2, 0.0f,
                    1.5f * (1.0f - std::cos(2.0f * t)), 0.0f, kp, kd);

    motor.poll_states(2);
    bool ok1 = motor.get_state(id1, s1);
    bool ok2 = motor.get_state(id2, s2);

    if (ok1 && ok2) {
      std::cout << "\033c" << std::flush;
      std::cout << "[M1] Pos: " << s1.pos << " rad, "
                << "Spd: " << s1.speed << " rad/s, "
                << "Tor: " << s1.torque << " Nm" << std::endl
                << "[M2] Pos: " << s2.pos << " rad, "
                << "Spd: " << s2.speed << " rad/s, "
                << "Tor: " << s2.torque << " Nm" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    t += 0.002f;
  }

  // Graceful stop
  std::cout << "\nCaught Ctrl+C, stopping motors..." << std::endl;
  motor.send_cmd(id1, 0, 0, 0, 0, 0, MotorStatus::DISABLE);
  motor.send_cmd(id2, 0, 0, 0, 0, 0, MotorStatus::DISABLE);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  motor.close();
  std::cout << "Motors stopped." << std::endl;
  return 0;
}
