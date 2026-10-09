// adi_digital.hpp
// Why: a 3-wire digital output, e.g. a pneumatic solenoid.
// Owns: the port and the last state written.
#pragma once

#include <cstdint>

namespace cardLib {

class AdiDigitalOut {
 public:
  // Port 1 to 8 ('A' to 'H' also work).
  AdiDigitalOut(std::uint8_t port, bool initialState = false)
      : m_port(port), m_state(initialState) {}

  // Sets the port to digital out and writes the initial state. False if the write failed.
  bool initialize();

  bool set(bool state);
  bool toggle() { return set(!m_state); }
  bool state() const { return m_state; }
  std::uint8_t port() const { return m_port; }

 private:
  std::uint8_t m_port;
  bool m_state;
};

}  // namespace cardLib
