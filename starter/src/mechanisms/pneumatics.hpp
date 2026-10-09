// pneumatics.hpp
// Why: names each piston by what it does (extend = clamp closed), so code never writes raw
//      true/false to a 3-wire port.
// Owns: Piston and the robot's pistons.
#pragma once

#include "cardLib/api.hpp"

class Piston {
 public:
  explicit Piston(cardLib::AdiDigitalOut& solenoid) : m_solenoid(solenoid) {}

  // Sets the port to digital out. Call once from initialize().
  bool initialize() { return m_solenoid.initialize(); }

  void extend() { m_solenoid.set(true); }
  void retract() { m_solenoid.set(false); }
  void toggle() { m_solenoid.toggle(); }
  bool isExtended() const { return m_solenoid.state(); }

 private:
  cardLib::AdiDigitalOut& m_solenoid;
};

extern Piston clamp;
