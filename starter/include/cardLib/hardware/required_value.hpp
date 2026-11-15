// required_value.hpp
// Why: a config field with no default, so leaving it out is a compile error, not a silent 0
//      (SPEC 20.14: wheel diameter, gearing and roll angle).
// Owns: RequiredValue<T>.
#pragma once

namespace cardLib {

template <typename T>
class RequiredValue {
 public:
  constexpr RequiredValue(T value) : m_value(value) {}

  constexpr operator T() const { return m_value; }
  constexpr T value() const { return m_value; }

 private:
  T m_value;
};

}  // namespace cardLib
