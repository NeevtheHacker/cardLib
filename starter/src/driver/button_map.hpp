// button_map.hpp
// Why: every controller binding in one place, so remapping a button is a one-line change.
// Owns: the stick and button constants.
#pragma once

#include "pros/misc.h"

constexpr pros::controller_analog_e_t FORWARD_STICK = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr pros::controller_analog_e_t TURN_STICK = pros::E_CONTROLLER_ANALOG_RIGHT_X;

constexpr pros::controller_digital_e_t INTAKE_BUTTON = pros::E_CONTROLLER_DIGITAL_R1;
constexpr pros::controller_digital_e_t OUTTAKE_BUTTON = pros::E_CONTROLLER_DIGITAL_R2;
constexpr pros::controller_digital_e_t LOW_STORAGE_BUTTON = pros::E_CONTROLLER_DIGITAL_L2;
constexpr pros::controller_digital_e_t CLAMP_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;
constexpr pros::controller_digital_e_t HOLD_BRAKE_BUTTON = pros::E_CONTROLLER_DIGITAL_B;
