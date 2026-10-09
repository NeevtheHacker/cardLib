// ports.hpp
// Why: every port number in one place, checked for range and duplicates when the code compiles.
// Owns: the port constants. Change these to match your robot.
#pragma once

#include <array>
#include <cstdint>

#include "cardLib/hardware/port_registry.hpp"

// Drive (Smart Ports 1 to 21).
constexpr std::uint8_t LEFT_FRONT_PORT = 1;
constexpr std::uint8_t LEFT_MIDDLE_PORT = 2;
constexpr std::uint8_t LEFT_BACK_PORT = 3;
constexpr std::uint8_t RIGHT_FRONT_PORT = 4;
constexpr std::uint8_t RIGHT_MIDDLE_PORT = 5;
constexpr std::uint8_t RIGHT_BACK_PORT = 6;

// Mechanisms.
constexpr std::uint8_t INTAKE_PORT = 7;

// Sensors.
constexpr std::uint8_t IMU_PORT = 16;
constexpr std::uint8_t VERTICAL_TRACKING_PORT = 17;
constexpr std::uint8_t HORIZONTAL_TRACKING_PORT = 18;

// 3-wire ports ('A' to 'H').
constexpr std::uint8_t CLAMP_PISTON_PORT = 'A';

constexpr std::array<std::uint8_t, 10> SMART_PORTS{
    LEFT_FRONT_PORT, LEFT_MIDDLE_PORT, LEFT_BACK_PORT, RIGHT_FRONT_PORT,
    RIGHT_MIDDLE_PORT, RIGHT_BACK_PORT, INTAKE_PORT, IMU_PORT,
    VERTICAL_TRACKING_PORT, HORIZONTAL_TRACKING_PORT};
constexpr std::array<std::uint8_t, 1> ADI_PORTS{CLAMP_PISTON_PORT};

static_assert(cardLib::smartPortsValid(SMART_PORTS), "a Smart Port is out of range or used twice");
static_assert(cardLib::adiPortsValid(ADI_PORTS), "a 3-wire port is invalid or used twice");
