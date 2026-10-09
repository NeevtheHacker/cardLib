// auton_list.cpp
// Why: the selector menu: one line per auton run.
// Owns: AUTONS.
#include "autons/auton_list.hpp"

namespace {

const cardLib::AutonEntry AUTONS[] = {
    {"Left", leftAuton},
    {"Right", rightAuton},
    {"Skills right", skillsRight},
};

}  // namespace

std::span<const cardLib::AutonEntry> autonList() { return AUTONS; }
