// auton_list.hpp
// Why: every auton run, so the screen selector can offer them by name.
// Owns: the auton declarations. One file per auton in this folder.
#pragma once

#include <span>

#include "cardLib/api.hpp"

void leftAuton();
void rightAuton();
void skillsRight();

// The list the selector shows. To add a run: copy _new_auton_template.cpp, then add one line
// to AUTONS in auton_list.cpp.
std::span<const cardLib::AutonEntry> autonList();
