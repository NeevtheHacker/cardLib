// screen_lines.hpp
// Why: what this robot shows on the User page and the controller screen.
// Owns: nothing; screen_lines.cpp chooses the lines.
#pragma once

// Hands the current values to the display. Cheap: it only copies numbers.
void showScreenLines();
