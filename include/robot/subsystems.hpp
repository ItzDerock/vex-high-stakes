#pragma once

#include <atomic>

namespace subsystems {

void initInakeTask();

enum Color { RED, BLUE, NONE };
extern std::atomic<bool> lock_intake_controls;
extern std::atomic<Color> current_team;

} // namespace subsystems
