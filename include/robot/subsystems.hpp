#pragma once

#include <atomic>

namespace subsystems {

/* INTAKE Subsystem */

void initInakeTask();

enum Color { RED, BLUE, NONE };
extern std::atomic<bool> lockIntakeControls;
extern std::atomic<Color> currentTeam;
extern std::atomic<bool> intakeRedirectMode;

/* LIFT Subsystem */

void initLiftTask();
void setTargetLiftPosition(double position);
void cycleLiftPosition();

} // namespace subsystems
