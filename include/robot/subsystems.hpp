#pragma once

#include <atomic>

namespace subsystems {

/* INTAKE Subsystem */

void initInakeTask();

enum Color { RED, BLUE, NONE };
extern std::atomic<bool> lockIntakeControls;
extern std::atomic<Color> currentTeam;
extern std::atomic<bool> intakeRedirectMode;
extern std::atomic<double> maxIntakePower;

/* LIFT Subsystem */

extern double liftPositions[3];
void initLiftTask();
void setTargetLiftPosition(double position);
void cycleLiftPosition();
void moveLift(double power);

} // namespace subsystems
