#include "../config.hpp"
#include "pros/rtos.hpp"
#include "robot/PID.hpp"
#include "robot/subsystems.hpp"
#include <algorithm>
#include <atomic>

// #define LIFT_KP 0.2
// #define LIFT_KI 0
// #define LIFT_KD 0.8
#define LIFT_KP 0.02
#define LIFT_KI 0
#define LIFT_KD 0.15
#define LIFT_UPPER_LIMIT 3.5 * 360
#define LIFT_LOWER_LIMIT 0

// PROS brake mode doesn't work, so we have to PID ourselves
// Each motor gets its own PID, target is negative of each other
PIDController liftPID(LIFT_KP, LIFT_KI, LIFT_KD);

std::atomic<double> targetLiftPosition;
std::atomic<bool> manualLiftOverride(false);

// lift positions {REST, HIGH, ALLIANCE}
double subsystems::liftPositions[3] = {164, 41, -1.4 * 360};
int currentLiftPosition = 0;

void internalLiftLoop() {
  while (true) {
    if (manualLiftOverride.load()) {
      pros::delay(10);
      continue;
    }

    // run loop
    double liftError = lift_sensor.get_position() - targetLiftPosition.load();
    double liftOut = liftPID.update(liftError);
    lift.move(std::max(std::min(liftOut, 127.0), -127.0));

    pros::delay(10);
  }
}

pros::Task *liftTask;
void subsystems::initLiftTask() {
  // set motor to hold mode
  lift.set_brake_mode_all(pros::MotorBrake::hold);
  lift_sensor.reset_position();

  // start task
  liftTask = new pros::Task(internalLiftLoop, "lift");
}

void subsystems::setTargetLiftPosition(double position) {
  targetLiftPosition = position;
}

void subsystems::cycleLiftPosition() {
  currentLiftPosition++;
  if (currentLiftPosition >= 3) {
    currentLiftPosition = 0;
  }

  setTargetLiftPosition(liftPositions[currentLiftPosition]);
}

/**
 * Power from -127 to 127
 * Temporarily disables PID, PID is only used to hold lift position
 */
void subsystems::moveLift(double power) {
  // If lift was previously moving, but is now done moving, set target to
  // current position
  if (power == 0 && manualLiftOverride.load()) {
    setTargetLiftPosition(lift_sensor.get_position());
    manualLiftOverride.store(false);
    return;
  }

  // otherwise, if power is 0, do nothing, let bg PID handle
  if (power == 0)
    return;

  // lastly, move it and prevent PID from interfering
  manualLiftOverride.store(true);
  lift.move(power);
}
