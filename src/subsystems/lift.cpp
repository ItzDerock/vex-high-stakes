#include "../config.hpp"
#include "pros/rtos.hpp"
#include "robot/PID.hpp"
#include "robot/subsystems.hpp"
#include <algorithm>
#include <atomic>

#define LIFT_KP 1.4
#define LIFT_KI 0
#define LIFT_KD 1.8
#define LIFT_UPPER_LIMIT 3.5 * 360
#define LIFT_LOWER_LIMIT 0

// PROS brake mode doesn't work, so we have to PID ourselves
// Each motor gets its own PID, target is negative of each other
PIDController leftLiftPID(LIFT_KP, LIFT_KI, LIFT_KD);
PIDController rightLiftPID(LIFT_KP, LIFT_KI, LIFT_KD);

std::atomic<double> targetLiftPosition;
std::atomic<bool> manualLiftOverride(false);

// lift positions {REST, HIGH, ALLIANCE}
double subsystems::liftPositions[3] = {0, 4.45 * 360, 2.7 * 360};
int currentLiftPosition = 0;

void internalLiftLoop() {
  while (true) {
    if (manualLiftOverride.load()) {
      pros::delay(10);
      continue;
    }

    // left lift is more positive as going up
    // right lift is more negative as going up
    // so we invert so both are positive
    double leftPosition = lift_left.get_position();
    double rightPosition = lift_right.get_position();

    // calculate error
    double leftError = leftPosition - targetLiftPosition;
    double rightError = rightPosition - targetLiftPosition;

    double leftOut = leftLiftPID.update(-1 * leftError);
    double rightOut = rightLiftPID.update(-1 * rightError);

    leftOut = std::max(std::min(leftOut, 127.0), -70.0);
    rightOut = std::max(std::min(rightOut, 127.0), -70.0);

    lift_left.move(leftOut);
    lift_right.move(rightOut);

    pros::delay(10);
  }
}

pros::Task *liftTask;
void subsystems::initLiftTask() {
  // set motor to hold mode
  lift_left.set_brake_mode_all(pros::MotorBrake::hold);
  lift_right.set_brake_mode_all(pros::MotorBrake::hold);

  // tare motors
  lift_left.tare_position();
  lift_right.tare_position();

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
    setTargetLiftPosition(lift_left.get_position());
    manualLiftOverride.store(false);
    return;
  }

  // otherwise, if power is 0, do nothing, let bg PID handle
  if (power == 0)
    return;

  // lastly, move it and prevent PID from interfering
  manualLiftOverride.store(true);
  lift_left.move(power);
  lift_right.move(power);
}
