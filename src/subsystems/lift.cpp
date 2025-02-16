#include "../config.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "robot/PID.hpp"
#include "robot/subsystems.hpp"
#include <algorithm>
#include <atomic>

#define LIFT_KP 0.1
#define LIFT_KI 0
#define LIFT_KD 0.16
#define LIFT_UPPER_LIMIT 3.5 * 360
#define LIFT_LOWER_LIMIT 0

// PROS brake mode doesn't work, so we have to PID ourselves
// Each motor gets its own PID, target is negative of each other
PIDController liftPID(LIFT_KP, LIFT_KI, LIFT_KD);

std::atomic<double> targetLiftPosition;
std::atomic<bool> manualLiftOverride(false);

// lift positions {REST, HIGH, ALLIANCE}
double subsystems::liftPositions[3] = {1186, 1410, 2746};
int currentLiftPosition = 0;

void internalLiftLoop() {
  while (true) {
    if (manualLiftOverride.load()) {
      pros::delay(10);
      continue;
    }

    errno = 0;

    // run loop
    double liftError = lift_position.get_value() - targetLiftPosition.load();
    double liftOut = -liftPID.update(liftError);
    lift.move(std::max(std::min(liftOut, 127.0), -127.0));

    // std::cout << "Lift position: " << lift_position.get_value() << std::endl
    //           << "Lift target: " << targetLiftPosition.load() << std::endl
    //           << "Lift error: " << liftError << std::endl
    //           << "Lift out: " << liftOut << std::endl;

    if (errno != 0) {
      std::cout << "Lift PID error: " << strerror(errno) << "(" << errno << ")"
                << std::endl;
    }

    pros::delay(10);
  }
}

pros::Task *liftTask;
void subsystems::initLiftTask() {
  // set motor to hold mode
  lift.set_brake_mode_all(pros::MotorBrake::hold);
  targetLiftPosition.store(liftPositions[0]);

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
    setTargetLiftPosition(lift_position.get_value());
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
