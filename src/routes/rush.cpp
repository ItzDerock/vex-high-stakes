#include "../config.hpp"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"
#include "robot/subsystems.hpp"

void chassis::runRushPath() {
  // odom::moveDistance(-3, 5'000, {.maxSpeed = 30});
  subsystems::moveLift(127);
  pros::delay(800);
  subsystems::moveLift(0);
  subsystems::setTargetLiftPosition(0);
  odom::moveDistance(-24, 3'000, {.maxSpeed = 60});
  // odom::moveDistance(-36);
  // odom::moveTo(-8.5, -48, 270, 5'000,
  //              {.lead = 0.4,
  //               .forwards = false,
  //               .maxSpeedWhenClose = 60,
  //               .closeThreshold = 3});
  // grabber_1.extend();
  // grabber_2.extend();
  // pros::delay(200);

  // intake_motor_stg2.move(127);
  // odom::moveDistance(3);
  // odom::moveTo(-23, -47, 270, 5'000, {.maxSpeed = 80});
}
