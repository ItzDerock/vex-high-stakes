#include "../config.hpp"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"

void chassis::runRushPath() {
  odom::moveTo(-8.5, -48, 270, 5'000,
               {.lead = 0.4,
                .forwards = false,
                .maxSpeedWhenClose = 60,
                .closeThreshold = 3});
  grabber_1.extend();
  grabber_2.extend();
  pros::delay(200);

  intake_motor_stg2.move(127);
  odom::moveDistance(3);
  odom::moveTo(-23, -47, 270, 5'000, {.maxSpeed = 80});
}
