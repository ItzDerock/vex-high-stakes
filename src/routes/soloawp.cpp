#include "../config.hpp"
#include "pros/rtos.hpp"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"

void chassis::runSoloAWPPath() {
  odom::startChainedMovement(6);

  // grab mid
  odom::moveTo(-24, 24, 310, 2'500, {.maxSpeed = 80, .forwards = false});
  grabber_1.extend();
  grabber_2.extend();
  pros::delay(250);

  // start intake
  intake_motor_stg1.move(127);
  intake_motor_stg2.move(127);

  // grab bottom ring
  odom::moveTo(-24, 48, 0, 5'000, {.maxSpeed = 100});
  pros::delay(200);
  pros::Task([] {
    pros::delay(600);
    intake_motor_stg1.move(-127);
  });

  // line up for 2nd goal
  odom::moveTo(-47, -10, 180, 2'500, {.maxSpeed = 127, .lead = 0.4});
  grabber_1.retract();
  grabber_2.retract();
  pros::delay(250);

  // grab 2nd
  odom::moveTo(-24, -24, 300, 2'500, {.maxSpeed = 75, .forwards = false});
  grabber_1.extend();
  grabber_2.extend();
  pros::delay(250);
  intake_motor_stg1.move(127);

  // grab ring to score
  odom::moveTo(-24, -48, 180, 5'000, {.maxSpeed = 100});
  pros::delay(500);
  pros::Task([] {
    pros::delay(400);
    grabber_1.retract();
    grabber_2.retract();
  });
  odom::moveTo(-24, 0, 0, 5'000, {.maxSpeed = 80});
}
