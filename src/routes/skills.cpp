#include "../config.hpp"
#include "pros/device.hpp"
#include "pros/rtos.hpp"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"
#include "robot/subsystems.hpp"

// #define GO_FOR_STAKES

void chassis::runSkillsPath() {
  odom::startChainedMovement(8);

  // 1. Back up to grab the mogo
  odom::moveDistance(-8, 600, {.chasePower = 30});
  odom::turnTo(180);
  odom::moveTo(-50, 16, 180, 4'000,
               {
                   .chasePower = 15,
                   .lead = 0.5,
                   .forwards = false,
               });

  grabber_1.extend();
  grabber_2.extend();
  pros::delay(250);

  // 2. Turn to face 90 degrees
  odom::turnTo(90);

  // 3. Start up the intake
  intake_motor_stg1.move(127);
  intake_motor_stg2.move(127);

  // 4. Grab 2nd ring
  odom::moveTo(-23, 23, 90, 5'000, {.chasePower = 100});

  // Grab 3 more rings
  odom::moveTo(-23, 42, 0, 2'500, {.chasePower = 80, .lead = 0.5});
  pros::delay(300);
  odom::moveTo(-65, 42, 270, 5'000,
               {.maxSpeed = 40, .chasePower = 40, .exitOnStall = true});

  // last ring
  odom::moveDistance(-24);
  pros::delay(250);
  odom::moveTo(-55, 55, 0, 2'000, {.maxSpeed = 60});
  pros::delay(1000);
  odom::moveDistance(-12);
  pros::delay(100);

  // score the goal
  intake_motor_stg2.move(-127);
  odom::moveTo(-70, 70, 135, 4'000, {.forwards = false});
  grabber_1.retract();
  grabber_2.retract();
  pros::delay(500);
  odom::moveDistance(10);
  intake_motor_stg2.move(127);

  // FIRST 6 RING COMPLETE!
  odom::startChainedMovement(8);

  odom::moveTo(-53, -13, 0, 5'000, {.forwards = false});
  odom::moveTo(-53, -23, 0, 2'000, {.maxSpeed = 60, .forwards = false});
  grabber_2.extend();
  grabber_1.extend();

  odom::moveTo(-23, -25, 90, 5'000,
               {.chasePower = 100}); // center square ring, 1
  odom::moveTo(-5, -50, 180, 2'000, {});
  odom::moveTo(-5, -58, 180, 2'000, {}); // mid ring, 2
  odom::turnTo(270);
  odom::moveTo(
      -65, -43, 270, 7'000,
      {.maxSpeed = 40, .lead = 0.7, .exitOnStall = true}); // long thingy

  odom::moveDistance(-24); // last ring
  odom::moveTo(-55, -58, 200, 2'000, {.maxSpeed = 60});
  pros::delay(1500);
  odom::moveDistance(-15);
  odom::turnTo(45);

  // score goal 2
  intake_motor_stg2.move(-127);
  odom::moveTo(-70, -70, 45, 1'000, {.forwards = false, .exitOnStall = true});
  grabber_1.retract();
  grabber_2.retract();
  pros::delay(500);
  odom::moveDistance(10);
  intake_motor_stg2.move(127);

  // CROSS MAP TIME
  // odom::moveTo(44, -30, 225, 5'000, {});

  // odom::moveTo(-23, -44, 180, 2'500, {.chasePower = 80, .lead = 0.5});
  // odom::moveTo(-50, -55, 45, 2'000, {.maxSpeed = 60});
}
