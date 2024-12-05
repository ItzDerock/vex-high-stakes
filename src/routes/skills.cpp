#include "../config.hpp"
#include "pros/rtos.hpp"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"
#include "robot/subsystems.hpp"

// #define GO_FOR_STAKES

void chassis::runSkillsPath() {
  odom::startChainedMovement(8);

  // 1. Back up to grab the mogo
  odom::moveDistance(-8, 600, {.chasePower = 30});
  intake_motor_stg2.move(-127);
  odom::turnTo(180);
  odom::moveTo(-48, 16, 180, 4'000,
               {
                   .chasePower = 15,
                   .lead = 0.5,
                   .forwards = false,
               });

  grabber_1.extend();
  grabber_2.extend();

  // 2. Turn to face 90 degrees
  odom::turnTo(90);

  // 3. Start up the intake
  intake_motor_stg1.move(127);
  intake_motor_stg2.move(127);

  // slight deviation, pure pursuit is broken so follow points instead
  odom::moveTo(-23, 23, 90, 5'000, {.chasePower = 100});
  odom::moveTo(23.5, 45, 90, 5'000, {.lead = 0.6});
  odom::moveTo(0, 55, 0, 1'500, {.chasePower = 40});

  // middle
  odom::moveTo(3, 41, 0, 1'500, {.forwards = false});
  subsystems::setTargetLiftPosition(subsystems::liftPositions[1]);
  odom::moveTo(0, 50, 0, 1'500, {});

  // odom::moveTo(-23, 23, 90, 5'000, {.chasePower = 100}); // red 1
  // odom::turnTo(45, 500);

  // // 4. middle
  // odom::moveTo(3, 50, 0, 5'000, {.chasePower = 40}); // red 2
  // odom::turnTo(180);

  // // line up for 3 ring
  // odom::moveTo(
  //     -50, 37, 270, 4'000,
  //     {.maxSpeed = 60, .chasePower = 5, .lead = 0.2}); // grab other 3 rings

  // pros::delay(500);

  // odom::turnTo(145);
  // odom::moveDistance(-12, 1'000, {.exitOnStall = true});
  // grabber_1.retract();
  // grabber_2.retract();
  // odom::moveDistance(5);
}
