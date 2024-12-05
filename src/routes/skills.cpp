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
  odom::turnTo(180);
  odom::moveTo(-40, 15, 180, 4'000,
               {
                   .chasePower = 15,
                   .lead = 0.4,
                   .forwards = false,
               });

  grabber_1.extend();
  grabber_2.extend();

  // 2. Turn to face 90 degrees
  odom::turnTo(90);

  // 3. Start up the intake and follow first pure pursuit path
  intake_motor_stg1.move(127);
  intake_motor_stg2.move(127);

  // auto path = odom::getPath("/usd/skills/TLtTR3ring.txt");
  // odom::follow(path, 3, 10'000, true, false);

  // slight deviation, pure pursuit is broken so follow points instead
  odom::moveTo(-23, 23, 90, 5'000, {.chasePower = 100}); // red 1
  odom::turnTo(45, 500);

  // 4. middle
  odom::moveTo(3, 50, 0, 5'000, {.chasePower = 40}); // red 2
  odom::turnTo(180);

  // line up for 3 ring
  odom::moveTo(
      -50, 37, 270, 4'000,
      {.maxSpeed = 60, .chasePower = 5, .lead = 0.2}); // grab other 3 rings

  pros::delay(500);

  odom::turnTo(145);
  odom::moveDistance(-12, 1'000, {.exitOnStall = true});
  grabber_1.retract();
  grabber_2.retract();
  odom::moveDistance(5);
}
