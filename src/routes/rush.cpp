#include "../config.hpp"
#include "pros/rtos.hpp"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"
#include "robot/position.hpp"
#include "robot/subsystems.hpp"
#include "robot/utils.hpp"

// built for BLUE POSITIVE

odom::RobotPosition flipForRed(odom::RobotPosition bluePosition) {
  if (subsystems::currentTeam.load() == subsystems::RED) {
    bluePosition.x *= -1;
    bluePosition.theta *= -1;
  }

  bluePosition.theta =
      utils::radToDeg(utils::angleSquish(utils::degToRad(bluePosition.theta)));

  return bluePosition;
}

void chassis::runRushPath() {
  odom::startChainedMovement(5);
  grabber_1.extend();
  grabber_2.extend();
  odom::moveDistance(-20);
  odom::moveTo(flipForRed({-8, 51, -45}), 2'000,
               {.lead = 0.2, .forwards = false, .maxSpeedWhenClose = 70});
  grabber_1.retract();
  grabber_2.retract();
  pros::delay(200);

  // ring
  intake_motor_stg1.move(-127);
  intake_motor_stg2.move(-127);
  odom::moveTo(flipForRed({-24, 47, 270}), 1'500, {.lead = 0});

  odom::moveTo(flipForRed({-46, 22, 180}), 1'500, {.lead = 0.4});

  intake_motor_stg1.move(0);
  intake_motor_stg2.move(0);
  odom::moveTo(flipForRed({-23, 0, 90 + 45}), 1'500, {.lead = 0.4});
}
