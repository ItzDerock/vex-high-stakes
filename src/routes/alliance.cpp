#include "robot/chassis.hpp"
#include "robot/odom.hpp"
#include "robot/subsystems.hpp"

void chassis::runAlliancePath() {
  odom::moveDistance(16);
  if (subsystems::currentTeam.load() == subsystems::RED) {
    odom::turnTo(90);
  } else {
    odom::turnTo(270);
  }

  subsystems::setTargetLiftPosition(4200);
  pros::delay(800);
  odom::moveDistance(-20);
  subsystems::moveLift(-127);
  pros::delay(100);
  subsystems::setTargetLiftPosition(subsystems::liftPositions[0]);
}
