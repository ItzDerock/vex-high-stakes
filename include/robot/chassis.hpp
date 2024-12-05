#pragma once

#include "main.h"
#include "robot/odom.hpp"

namespace chassis {

pros::Mutex movementMutex();

struct PurePursuitPoint {
  odom::RobotPosition position;
  float velocity;
};

void moveVelocity(double left, double right);
void move(double left, double right);
void setChassisBrake(pros::motor_brake_mode_e_t mode);

// AUTON PATHS
void initAutonRoutes();
void runSkillsPath();

} // namespace chassis
