#include "robot/chassis.hpp"
#include "../config.hpp"

/**
 * @brief Moves the drivetrain (velocity control)
 */
void chassis::moveVelocity(double left, double right) {
  drive_left_back.move_velocity(left);
  drive_left_front.move_velocity(left);
  drive_left_pto.move_velocity(left);
  drive_right_back.move_velocity(right);
  drive_right_front.move_velocity(right);
  drive_right_pto.move_velocity(right);
}

/**
 * @brief Moves the drivetrain (power control)
 */
void chassis::move(double left, double right) {
  drive_left_back.move(left);
  drive_left_front.move(left);
  drive_left_pto.move(left);
  drive_right_back.move(right);
  drive_right_front.move(right);
  drive_right_pto.move(right);
}

/**
 * @brief Sets the drivetrain brake mode
 */
void chassis::setChassisBrake(pros::motor_brake_mode_e_t mode) {
  // use array to set all motors at once
  pros::Motor *motors[] = {&drive_left_back,   &drive_left_front,
                           &drive_left_pto,    &drive_right_back,
                           &drive_right_front, &drive_right_pto};

  // unroll loop for better performance
#pragma GCC unroll 6
  for (auto motor : motors) {
    motor->set_brake_mode(mode);
  }
}
