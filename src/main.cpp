#include "main.h"

#include "config.hpp"
#include "pros-mpeg/mpeg.hpp"
#include "pros/misc.h"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"
#include "robot/screen.hpp"
#include "robot/subsystems.hpp"
#include "robot/utils.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  odom::initalize();
  odom::reset({0, 0, 0});

  // load pure pursuit paths
  odom::loadPaths({"/usd/skills/push-left.txt", "/usd/pathtest.txt"});

  static MPEGPlayer mpeg("/usd/game.mpeg", lv_scr_act());
  screen::initAutonSelector(&mpeg);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  odom::RobotPosition start = odom::getPosition();
  // TODO: implement autonomous routines for this competition
}

/**
 * Runs the operator control code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart
 * the task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  bool dtReversed = false;

  while (true) {
    // one stick is throttle, one controls turning radius
    // so we can speed up while maintaining a tight turn
    double throttle = master.get_analog(ANALOG_RIGHT_Y);
    double turn = master.get_analog(ANALOG_LEFT_X);

    // small deadzone
    if (fabs(throttle) < 4) {
      double leftPower = utils::driveCurve(turn);

      chassis::move(leftPower, -leftPower);
    } else {
      double leftPower = throttle + (std::abs(throttle) * turn) / 127.0;
      double rightPower = throttle - (std::abs(throttle) * turn) / 127.0;

      leftPower = utils::driveCurve(leftPower);
      rightPower = utils::driveCurve(rightPower);

      if (dtReversed) {
        double originalLeftPower = leftPower;
        leftPower = -rightPower;
        rightPower = -originalLeftPower;

        // leftPower += rightPower; // x = x + y
        // rightPower -= leftPower; // y = y - x
        // leftPower = -leftPower - rightPower; // x = -x - y
        // swaps x and y and flips signs without temporary variables
      }

      chassis::move(leftPower, rightPower);
    }

    // intake
    // DIGITAL_R1: intake in
    // DIGITAL_L1: intake out
    intake_motor.move(
        127 * master.get_digital(DIGITAL_R1) -
        (master.get_digital(DIGITAL_L1) || master.get_digital(DIGITAL_R2)) *
            127);

    // grabber on DIGITAL_L2
    if (master.get_digital_new_press(DIGITAL_L2)) {
      blocker_1.toggle();
      blocker_2.toggle();
    }

    // reversing the drivetrain
    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      dtReversed = !dtReversed;
      master.rumble(dtReversed ? "-" : ".");
    }

    pros::delay(10);
  }
}