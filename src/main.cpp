#include "main.h"

#include "config.hpp"
#include "pros-mpeg/mpeg.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
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

  subsystems::initInakeTask();
  subsystems::initLiftTask();

  // load pure pursuit paths
  odom::loadPaths({"/usd/skills/TLtTR3ring.txt", "/usd/pathtest.txt"});

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

  switch (odom::autonomous) {
  case odom::Autonomous::Skills:
    chassis::runSkillsPath();
    break;

  default:
    printf("Invalid autonomous mode\n");
  }
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
    // printf("dist: %d\n", intake_sensor.get_proximity());
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
        // less efficient and unreadable, but cool
      }

      chassis::move(leftPower, rightPower);
    }

    // intake
    // DIGITAL_R1: intake in
    // DIGITAL_L1: intake out
    double intake_speed = 127 * master.get_digital(DIGITAL_R1) -
                          (master.get_digital(DIGITAL_R2)) * 127;

    intake_motor_stg1.move(intake_speed);
    if (!subsystems::lockIntakeControls.load()) {
      intake_motor_stg2.move(intake_speed);
    }

    // toggle redirect
    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      subsystems::intakeRedirectMode.store(
          !subsystems::intakeRedirectMode.load());
    }

    // lift
    if (master.get_digital_new_press(DIGITAL_L1)) {
      subsystems::cycleLiftPosition();
    }

    // grabber on DIGITAL_L2
    if (master.get_digital_new_press(DIGITAL_L2)) {
      grabber_1.toggle();
      grabber_2.toggle();
    }

    // reversing the drivetrain
    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      dtReversed = !dtReversed;
      master.rumble(dtReversed ? "-" : ".");
    }

    // print the current team and redirect status onto the controller
    std::string team_str =
        subsystems::currentTeam == subsystems::Color::RED    ? "Red "
        : subsystems::currentTeam == subsystems::Color::BLUE ? "Blue"
                                                             : "None";

    std::string redirect_str =
        subsystems::intakeRedirectMode.load() ? "REDIRECTING" : "           ";

    master.print(0, 0, (team_str + redirect_str).c_str());

    // DIGITAL_UP to change the current team
    // long rumble = now color is red, short rumble = now color is blue
    if (master.get_digital_new_press(DIGITAL_UP)) {
      subsystems::currentTeam =
          subsystems::currentTeam == subsystems::Color::RED
              ? subsystems::Color::BLUE
              : subsystems::Color::RED;

      master.rumble(subsystems::currentTeam == subsystems::Color::RED ? "-"
                                                                      : ".");
    }

    pros::delay(10);
  }
}
