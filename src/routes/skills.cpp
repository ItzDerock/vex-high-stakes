
#include "../config.hpp"
#include "pros/rtos.hpp"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"
#include "robot/position.hpp"
#include "robot/subsystems.hpp"

#define GOAL_TO_ROBOT_CENTER_DISTANCE_IN 11
#define GOAL_SCORED_CENTER_X 65
#define GOAL_SCORED_CENTER_Y 65

// fwd decl
void doPolarReset(const odom::RobotPosition center, double radius);

// impl
void chassis::runSkillsPath() {
  grabber_1.extend();
  grabber_2.extend();
  subsystems::setTargetLiftPosition(4200);
  pros::delay(800);
  odom::startChainedMovement(8);

  // 1. Back up to grab the mogo
  odom::moveDistance(-5.5, 600, {.chasePower = 60});
  subsystems::setTargetLiftPosition(subsystems::liftPositions[0]);
  odom::turnTo(180);
  odom::moveTo(-48.5, 22, 180, 4'000,
               {
                   .chasePower = 15,
                   .lead = 0.5,
                   .forwards = false,

               },
               true);
  pros::delay(700);
  grabber_1.retract();
  grabber_2.retract();
  pros::delay(250);
  odom::waitUntilSettled(1'000);

  // 2. Turn to face 90 degrees
  odom::turnTo(90);

  // 3. Start up the intake
  intake_motor_stg1.move(127);
  intake_motor_stg2.move(127);

  // 4. Grab 2nd ring
  odom::moveTo(-23, 23, 90, 5'000, {.chasePower = 100});
  pros::delay(200);
  odom::moveTo(-1, 55, 30, 5'000, {}); // center ring
  odom::moveDistance(-5);

  // Grab 3 more rings
  odom::moveTo(-23, 50, 270, 2'500, {.chasePower = 70, .lead = 0.5});
  pros::delay(300);
  odom::moveTo(-65, 50, 270, 5'000,
               {.maxSpeed = 80, .chasePower = 40, .exitOnStall = true});

  // last ring
  pros::delay(250);
  odom::moveDistance(-24);
  pros::delay(250);
  odom::moveTo(-54, 60, 315, 2'000, {.maxSpeed = 60});
  odom::moveDistance(2);
  pros::delay(800);
  odom::moveDistance(-12);
  pros::delay(100);

  // score the goal
  intake_motor_stg2.move(-127);
  odom::moveTo(-70, 70, 135, 4'000, {.forwards = false});
  odom::moveDistance(-3, 2'000, {.forwards = false, .exitOnStall = true});
  grabber_1.extend();
  grabber_2.extend();
  pros::delay(250);
  doPolarReset({-GOAL_SCORED_CENTER_X, GOAL_SCORED_CENTER_Y, 0},
               GOAL_TO_ROBOT_CENTER_DISTANCE_IN);
  pros::delay(250);
  std::cout << "resume at " << odom::getPosition(true, false) << std::endl;
  odom::moveDistance(10);
  intake_motor_stg2.move(127);

  // FIRST 6 RING COMPLETE!
  odom::startChainedMovement(8);

  odom::moveTo(-48, -15, 0, 5'000, {.forwards = false});
  odom::moveTo(-48, -24, 0, 2'000, {.maxSpeed = 60, .forwards = false});
  grabber_2.retract();
  grabber_1.retract();

  odom::moveTo(-23, -25, 90, 5'000,
               {.chasePower = 100}); // center square ring, 1
  odom::moveTo(0, -50, 180, 2'000, {});
  odom::moveTo(0, -60, 180, 2'000, {}); // mid ring, 2
  odom::turnTo(270);
  odom::moveTo(
      -66.5, -51, 270, 7'000,
      {.maxSpeed = 40, .lead = 0.7, .exitOnStall = true}); // long thingy

  odom::moveDistance(-24); // last ring
  odom::moveTo(-55, -58, 200, 2'000, {.maxSpeed = 60});
  pros::delay(1500);
  odom::moveDistance(-15);
  odom::turnTo(45);

  // score goal 2
  intake_motor_stg2.move(-127);
  odom::moveTo(-70, -70, 45, 1'000, {.forwards = false, .exitOnStall = true});
  chassis::move(-100, -100);
  pros::delay(1000);
  chassis::move(0, 0);
  grabber_1.extend();
  grabber_2.extend();
  pros::delay(250);
  doPolarReset({-GOAL_SCORED_CENTER_X, -GOAL_SCORED_CENTER_Y, 0},
               GOAL_TO_ROBOT_CENTER_DISTANCE_IN);
  pros::delay(250);
  odom::moveDistance(10);
  intake_motor_stg2.move(80);

  return;

  // CROSS MAP TIME
  odom::moveTo(25, -35, 90, 5'000, {.lead = 0.4});
  intake_motor_stg2.move(0);
  odom::turnTo(236);

  // grab the blue thingy
  odom::moveTo(42, -35, 240, 6'000, {.maxSpeed = 60, .forwards = false});
  odom::moveTo(60, -53, 0, 4'000, {.maxSpeed = 90, .forwards = false});
  chassis::move(-127, -127);

  pros::delay(3000); // let go
  intake_motor_stg2.move(-127);
  grabber_1.extend();
  grabber_2.extend();
  pros::delay(500);
  odom::turnTo(305);
  odom::moveDistance(12);

  // odom::moveTo(51, -68, 0, 4'000, {.forwards = false});

  // drop it off
  // grabber_1.retract();
  // grabber_2.retract();
  // pros::delay(250);
  // odom::turnTo(305, 0.5);
  // odom::moveTo(54, -35, 0, 2'000, {});
}

/**
 * Resets the position of the robot for use mid-auton when in a known radius
 * location (i.e. when scoring a goal, and in the corner, where wheels might
 * slip)
 *
 * Since the IMU is accurate, we use that combined with a center point to find
 * a point on that radius circle to reset to.
 */
void doPolarReset(const odom::RobotPosition center, double radius) {
  odom::RobotPosition current = odom::getPosition(false, true);

  double offset_x = radius * cos(current.theta);
  double offset_y = radius * sin(current.theta);

  odom::RobotPosition newPose = {
      center.x + offset_x,
      center.y + offset_y,
      odom::getPosition(false, false).theta,
  };

  odom::setPosition(newPose);

  std::cout << "Polar reset to " << newPose << std::endl;
}
