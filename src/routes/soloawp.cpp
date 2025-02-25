#include "../config.hpp"
#include "pros/rtos.hpp"
#include "robot/chassis.hpp"
#include "robot/odom.hpp"
#include "robot/subsystems.hpp"

void chassis::runSoloAWPPath() {
  grabber_1.extend();
  grabber_2.extend();

  odom::moveTo(-33, -29.5, 360 - 45, 5'000, {.lead = 0.3, .forwards = false});
  odom::moveDistance(-4);
  grabber_1.retract();
  grabber_2.retract();
  pros::delay(300);

  // grab rings
  intake_motor_stg1.move(127);
  intake_motor_stg2.move(127);

  odom::moveTo(-23, -47, 0, 5'000, {.lead = 0.2});

  // touch
  odom::moveTo(-23.6, 0, 180, 5'000, {.lead = 0.2});
  intake_motor_stg1.move(0);
  intake_motor_stg2.move(0);
}

// #define TOP_SIDE_DOUBLE false

// void chassis::runSoloAWPPath() {
//   grabber_1.extend();
//   grabber_2.extend();
//   // subsystems::setTargetLiftPosition(4200);
//   // odom::startChainedMovement(6);
//   // odom::moveDistance(3);
//   // pros::delay(800);
//   // subsystems::setTargetLiftPosition(subsystems::liftPositions[0]);

//   // top left mogo
//   odom::moveTo(-34, -14, 295, 5'000, {.forwards = false}); // cm1
//   odom::moveDistance(-6, 2'000, {.maxSpeed = 80});         // cm2
//   grabber_1.retract();
//   grabber_2.retract();
//   pros::delay(200);

//   // grab rings
//   intake_motor_stg1.move(127);
//   intake_motor_stg2.move(127);

//   // #ifdef TOP_SIDE_DOUBLE
//   //   odom::moveTo(-11, -40, 45, 5'000, {.maxSpeed = 100, .lead = 0}); //
//   cm3
//   //   odom::moveDistance(-10);                                         //
//   cm4
//   // #endif
//   odom::moveTo(-23, -50, 180, 5'000, {.lead = 0}); // cm5
//   pros::delay(1000);

//   odom::moveTo(-52, 0, 0, 5'000, {.lead = 0.3}); // cm6

//   // touch
//   odom::moveTo(-24, 4, 90, 5'000, {.lead = 0});

//   return;

//   // cross map
//   odom::startChainedMovement(6);
//   odom::moveTo(-52, -24, 180, 5'000, {}); // cm1
//   grabber_1.extend();
//   grabber_2.extend();
//   intake_motor_stg1.move(0);
//   intake_motor_stg2.move(0);
//   odom::moveTo(-52, -35, 180, 5'000, {}); // cm2

//   // grab secong mogo
//   odom::moveTo(-38, -32, 180, 5'000,
//                {.lead = 0.3, .maxSpeedWhenClose = 60}); // cm3
//   grabber_1.retract();
//   grabber_2.retract();
// }

// // void chassis::runSoloAWPPath() {
// //   odom::startChainedMovement(6);

// //   // grab mid
// //   odom::moveTo(-24, 24, 310, 2'500, {.maxSpeed = 80, .forwards = false});
// //   grabber_1.extend();
// //   grabber_2.extend();
// //   pros::delay(250);

// //   // start intake
// //   intake_motor_stg1.move(127);
// //   intake_motor_stg2.move(127);

// //   // grab bottom ring
// //   odom::moveTo(-24, 48, 0, 5'000, {.maxSpeed = 100});
// //   pros::delay(200);
// //   pros::Task([] {
// //     pros::delay(600);
// //     intake_motor_stg1.move(-127);
// //   });

// //   // line up for 2nd goal
// //   odom::moveTo(-47, -10, 180, 2'500, {.maxSpeed = 127, .lead = 0.4});
// //   grabber_1.retract();
// //   grabber_2.retract();
// //   pros::delay(250);

// //   // grab 2nd
// //   odom::moveTo(-24, -24, 300, 2'500, {.maxSpeed = 75, .forwards = false});
// //   grabber_1.extend();
// //   grabber_2.extend();
// //   pros::delay(250);
// //   intake_motor_stg1.move(127);

// //   // grab ring to score
// //   odom::moveTo(-24, -48, 180, 5'000, {.maxSpeed = 100});
// //   pros::delay(500);
// //   pros::Task([] {
// //     pros::delay(400);
// //     grabber_1.retract();
// //     grabber_2.retract();
// //   });
// //   odom::moveTo(-24, 0, 0, 5'000, {.maxSpeed = 80});
// // }
