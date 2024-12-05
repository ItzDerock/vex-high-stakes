#pragma once
#include "main.h"
#include "pros/adi.hpp"

// ODOMETRY
#define ODOMETRY_TICKS_PER_INCH 360.0 // ticks per inch
#define ODOMETRY_WHEEL_DIAMETER 3.25  // inches
// #define ODOM_MIDDLE_PORT 'c', 'd'
#define ODOM_INERTIAL 7

// DRIVETRAIN
#define DRIVETRAIN_GEAR_RATIO /* input 36 -> output 72 */ (double)36 / 72
#define DRIVETRAIN_GEARBOX pros::v5::MotorGear::blue
#define DRIVE_LEFT_FRONT 1
#define DRIVE_LEFT_BACK 2
#define DRIVE_LEFT_PTO 3
#define DRIVE_RIGHT_FRONT 6
#define DRIVE_RIGHT_BACK 4
#define DRIVE_RIGHT_PTO 5
#define DRIVE_TRACK_WIDTH 12.3 // inches

// LIFT
#define LIFT_LEFT 9
#define LEFT_RIGHT 8

// INTAKE
#define INTAKE_PORT_1 12
#define INTAKE_PORT_2 10
#define INTAKE_SENSOR_PORT 11

// WINGS
#define BLOCKER_1 'g'
#define BLOCKER_2 'h'

/*************************
 * VARIABLE DECLARATIONS *
 *************************/

//// Odometry

struct BasicOdomSensor {
  double offset;
  double gear_ratio;
  double wheel_size;
};

struct OdomSensor : BasicOdomSensor {
  pros::adi::Encoder sensor;

  OdomSensor(pros::adi::Encoder &sensor, double offset, double gear_ratio,
             double wheel_size)
      : BasicOdomSensor{offset, gear_ratio, wheel_size}, sensor(sensor) {}
};

struct OdomIntegratedSensor : BasicOdomSensor {
  pros::Motor sensor;

  OdomIntegratedSensor(pros::Motor &sensor, double offset, double gear_ratio,
                       double wheel_size)
      : BasicOdomSensor{offset, gear_ratio, wheel_size}, sensor(sensor) {}
};

// odometry sensors
// no need to use shared pointers
extern OdomIntegratedSensor odom_left;
extern OdomIntegratedSensor odom_right;
// extern OdomSensor odom_middle;
extern pros::Imu inertial;

//// Drivetrain
extern pros::Motor drive_left_front;
extern pros::Motor drive_left_back;
extern pros::Motor drive_left_pto;
extern pros::Motor drive_right_front;
extern pros::Motor drive_right_back;
extern pros::Motor drive_right_pto;
extern std::vector<pros::Motor *> drive_left;
extern std::vector<pros::Motor *> drive_right;

//// Catapult
extern pros::Motor catapult_motor;
extern pros::Rotation catapult_position;

///// Wings
extern pros::adi::Pneumatics wings_left;
extern pros::adi::Pneumatics wings_right;
// todo: separate wings

///// Blocker
extern pros::adi::Pneumatics grabber_1;
extern pros::adi::Pneumatics grabber_2;

///// Intake
extern pros::Motor intake_motor_stg1; // outer intake
extern pros::Motor intake_motor_stg2; // disk ring intake
extern pros::Optical intake_sensor;

//// Lift
extern pros::Motor lift_left;
extern pros::Motor lift_right;

// undefine SHARED macro to prevent accidental use
#undef SHARED
