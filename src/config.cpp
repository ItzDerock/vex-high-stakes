/**
 * This file just defines the global variables declared in config.hpp.
 * Please edit config.hpp instead of this file for port changes.
 */

#include "config.hpp"

#include "pros/adi.hpp"

/** helper function that defines shared pointers */
#define SHARED(type, name, ...)                                                \
  std::shared_ptr<type> name = std::make_shared<type>(__VA_ARGS__)

pros::Motor drive_left_front(-DRIVE_LEFT_FRONT, DRIVETRAIN_GEARBOX);
pros::Motor drive_left_back(-DRIVE_LEFT_BACK, DRIVETRAIN_GEARBOX);
pros::Motor drive_left_pto(DRIVE_LEFT_PTO, DRIVETRAIN_GEARBOX);
pros::Motor drive_right_front(DRIVE_RIGHT_FRONT, DRIVETRAIN_GEARBOX);
pros::Motor drive_right_back(DRIVE_RIGHT_BACK, DRIVETRAIN_GEARBOX);
pros::Motor drive_right_pto(-DRIVE_RIGHT_PTO, DRIVETRAIN_GEARBOX);

std::vector<pros::Motor *> drive_left = std::vector<pros::Motor *>{
    &drive_left_front, &drive_left_back, &drive_left_pto};

std::vector<pros::Motor *> drive_right = std::vector<pros::Motor *>{
    &drive_right_front, &drive_right_back, &drive_right_pto};

// can't use a designator until P2287R1 is merged
// https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2021/p2287r1.html
// which won't make it to C++23

// OdomIntegratedSensor odom_left{
//     .sensor = drive_left_back,
//     .offset = DRIVE_TRACK_WIDTH / 2,
//     .gear_ratio = DRIVETRAIN_GEAR_RATIO,
//     .wheel_size = 4,
// };

// OdomIntegratedSensor odom_right{
//     .sensor = drive_right_back,
//     .offset = DRIVE_TRACK_WIDTH / 2,
//     .gear_ratio = DRIVETRAIN_GEAR_RATIO,
//     .wheel_size = 4,
// };

// offset, gear ratio, wheel size
OdomIntegratedSensor odom_left(drive_left_back, (double)DRIVE_TRACK_WIDTH / 2,
                               DRIVETRAIN_GEAR_RATIO, 4.125);
OdomIntegratedSensor odom_right(drive_right_back, (double)DRIVE_TRACK_WIDTH / 2,
                                DRIVETRAIN_GEAR_RATIO, 4.125);

// pros::adi::Encoder odom_middle_sensor(ODOM_MIDDLE_PORT, false);
// OdomSensor odom_middle(odom_middle_sensor, 4, 1, ODOMETRY_WHEEL_DIAMETER);

pros::Imu inertial(ODOM_INERTIAL);
pros::adi::Pneumatics grabber_1(BLOCKER_1, false, true);
pros::adi::Pneumatics grabber_2(BLOCKER_2, false, true);

pros::Optical intake_sensor(INTAKE_SENSOR_PORT);

// we need individual control of the intake motors
pros::Motor intake_motor_stg1(INTAKE_PORT_1, pros::v5::MotorGear::green);
pros::Motor intake_motor_stg2(-INTAKE_PORT_2, pros::v5::MotorGear::green);

// Lift
pros::Motor lift(LIFT);
pros::Rotation lift_sensor(LIFT_ROT_SENSOR_PORT);

#undef SHARED
