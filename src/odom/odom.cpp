#include "robot/odom.hpp"

#include <atomic>
#include <math.h>

#include "../config.hpp"
#include "pros/abstract_motor.hpp"
#include "robot/utils.hpp"

// #define ODOM_DEBUG true
#define ODOM_UPDATE_INTERVAL 10 /* ms */

/**
 * Odometry implementation is based on the following paper:
 * http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
 *
 * Movement functions are not based on that paper.
 */

// Task to update the odom
pros::Task *odomTask = nullptr;
pros::Mutex odom::mutex;

struct {
  double left, right, center, theta;
} prevSensors = {0, 0, 0, 0};

struct {
  double left, right, theta;
} resetValues = {0, 0, 0};

odom::RobotPosition *state = new odom::RobotPosition({0, 0, 0});
std::atomic<double> velocity = 0;

/**
 * Since part of our odometry is based on internal sensors in the motors,
 * we need to normalize the data to account for different gear ratios.
 *
 * @note - marked inline to reduce overhead of function calls
 *       - marked static so only accessible in odom.cpp
 *
 * @param sensor The sensor to normalize
 */
inline static double normalizeSensorData(double position,
                                         BasicOdomSensor sensor) {
  double wheelRotations = position / 360.0;
  double adjustedWheelRotations = wheelRotations * sensor.gear_ratio;
  double wheelCircumference = sensor.wheel_size * M_PI;

  return adjustedWheelRotations * wheelCircumference;
}

double readDrivetrainSensor(std::vector<pros::Motor *> const &motors) {
  // find median of the three drivetrain motors
  // Median is resistant to outliers, so if a DT sensor is bugged and reads 0,
  // it won't skew the data
  double values[3] = {motors[0]->get_position(), motors[1]->get_position(),
                      motors[2]->get_position()};

#ifdef ODOM_DEBUG
  printf("[odom] dt sensors read: %f, %f, %f", values[0], values[1], values[2]);
#endif

  // https://stackoverflow.com/questions/1582356/fastest-way-of-finding-the-middle-value-of-a-triple
  // no need for sorted array!
  return std::max(std::min(values[0], values[1]),
                  std::min(std::max(values[0], values[1]), values[2]));
}

void odom::update() {
  // lock mutex
  mutex.take();

  // 1. Store the current encoder values
  double left = readDrivetrainSensor(drive_left);
  double right = readDrivetrainSensor(drive_right);
  // double center = (double)odom_middle.sensor->get_value();

#ifdef ODOM_DEBUG
  printf("[odom] drivetrain LR raw: %f, %f\n", left, right);
#endif

  //  (1.1) Convert to distance of wheel travel (inches)
  left = normalizeSensorData(left, odom_left);
  right = normalizeSensorData(right, odom_right);
  // center = normalizeSensorData(center, odom_middle);

  // 2. Calculate delta values
  double dL = left - prevSensors.left;
  double dR = right - prevSensors.right;

#ifdef ODOM_DEBUG
  printf("[odom] drivetrain LR inches: %f, %f\n", left, right);
  printf("[odom] resulting in dL, dR: %f, %f\n", dL, dR);
#endif

  // 3. Update the previous values
  prevSensors.left = left;
  prevSensors.right = right;

  // 5. Calculate new orientation
  double newTheta = resetValues.theta + inertial.get_heading() * M_PI / 180;
  if (newTheta > 2 * M_PI)
    newTheta -= 2 * M_PI;

  // 6. Calculate change in orientation
  double dTheta = newTheta - state->theta;
  double d = (dL + dR) / 2;

  // copy the state so we can calculate the velocity
  RobotPosition prevState = *state;

  // 7. Update the state
  state->y += d * cos(state->theta + dTheta / 2);
  state->x += d * sin(state->theta + dTheta / 2);
  state->theta = newTheta;

  // calculate the velocity
  // divide by 10 because this function is called every 10ms
  // multiply by 1000 to get it in inches per second
  velocity = (state->distance(prevState) / ODOM_UPDATE_INTERVAL) * 1000;

  // The following is the legacy odometry algorithm
  // it has been replaced with the above code

  /*
    // 7. Calculate local offset for dTheta = 0
    RobotPosition localOffset = {0, 0, 0};

    if (dTheta == 0) {
      localOffset.x = dC;
      localOffset.y = dR;
    } else {
      // 8. Otherwise, calculate local offset with formula.
      localOffset.x = 2 * sin(dTheta / 2) * (dC / dTheta +
    (odom_middle.offset));

      localOffset.y = 2 * sin(dTheta / 2) * (dR / dTheta + (odom_right.offset));
    }

    // 9. Calculate the average orientation
    double thetam = state->theta + dTheta / 2;

    // 10. Calculate the global offset
    RobotPosition globalOffset = {0, 0, 0};

    // convert local offset to polar coordinates
    double r =
        sqrt(localOffset.x * localOffset.x + localOffset.y * localOffset.y);
    double theta = atan2(localOffset.y, localOffset.x);

    // subtract thetam from the angle component
    theta -= thetam;

    // convert back to Cartesian coordinates
    globalOffset.x = r * cos(theta);
    globalOffset.y = r * sin(theta);

    // 11. Update the global position
    state->x += globalOffset.x;
    state->y += globalOffset.y;
    state->theta = newTheta;
  */

  // #if ODOM_DEBUG
  //   logger::log(logger::Route::RobotPosition, {state->x, state->y,
  //   state->theta});

  //   // I wish there was a more elegant way to do this
  //   logger::log(logger::Route::RobotVelocity,
  //               {
  //                   drive_left_back->get_actual_velocity(),
  //                   (double)drive_left_back->get_target_velocity(),
  //                   drive_right_back->get_actual_velocity(),
  //                   (double)drive_right_back->get_target_velocity(),
  //               });
  // #endif

  // unlock mutex
  mutex.give();
}

void odom::updateLoop() {
  while (true) {
    update();
    pros::delay(ODOM_UPDATE_INTERVAL);
  }
}

void odom::initalize() {
  if (odomTask != nullptr) {
    std::cout << "WARNING: odom::init() called when odomTask is not null"
              << std::endl;
    return;
  }

  odomTask = new pros::Task(updateLoop);
}

// macro to handle errors properly
#define CHECK_SUCCESS(fn, name)                                                \
  if (fn != 1) {                                                               \
    std::cerr << "FAILED TO RESET ODOMETRY!" << std::endl                      \
              << "errorno: " << errno << std::endl                             \
              << "at: " << name << std::endl;                                  \
  }

void odom::reset(odom::RobotPosition startState) {
  // aquire mutex
  mutex.take();

  // stop task
  bool taskRunning = odomTask != nullptr;
  if (taskRunning) {
    odomTask->remove();
    odomTask = nullptr;
  }

  // reset encoders
  for (auto motor : drive_left) {
    motor->set_encoder_units(pros::MotorUnits::degrees);
    CHECK_SUCCESS(motor->set_zero_position(0), "drive_left");
  }

  for (auto motor : drive_right) {
    motor->set_encoder_units(pros::MotorUnits::degrees);
    CHECK_SUCCESS(motor->set_zero_position(0), "drive_right");
  }

  // CHECK_SUCCESS(odom_middle.sensor.reset(), "odom_middle");
  CHECK_SUCCESS(inertial.reset(true), "odom_imu");

  // reset state
  // state = startState;
  state->x = startState.x;
  state->y = startState.y;
  state->theta = startState.theta;
  resetValues.theta = startState.theta;

  // reset prevSensors
  prevSensors = {0, 0, 0, 0};

  // delay 10ms to let the sensors reset
  pros::delay(10);

  // restart task
  if (taskRunning)
    initalize();

  // release mutex
  mutex.give();
}

void odom::reset() { reset({0, 0, 0}); }

odom::RobotPosition odom::getPosition(bool degrees, bool standardPos) {
  // take mutex to prevent reading while writing
  // technically not necessary as the cpu is single threaded
  // but it's good practice
  mutex.take();

  // get the state
  RobotPosition returnState =
      degrees ? RobotPosition(state->x, state->y, state->theta * (180 / M_PI))
              : *state;

  mutex.give();

  // bearing -> standard form
  if (standardPos) {
    returnState.theta = utils::angleSquish(M_PI_2 - returnState.theta);
  }

  return returnState;
}

float odom::getVelocity() {
  // variable is atomic so no need to lock mutex
  return velocity;
}
