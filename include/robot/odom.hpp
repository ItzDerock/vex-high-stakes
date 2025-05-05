#pragma once
#include "ExitCondition.hpp"
#include "PID.hpp"
#include "position.hpp"
#include "pros/rtos.hpp"
#include <string>
#include <sys/types.h>
#include <vector>

/****************
 * CONSTANTS for settlement behavior
 * Format: range, time
 ****************/
#define LATERAL_LARGE_EXIT 4, 300
#define LATERAL_SMALL_EXIT 1.2, 100
#define ANGULAR_LARGE_EXIT 3, 300
#define ANGULAR_SMALL_EXIT 2, 150

#define CHAINED_LATERAL_LARGE_EXIT 4, 100
#define CHAINED_LATERAL_SMALL_EXIT 1, 50
#define CHAINED_ANGULAR_LARGE_EXIT 3, 100
#define CHAINED_ANGULAR_SMALL_EXIT 2, 50

namespace odom {

/**
 * PID controller for odom-related tasks.
 */
extern PIDController turnPID;
extern PIDController drivePID;
extern ExitCondition lateralLargeExit;
extern ExitCondition lateralSmallExit;
extern ExitCondition angularLargeExit;
extern ExitCondition angularSmallExit;

/**
 * Returns the current robot location, by default in radians.
 * @param degrees whether to return the angle in degrees
 * @param standardPos whether to return the angle in standard form (0 deg is +x)
 */
RobotPosition getPosition(bool degrees = false, bool standardPos = false);

/**
 * Returns the current robot velocity in inches/sec.
 */
float getVelocity();

/**
 * Updates the odoemtry position
 */
void update();
void updateLoop();

/**
 * Resets the odometry to a given position.
 */
void reset(RobotPosition startState);
void reset();

/**
 * Sets the position of the robot.
 */
void setPosition(const odom::RobotPosition &position);

/**
 * Initializes the odometry task.
 */
void initalize();

/**
 * turn to a given angle (in degrees)
 */
void turnTo(double degrees, double timeout = 2.5, double maxSpeed = 127.0);

struct MoveToPoseParams {
  float maxSpeed = 127;
  float minSpeed = 0;
  float chasePower = 10;
  float lead = 0;
  float earlyExitRange = 0;
  float slew = 3.5;
  bool forwards = true;

  // stall detection (in ms)
  bool exitOnStall = false;
  float stallTime = 50;
  float stallThreshold = 0.5; // inches/sec

  // especially useful in this comp
  float maxSpeedWhenClose = 127;
  float closeThreshold = 5;
};

/**
 * Drive a certain distance (in inches)
 */
void moveDistance(double distance, double timeout = 10'000,
                  MoveToPoseParams params = {}, bool async = false);

/**
 * @brief Move the chassis towards the target pose
 *
 * Uses the boomerang controller
 *
 * @param x x location
 * @param y y location
 * @param theta target heading in degrees.
 * @param timeout longest time the robot can spend moving
 *
 * @param maxSpeed the maximum speed the robot can move at. 127 at default
 * @param async whether the function should be run asynchronously. true by
 * default
 */
void moveTo(float x, float y, float theta, int timeout, MoveToPoseParams params,
            bool async = false);

void moveTo(odom::RobotPosition position, int timeout, MoveToPoseParams params,
            bool async = false);

/**
 * Lowers the requirements for settlement for the next `amount` movement
 * commands. This is useful for chaining movements together without having to
 * wait for the robot to fully settle.
 *
 * @param amount the amount of movement commands to lower the requirements for
 */
void startChainedMovement(int amount);

/**
 * The odometry mutex. Use whenever you are reading values.
 */
extern pros::Mutex mutex;

/**
 * Holds the robot facing a certain angle.
 * @param angle the angle to hold the robot at (-1 to disable)
 */
void holdAngle(double angle);

/**
 * Waits until the robot is settled
 */
void waitUntilSettled(uint timeout = 10'000);

/**
 * Waits until a certain distance is reached
 */
void waitUntilDistance(double distance, uint timeout = 10'000);

/**
 * Autonomous enum
 */
enum class Autonomous { None, Skills, SoloAWP, Rush, Alliance };

/**
 * follow pure pursuit path
 */
void follow(std::vector<odom::RobotPosition> *pathPoints, float lookahead,
            int timeout, bool forwards, bool async);

/**
 * Loads the given paths into the cache.
 */
void loadPaths(std::vector<std::string> const &files);

/**
 * Returns a path from the cache.
 */
std::vector<odom::RobotPosition> *getPath(std::string const &path);

extern Autonomous autonomous;

} // namespace odom
