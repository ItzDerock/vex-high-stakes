#include "../config.hpp"
#include "robot/subsystems.hpp"
#include "robot/utils.hpp"
#include <atomic>
#include <cstdint>
#include <queue>

#define NEAR_THRESHOLD 200
#define COLOR_RED 0
#define COLOR_BLUE 229
#define COLOR_THRESHOLD 20
#define MOVES_TILL_STOP 0
#define MOVES_TILL_REDIRECT 0
#define MOVES_TILL_SLOWDOWN 0

/**
 * Changes the behavior of how the color is detected.
 * Rising edge = immediately when the color sensor detects a ring.
 * Falling edge = wait for when the color sensor finishes detecting a ring.
 *
 * |     |-- Ring (distance) --|
 * | ____|                     |______
 * d     ^ rising edge         ^ falling edge
 */
#define INTAKE_DETECT_ON_RISING_EDGE true

// #define INTAKE_DEBUG true

struct Interrupt {
  enum Type { SORT, REDIRECT, SLOW_DOWN };
  Type type;
  double position;

  // prority queue sorted by rotations
  // so need to implement comparison operator
  bool operator<(const Interrupt &other) const {
    return position < other.position;
  }
};

// priority queue sorted by rotations
std::priority_queue<Interrupt> interrupts;
std::atomic<bool> subsystems::lockIntakeControls(false);
std::atomic<bool> subsystems::intakeRedirectMode(false);
std::atomic<subsystems::Color> subsystems::currentTeam(NONE);
std::atomic<double> subsystems::maxIntakePower(127);

/**
 * Compares a given hue with the COLOR_{RED,BLUE} constants
 */
subsystems::Color classify_color(double hue) {
  return (
      // hue is a wheel, [0, 360] so we can take advantage of angle
      // functions
      fabs(utils::angleError(hue, COLOR_RED)) < COLOR_THRESHOLD
          ? subsystems::RED
      : fabs(utils::angleError(hue, COLOR_BLUE)) < COLOR_THRESHOLD
          ? subsystems::BLUE
          : subsystems::NONE);
}

void check_for_interrupts() {
  while (true) {
    while (!interrupts.empty()) {
      Interrupt next_interrupt = interrupts.top();
      double motor_position = intake_motor_stg2.get_position();

      if (next_interrupt.position > motor_position) {
        break;
      }

      std::cout << "interrupt at " << next_interrupt.position
                << " and motor is " << motor_position << std::endl;

      // lock the controls to prevent driver from interfering
      subsystems::lockIntakeControls.store(true);
      double current_velocity = intake_motor_stg2.get_target_velocity();

      switch (next_interrupt.type) {
      case Interrupt::SORT:
        intake_motor_stg2.move(-127);
        pros::delay(100);
        break;

      case Interrupt::REDIRECT:
        intake_motor_stg2.move(-127);
        pros::delay(400);
        subsystems::maxIntakePower.store(127);
        break;

      case Interrupt::SLOW_DOWN:
        intake_motor_stg2.move(80);
        subsystems::maxIntakePower.store(80);
        break;
      }

      // and unlock the controls
      intake_motor_stg2.move_velocity(current_velocity);
      subsystems::lockIntakeControls.store(false);
      interrupts.pop();
    }

    pros::delay(5);
  }
}

void intake_loop() {
  while (true) {
    uint32_t distance = intake_sensor.get_proximity();

#ifdef INTAKE_DEBUG
    printf("d: %d\n", distance);
#endif

    // check if NEAR on intake sensor
    if (distance < NEAR_THRESHOLD) {
      pros::delay(10);
      continue;
    }

#if INTAKE_DETECT_ON_RISING_EDGE
    // loop until we get a valid color
    subsystems::Color detected_color = subsystems::NONE;
    do {
      double color = intake_sensor.get_hue();
      detected_color = classify_color(color);
    } while (detected_color == subsystems::NONE &&
             intake_sensor.get_proximity() < NEAR_THRESHOLD);
#else
    // calculate an average color
    uint32_t color_sum = 0; // color * 100, since is integer and we want to keep
                            // 2 decimal precision
    uint16_t color_count = 0;
    int isGoodFor = 0;

    // attempt to classify the color
    do {
      double color = intake_sensor.get_hue();

      // prevent overflow
      if ((color_sum > UINT32_MAX - color) || (color_count > UINT16_MAX - 1)) {
        continue;
      }

      color_sum += color * 100;
      color_count++;

      pros::delay(10);

      if (intake_sensor.get_proximity() < NEAR_THRESHOLD && (isGoodFor++) > 5) {
        break;
      }

#ifdef INTAKE_DEBUG
      printf("d: %d\n", intake_sensor.get_proximity());
#endif
    } while (true);

    double average_color = (double)color_sum / (100 * color_count);
    printf("Finished getting avg color, proximity is %d\n",
           intake_sensor.get_proximity());
#endif

    printf("classified color: %d\n", detected_color);
    printf("motor position is: %f\n", intake_motor_stg2.get_position());

    // if current team is NONE, ignore
    // otherwise, if differing ring color, eject it
    if (detected_color != subsystems::currentTeam.load() &&
        subsystems::currentTeam.load() != subsystems::NONE) {
      interrupts.push(Interrupt{.type = Interrupt::SORT,
                                .position = intake_motor_stg2.get_position() +
                                            MOVES_TILL_STOP});
    }

// if we're doing rising edge detection, we need to wait until the falling
// edge to prevent
#if INTAKE_DETECT_ON_RISING_EDGE
    while (intake_sensor.get_proximity() > NEAR_THRESHOLD &&
           classify_color(intake_sensor.get_hue()) == detected_color) {
      pros::delay(10);
    }
#endif

    pros::delay(10);
  }
}

pros::Task *intake_task = nullptr;
pros::Task *interrupt_task = nullptr;

void subsystems::initInakeTask() {
  intake_sensor.set_led_pwm(100);
  // create a task that runs the intake loop
  intake_task = new pros::Task(intake_loop, "intake_task");
  interrupt_task = new pros::Task(check_for_interrupts, "interrupt_task");
}
