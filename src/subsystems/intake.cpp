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
#define MOVES_TILL_STOP 4.1 * 360
#define MOVES_TILL_REDIRECT 3.8 * 360
#define MOVES_TILL_SLOWDOWN 3.2 * 360

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

      // if (next_interrupt.type == Interrupt::SORT) {
      //   // if sort interrupt, stop motor for 10millis
      //   intake_motor_stg2.move(-127);
      //   pros::delay(100);
      // } else {
      //   // if redirect interrupt, move backwards for 250ms
      //   intake_motor_stg2.move(-127);
      //   pros::delay(500);
      // }

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

    // check if NEAR on bottom intake sensor
    if (distance < NEAR_THRESHOLD) {
      pros::delay(10);
      continue;
    }

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

    // hue is a wheel, [0, 360] so we can take advantage of angle functions
    subsystems::Color detected_color =
        fabs(utils::angleError(average_color, COLOR_RED)) < COLOR_THRESHOLD
            ? subsystems::RED
        : fabs(utils::angleError(average_color, COLOR_BLUE)) < COLOR_THRESHOLD
            ? subsystems::BLUE
            : subsystems::NONE;

#ifdef INTAKE_DEBUG
    printf("angle error red: %f\n",
           fabs(utils::angleError(average_color, COLOR_RED)));
    printf("angle error blue: %f\n",
           fabs(utils::angleError(average_color, COLOR_BLUE)));
#endif

    printf("classified color: %d\n", detected_color);
    printf("motor position is: %f\n", intake_motor_stg2.get_position());

    // check team

    // if current team is NONE, ignore
    // otherwise, if differing ring color, eject it
    if (detected_color != subsystems::currentTeam.load() &&
        subsystems::currentTeam.load() != subsystems::NONE) {
      interrupts.push(Interrupt{.type = Interrupt::SORT,
                                .position = intake_motor_stg2.get_position() +
                                            MOVES_TILL_STOP});
    }

    // if redirect mode is on, append redirect and slowdown interrupt
    else if (subsystems::intakeRedirectMode.load() &&
             detected_color == subsystems::currentTeam.load()) {
      interrupts.push(Interrupt{.type = Interrupt::SLOW_DOWN,
                                .position = intake_motor_stg2.get_position() +
                                            MOVES_TILL_SLOWDOWN});

      interrupts.push(Interrupt{.type = Interrupt::REDIRECT,
                                .position = intake_motor_stg2.get_position() +
                                            MOVES_TILL_REDIRECT});
    }

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
