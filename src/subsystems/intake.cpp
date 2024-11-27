#include "../config.hpp"
#include "main.h"
#include "robot/subsystems.hpp"
#include "robot/utils.hpp"
#include <atomic>
#include <cstdint>
#include <queue>

#define NEAR_THRESHOLD 240
#define COLOR_RED 0
#define COLOR_BLUE 229
#define COLOR_THRESHOLD 20
#define MOVES_TILL_STOP 1.65 * 360

// priority queue sorted by rotations
std::priority_queue<double> interrupts;
std::atomic<bool> subsystems::lock_intake_controls(false);
std::atomic<subsystems::Color> subsystems::current_team(NONE);

void check_for_interrupts() {
  while (true) {
    while (!interrupts.empty()) {
      double next_interrupt = interrupts.top();
      double motor_position = intake_motor_stg2.get_position();

      // printf("next interrupt: %f\n", next_interrupt);
      // printf("current motor position: %f\n", motor_position);

      if (next_interrupt > motor_position) {
        break;
      }

      // stop motor for 10millis
      subsystems::lock_intake_controls.store(true);
      double current_velocity = intake_motor_stg2.get_target_velocity();
      intake_motor_stg2.move(-127);
      pros::delay(100);
      intake_motor_stg2.move_velocity(current_velocity);
      interrupts.pop();
      subsystems::lock_intake_controls.store(false);
    }

    pros::delay(5);
  }
}

void intake_loop() {
  while (true) {
    uint32_t distance = intake_sensor.get_proximity();

    // check if NEAR on bottom intake sensor
    if (distance < NEAR_THRESHOLD) {
      pros::delay(10);
      continue;
    }

    // calculate an average color
    uint32_t color_sum = 0; // color * 100, since is integer and we want to keep
                            // 2 decimal precision
    uint16_t color_count = 0;

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
    } while (intake_sensor.get_proximity() > NEAR_THRESHOLD);

    double average_color = (double)color_sum / (100 * color_count);
    // printf("detected color: %f\n", average_color);
    // printf("broke cus proximity is %d\n", intake_sensor.get_proximity());

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
    printf("classified color: %d\n", detected_color);
#endif

    if (detected_color != subsystems::current_team) {
      interrupts.push(intake_motor_stg2.get_position() + MOVES_TILL_STOP);
      printf("publishing interrupt\n");
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
