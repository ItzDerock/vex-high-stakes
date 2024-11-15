#pragma once

class PIDController {
public:
  /**
   * Creates a new PIDController with the given gains and integral max
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param Imax max integral value to prevent windup
   * @param debug if true, prints debug info
   */
  PIDController(double kP, double kI, double kD);
  PIDController(double kP, double kI, double kD, double Imax);
  PIDController(double kP, double kI, double kD, double Imax, bool debug);
  double update(double error);
  void reset();

private:
  bool debug;
  double _kP, _kI, _kD;
  double _Imax = 0;
  double _previousError = 0;
  double _integral = 0;
};
