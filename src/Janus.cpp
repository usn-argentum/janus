#include <Arduino.h>
#include "Janus.h"

// https://forum.arduino.cc/t/sgn-sign-signum-function-suggestions/602445/2
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace Hardware
{

void Motor::init()
{
  pinMode(pin_direction, OUTPUT);
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_enable, OUTPUT);

  digitalWrite(pin_enable, HIGH);
}

void Motor::set_period(unsigned int s)
{
  period = s;
}

unsigned int Motor::get_period()
{
  return period;
}

void Motor::set_direction(char dir)
{
  direction = dir;
}

char Motor::get_direction()
{
  return direction;
}

void Motor::disable()
{
  digitalWrite(pin_enable, LOW);
}

void Motor::commit()
{
  digitalWrite(pin_direction, direction > 0);
  analogWrite(pin_pwm, period);
}

void Steering::init()
{
  pinMode(pin_pwm, OUTPUT);
}

void Steering::set_period(unsigned int p)
{
  period = p;
}

unsigned int Steering::get_period()
{
  return period;
}

} // namespace Hardware

namespace Driver
{

void MotorDriver::init()
{
  motor->init();
}

double MotorDriver::get_speed()
{
  return speed;
}

void MotorDriver::set_speed(double s)
{
  if (speed < -1 || speed > 1) { return; }
  speed = s;

  motor->set_direction(sign(speed));
  // assumes pwm signaling is "linear"; 0% = 0 speed, 100% = max speed
  motor->set_period(abs(speed) * 255.0);
  motor->commit();
}

void SteeringDriver::init()
{
  steering->init();
}

double SteeringDriver::get_angle()
{
  return angle;
}

void SteeringDriver::set_angle(double a)
{

}

} // namespace Driver

// hack to make 'pio run' compile the library for testing
#ifdef DEBUG_BUILD
void setup() {}
void loop() {}
#endif

