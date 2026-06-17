#include <Arduino.h>
#include "Janus.h"

// https://forum.arduino.cc/t/sgn-sign-signum-function-suggestions/602445/2
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace Hardware
{

/*void set_pwm_depth(unsigned int d)
{
  pwm_depth = d;
  analogWriteResolution(d);
}*/

void init()
{
  //set_pwm_depth(8);
}

inline bool Component::is_armed()
{
  return armed;
}

bool Component::was_error()
{
  return last_status != StatusCode::OK;
}

bool Component::is_ok()
{
  return !was_error();
}

void Component::set_status(StatusCode s)
{
  last_status = s;
}

StatusCode Component::get_status()
{
  return last_status;
}

inline void Component::clear_status()
{
  set_status(StatusCode::OK);
}

void Component::arm()
{
  armed = true;
  update();
}

void Component::disarm()
{
  armed = false;
  update();
}

void PWMMotor::init()
{
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_direction, OUTPUT);
  pinMode(pin_enable, OUTPUT);

  disarm();
}

unsigned int PWMMotor::get_period()
{
  return period;
}

void PWMMotor::set_period(unsigned int p)
{
  clear_status();

  if (p < 0 || p >= (1 << 8)) { 
    set_status(StatusCode::HardwareInvalidValue);
    return;
  }

  period = p;
  update();
}

void PWMMotor::set_invert(bool inv)
{
  inverted = inv;
}

bool PWMMotor::get_direction()
{
  return direction ^ inverted;
}

void PWMMotor::set_direction(bool dir)
{
  clear_status();

  direction = dir ^ inverted;
  update();
}

void PWMMotor::update()
{
  clear_status();

  if (armed)
  {
    analogWrite(pin_pwm, period);
    digitalWrite(pin_direction, direction);
    digitalWrite(pin_enable, HIGH);
  }
  else
  {
    digitalWrite(pin_enable, LOW);
  }
}

void Servo::init()
{
  pinMode(pin_pwm, OUTPUT);

  disarm();
}

void Servo::update()
{
  clear_status();

  if(armed)
  {
    analogWrite(pin_pwm, period);
  }
  else
  {
    //TODO implement
  }
}

void Servo::set_period(unsigned int p)
{
  clear_status();

  if (p < 0 || p >= (1 << 8))
  {
    set_status(StatusCode::HardwareInvalidValue);
    return;
  }

  period = p;
  update();
}

unsigned int Servo::get_period()
{
  return period;
}

/*
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

*/
} // namespace Hardware

namespace Driver
{

template <typename T>
void Driver<T>::set_status(StatusCode s)
{
  last_status = s;
}

template <typename T>
void Driver<T>::clear_status()
{
  set_status(StatusCode::OK);
}

template <typename T>
void Driver<T>::init()
{
  child->init();
}

template <typename T>
void Driver<T>::set_enable(bool en)
{
  clear_status();

  enable = en;

  if (enable)
  {
    child->arm();
  }
  else
  {
    child->disarm();
  }
}

template <typename T>
StatusCode Driver<T>::get_status()
{
  return last_status;
}

void ESCON50Driver::init()
{
  child->init();
  child->arm();
}

void ESCON50Driver::set_speed(double s)
{
  clear_status();

  if (s > 1.0f || s < -1.0f)
  {
    set_status(StatusCode::DriverInvalidValue);
    return;
  }

  speed = s;

  child->set_period(abs(speed) * 255);
  child->set_direction(sign(speed));
  child->update();
}

/*
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
*/

} // namespace Driver

// hack to make 'pio run' compile the library for testing
#ifdef DEBUG_BUILD
void setup() {}
void loop() {}
#endif

