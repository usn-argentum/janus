#include <Arduino.h>
#include "Janus.h"
#include <Servo.h>

//#define DEBUG_PRINTS

// https://forum.arduino.cc/t/sgn-sign-signum-function-suggestions/602445/2
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace Hardware
{

unsigned int pwm_depth = 8;
void set_pwm_depth(unsigned int d)
{
  pwm_depth = constrain(d, 8, 15);
  analogWriteResolution(d);
}

void init()
{
  set_pwm_depth(12);
}

/*
void set_pwm_depth(unsigned int d)
{
  pwm_depth = d;
  analogWriteResolution(d);
}

void init()
{
  set_pwm_depth(8);
}
*/
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
#ifdef DEBUG_PRINTS
  Serial.print("Component set status: ");
  Serial.println(static_cast<int>(s));
#endif

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
#ifdef DEBUG_PRINTS
  Serial.println("Component armed");
#endif
  armed = true;
  update();
}

void Component::disarm()
{
#ifdef DEBUG_PRINTS
  Serial.println("Component disarmed");
#endif
  armed = false;
  update();
}

void PWMMotor::init()
{
#ifdef DEBUG_PRINTS
  Serial.println("Motor init");
#endif
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_direction, OUTPUT);
  pinMode(pin_enable, OUTPUT);

  disarm();
  //arm(); // this is dangerous at best
}

unsigned int PWMMotor::get_period()
{
  return period;
}

void PWMMotor::set_period(unsigned int p)
{
  clear_status();

  if (p < 0 || p >= (1 << Hardware::pwm_depth)) { 
#ifdef DEBUG_PRINTS
    Serial.println("Invalid motor pwm period");
#endif 
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
#ifdef DEBUG_PRINTS
  Serial.print("Set motor direction: ");
  Serial.println(dir);
#endif
  clear_status();

  direction = dir ^ inverted;
  update();
}

void PWMMotor::update()
{
  clear_status();

#ifdef DEBUG_PRINTS
  Serial.print("Motor update: ");
#endif
  if (armed)
  {
    //Serial.println(constrain(period, 25, 230));
    analogWrite(pin_pwm, period); // should be constrained here and report error if not in range
    digitalWrite(pin_direction, direction);
    digitalWrite(pin_enable, HIGH);
  }
  else
  {
    digitalWrite(pin_enable, LOW);
  }
}

void CustomServo::init()
{
  s.attach(pin_pwm);

  disarm();
}

void CustomServo::update()
{
  clear_status();

  if(armed)
  {
    //analogWrite(pin_pwm, period);
    unsigned int p = map(period, 0, (1<<Hardware::pwm_depth), prd_lower_bound, prd_upper_bound);
    s.writeMicroseconds(p);
  }
  else
  {
    //TODO implement
  }
}

void CustomServo::set_period(unsigned int p)
{
  clear_status();

  if (p < 0 || p >= (1 << Hardware::pwm_depth))
  {
    set_status(StatusCode::HardwareInvalidValue);
    return;
  }

  period = p;
  update();
}

unsigned int CustomServo::get_period()
{
  return period;
}

void OpenCRSerialDynamixel::send_control_packet(control_packet p)
{
  uint8_t tx_packet[sizeof(p) + 1];
  tx_packet[0] = PKT_CONTROL;
  memcpy(tx_packet + 1, &p, sizeof(p));
  packet_serial.send(tx_packet, sizeof(p) + 1);
}

void OpenCRSerialDynamixel::send_control_packet(dynamixel_state s_1, dynamixel_state s_2)
{
  control_packet p;
  convert_dxl_to_packet(0, s_1, &p);
  convert_dxl_to_packet(1, s_2, &p);
  send_control_packet(p);
}

void OpenCRSerialDynamixel::send_status_packet()
{
  uint8_t tx_packet[1];
  tx_packet[0] = PKT_STATUS;
  packet_serial.send(tx_packet, sizeof(tx_packet));
}

void OpenCRSerialDynamixel::send_arm_packet(bool armed)
{
  uint8_t tx_packet[2];
  tx_packet[0] = PKT_ARM;
  tx_packet[1] = armed;
  packet_serial.send(tx_packet, sizeof(tx_packet));
}

void OpenCRSerialDynamixel::convert_dxl_to_packet(int motor_number, dynamixel_state s, control_packet *p)
{
  p->goal[motor_number] = (s.radians / M_TWOPI) * 4095.0f;
  p->velocity[motor_number] = s.velocity / 0.229f;
  p->acceleration[motor_number] = s.acceleration / 214.577f;
}

void OpenCRSerialDynamixel::on_packet_received(const uint8_t *buffer, size_t size)
{
  status_packet packet;
  memcpy(&packet, buffer, size);
}

void OpenCRSerialDynamixel::transmit_motor_state()
{
  send_control_packet(motor_1, motor_2);
}

void OpenCRSerialDynamixel::arm()
{
  armed = true;
  send_arm_packet(true);
}

void OpenCRSerialDynamixel::disarm()
{
  armed = false;
  send_arm_packet(false);
}

void OpenCRSerialDynamixel::init()
{
  static_cast<HardwareSerial*>(ser_obj)->begin(baudrate);
  packet_serial.setStream(static_cast<HardwareSerial*>(ser_obj));
  packet_serial.setPacketHandler(&on_packet_received);
}

void OpenCRSerialDynamixel::update()
{
  unsigned long t = millis();
  static unsigned long last_status_tick;

  packet_serial.update();

  if (t - last_status_tick >= 100) {
    last_status_tick = t;
    send_status_packet();
  }
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
  //Serial.print("Driver set status: ");
  //Serial.println(static_cast<int>(s));
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
#ifdef DEBUG_PRINTS
  Serial.println("Inited child");
#endif
  child->init();
}

template <typename T>
void Driver<T>::set_enable(bool en)
{
#ifdef DEBUG_PRINTS
  Serial.print("Driver set enable: ");
  Serial.println(en);
#endif
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
  //Serial.println("ESCON init");
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

  unsigned int p = map(abs(speed) * (1 << Hardware::pwm_depth), 0, (1 << Hardware::pwm_depth), lower_period, upper_period);
  child->set_period(p);
  child->set_direction((speed >= 0));
#ifdef DEBUG_PRINTS
  Serial.print("Motor direction: ");
  Serial.println(child->get_direction());
  Serial.println((speed >= 0));
#endif
  //child->update();
}

double ServoDriver::angle_to_steering_value(double deg)
{
  return (deg / 180.00) * ((1 << Hardware::pwm_depth) - 1);
}

void ServoDriver::init()
{
  clear_status();

  child->init();
  child->arm();
}

void ServoDriver::update()
{
  clear_status();
  child->update();
}

double ServoDriver::get_angle()
{
  return angle;
}

void ServoDriver::set_angle(double deg)
{
  clear_status();

  if (deg < 0 || deg > 180)
  {
    set_status(StatusCode::DriverInvalidValue);
    return;
  }

  angle = deg + trim_angle;

  child->set_period(angle_to_steering_value(angle));
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

