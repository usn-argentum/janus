// Janus HAL library
// USN Argentum
//
// This library provides a HAL for controlling different types of actuators. It
// aims to make hardware components easily changeable and tuneable

#pragma once

enum class StatusCode 
{
  OK = 1,
  DriverInvalidValue,
  DriverWarning,
  DriverError,

  HardwareInvalidValue,
  HardwareWarning,
  HardwareError,

  CriticalFailure
};

namespace Hardware 
{
  unsigned int pwm_depth;
  void set_pwm_depth(unsigned int d);
  void init();

  class Component
  {
    protected: // not proud of using protected but bleh
      StatusCode last_status;
      bool armed;

      inline void set_status(StatusCode s);
      inline void clear_status();

    public:
      virtual void init();
      virtual void update();
      
      inline StatusCode get_status();
      inline bool was_error();
      inline bool is_ok();

      void arm();
      void disarm();
      inline bool is_armed();
  };

  class PWMMotor : public Component
  {
    private:
      size_t pin_pwm;
      size_t pin_direction;
      size_t pin_enable;

      unsigned int period;
      bool direction;
      bool inverted;
 
    public:
      PWMMotor(size_t pwm_pin, size_t dir_pin, size_t en_pin) : Component(), pin_pwm{ pwm_pin }, pin_direction{ dir_pin }, pin_enable { en_pin } {};
      void init() override;
      void update() override;

      unsigned int get_period();
      void set_period(unsigned int p);

      void set_invert(bool inv);
      
      bool get_direction();
      void set_direction(bool dir);
  };

  class Servo : public Component
  {
    private:
      size_t pin_pwm;
      unsigned int period = 0;

    public:
      Servo(size_t pwm_pin) : Component(), pin_pwm{ pwm_pin } {};
      void init() override;
      void update() override;

      void set_period(unsigned int p);
      unsigned int get_period();
  };

  class Stepper : public Component
  {
    private:
      size_t pin_a;
      size_t pin_b;
      size_t pin_c;
      size_t pin_d;

      long steps;

    public:
      Stepper(size_t pa, size_t pb, size_t pc, size_t pd) 
        : Component(), pin_a{ pa }, pin_b{ pb }, pin_c{ pc }, pin_d{ pd } {};
      void init() override;
      void update() override;

      void step(int direction);
  };

  class OpenCRSerialDynamixel : public Component
  {
    private:
      Stream* serial_port;

    public:
    OpenCRSerialDynamixel(Stream* serial) : Component(), serial_port{ serial } {};
    void init() override;
    void update() override;


  };
}

namespace Driver
{
  template <typename T>
  class Driver
  {
    protected:
      StatusCode last_status;
      bool enable;
      virtual void set_status(StatusCode s);
      void clear_status();
      T* child;

    public:
      Driver(T* c) : child { c } {};
      virtual void init();
      bool was_error();
      bool is_ok();
      void set_enable(bool en);
      StatusCode get_status();
  };

  class ESCON50Driver: public Driver<Hardware::PWMMotor>
  {
    private:
      double speed = 0.00;
      double lower_pwm_bound = 0.1;
      unsigned int lower_period = Hardware::pwm_depth * lower_pwm_bound;
      double upper_pwm_bound = 0.9;
      unsigned int upper_period = Hardware::pwm_depth * upper_period;

    public:
      ESCON50Driver(Hardware::PWMMotor* m) : 
        Driver<Hardware::PWMMotor>( m ) {
          lower_pwm_bound = 0.1;
          lower_period = Hardware::pwm_depth * lower_pwm_bound;
          upper_pwm_bound = 0.9;
          upper_period = Hardware::pwm_depth * upper_period;
        };
      void init() override;
      void set_speed(double s);
      void update();
  };

  class ServoDriver : public Driver<Hardware::Servo>
  {
    private:
      double angle = 0.00;
      double trim_angle = 0.00;

      double angle_to_steering_value(double deg);
      double steering_value_to_angle(double steer);
      
    public:
      ServoDriver(Hardware::Servo* s) : Driver<Hardware::Servo>( s ) {};
      void init();
      double get_angle();
      void set_angle(double deg);
  };

  class DynamixelDriver : public Driver<Hardware::OpenCRSerialDynamixel>
  {
    private:
      double angle = 0.00;
      double offset_angle = 0.00;

    public:
      DynamixelDriver(Hardware::OpenCRSerialDynamixel* d) : Driver<Hardware::OpenCRSerialDynamixel>( d ) {};
      void init();
      void update();
      void get_angle();
      void set_angle();
  };

}
