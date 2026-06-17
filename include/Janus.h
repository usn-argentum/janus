// Janus HAL library
// USN Argentum
//
// This library provides a HAL for controlling different types of actuators. It
// aims to make hardware components easily changeable and tuneable

enum class StatusCode 
{
  OK = 1,
  DriverInvalidValue,
  DriverWarning,
  DriverError,

  HardwareInvalidValue,
  HardwareWarning,
  HardwareError
};

namespace Hardware 
{
  unsigned int pwm_depth;
  void set_pwm_depth(unsigned int d);
  void init();

  class Component
  {
    protected: // not proud of this but bleh
      StatusCode last_status;
      bool armed;
      inline void set_status(StatusCode s);
      inline void clear_status();

    public:
      virtual void init();
      inline bool was_error();
      inline bool is_ok();
      inline StatusCode get_status();
      inline bool is_armed();
      void arm();
      void disarm();
      virtual void update();
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
      unsigned int get_period();
      void set_period(unsigned int p);
      void set_invert(bool inv);
      bool get_direction();
      void set_direction(bool dir);
      void update() override;
  };

  class Steering : public Component
  {
    private:
      size_t pin_pwm;
      unsigned int period = 0;

    public:
      void init();
      void set_period(unsigned int a);
      unsigned int get_period();
  };

}

namespace Driver
{
  class Driver
  {
    private:
      StatusCode last_status;
      bool enable;
      virtual void set_status(StatusCode s);
      Hardware::Component* child;

    public:
      Driver(Hardware::Component* c) : child { c } {};
      virtual void init();
      bool was_error();
      bool is_ok();
      StatusCode get_status();
  };

  class ESCON50Driver: public Driver
  {
    private:
      Hardware::PWMMotor* motor;
      double speed;

    public:
      ESCON50Driver(Hardware::PWMMotor* m) : 
        Driver( m ) {};
      void init() override;
  };

  class SteeringDriver
  {
    private:
      double angle = 0.00;
      double trim_angle = 0.00;
      double turning_speed = 0.00;
      Hardware::Steering* steering;

      double angle_to_steering_value(double a);
      double steering_value_to_angle(double s);
      
    public:
      SteeringDriver() {};
      SteeringDriver(Hardware::Steering* s) : steering{ s } {};
      void init();
      double get_angle();
      void set_angle(double a);
      double set_turning_speed(double s);
  };

}
