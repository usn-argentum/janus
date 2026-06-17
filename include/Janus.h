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
  };
}

namespace Driver
{
  class Driver
  {
    protected:
      StatusCode last_status;
      bool enable;
      virtual void set_status(StatusCode s);
      Hardware::Component* child;

    public:
      Driver(Hardware::Component* c) : child { c } {};
      virtual void init();
      bool was_error();
      bool is_ok();
      void set_enable(bool en);
      StatusCode get_status();
  };

  class ESCON50Driver: public Driver
  {
    private:
        double speed = 0.00;

    public:
      ESCON50Driver(Hardware::PWMMotor* m) : 
        Driver( m ) {};
      void init() override;
      void set_speed(double s);
      void update();
  };

  class ServoDriver : public Driver
  {
    private:
      double angle = 0.00;
      double trim_angle = 0.00;

      double angle_to_steering_value(double a);
      double steering_value_to_angle(double s);
      
    public:
      ServoDriver(Hardware::Servo* s) : Driver( s ) {};
      void init();
      double get_angle();
      void set_angle(double a);
  };

}
