enum class ReturnCode 
{
  OK = 1,
  DriverWarning,
  DriverError,
  DriverInvalidValue,

  HardwareWarning,
  HardwareError
};

namespace Hardware 
{

  class Motor
  {
    private:
      size_t pin_pwm;
      size_t pin_direction;
      size_t pin_enable;
      unsigned int period;
      char direction;

    public:
      Motor(size_t pwm_pin, size_t dir_pin, size_t en_pin) : pin_pwm{ pwm_pin }, pin_direction{ dir_pin }, pin_enable { en_pin } {};
      void init();
      void set_period(unsigned int s);
      unsigned int get_period();
      void set_direction(char dir);
      char get_direction();
      void disable();
      void commit();
  };

  class Steering
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

  class MotorDriver
  {
    private:
      double speed = 0;
      Hardware::Motor* motor;

    public:
      MotorDriver() {};
      MotorDriver(Hardware::Motor* m) : motor{ m } {};
      void init();
      double get_speed();
      void set_speed(double s);
      double get_position();
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
