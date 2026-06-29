# Janus HAL
*version 2*

A HAL for quick, easy and safe control of different actuators. (Safety will come later :P)

## Features and documentation

| Actuator                 | Status           | Comment                                                                        |
| ------------------------ | ---------------- | ------------------------------------------------------------------------------ |
| ESCON 50/8 ESC           | Done             | Works using set_speed with custom rpm and pwm ramp settings                    |
| Microservos              | Done             | Uses the servo library under the hood. Controlled with set_angle               |
| Dynamixel OpenCR brigde  | Partial support  | Commands can be sent (set_angle + vel, acc). No telemetry.                     |
| Stepper motors           | Work in progress | Plan: similar interface to servos                                              |
| Gear ratios & scaling    | Incomplete       | Plan: implement more abstract controllers for these purposes                   |
| PID / control algorithms | Incomplete       | Plan: do research. Maybe separate into it's own library (or use a premade one) |

## Basic constructs
### PWMConfig
*Only use one of these objects per project. Currently each object modifies the same hardware.*

Holds PWM configuration. 

| Function                                  | Description                                                                                     |
| ----------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `void set_resolution(unsigned int depth)` | Sets the global PWM resolution. Constrained between 8 - 15 bits.                                |
| `unsigned int get_resolution()`           | Returns the number of bits the PWM is configured to.                                            |
| `unsigned int max_value()`                | Returns (1 << bits) - 1. The maximum value that analogWrite can take at the curernt resolution. |


### TimerConfig
*Not complete. Planned to be mainly used by stepper motors*

Acts as a wrapper for interval timers. 

**TODO**
- Implement and make work without hanging the system.

### Escon50Config
Holds configuration parameters for an ESCON 50 / 8 ESC. Note that the library cannot set these parameters on the ESC. It still has to be programmed manually with the manufacturers software.

### VelocityMotor
Base class for motors that are controlled by setting a speed. In Janus the default unit is RPM. Note that this might be changed to Rad/s for SI compatibility. 

| Function                  | Description                                                                                        |
| ------------------------- | -------------------------------------------------------------------------------------------------- |
| `void init()`             | Inits the motor. Often, specific implementations take pin numbers and other config parameters here. |
| `void set_rpm(float rpm)` | Sets the target RPM.                                                                             |
| `float get_rpm()`        | Returns the target RPM.                                                               |

**TODO**
- Convert to SI units
- Provide clearer function names
  - Allow getting actual measured or calculated speed in addition to the target speed.

### PositionMotor
Base class for motors that are controlled by setting a position. The unit used is radians. 

| Function                           | Description                                                                                         |
| ---------------------------------- | --------------------------------------------------------------------------------------------------- |
| `void init()`                      | Inits the motor. Often, specific implementations take pin numbers and other config parameters here. |
| `void set_position(float radians)` | Sets the target position                                                                            |
| `float get_position()`             | Returns the target position.                                                                        |

**TODO**
- Allow getting actual measured or calculated position in addition to the target.

### OpenCRDynamixelBridge
This is a special class only meant to communicate with the OpenCR to control Dynamixel servos.
Currently it sits somewhere between an abstract class and a driver / implementation and that's not great.

| Function                                               | Description                                                                                                                                         |
| ------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `void init()`                                          | Inits the bridge. Often, specific implementations take pin numbers and other config parameters here.                                                |
| `void update()`                                        | Updates the underlying serial communication. This needs to be called regularly, but not too often as to overload the bus. ~40Hz usually works well. |
| `void send_arm(bool armed)`                            | Sends an arm / disarm packet. This takes a long time so don't use it too regularly and only when the servos are not to be moved for a little while. |
| `void send_motors()`                                   | Sends a motor control packet. It can be called about as often as `update()`                                                                         |
| `void id_set_state(unsigned char id, dynamixel_state d)` | Sets the internal state of the dynamixel to be sent on the next call to `send_motors()` |

### ESCON 50/8
...