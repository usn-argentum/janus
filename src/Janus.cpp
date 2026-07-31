#include <Arduino.h>
#include "Janus.h"

void PWMConfig::set_resolution(unsigned int depth)
{
    bit_depth = constrain(static_cast<int>(depth), 8, 15);
    analogWriteResolution(bit_depth);
}

unsigned int PWMConfig::get_resolution()
{
    return bit_depth;
}

unsigned int PWMConfig::max_value()
{
    return (1 << bit_depth) - 1;
}

unsigned long TimerConfig::freqency_to_period_us(float freq)
{
    return (1 / freq) * 1'000'000;
}

void TimerConfig::init(float freq, void (*cb)())
{
    frequency = freq;
    interrupt_callback = cb;
    tmr->begin(interrupt_callback, 1000000); // initially start at 1ms 
}

void TimerConfig::start()
{
    tmr->begin(interrupt_callback, freqency_to_period_us(frequency));
}

void TimerConfig::stop()
{
    tmr->end();
}

void TimerConfig::set_frequency_hz(float freq)
{
    frequency = freq;
    tmr->update(freqency_to_period_us(frequency));
}

float Escon50Config::rpm_to_dutycycle(float rpm)
{
    float rpm_norm = (rpm - rpm_ramp_low) / (rpm_ramp_high - rpm_ramp_low);
    return pwm_ramp_low + rpm_norm * (pwm_ramp_high - pwm_ramp_low);
}

float Escon50Config::max_rpm()
{
    return rpm_ramp_high;
}

void EsconPWMMotor::init()
{
    pinMode(pin_pwm, OUTPUT);
    pinMode(pin_enable, OUTPUT);
    pinMode(pin_direction, OUTPUT);

    //set_enable(false);
}

void EsconPWMMotor::set_rpm(float rpm)
{
    if (rpm < -1000.0f || rpm > 1000.0f) { return; }  // don't accept invalid values

    target_rpm = rpm;

    float abs_rpm = fabsf(rpm);
    float duty_cycle = esc_config->rpm_to_dutycycle(abs_rpm);
    if (duty_cycle > 0.9f || duty_cycle < 0.1f) { return; } // don't accept invalid values

    unsigned int period = floor(duty_cycle * pwm_config->max_value());
    analogWrite(pin_pwm, constrain(period, pwm_low, pwm_high));
    digitalWrite(pin_direction, rpm < 0);
}

float EsconPWMMotor::get_rpm()
{
    return target_rpm;
}

void EsconPWMMotor::set_enable(bool enabled)
{
    digitalWrite(pin_enable, enabled ? HIGH : LOW);
}

void ServoMotor::init()
{
    pinMode(pin_pwm, OUTPUT);
    s.attach(pin_pwm);
}

void ServoMotor::set_position(float radians)
{
    angle = constrain(radians, 0.0f, M_PI); // constrain 0 - 180 degrees
    float servo_angle = angle * (180.0f / M_PI);
    s.write(servo_angle);
}

float ServoMotor::get_position()
{
    return angle;
}

/*void OpenCRDynamixelBridge::send_control_packet(control_packet p)
{
    uint8_t tx_packet[sizeof(p) + 1];
    tx_packet[0] = PKT_CONTROL;
    memcpy(tx_packet + 1, &p, sizeof(p));
    packet_serial.send(tx_packet, sizeof(p) + 1);
}

void OpenCRDynamixelBridge::send_control_packet(dynamixel_state s_1, dynamixel_state s_2)
{
    control_packet p;
    convert_dxl_to_packet(0, s_1, &p);
    convert_dxl_to_packet(1, s_2, &p);
    send_control_packet(p);
}

void OpenCRDynamixelBridge::send_status_packet()
{
    uint8_t tx_packet[1];
    tx_packet[0] = PKT_STATUS;
    packet_serial.send(tx_packet, sizeof(tx_packet));
}

void OpenCRDynamixelBridge::send_arm_packet(bool armed)
{
    uint8_t tx_packet[2];
    tx_packet[0] = PKT_ARM;
    tx_packet[1] = armed;
    packet_serial.send(tx_packet, sizeof(tx_packet));
}

void OpenCRDynamixelBridge::convert_dxl_to_packet(int motor_number, dynamixel_state s, control_packet *p)
{
    p->goal[motor_number] = (s.radians / M_TWOPI) * 4095.0f;
    p->velocity[motor_number] = s.velocity / 0.229f;
    p->acceleration[motor_number] = s.acceleration / 214.577f;
}

void OpenCRDynamixelBridge::on_packet_received(const uint8_t *buffer, size_t size)
{
    status_packet packet;
    memcpy(&packet, buffer, size);
}

void OpenCRDynamixelBridge::init()
{
    static_cast<HardwareSerial*>(serial)->begin(baudrate);
    packet_serial.setStream(serial);
    packet_serial.setPacketHandler(on_packet_received);
}

void OpenCRDynamixelBridge::update()
{
    unsigned long time = millis();
    static unsigned long last_status_packet;
    
    packet_serial.update();

    if (time - last_status_packet >= 1'000) {
        last_status_packet = time;
        send_status_packet();
    }
}

void OpenCRDynamixelBridge::send_arm(bool armed)
{
    send_arm_packet(armed);
}

void OpenCRDynamixelBridge::send_motors()
{
    send_control_packet(motor_states);
}

void OpenCRDynamixelBridge::id_set_state(unsigned char id, dynamixel_state d)
{
    convert_dxl_to_packet(id, d, &motor_states);
}

void OpenCRDynamixelMotor::init()
{
}

void OpenCRDynamixelMotor::set_position(float rad)
{
    radians = rad;
}

void OpenCRDynamixelMotor::set_offset(float o)
{
    offset = o;
}

float OpenCRDynamixelMotor::get_position()
{
    return radians;
}

void OpenCRDynamixelMotor::update_bridge()
{
    dynamixel_state s;
    s.radians = radians + offset;
    s.velocity = velocity;
    s.acceleration = acceleration;
    bridge->id_set_state(id, s);
}

long StepperMotor::angle_to_step(float radians)
{
    return (radians / M_TWOPI) * steps_per_revolution;
}

float StepperMotor::step_to_angle(long step)
{
    return (step / steps_per_revolution) * M_TWOPI;
}

void StepperMotor::init()
{
    pinMode(pin_direction, OUTPUT);
    pinMode(pin_enable, OUTPUT);
    pinMode(pin_step, OUTPUT);

    digitalWrite(pin_enable, LOW);
    
    stepper = new TS4::Stepper(pin_step, pin_direction);
    stepper->setMaxSpeed(1000.0);
    stepper->setAcceleration(2000.0);

    pid_controller = new PID(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, PID::Direct);
    pid_controller->SetOutputLimits(-3000, 3000);
    pid_controller->SetMode(PID::Automatic);
    pid_controller->SetSampleTime(5);

    //int32_t current_position = stepper->getPosition();
    //stepper->moveAbsAsync(0);
}

void StepperMotor::set_position(float radians)
{
    steps_target = angle_to_step(radians);
    pid_setpoint = (double)steps_target;
    //pid_controller->Setpoint(steps_target);
    stepper->moveAbsAsync(angle_to_step(radians));
}

float StepperMotor::get_position()
{
    return step_to_angle(stepper->getPosition());
}

int32_t StepperMotor::update()
{
    pid_input = (double)stepper->getPosition();
    if (pid_controller->Compute()) {
        int32_t dynamic_target = steps_target + (int32_t)pid_output;
        stepper->setTargetAbs(dynamic_target);
        return steps_target + (int32_t)pid_output;
    }
    pid_input = (double)stepper->getPosition();
    pid_controller->Compute();
    
    // Return calculated absolute coordinate target
    return steps_target + (int32_t)pid_output;
}

void StepperMotor::home(size_t sw_pin)
{
    //stepper->moveRelAsync()
}*/

// controls the stepper drivers. called at 200kHz
void FASTRUN global_stepper_isr() {
    for (uint32_t i = 0; i < N_STEPPERS; i++) {
        StepperMotor* s = janus_stepper.motors[i];

        if ((s->steps + s->offset *0) == s->goal) { continue; }

        s->accumulator++;
        if (s->accumulator >= s->acc_goal) {
            s->accumulator = 0;

            if ((s->steps + s->offset*0) < s->goal) {
                digitalWriteFast(s->direction_pin, HIGH);
                s->steps++;
            }
            else {
                digitalWriteFast(s->direction_pin, LOW);
                s->steps--;
            }

            // pulse pin until next interrupt
            digitalWriteFast(s->pulse_pin, HIGH);
        }
        else {
            digitalWriteFast(s->pulse_pin, LOW);
        }
    }
}

void StepperManager::init()
{
    stepper_timer.begin(global_stepper_isr, 5.0);
}

void StepperManager::homing_sequence(StepperMotor& motor, size_t signal_pin, int tickspeed, float home_angle)
{
    uint32_t previous_acc_goal = motor.acc_goal;

    motor.steps = 0;
    motor.acc_goal = abs(tickspeed);

    motor.goal = (tickspeed > 0) ? INT32_MAX : INT32_MIN;
    while(!digitalRead(signal_pin)) {}

    motor.goal = (tickspeed > 0) ? INT32_MIN : INT32_MAX;
    motor.acc_goal = abs(tickspeed) * 4;
    while(digitalRead(signal_pin)) {}

    motor.offset = -motor.steps + home_angle * motor.angle_to_steps;
    motor.steps = home_angle * motor.angle_to_steps;
    motor.goal = motor.steps;
    motor.acc_goal = previous_acc_goal;
}

void StepperMotor::init()
{
    pinMode(enable_pin, OUTPUT);
    pinMode(direction_pin, OUTPUT);
    pinMode(pulse_pin, OUTPUT); 

    digitalWrite(enable_pin, LOW); // disable at start
}

void StepperMotor::set_position(float radians)
{
    goal = radians * angle_to_steps + offset;
}

float StepperMotor::get_position()
{
    return 0.0f;
}

int32_t StepperMotor::get_rawsteps()
{
    return steps;
}

int32_t StepperMotor::get_goalsteps()
{
    return goal;
}

float* DynamixelServo::get_position_ptr()
{
    return &angle;
}

void DynamixelManager::init()
{
    packet_serial.setStream(serial);
    if (callback != nullptr) {
        packet_serial.setPacketHandler(callback);
    }
}

#ifdef BUILDING_LOCAL_TEST

DynamixelServo left(0.0f);
DynamixelServo right(0.0f);
DynamixelManager steering_manager(&Serial1, 115200);
DynamixelSteering steering(&steering_manager, &left, &right);

DynamixelServo claw_dxl(0.0f);
DynamixelManager claw_manager(&Serial2, 115200);
DynamixelClaw claw(&claw_manager, &claw_dxl);

StepperMotor joint_a(18, 3200 / M_PI, 11, 10, 9);

unsigned long t_start;
void setup() {
    Serial.begin(115200);

    janus_stepper.motors[0] = &joint_a;
    joint_a.init();

    janus_stepper.init();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(8, OUTPUT);

    Serial1.begin(115200);
    steering.set_angles(0.0f, 0.0f);
    Serial2.begin(115200);
    claw.set_angle(0.0f);

    pinMode(32, INPUT_PULLDOWN);
    janus_stepper.homing_sequence(joint_a, 32, -200, 1.0f);
    t_start = millis();
}

void loop() {
    float p = ((millis() - t_start) % 10000) / 10000.0f;

    joint_a.set_position(sin(p * M_TWOPI) * M_TWOPI);
    
    steering.set_angles(sin(p * M_TWOPI) * 0.0f, sin(p * M_TWOPI) * 0.0f);
    steering.update();

    claw.set_angle(sin(p * M_TWOPI * 2) * 0.5f);
    claw.update();

    static unsigned long last_print;
    if ((millis() - t_start) - last_print >= 1000) {
        last_print = millis() - t_start;
        Serial.print(joint_a.get_rawsteps());
        Serial.print("\t");
        Serial.println(joint_a.get_goalsteps());
    }
}
#endif
