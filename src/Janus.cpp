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
    if (rpm > 1.0f || rpm < -1.0f) { return; } // don't accept invalid values
    target_rpm = rpm;

    float abs_rpm = fabsf(rpm);
    unsigned int period = floor(esc_config->rpm_to_dutycycle(abs_rpm) * pwm_config->max_value());

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

void OpenCRDynamixelBridge::send_control_packet(control_packet p)
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

float OpenCRDynamixelMotor::get_position()
{
    return radians;
}

void OpenCRDynamixelMotor::update_bridge()
{
    dynamixel_state s;
    s.radians = radians;
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
    TS4::begin();
    pinMode(pin_direction, OUTPUT);
    pinMode(pin_enable, OUTPUT);
    pinMode(pin_step, OUTPUT);

    digitalWrite(pin_enable, LOW);

    stepper = new TS4::Stepper(pin_step, pin_direction);
    stepper->setMaxSpeed(5000.0);
    stepper->setAcceleration(12000.0);
}

void StepperMotor::set_position(float radians)
{
    stepper->moveAbsAsync(angle_to_step(radians));
}

float StepperMotor::get_position()
{
    return step_to_angle(stepper->getPosition());
}

void StepperMotor::home(size_t sw_pin)
{
    //stepper->moveRelAsync()
}

#ifdef BUILDING_LOCAL_TEST

StepperMotor s1(1, 0, 2, 6400);
StepperMotor s2(4, 3, 5, 6400);
StepperMotor s3(7, 6, 8, 6400);

// pio compilation fix dont upload this it wont do anything
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(10, INPUT_PULLUP);

    delay(500);
    s1.init();
    s2.init();
    s3.init();

    s1.set_position(M_TWOPI);
    s2.set_position(M_PI);
    s3.set_position(M_PI_2);
}
void loop() {
    unsigned long time = millis();
    static unsigned long last_stepper_update;
    static unsigned long last_led_blink;
    
    if (time - last_led_blink >= 500) {
        last_led_blink = time;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    if (time - last_stepper_update >= 5000) {
        last_stepper_update = time;

        s1.set_position(((time - 5000) / 5000.0f) * M_PI);
        s2.set_position(M_PI);
        s3.set_position(M_PI);
    }
}
#endif


