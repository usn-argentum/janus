// Janus HAL library
// USN Argentum
//
// This library provides a HAL for controlling different types of actuators. It
// aims to make hardware components easily changeable and tuneable

#pragma once

#include <Arduino.h>
#include <IntervalTimer.h>
#include <PacketSerial.h>
#include <Servo.h>

#ifdef BUILDING_LOCAL_TEST
    #define N_STEPPERS 1
#endif

struct dynamixel_state {
    float radians;
    float velocity;
    float acceleration;
};

class PWMConfig {
    private:
        unsigned int bit_depth = 8;

    public:
        void set_resolution(unsigned int depth);
        unsigned int get_resolution();
        unsigned int max_value();
};

class TimerConfig {
    private:
        IntervalTimer* tmr;
        float frequency;
        void (*interrupt_callback)();
        unsigned long freqency_to_period_us(float freq);

    public:
        void init(float freq, void (*cb)());    
        void start();
        void stop();
        void set_frequency_hz(float freq);
};

class Escon50Config {
    private:
        float rpm_ramp_low = 0.0f;
        float rpm_ramp_high = 500.0f;
        float pwm_ramp_low = 0.1;
        float pwm_ramp_high = 0.9;
    
    public:
        Escon50Config(float low_rpm, float high_rpm, float low_pwm, float high_pwm) :
            rpm_ramp_low{ low_rpm }, rpm_ramp_high{ high_rpm }, pwm_ramp_low{ low_pwm }, pwm_ramp_high{ high_pwm } {};
        float rpm_to_dutycycle(float rpm);
        float max_rpm();
};

class VelocityMotor {
    public:
        virtual ~VelocityMotor() = default;
        virtual void init() = 0;
        virtual void set_rpm(float rpm) = 0;
        virtual float get_rpm();
};

class EsconPWMMotor : public VelocityMotor {
    private:
        unsigned int pin_direction;
        unsigned int pin_enable;
        unsigned int pin_pwm;
        Escon50Config* esc_config;
        PWMConfig* pwm_config;
        float target_rpm;
        unsigned int pwm_low = 102;
        unsigned int pwm_high = 920;

    public:
        EsconPWMMotor(unsigned int p_dir, unsigned int p_ena, unsigned int p_pwm, Escon50Config* esc_conf, PWMConfig* pwm_conf) :
            pin_direction{ p_dir }, pin_enable{ p_ena }, pin_pwm{ p_pwm }, esc_config{ esc_conf }, pwm_config{ pwm_conf } {};
        
        void init() override;
        void set_rpm(float rpm) override;
        float get_rpm() override;
        void set_enable(bool enabled);
};

class PositionMotor {
    public:
        virtual ~PositionMotor() = default;
        virtual void init() = 0;
        virtual void set_position(float radians) = 0;
        virtual float get_position() = 0;
};

class ServoMotor : public PositionMotor{
    private:
        float angle;
        unsigned int pin_pwm;
        PWMConfig* pwm_config;
        Servo s;

    public:
        ServoMotor(unsigned int p_pwm, PWMConfig* pwm_cfg) : pin_pwm{ p_pwm }, pwm_config{ pwm_cfg } {};
        void init() override;
        void set_position(float radians) override;
        float get_position() override;
};

/*class StepperMotor : public PositionMotor {
    private:
        unsigned int pin_direction;
        unsigned int pin_enable;
        unsigned int pin_step;
        TimerConfig* tmr_config;

        TS4::Stepper* stepper;
        int steps_per_revolution = 3200;

        double pid_input;
        double pid_output;
        double pid_setpoint;

        double Kp = 2;
        double Ki = 5;
        double Kd = 1;
        PID* pid_controller;

        int32_t steps_target;

        long angle_to_step(float radians);
        float step_to_angle(long step);
    
    public:
        StepperMotor(unsigned int p_dir, unsigned int p_en, unsigned int p_step, int per_rev = 3200) : pin_direction{ p_dir }, pin_enable{ p_en }, pin_step{ p_step }, steps_per_revolution{ per_rev } {};
        void init() override;
        void set_position(float radians) override;
        float get_position() override;  
        void home(size_t sw_pin);
        int32_t update();
        TS4::Stepper& get_stepper() { return *stepper; }
};

class OpenCRDynamixelBridge {
    private:
        struct control_packet {
            uint32_t goal[2];
            uint32_t velocity[2];
            uint32_t acceleration[2];
        };

        struct status_packet {
            int32_t current_position[2];
            int32_t current_velocity[2];
            int32_t current[2];
            int32_t input_voltage[2];
            int32_t temperature[2];
            int32_t moving[2];
        };

        enum PacketTypes {
            PKT_CONTROL = 0xff,
            PKT_ARM = 0x80,
            PKT_STATUS = 0x00
        };

        Stream* serial;
        unsigned long baudrate;
        PacketSerial packet_serial;
        control_packet motor_states;

        void send_control_packet(control_packet p);
        void send_control_packet(dynamixel_state s_1, dynamixel_state s_2);
        void send_status_packet();
        void send_arm_packet(bool armed);
        void convert_dxl_to_packet(int motor_number, dynamixel_state s, control_packet* p);
        static void on_packet_received(const uint8_t* buffer, size_t size);
    
    public:
        OpenCRDynamixelBridge(Stream* ser, unsigned long baudrate) :
            serial{ ser }, baudrate{ baudrate } {};
        void init();
        void update();
        void send_arm(bool armed);
        void send_motors();
        void id_set_state(unsigned char id, dynamixel_state d);
};

class OpenCRDynamixelMotor : public PositionMotor {
    private:
        unsigned char id;
        float radians;
        float offset;
        float velocity;
        float acceleration;
        OpenCRDynamixelBridge* bridge;
    
    public:
        OpenCRDynamixelMotor(unsigned char motor_id, float velocity, float acceleration, OpenCRDynamixelBridge* opencr_bridge) :
            id{ motor_id }, velocity{ velocity }, acceleration{ acceleration }, bridge{ opencr_bridge } {};

        void init() override;
        void set_position(float rad) override;
        float get_position() override;
        void set_offset(float o);
        void update_bridge();
};*/

class StepperMotor;

class StepperManager {
    private:
        IntervalTimer stepper_timer;

    public:
        StepperMotor* motors[N_STEPPERS];
        void init();
        void homing_sequence(StepperMotor& motor, size_t signal_pin, int tickspeed, float end_angle);
};

class StepperMotor : public PositionMotor {
    private:        
        volatile int32_t steps = 0;
        volatile int32_t offset = 0;
        volatile int32_t goal = 0;
        volatile uint32_t accumulator = 0;
        volatile uint32_t acc_goal = 0;
        float angle_to_steps = 3200 / M_TWOPI;
        size_t enable_pin;
        size_t direction_pin;
        size_t pulse_pin;
    
    public:
        friend void FASTRUN global_stepper_isr();
        friend void StepperManager::homing_sequence(StepperMotor& motor, size_t signal_pin, int tickspeed, float end_angle);
        StepperMotor(uint32_t isr_ticks, float angle_to_steps, size_t p_en, size_t p_dir, size_t p_pulse) : acc_goal{ isr_ticks }, angle_to_steps{ angle_to_steps }, enable_pin{ p_en }, direction_pin{ p_dir }, pulse_pin{ p_pulse } {};
        void init() override;
        void set_position(float radians) override;
        float get_position() override;
        int32_t get_rawsteps();
        int32_t get_goalsteps();
};

// From Dynamixel-Bridge
struct __attribute__((packed)) ClawPacket {
  uint32_t packet_num;
  bool armed;
  float openness; // 0 - 1 -> how open is the claw. TODO: replace with measured distances
};

struct __attribute__((packed)) SteeringPacket {
  uint32_t packet_num;
  bool armed;
  float left_angle;
  float right_angle;
};

class DynamixelServo : public PositionMotor {
    private:
        float angle;
        float offset;

    public:
        DynamixelServo(float offset) : offset{ offset } {}
        void init() override {};
        void set_position(float radians) override {
            angle = radians;
        };
        float get_position() override {
            return angle;
        };
        float* get_position_ptr();
};

class DynamixelManager {
    private:
        Stream* serial;
        PacketSerial packet_serial;
        unsigned int baudrate;
        void (*callback)(const uint8_t*, size_t) = nullptr;

    public:
        DynamixelManager(Stream* stream, unsigned int baud) : serial{ stream }, baudrate{ baud } {
            packet_serial.setStream(stream);
        };
        void init();
        void set_callback(void (*cb)(const uint8_t*, size_t)) { callback = cb; }
        void update() {
            packet_serial.update();
        }
        void send(const uint8_t* buf, size_t size) {
            packet_serial.send(buf, size);
        }
};

extern StepperManager janus_stepper;

class DynamixelSteering {
    private:
        DynamixelManager* manager;
        SteeringPacket state;
        DynamixelServo* left;
        DynamixelServo* right;
        unsigned int tx_period = 20;

        static void packet_callback(const uint8_t*, size_t) {};

    public:
        DynamixelSteering(DynamixelManager* man, DynamixelServo* left, DynamixelServo* right) : manager{ man }, left{ left }, right{ right } {
            manager->set_callback(packet_callback);
        };

        void set_angles(float left_angle, float right_angle) {
            left->set_position(left_angle);
            right->set_position(right_angle);
        }

        void update() {
            static unsigned long last_tx;
            manager->update();

            if (millis() - last_tx >= tx_period) {
                last_tx = millis();

                state.left_angle = left->get_position();
                state.right_angle = right->get_position();
                state.packet_num++;
                manager->send((uint8_t*)&state, sizeof(state));
            }
        }
};

class DynamixelClaw {
    private:
        DynamixelManager* manager;
        ClawPacket state;
        DynamixelServo* claw;
        unsigned int tx_period = 20;

        static void packet_callback(const uint8_t*, size_t) {};

    public:
        DynamixelClaw(DynamixelManager* man, DynamixelServo* claw) : manager{ man }, claw{ claw } {
            manager->set_callback(packet_callback);
        };

        void set_angle(float claw_angle) {
            claw->set_position(claw_angle);
        }

        void update() {
            static unsigned long last_tx;
            manager->update();

            if (millis() - last_tx >= tx_period) {
                last_tx = millis();

                state.openness = claw->get_position();
                state.packet_num++;
                manager->send((uint8_t*)&state, sizeof(state));
            }
        }
};
