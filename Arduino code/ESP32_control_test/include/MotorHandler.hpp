#ifndef MOTOR_HANDLER_HPP
#define MOTOR_HANDLER_HPP

#include "SystemConfig.hpp"
#include "driver/pcnt.h"

class MotorHandler {
   private:
    pcnt_unit_t pcnt_unit = PCNT_UNIT_0;
    volatile long targetPosition;
    bool isHoming;
    int lastSpeed = -1;
    int lastDirection = 0;

    void setMotorDrive(int direction, int speed) {
        if (speed == 0) {
            stopMotor();
            return;
        }

        // Only update if state has changed
        if (speed != lastSpeed || direction != lastDirection) {
            digitalWrite(SLEEP_PIN, HIGH);  // Wake up

            if (direction > 0) {
                // Forward
                analogWrite(M1_PIN, speed);
                analogWrite(M2_PIN, 0);
                // Serial.println("{\"motor\":\"forward\"}");
            } else {
                // Reverse
                analogWrite(M1_PIN, 0);
                analogWrite(M2_PIN, speed);
                // Serial.println("{\"motor\":\"reverse\"}");
            }

            lastSpeed = speed;
            lastDirection = direction;
        }
    }

    long distanceToEncoderCounts(float distanceMM) {
        return (long)((distanceMM - DISTANCE_OFFSET) / DISTANCE_SLOPE);
    }

   public:
    MotorHandler() : targetPosition(0), isHoming(false) {}

    void setup() {
        pinMode(M1_PIN, OUTPUT);
        pinMode(M2_PIN, OUTPUT);

        // Setup Sleep Pin
        pinMode(SLEEP_PIN, OUTPUT);
        digitalWrite(SLEEP_PIN, HIGH);  // Enable Driver

        pinMode(BUTTON_HOME_PIN, INPUT_PULLUP);
        pinMode(BUTTON_CONTROL_PIN, INPUT_PULLUP);

        stopMotor();

        // Standard Quadrature Decoder configuration for Legacy PCNT
        // Unit 0, Channel 0
        pcnt_config_t pcnt_config_a = {};
        pcnt_config_a.pulse_gpio_num = ENC_A_PIN;
        pcnt_config_a.ctrl_gpio_num = ENC_B_PIN;
        pcnt_config_a.channel = PCNT_CHANNEL_0;
        pcnt_config_a.unit = pcnt_unit;
        pcnt_config_a.pos_mode = PCNT_COUNT_INC;       // Count up on rising edge
        pcnt_config_a.neg_mode = PCNT_COUNT_DEC;       // Count down on falling edge
        pcnt_config_a.lctrl_mode = PCNT_MODE_REVERSE;  // Reverse counting direction if low
        pcnt_config_a.hctrl_mode = PCNT_MODE_KEEP;     // Keep counting direction if high
        pcnt_config_a.counter_h_lim = 32000;
        pcnt_config_a.counter_l_lim = -32000;
        pcnt_unit_config(&pcnt_config_a);

        // Unit 0, Channel 1 - for 4x resolution (counting edges of B as well)
        pcnt_config_t pcnt_config_b = {};
        pcnt_config_b.pulse_gpio_num = ENC_B_PIN;
        pcnt_config_b.ctrl_gpio_num = ENC_A_PIN;
        pcnt_config_b.channel = PCNT_CHANNEL_1;
        pcnt_config_b.unit = pcnt_unit;
        pcnt_config_b.pos_mode = PCNT_COUNT_INC;
        pcnt_config_b.neg_mode = PCNT_COUNT_DEC;
        pcnt_config_b.lctrl_mode = PCNT_MODE_KEEP;
        pcnt_config_b.hctrl_mode = PCNT_MODE_REVERSE;
        pcnt_config_b.counter_h_lim = 32000;
        pcnt_config_b.counter_l_lim = -32000;
        pcnt_unit_config(&pcnt_config_b);

        // Filter
        pcnt_set_filter_value(pcnt_unit, 100);
        pcnt_filter_enable(pcnt_unit);

        pcnt_counter_pause(pcnt_unit);
        pcnt_counter_clear(pcnt_unit);
        pcnt_counter_resume(pcnt_unit);
    }

    void stopMotor() {
        analogWrite(M1_PIN, 0);
        analogWrite(M2_PIN, 0);
        digitalWrite(SLEEP_PIN, LOW);  // Sleep
        lastSpeed = 0;
        lastDirection = 0;
    }

    void update() {
        // Homing Stop Condition
        if (digitalRead(BUTTON_HOME_PIN) == LOW) {
            pcnt_counter_clear(pcnt_unit);
            if (isHoming) {
                stopMotor();
                targetPosition = 0;
                isHoming = false;
                return;
            }
        }

        long currentPos = getPosition();
        long error = targetPosition - currentPos;
        long absError = abs(error);

        if (absError <= STOP_TOLERANCE) {
            stopMotor();
            return;
        }

        int direction = (error > 0) ? 1 : -1;
        int speed = FAST_SPEED;

        if (absError <= CREEP_DISTANCE)
            speed = CREEP_SPEED;
        else if (absError <= SLOW_DISTANCE)
            speed = SLOW_SPEED;

        setMotorDrive(direction, speed);
    }

    void setTargetDistance(float distanceMM) {
        if (distanceMM < MIN_DISTANCE_MM) distanceMM = MIN_DISTANCE_MM;
        if (distanceMM > MAX_DISTANCE_MM) distanceMM = MAX_DISTANCE_MM;
        setTargetPosition(distanceToEncoderCounts(distanceMM));
    }

    void setTargetPosition(long target) {
        if (!isHoming) {
            if (target < MIN_POSITION)
                target = MIN_POSITION;
            else if (target > MAX_POSITION)
                target = MAX_POSITION;
        }
        targetPosition = target;
    }

    long getPosition() {
        int16_t count = 0;
        pcnt_get_counter_value(pcnt_unit, &count);
        return (long)count;
    }
    long getTarget() const { return targetPosition; }

    float getDistanceMM() { return (getPosition() * DISTANCE_SLOPE) + DISTANCE_OFFSET; }

    void home() {
        isHoming = true;
        // Move in reverse direction indefinitely (until switch)
        // using a target far beyond reachable bounds
        targetPosition = -100000;
    }
};

#endif  // MOTOR_HANDLER_HPP
