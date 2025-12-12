#ifndef MOTOR_HANDLER_HPP
#define MOTOR_HANDLER_HPP

#include <Encoder.h>
#include "SystemConfig.hpp"

class MotorHandler {
   private:
    Encoder myEncoder;
    volatile long targetPosition;
    bool isHoming;
    int lastSpeed = -1;
    int lastDirection = 0;
    long lastError = 0;
    unsigned long lastTime = 0;

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
            } else {
                // Reverse
                analogWrite(M1_PIN, 0);
                analogWrite(M2_PIN, speed);
            }

            lastSpeed = speed;
            lastDirection = direction;
        }
    }

    long distanceToEncoderCounts(float distanceMM) {
        return (long)((distanceMM - DISTANCE_OFFSET) / DISTANCE_SLOPE);
    }

   public:
    MotorHandler() : myEncoder(ENC_A_PIN, ENC_B_PIN), targetPosition(0), isHoming(false) {}

    void setup() {
        pinMode(M1_PIN, OUTPUT);
        pinMode(M2_PIN, OUTPUT);

        // Setup Sleep Pin
        pinMode(SLEEP_PIN, OUTPUT);
        digitalWrite(SLEEP_PIN, HIGH);  // Enable Driver

        pinMode(BUTTON_HOME_PIN, INPUT_PULLUP);
        pinMode(BUTTON_CONTROL_PIN, INPUT_PULLUP);

        stopMotor();
        
        // Encoder library handles pin setup automatically
        myEncoder.write(0);
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
            myEncoder.write(0);
            if (isHoming) {
                stopMotor();
                targetPosition = 0;
                isHoming = false;
                return;
            }
        }

        if (isHoming) {
            setMotorDrive(-1, MIN_PWM);
            return;
        }

        long currentPos = getPosition();
        long error = targetPosition - currentPos;

        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0;
        if (dt <= 0) dt = 0.001;  // Prevent division by zero if called too fast

        float derivative = (error - lastError) / dt;

        // PD Output
        float output = (KP * error) + (KD * derivative);

        // Update state
        lastError = error;
        lastTime = currentTime;

        // Deadband / Stop Condition
        if (abs(error) <= STOP_TOLERANCE) {
            stopMotor();
            return;
        }

        int speed = abs(output);
        int direction = (output > 0) ? 1 : -1;

        // Constraint Speed
        if (speed > 255) speed = 255;

        // Minimum PWM to overcome friction
        if (speed < MIN_PWM && speed > 0) {
            speed = MIN_PWM;
        }

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
        return myEncoder.read();
    }
    long getTarget() const { return targetPosition; }

    float getDistanceMM() { return (getPosition() * DISTANCE_SLOPE) + DISTANCE_OFFSET; }

    void home() { isHoming = true; }
};

#endif  // MOTOR_HANDLER_HPP
