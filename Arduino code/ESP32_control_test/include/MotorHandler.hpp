#ifndef MOTOR_HANDLER_HPP
#define MOTOR_HANDLER_HPP

#include "SystemConfig.hpp"

class MotorHandler {
private:
    volatile long encoderCount;
    volatile long targetPosition;
    volatile uint8_t lastEncoded;
    bool isHoming;

    // Static instance pointer for ISR
    static MotorHandler* instance;

    void setMotorDirection(int direction) {
        if (direction > 0) {
            // Forward
            digitalWrite(M1_PIN, LOW);
            digitalWrite(M2_PIN, HIGH);
        } else {
            // Reverse
            digitalWrite(M1_PIN, HIGH);
            digitalWrite(M2_PIN, LOW);
        }
    }

    void setMotorSpeed(int speed) {
        analogWrite(ENA_PIN, speed);
    }

    long distanceToEncoderCounts(float distanceMM) {
        return (long)((distanceMM - DISTANCE_OFFSET) / DISTANCE_SLOPE);
    }

public:
    MotorHandler() : encoderCount(0), targetPosition(0), lastEncoded(0), isHoming(false) {
        instance = this;
    }

    void setup() {
        pinMode(M1_PIN, OUTPUT);
        pinMode(M2_PIN, OUTPUT);
        pinMode(ENA_PIN, OUTPUT);
        pinMode(ENC_A_PIN, INPUT);
        pinMode(ENC_B_PIN, INPUT);
        pinMode(BUTTON_HOME_PIN, INPUT_PULLUP);

        stopMotor();
        
        attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleInterrupt, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), handleInterrupt, CHANGE);
    }

    // Static ISR handler
    static void handleInterrupt() {
        if (instance) {
            instance->updateEncoder();
        }
    }

    void updateEncoder() {
        static const int8_t encoderStates[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
        uint8_t a = digitalRead(ENC_A_PIN);
        uint8_t b = digitalRead(ENC_B_PIN);
        uint8_t currentEncoded = (a << 1) | b;
        uint8_t sum = (lastEncoded << 2) | currentEncoded;
        encoderCount += encoderStates[sum];
        lastEncoded = currentEncoded;
    }

    void stopMotor() {
        digitalWrite(M1_PIN, LOW);
        digitalWrite(M2_PIN, LOW);
        analogWrite(ENA_PIN, 0);
    }

    void update() {
        if (isHoming) return; // Homing handles its own movement or delegates differently

        long currentPos = encoderCount;
        
        // Boundaries check
        if (currentPos < MIN_POSITION && targetPosition < MIN_POSITION) {
            targetPosition = MIN_POSITION;
        } else if (currentPos > MAX_POSITION && targetPosition > MAX_POSITION) {
            targetPosition = MAX_POSITION;
        }

        long error = targetPosition - currentPos;
        long absError = abs(error);

        if (absError <= STOP_TOLERANCE) {
            stopMotor();
            return;
        }

        int direction = (error > 0) ? 1 : -1;
        int speed = FAST_SPEED;
        
        if (absError <= CREEP_DISTANCE) speed = CREEP_SPEED;
        else if (absError <= SLOW_DISTANCE) speed = SLOW_SPEED;

        setMotorDirection(direction);
        setMotorSpeed(speed);
    }

    void setTargetDistance(float distanceMM) {
        if (distanceMM < MIN_DISTANCE_MM) distanceMM = MIN_DISTANCE_MM;
        if (distanceMM > MAX_DISTANCE_MM) distanceMM = MAX_DISTANCE_MM;
        setTargetPosition(distanceToEncoderCounts(distanceMM));
    }

    void setTargetPosition(long target) {
        if (!isHoming) {
             if (target < MIN_POSITION) target = MIN_POSITION;
             else if (target > MAX_POSITION) target = MAX_POSITION;
        }
        targetPosition = target;
    }

    long getPosition() const { return encoderCount; }
    long getTarget() const { return targetPosition; }
    
    float getDistanceMM() {
        return (encoderCount * DISTANCE_SLOPE) + DISTANCE_OFFSET;
    }

    void home() {
        isHoming = true;
        targetPosition = -10000;
        
        // Initial approach
        while (digitalRead(BUTTON_HOME_PIN) == HIGH) {
            update(); // Use standard update logic which will try to go to -10000
            delay(10);
        }
        
        // Final approach
        setMotorDirection(-1);
        setMotorSpeed(CREEP_SPEED);
        delay(850); // Hardcoded delay from original logic
        
        stopMotor();
        delay(100);
        
        encoderCount = 0;
        targetPosition = 0;
        isHoming = false;
    }
};

MotorHandler* MotorHandler::instance = nullptr;

#endif // MOTOR_HANDLER_HPP
