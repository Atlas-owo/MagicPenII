#include <Arduino.h>

#include "MotorHandler.hpp"
#include "SerialHandler.hpp"
#include "SystemConfig.hpp"

// Global objects
MotorHandler motor;
SerialHandler serialHandler(motor);

// Timer variables for non-blocking loop
unsigned long lastMotorUpdate = 0;
unsigned long lastSerialRxUpdate = 0;
unsigned long lastSerialTxUpdate = 0;

void setup() {
    serialHandler.setup();
    Serial.println("{\"status\":\"booting\"}");

    // Initialize Modules
    motor.setup();

    // Initial Homing:
    if (AUTO_HOME_ON_BOOT) {
        motor.home();
    } else {
        motor.stopMotor();
    }
}

void loop() {
    unsigned long currentMillis = millis();

    // Serial Rx Task
    if (currentMillis - lastSerialRxUpdate >= SERIAL_RX_TASK_DELAY) {
        serialHandler.update();
        lastSerialRxUpdate = currentMillis;
    }

    // Motor Task
    if (currentMillis - lastMotorUpdate >= MOTOR_TASK_DELAY) {
        motor.update();
        lastMotorUpdate = currentMillis;
    }

    // Serial Tx Task
    if (currentMillis - lastSerialTxUpdate >= SERIAL_TX_TASK_DELAY) {
        serialHandler.sendStatus();
        lastSerialTxUpdate = currentMillis;
    }
}