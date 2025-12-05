#include <Arduino.h>
#include "SystemConfig.hpp"
#include "MotorHandler.hpp"
#include "SerialHandler.hpp"

// Global objects
MotorHandler motor;
SerialHandler serialHandler(motor);

// FreeRTOS Task Handles
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t serialTaskHandle = NULL;

// Task Definitions
void motorTask(void *pvParameters) {
    for (;;) {
        motor.update();
        vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK_DELAY));
    }
}

void serialTask(void *pvParameters) {
    for (;;) {
        serialHandler.update();
        serialHandler.sendStatus();
        vTaskDelay(pdMS_TO_TICKS(SERIAL_TASK_DELAY));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("{\"status\":\"booting\"}");

    // Initialize Modules
    motor.setup();
    serialHandler.setup();

    // Create Tasks
    xTaskCreate(
        motorTask,          // Task function
        "Motor Control",    // Name
        4096,               // Stack size
        NULL,               // Parameters
        2,                  // Priority (High)
        &motorTaskHandle    // Handle
    );

    xTaskCreate(
        serialTask,         // Task function
        "Serial Comms",     // Name
        4096,               // Stack size
        NULL,               // Parameters
        1,                  // Priority (Low)
        &serialTaskHandle   // Handle
    );

    // Initial Homing
    // Note: Homing is blocking in current implementation. 
    // It might be better to trigger it via command, but let's keep original behavior if safe.
    // However, since we are in tasks, blocking setup might delay task start.
    // Let's run homing in the motor task or just once here before scheduler takes over?
    // FreeRTOS scheduler starts automatically on ESP32 in Arduino.
    
    // We can just send a "Ready" message. Homing should ideally be a command.
    // Original code did: stopMotor(); homeMotor();
    motor.stopMotor();
    // motor.home(); // Optional: Uncomment if auto-home on boot is strictly required. 
                  // Better to let user send {"cmd":"HOME"}.
}

void loop() {
    // Empty. Tasks handle everything.
    vTaskDelete(NULL); 
}