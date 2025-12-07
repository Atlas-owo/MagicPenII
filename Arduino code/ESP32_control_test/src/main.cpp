#include "MotorHandler.hpp"
#include "SerialHandler.hpp"
#include "SystemConfig.hpp"
#include <Arduino.h>

// Global objects
MotorHandler  motor;
SerialHandler serialHandler( motor );

// FreeRTOS Task Handles
TaskHandle_t motorTaskHandle  = NULL;
TaskHandle_t serialTaskHandle = NULL;

// Task Definitions
// Task Definitions
void motorTask( void* pvParameters ) {
    for ( ;; ) {
        motor.update();
        vTaskDelay( pdMS_TO_TICKS( MOTOR_TASK_DELAY ) );
    }
}

void serialRxTask( void* pvParameters ) {
    for ( ;; ) {
        serialHandler.update();
        vTaskDelay( pdMS_TO_TICKS( SERIAL_RX_TASK_DELAY ) );
    }
}

void serialTxTask( void* pvParameters ) {
    for ( ;; ) {
        serialHandler.sendStatus();
        vTaskDelay( pdMS_TO_TICKS( SERIAL_TX_TASK_DELAY ) );
    }
}

void setup() {
    serialHandler.setup();
    Serial.println( "{\"status\":\"booting\"}" );

    // Initialize Modules
    motor.setup();

    // Create Tasks
    xTaskCreate( motorTask,        // Task function
                 "Motor Control",  // Name
                 4096,             // Stack size
                 NULL,             // Parameters
                 2,                // Priority (High)
                 &motorTaskHandle  // Handle
    );

    xTaskCreate( serialRxTask,  // Task function
                 "Serial Rx",   // Name
                 4096,          // Stack size
                 NULL,          // Parameters
                 1,             // Priority (Low)
                 NULL           // Handle (not stored)
    );

    xTaskCreate( serialTxTask,      // Task function
                 "Serial Tx",       // Name
                 4096,              // Stack size
                 NULL,              // Parameters
                 1,                 // Priority (Low)
                 &serialTaskHandle  // Handle (using existing handle var or NULL if OK)
    );

    // Initial Homing:
    // Initial Homing:
    if (AUTO_HOME_ON_BOOT) {
        motor.home();
    } else {
        motor.stopMotor();
    }
}

void loop() {
    // Empty. Tasks handle everything.
    vTaskDelete( NULL );
}