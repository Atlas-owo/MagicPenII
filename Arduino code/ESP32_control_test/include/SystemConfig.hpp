#ifndef SYSTEM_CONFIG_HPP
#define SYSTEM_CONFIG_HPP

#include <Arduino.h>

// ----------------- Pin Definitions -----------------
constexpr uint8_t M1_PIN = 8;
constexpr uint8_t M2_PIN = 7;
constexpr uint8_t ENA_PIN = 9;

constexpr uint8_t ENC_A_PIN = 3;
constexpr uint8_t ENC_B_PIN = 2;

constexpr uint8_t PRESSURE_SENSOR_PIN = A1;
constexpr uint8_t BUTTON_HOME_PIN = 11;
constexpr uint8_t BUTTON_CONTROL_PIN = 10;

// ----------------- Movement Parameters -----------------
constexpr int FAST_SPEED = 255;
constexpr int SLOW_SPEED = 150;
constexpr int CREEP_SPEED = 80;

constexpr int SLOW_DISTANCE = 300;
constexpr int CREEP_DISTANCE = 200;
constexpr int STOP_TOLERANCE = 10;

// ----------------- Limits & conversion -----------------
constexpr long MIN_POSITION = 50;
constexpr long MAX_POSITION = 10000;

constexpr float DISTANCE_SLOPE = 0.0084;
constexpr float DISTANCE_OFFSET = 0.5;
constexpr float MIN_DISTANCE_MM = 0.0;
constexpr float MAX_DISTANCE_MM = 90.0;

// ----------------- Task Settings -----------------
constexpr uint32_t SERIAL_TASK_DELAY = 50;  // ms
constexpr uint32_t MOTOR_TASK_DELAY = 10;   // ms

#endif // SYSTEM_CONFIG_HPP
