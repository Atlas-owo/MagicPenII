#ifndef SYSTEM_CONFIG_HPP
#define SYSTEM_CONFIG_HPP

#include <Arduino.h>

// ----------------- Pin Definitions -----------------
constexpr uint8_t M1_PIN = 32;
constexpr uint8_t M2_PIN = 33;
constexpr uint8_t SLEEP_PIN = 25;

constexpr uint8_t ENC_A_PIN = 26;
constexpr uint8_t ENC_B_PIN = 27;

constexpr uint8_t PRESSURE_SENSOR_PIN = 34;
constexpr uint8_t BUTTON_HOME_PIN = 12;
constexpr uint8_t BUTTON_CONTROL_PIN = 14;

// ----------------- Movement Parameters -----------------
constexpr float KP = 0.7;
constexpr float KD = 0.05;
constexpr int MIN_PWM = 100;
constexpr int STOP_TOLERANCE = 10;

// ----------------- Limits & conversion -----------------
constexpr long MIN_POSITION = 50;
constexpr long MAX_POSITION = 10000;

constexpr float DISTANCE_SLOPE = 0.0084;
constexpr float DISTANCE_OFFSET = 0.5;
constexpr float MIN_DISTANCE_MM = 0.0;
constexpr float MAX_DISTANCE_MM = 90.0;

// ----------------- Task Settings -----------------
constexpr uint32_t SERIAL_RX_TASK_DELAY = 1;     // ms (Fast Read)
constexpr uint32_t SERIAL_TX_TASK_DELAY = 1000;  // ms (Slow Write)
constexpr uint32_t MOTOR_TASK_DELAY = 1;         // ms

constexpr bool AUTO_HOME_ON_BOOT = true;

#endif  // SYSTEM_CONFIG_HPP
