// config.h
#pragma once

// Wi-Fi Credentials
// #define WIFI_SSID "ESP32_AP_RACKET"
// #define WIFI_SSID "ESP32_AP_WRIST"
#define WIFI_SSID "ESP32_AP_TEST"
#define WIFI_PASS "password"

// Web Server Endpoint
// #define SERVER_URL "http://192.168.4.2:4000/data" // test server URL
#define SERVER_URL "http://192.168.4.2:4000/data"// FILL IN (WEB APP SUBSYSTEM)

// IMU & Buffer Settings
#define IMU_SAMPLE_RATE_HZ 100
#define BUFFER_SIZE 150 // number of IMU data points to buffer
#define THRESHOLD_ACCELERATION_MSS 50.0f // m/s²
#define SWING_TIMEOUT_MS 700 // ms

// BNO08x I2C address
#define BNO08X_I2C_ADDR 0x4B

// Pin definitions
#define BNO08X_RESET -1 // -1 = not used
#define BNO08X_INT_PIN 2 // GPIO pin connected to BNO085's interrupt pin
#define ESP32_POWER_FLAG_PIN 3 // GPIO pin power flag (cirucuit reads high = powered on, circuit reads low = power off)
#define ESP32_POWER_BUTTON_PIN 4 // GPIO pin connected to the power button