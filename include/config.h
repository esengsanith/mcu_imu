// config.h
#pragma once

// Wi-Fi Credentials
#define WIFI_SSID "ESP32_AP2"
#define WIFI_PASS "password"

// Web Server Endpoint
#define SERVER_URL "http://192.168.4.2:4000/data"// FILL IN (WEB APP SUBSYSTEM)

// IMU & Buffer Settings
#define IMU_SAMPLE_RATE_HZ 100
#define DATA_TRANSMISSION_INTERVAL_MS 2000
// Buffer holds 2 seconds of data @ 100Hz (~200) + a small safety margin (10)
#define BUFFER_SIZE 210

// Pin definitions
#define BNO08X_RESET -1 // -1 = not used
#define BNO08X_INT_PIN 2 // GPIO pin connected to BNO085's INT pin