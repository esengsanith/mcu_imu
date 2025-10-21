// imu_handler.h
#pragma once
#include <cstdint>

// define the GPIO pin connected to the BNO085's INT pin
// #define BNO08X_INT_PIN 2

// struct to hold a single snapshot of sensor data
struct IMUDataPoint {
    uint64_t timestamp_us; // Microsecond timestamp
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
};

// initialize the I2C bus and the BNO085 sensor
bool setup_imu();

// reads a single packet from the IMU and populates the struct
bool read_sensor_data(IMUDataPoint& dataPoint);

