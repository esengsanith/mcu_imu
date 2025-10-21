#include "imu_handler.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "config.h"
#include <esp_timer.h>

// BNO08x I2C address
#define BNO08X_I2C_ADDR 0x4B

SemaphoreHandle_t imuDataSemaphore = NULL;

void IRAM_ATTR imu_interrupt_handler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Give the semaphore to unblock the IMU reading task
  xSemaphoreGiveFromISR(imuDataSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) { // if a higher priority task was woken, yield
    portYIELD_FROM_ISR();
  }
}

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

bool setup_imu() {
    // init and set to fast mode (400kHz)
    Wire.begin();
    Wire.setClock(400000);

    // create the semaphore for signaling new data availability
    imuDataSemaphore = xSemaphoreCreateBinary();
    // configure the interrupt pin
    pinMode(BNO08X_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BNO08X_INT_PIN), imu_interrupt_handler, FALLING);

    //init the IMU
    if (!bno08x.begin_I2C(BNO08X_I2C_ADDR)) {
        Serial.println("Failed to find BNO08x chip.");
        return false;
    }
    Serial.println("BNO08x found.");

    long report_interval_us = 1000000L / IMU_SAMPLE_RATE_HZ; // interval report in microseconds

    // enable the accel and gyro reports
    if (!bno08x.enableReport(SH2_ACCELEROMETER, report_interval_us)) {
        Serial.println("Could not enable accelerometer.");
        return false;
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval_us)) {
        Serial.println("Could not enable gyroscope.");
        return false;
    }

    Serial.println("IMU configured successfully.");
    return true;
}

/**
 * @brief Reads a sensor event and assembles a complete data point
 * This function remembers partial data between calls.
 * @param dataPoint Reference to the struct where a completed data point will be stored
 * @return True ONLY when a complete accel + gyro pair has been assembled
 */
bool read_sensor_data(IMUDataPoint& dataPoint) {
    // variables to remember their values between function calls
    static IMUDataPoint partial_data;
    static bool has_accel = false;
    static bool has_gyro = false;

    // interrupt fired, check for new events
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                partial_data.accelX = sensorValue.un.accelerometer.x;
                partial_data.accelY = sensorValue.un.accelerometer.y;
                partial_data.accelZ = sensorValue.un.accelerometer.z;
                has_accel = true;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                partial_data.gyroX = sensorValue.un.gyroscope.x;
                partial_data.gyroY = sensorValue.un.gyroscope.y;
                partial_data.gyroZ = sensorValue.un.gyroscope.z;
                has_gyro = true;
                break;
        }

        // check if we have both accel and gyro data
        if (has_accel && has_gyro) {
            // copy the completed data to the output
            dataPoint = partial_data;
            dataPoint.timestamp_us = esp_timer_get_time();
            
            // reset for next point
            has_accel = false;
            has_gyro = false;
            return true; // signal that a full data point is ready
        }
    }
    
    // do not have a complete data point yet
    return false;
}



