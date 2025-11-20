# mcu_imu
ESP32 C3 MINI and BNO085 code:
- initialize MCU and IMU
- configured data collection rate at 100 Hz
- configure IMU interrupt to alert MCU when data is ready
- MCU will read IMU data via I2C and format as JSON string: [{"ts":, "ax":, "ay":, "az":, "gx":, "gy":, "gz":}]
- MCU will being writing data once a swing is detected and will transmit after the swing has ended
