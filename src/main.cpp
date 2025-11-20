#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include "config.h"
#include "imu_handler.h"
#include "wifi_manager.h"
#include "data_buffer.h"
#include "ArduinoJson.h"
#include <LittleFS.h>
#include <esp_sleep.h>

// --- MODE SELECTOR ---
// Set to 1 for debug testing (prints JSON to Serial Monitor)
// Set to 0 for normal mode (sends data over Wi-Fi)
#define DEBUG_MODE 0

// --- ACCESS POINT TESTING --- (had priority over main application logic)
// Set to 1 for a simple Wi-Fi AP test, 0 for the full application
#define AP_TEST_MODE 0
// ----------------------------

#if AP_TEST_MODE == 1
/** *@brief code for testing ESP32 in Access Point mode
 * This code creates a WIFI access point and waits for the user to connect
*/
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Simple Wi-Fi AP Test ---");

  Serial.print("Creating Access Point named: ");
  Serial.println(WIFI_SSID);

  WiFi.softAP(WIFI_SSID, WIFI_PASS);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.println("------------------------------------");
  Serial.println("Waiting for a client to connect...");
}

void loop() {
  if (WiFi.softAPgetStationNum() > 0) {
    Serial.println("\nSUCCESS: A client has connected!");
    while(true) { delay(1000); } // halt
  } else {
    Serial.print(".");
  }
  delay(2000);
}

#else // Full Application Logic

// Make the semaphore from imu_handler.cpp available to this file
extern SemaphoreHandle_t imuDataSemaphore;

DataBuffer sensor_data_buffer;

/**
 * @brief Formats a local buffer of data points into a JSON string
 * @param local_buffer Pointer to the buffer containing data points
 * @param count Number of data points in the buffer
 * @return Formatted JSON string
 */
std::string format_local_buffer_as_json(const IMUDataPoint* local_buffer, int count) {
    if (count == 0) {
        return "";
    }
    JsonDocument doc;
    JsonArray dataArray = doc.to<JsonArray>();
    for (int i = 0; i < count; i++) {
        JsonObject dataPoint = dataArray.add<JsonObject>();
        dataPoint["ts"] = local_buffer[i].timestamp_us;
        dataPoint["ax"] = local_buffer[i].accelX;
        dataPoint["ay"] = local_buffer[i].accelY;
        dataPoint["az"] = local_buffer[i].accelZ;
        dataPoint["gx"] = local_buffer[i].gyroX;
        dataPoint["gy"] = local_buffer[i].gyroY;
        dataPoint["gz"] = local_buffer[i].gyroZ;
    }
    std::string output;
    serializeJson(doc, output);
    return output;
}

SemaphoreHandle_t transmitDataSemaphore = NULL;
bool collectingSwingData = false;

/**
 * @brief Interrupt-driven task for reading sensor data
 * This task sleeps until woken by the IMU's hardware interrupt via a semaphore
 */
void imu_read_task(void* pvParameters) {
    unsigned long start_time;
    for (;;) {
        // wait for semaphore indefinitely
        // sleep until interrupt fires
        if (xSemaphoreTake(imuDataSemaphore, portMAX_DELAY) == pdTRUE) {
            IMUDataPoint new_point;
            
            // interrupt fired; read available data
            if (read_sensor_data(new_point)) {
                sensor_data_buffer.addPoint(new_point);
                //check for swing event
                float total_accel = sqrt(
                    new_point.accelX * new_point.accelX +
                    new_point.accelY * new_point.accelY +
                    new_point.accelZ * new_point.accelZ
                );

                //Serial.println(total_accel);
                
                if (collectingSwingData){ // swing in progress
                    sensor_data_buffer.addPoint(new_point);
                    // check if swing ended
                    // if (total_accel < (THRESHOLD_ACCELERATION_MSS)) { // swing end detected
                    //     collectingSwingData = false; 
                    //     xSemaphoreGive(transmitDataSemaphore); // signal data ready for transmission
                    // } 
                    // check for timeout
                    if (millis() - start_time >= SWING_TIMEOUT_MS) { // swing timeout
                        collectingSwingData = false;
                        xSemaphoreGive(transmitDataSemaphore); // signal data ready for transmission
                    }
                }
                else if (total_accel >= THRESHOLD_ACCELERATION_MSS) { //swing start detected
                    collectingSwingData = true;
                    start_time = millis();
                }
                else{
                    // not collecting data, and no swing detected; do nothing
                }
            }
        }
    }
}

#if DEBUG_MODE == 1
/**
 * @brief "Offline" task that prints the data buffer to the Serial Monitor
 * This task waits for the transmitDataSemaphore to be given, then prints the buffered data.
 */
void data_output_task(void* pvParameters) {
    static IMUDataPoint local_data_buffer[BUFFER_SIZE];

    for (;;) {
        if (xSemaphoreTake(transmitDataSemaphore, portMAX_DELAY) == pdTRUE) {
            int point_count = sensor_data_buffer.copyAndClear(local_data_buffer);
            
            if (point_count > 0) {
                std::string payload = format_local_buffer_as_json(local_data_buffer, point_count);

                Serial.println("\n--- SWING CAPTURE ---");
                Serial.printf("Data points captured: %d\n", point_count);
                // Print the full JSON payload to verify the data
                Serial.println("JSON Payload:");
                Serial.println(payload.c_str());
                Serial.println("----------------------\n");
            }
        }
    }
}
#else
/**
 * @brief "Online" task for sending data over Wi-Fi.
 * This task waits for the transmitDataSemaphore to be given, then sends the buffered data.
 */
void wifi_transmission_task(void* pvParameters) {
    unsigned long start_time, end_time;
    static IMUDataPoint local_data_buffer[BUFFER_SIZE];

    for (;;) {
        if (xSemaphoreTake(transmitDataSemaphore, portMAX_DELAY) == pdTRUE) {
            int point_count = sensor_data_buffer.copyAndClear(local_data_buffer);
            if (point_count > 0) {
                std::string payload = format_local_buffer_as_json(local_data_buffer, point_count);
                
                Serial.println("\n--- SWING CAPTURE ---");
                Serial.printf("\nAttempting to send %d data points...\n", point_count);
                
                bool success = false;
                while (!success) {
                    start_time = millis();
                    success = send_http_post(payload);
                    end_time = millis();
                    if (success){
                        Serial.printf("SUCCESS: Data batch sent. Latency: %lu ms\n", end_time - start_time);
                        Serial.printf("Data Points Sent: %lu\n", point_count);
                        Serial.println("----------------------\n");
                    }
                    else {
                        Serial.println("ERROR: Data transmission failed. Retrying...");
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                }
            }
        }
    }
}
#endif

SemaphoreHandle_t powerButtonSemaphore = NULL;

/**
 * @brief Interrupt handler for power button press
 * Gives the semaphore to notify the power monitoring task
 */
void IRAM_ATTR power_button_interrupt_handler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Give the semaphore to unblock the power monitoring task
  xSemaphoreGiveFromISR(powerButtonSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) { // if a higher priority task was woken, yield
    portYIELD_FROM_ISR();
  }
}

void power_monitor_task(void* pvParameters) {
    Serial.println("Power monitor task started (interrupt-based).");
    for (;;) {
        // Wait indefinitely for the semaphore (signal from ISR)
        if (xSemaphoreTake(powerButtonSemaphore, portMAX_DELAY) == pdTRUE) {
            // debounce
            vTaskDelay(pdMS_TO_TICKS(250));
            xSemaphoreTake(powerButtonSemaphore, (TickType_t)0); 

            // send power flag low
            Serial.println("Power button pressed! Setting power flag LOW.");
            digitalWrite(ESP32_POWER_FLAG_PIN, LOW); // Signal external circuit to cut power
            Serial.println("Shutdown signal sent.");
        }
    }
}

/**
 * @brief Main setup function
 * Initializes power, serial, Wi-Fi, IMU, and starts tasks
 */
void setup() {
    // init power control pin
    pinMode(ESP32_POWER_FLAG_PIN, OUTPUT);
    digitalWrite(ESP32_POWER_FLAG_PIN, HIGH);
    Serial.println("Power flag pin set HIGH.");

    Serial.begin(115200);
    delay(2000); 


#if DEBUG_MODE == 1
    Serial.println("--- Tennis Racket Tracker (SERIAL PORT) ---");
#else
    Serial.println("--- Tennis Racket Tracker (WIFI) ---");
#endif

    // Initialize power button interrupt
    delay(1000);
    powerButtonSemaphore = xSemaphoreCreateBinary();
    pinMode(ESP32_POWER_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ESP32_POWER_BUTTON_PIN), power_button_interrupt_handler, FALLING);
    Serial.println("Power button interrupt pin configured.");
    xTaskCreate(power_monitor_task, "Power Monitor Task", 2048, NULL, 5, NULL);        

    // Initialize WiFi
#if DEBUG_MODE == 0
    // create an AP and wait for a client to connect.
    create_access_point();
    Serial.print("Access Point: ");
    Serial.println(WIFI_SSID);
    // Serial.print("WiFI Password: ");
    // Serial.println(WIFI_PASS);  
    Serial.println("Waiting for a client to connect...");
    while (WiFi.softAPgetStationNum() == 0) {
      Serial.print(".");
      delay(1000);
    }
    
    Serial.println("\nClient connected!");
#endif

    // Initialize the IMU  
    delay(1000);
    while (!setup_imu()) { // retry until successful
        Serial.println("IMU initialization failed.");
        Serial.println("Retrying IMU initialization in 2 seconds...");
        delay(2000);
    }
    Serial.println("IMU initialization successful.");

    transmitDataSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(imu_read_task, "IMU Task", 4096, NULL, 10, NULL);

#if DEBUG_MODE == 1
    xTaskCreate(data_output_task, "Data Output Task", 4096, NULL, 3, NULL);
#else
    xTaskCreate(wifi_transmission_task, "WiFi Task", 8192, NULL, 3, NULL);
#endif

}

/**
 * @brief Main loop function
 * Not used since tasks are handling all operations
 */
void loop() {
    vTaskDelete(NULL); 
}
#endif
