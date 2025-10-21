#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include "config.h"
#include "imu_handler.h"
#include "wifi_manager.h"
#include "data_buffer.h"
#include "ArduinoJson.h"

// --- MODE SELECTOR ---
// Set to 1 for offline testing (prints JSON to Serial Monitor)
// Set to 0 for online mode (sends data over Wi-Fi)
#define OFFLINE_TEST_MODE 0

// --- ACCESS POINT TESTING --- (had priority over main application logic)
// Set to 1 for a simple Wi-Fi AP test, 0 for the full application
#define AP_TEST_MODE 0
// ----------------------------


#if AP_TEST_MODE == 1
/** *@brief Simple code for testing ESP32 in Access Point mode
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

// Helper function to format a local buffer of data points into a JSON string
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

/**
 * @brief Interrupt-driven task for reading sensor data
 * This task sleeps until woken by the IMU's hardware interrupt via a semaphore
 */
void imu_read_task(void* pvParameters) {
    for (;;) {
        // wait for semaphore indefinitely
        // sleep until interrupt fires
        if (xSemaphoreTake(imuDataSemaphore, portMAX_DELAY) == pdTRUE) {
            IMUDataPoint new_point;
            // interrupt fired; read available data
            if (read_sensor_data(new_point)) {
                sensor_data_buffer.addPoint(new_point);
            }
        }
    }
}

#if OFFLINE_TEST_MODE == 1
/**
 * @brief "Offline" task that prints the data buffer to the Serial Monitor
 */
void data_output_task(void* pvParameters) {
    static IMUDataPoint local_data_buffer[BUFFER_SIZE];
    static unsigned long total_data_points = 0;
    static unsigned long total_packets_sent = 0;
    static unsigned long last_report_time = millis();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DATA_TRANSMISSION_INTERVAL_MS));
        int point_count = sensor_data_buffer.copyAndClear(local_data_buffer);
        
        if (point_count > 0) {
            std::string payload = format_local_buffer_as_json(local_data_buffer, point_count);

            Serial.println("\n--- SIMULATED SEND ---");
            Serial.printf("Data points captured: %d\n", point_count);
            // Print the full JSON payload to verify the data
            Serial.println("JSON Payload:");
            Serial.println(payload.c_str());
            Serial.println("----------------------\n");

            total_data_points += point_count;
            total_packets_sent++;
        }

        if (millis() - last_report_time > 120000) {
            Serial.println("\n--- 2-MINUTE SUMMARY ---");
            Serial.printf("Total Data Points Sent: %lu\n", total_data_points);
            Serial.printf("Total JSON Packets Sent: %lu\n", total_packets_sent);
            Serial.println("------------------------\n");
            last_report_time = millis();
        }
    }
}
#else
/**
 * @brief "Online" task for sending data over Wi-Fi.
 */
void wifi_transmission_task(void* pvParameters) {
    unsigned long start_time, end_time;
    static IMUDataPoint local_data_buffer[BUFFER_SIZE];
    static unsigned long total_data_points = 0;
    static unsigned long total_packets_sent = 0;
    static unsigned long last_report_time = millis();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DATA_TRANSMISSION_INTERVAL_MS));
        int point_count = sensor_data_buffer.copyAndClear(local_data_buffer);
        
        if (point_count > 0) {
            std::string payload = format_local_buffer_as_json(local_data_buffer, point_count);
            
            Serial.printf("\nAttempting to send %d data points...\n", point_count);
            
            start_time = millis();
            if (send_http_post(payload)) {
                end_time = millis();
                Serial.printf("SUCCESS: Data batch sent. Latency: %lu ms\n", end_time - start_time);
                total_data_points += point_count;
                total_packets_sent++;
            } else {
                Serial.println("FAILURE: Failed to send data batch.");
            }
        }

        if (millis() - last_report_time > 120000) {
            Serial.println("\n--- 2-MINUTE SUMMARY ---");
            Serial.printf("Total Data Points Sent: %lu\n", total_data_points);
            Serial.printf("Total JSON Packets Sent: %lu\n", total_packets_sent);
            Serial.println("------------------------\n");
            last_report_time = millis();
            total_data_points = 0;
            total_packets_sent = 0;
        }
    }
}
#endif

/**
 * @brief Checks the power state from the designated GPIO pin
 * @return The digital read value of the power flag pin
 */
int check_power_off(){
    // read GPIO4
    int power_state = digitalRead(ESP32_POWER_FLAG_PIN);
    if (power_state == LOW){

    }
    return power_state;
}

/**
 * @brief Main setup function
 * Initializes serial, Wi-Fi, IMU, and starts tasks
 */
void setup() {
    //set GPIO3 high as Tamer requested with pullup
    pinMode(ESP32_POWER_FLAG_PIN, OUTPUT);
    digitalWrite(ESP32_POWER_FLAG_PIN, HIGH);

    Serial.begin(115200);
    delay(2000); 


#if OFFLINE_TEST_MODE == 1
    Serial.println("--- Tennis Racket Tracker (INTERRUPT-DRIVEN OFFLINE MODE) ---");
#else
    Serial.println("--- Tennis Racket Tracker (INTERRUPT-DRIVEN ONLINE MODE) ---");
#endif
    
#if OFFLINE_TEST_MODE == 0
    // create an AP and wait for a client to connect.
    create_access_point(); 
    Serial.println("Waiting for a client to connect...");
    while (WiFi.softAPgetStationNum() == 0) {
      Serial.print(".");
      delay(1000);
    }
    // delay(1000); // give some time to stabilize
    Serial.println("\nClient connected!");
#endif

    delay(500);

    if (!setup_imu()) {
        Serial.println("IMU initialization failed! Halting.");
        while (1) { delay(100); }
    }
    Serial.println("IMU initialization successful.");


    Serial.println("Starting tasks...");
    xTaskCreate(imu_read_task, "IMU Task", 4096, NULL, 10, NULL);

#if OFFLINE_TEST_MODE == 1
    xTaskCreate(data_output_task, "Data Output Task", 4096, NULL, 3, NULL);
#else
    xTaskCreate(wifi_transmission_task, "WiFi Task", 8192, NULL, 3, NULL);
#endif

    xTaskCreate([](void*){
        for(;;){
            vTaskDelay(pdMS_TO_TICKS(500));
            if (check_power_off() == LOW){
                digitalWrite(ESP32_POWER_FLAG_PIN, LOW); // set the pin low to signal power off
                Serial.println("Power off detected! Shutting down...");
                esp_sleep_enable_timer_wakeup(1000000); // 1 second delay before sleep
                esp_deep_sleep_start();
            }
        }
    }, "Power Monitor Task", 2048, NULL, 5, NULL);

}

void loop() {
    vTaskDelete(NULL); 

}
#endif