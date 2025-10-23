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

// /**
//  * @brief Creates a JSON file in LittleFS with the given data
//  * @param json_data The JSON string to write to the file
//  */
// void create_json_file(const std::string& json_data) {
//     Serial.println("JSON Data:");
//     Serial.println(json_data.c_str());
//     // Write JSON data to internal flash using LittleFS
//     if (!LittleFS.begin()) {
//         Serial.println("LittleFS mount failed");
//         return;
//     }

//     File file = LittleFS.open("/data.json", FILE_WRITE);
//     if (!file) {
//         Serial.println("Failed to open file for writing");
//         LittleFS.end();
//         return;
//     }

//     size_t written = file.print(json_data.c_str());
//     file.close();
//     Serial.printf("Wrote %u bytes to /data.json\n", (unsigned)written);

//     LittleFS.end();
// }

// /**
//  * @brief Read the stored JSON file from LittleFS
//  * @return std::string containing file contents or empty string if not found
//  */
// std::string read_json_file() {
//     if (!LittleFS.begin()) {
//         Serial.println("LittleFS mount failed");
//         return std::string();
//     }

//     File file = LittleFS.open("/data.json", FILE_READ);
//     if (!file) {
//         Serial.println("No /data.json found");
//         LittleFS.end();
//         return std::string();
//     }

//     std::string content;
//     while (file.available()) {
//         content += (char)file.read();
//     }
//     file.close();
//     LittleFS.end();
//     return content;
// }

// /**
//  * @brief Remove the stored JSON file from LittleFS
//  */
// void delete_json_file() {
//     if (!LittleFS.begin()) return;
//     if (LittleFS.exists("/data.json")) {
//         LittleFS.remove("/data.json");
//     }
//     LittleFS.end();
// }

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
 * @brief FreeRTOS task to monitor the power button.
 * Implements debouncing and sets the power flag LOW to signal shutdown.
 */
void power_monitor_task(void* pvParameters) {
    Serial.println("Power monitor task started.");
    bool last_button_state = HIGH; // Assume button is not pressed initially
    unsigned long last_debounce_time = 0;
    const unsigned long debounce_delay = 50; // milliseconds for debounce

    for (;;) {
        // Read the current state of the button
        int reading = digitalRead(ESP32_POWER_BUTTON_PIN);

        // Check if the state has changed (with debouncing)
        if (reading != last_button_state) {
            last_debounce_time = millis(); // Reset debounce timer
        }

        // If the button state has been stable for longer than the debounce delay
        if ((millis() - last_debounce_time) > debounce_delay) {
            // If the stable state is LOW (pressed) and the previous stable state was HIGH
            if (reading == LOW && last_button_state == HIGH) {
                Serial.println("Power button pressed! Setting power flag LOW.");
                digitalWrite(ESP32_POWER_FLAG_PIN, LOW); // Signal external circuit to cut power

                // --- IMPORTANT ---
                // Do NOT enter deep sleep here. The external circuit will remove power.
                // We just stop the task from running further checks.
                Serial.println("Shutdown signal sent. Waiting for power cut...");
                vTaskDelete(NULL); // Delete this task to prevent further checks
            }
        }
        
        // Update the last stable button state
        last_button_state = reading;

        // Check the button state periodically (e.g., every 20ms)
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

/**
 * @brief Main setup function
 * Initializes serial, Wi-Fi, IMU, and starts tasks
 */
void setup() {
    // // --- Initialize Power Control Pins FIRST ---
    // // Set the power flag pin as output and set it HIGH immediately on boot
    // pinMode(ESP32_POWER_FLAG_PIN, OUTPUT);
    // digitalWrite(ESP32_POWER_FLAG_PIN, HIGH);
    // Serial.println("Power flag pin set HIGH.");

    // // Set the power button pin as input with an internal pull-up resistor
    // // Assumes the button connects the pin to GND when pressed
    // pinMode(ESP32_POWER_BUTTON_PIN, INPUT_PULLUP);
    // Serial.println("Power button pin configured.");
    // // --- End Power Control Init ---

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

    // // Start the power button monitoring task
    // Serial.println("Starting power monitor task...");
    // xTaskCreate(power_monitor_task,       // Function to implement the task
    //             "Power Monitor Task", // Name of the task
    //             2048,                 // Stack size in words
    //             NULL,                 // Task input parameter
    //             5,                    // Priority of the task
    //             NULL); 

}

/**
 * @brief Main loop function
 * Not used since tasks are handling all operations
 */
void loop() {
    vTaskDelete(NULL); 
}
#endif