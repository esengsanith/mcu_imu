#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>

/**
 * @brief Creates a Wi-Fi Access Point using the credentials from config.h
 */
void create_access_point() {
    Serial.print("Creating Access Point named: ");
    Serial.println(WIFI_SSID);

    // create the access point
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

/**
 * @brief Sends a data payload to the web server via HTTP POST
 * @param payload The string (JSON) payload to send
 * @return TRUE on a successful HTTP response (2xx), else FALSE
 */
bool send_http_post(const std::string& payload) {
    // check if a client is connected before trying to send
    if (WiFi.softAPgetStationNum() == 0) {
        Serial.println("No client connected, cannot send data.");
        return false;
    }

    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(payload.c_str());

    if (httpResponseCode > 0) {
        Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    } else {
        Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    http.end();
    return (httpResponseCode >= 200 && httpResponseCode < 300);
}
