#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>

/**
 * @brief Creates a Wi-Fi Access Point using the credentials from config.h
 */
void create_access_point() {
    Serial.print("Creating Access Point named: ");
    Serial.println(WIFI_SSID_AP);

    // create the access point
    // WiFi.softAP(WIFI_SSID, WIFI_PASS);
    WiFi.softAP(WIFI_SSID_AP, NULL, 1); // no password
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

/**
 * @brief Connects to a Wi-Fi network using the credentials from config.h
 */
void connect_to_wifi() {
    Serial.print("Connecting to Wi-Fi network: ");
    Serial.println(WIFI_SSID_USER);

    WiFi.begin(WIFI_SSID_USER, WIFI_PASS_USER);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }

    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
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
    
    // check if connected to Wi-Fi
    // while (WiFi.status() != WL_CONNECTED) {
    //     WiFi.begin(WIFI_SSID_USER, WIFI_PASS_USER);
    //     Serial.println("Wi-Fi not connected, retrying...");
    //     return false;
    // }

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

    delay(100); // brief delay
    
    return (httpResponseCode >= 200 && httpResponseCode < 300);
}
