#pragma once
#include <string>

// Creates a Wi-Fi Access Point using the credentials in config.h
void create_access_point();

// Sends the data payload to the connected client via HTTP POST
// Returns true on a successful HTTP response (2xx), false otherwise.
bool send_http_post(const std::string& payload);
