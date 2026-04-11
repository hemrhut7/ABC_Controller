#pragma once

// if enable BT_SERIAL, framework-arduinoespressif32 @ symlink://C:/PIO_Cores/esp32-bluepad32-4.1.0 in the platformio.ini should be commented
#define HAS_BT_SERIAL 0
#define HAS_WIFI_SERIAL 1

#define WIFI_SSID "TP-Link_E428"
#define WIFI_PASS "ssssssss"
#define AP_SSID "ABC_Controller_AP"
#define AP_PASS "ssssssss"
#define UDP_PORT 8000

