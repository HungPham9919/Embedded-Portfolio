#ifndef WIFI_H
#define WIFI_H
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <nvs_flash.h>

#define WIFI_SSID "Nokia 1280" // Write yours
#define WIFI_PASSWORD "123451235" // Write yours

void wifi_initialize(void);

#endif
