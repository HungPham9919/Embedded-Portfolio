#include <stdio.h>
#include <string.h>
#include "wifi.h"
#include "esp_log.h"

static void wifi_handler(void *arg, esp_event_base_t event_base, int32_t event_id,void *event_data){
    if(event_base == WIFI_EVENT){
        if(event_id == WIFI_EVENT_STA_START){
            ESP_LOGI("Wifi","Wifi connected");
            esp_wifi_connect();
        }
        else if(event_id == WIFI_EVENT_STA_DISCONNECTED){
            static int retry = 0;
            if(retry < 5){
                esp_wifi_connect();
                retry++;
                printf("Connecting to wifi after disconnected in %d time(s) \n", retry);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else {
                retry = 0;
                printf("Failed to connect to wifi \n");
            }
        }
    }

    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("IP","Got IPv4:"IPSTR,IP2STR(&event->ip_info.ip));

        esp_netif_dns_info_t dns;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if(esp_netif_get_dns_info(netif,ESP_NETIF_DNS_MAIN,&dns) == ESP_OK){
            ESP_LOGI("Net","DNS:"IPSTR,IP2STR(&dns.ip.u_addr.ip4));
        }
    }
}

void wifi_initialize(void){
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta(); // sta

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT(); // driver initialized
    esp_wifi_init(&wifi_cfg);

    esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID, &wifi_handler,NULL);
    esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&wifi_handler,NULL);

    wifi_config_t wf_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);

    esp_wifi_set_config(WIFI_IF_STA, &wf_cfg);
    esp_wifi_start();
    esp_wifi_connect();
}



