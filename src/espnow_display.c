#include "espnow_display.h"

#include <string.h>

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"

static const char *TAG = "ESPNOW_TX";

/* MAC de la pantalla */
static const uint8_t display_mac[6] =
{
    0x20, 0x50, 0x0D, 0x11, 0xBE, 0xD0
};

static void send_cb(const esp_now_send_info_t *tx_info,
                    esp_now_send_status_t status)
{
    ESP_LOGI(TAG,
             "ESP-NOW TX -> %s",
             status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

esp_err_t espnow_display_init(void)
{
    esp_err_t err;

    /* Inicializar ESP-NOW */
    err = esp_now_init();

ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));


    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error inicializando ESP-NOW (%s)",
                 esp_err_to_name(err));
        return err;
    }

    esp_now_peer_info_t peer = {0};

    memcpy(peer.peer_addr,
           display_mac,
           ESP_NOW_ETH_ALEN);

    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    err = esp_now_add_peer(&peer);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Error agregando peer (%s)",
                 esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(TAG, "Pantalla agregada como peer ESP-NOW");

    return ESP_OK;
}

esp_err_t espnow_display_send(const telemetry_packet_t *packet)
{
    return esp_now_send(
        display_mac,
        (const uint8_t *)packet,
        sizeof(telemetry_packet_t));
}