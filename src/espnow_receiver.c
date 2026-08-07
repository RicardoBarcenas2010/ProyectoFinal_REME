#include "espnow_receiver.h"

#include <string.h>

#include "communication_protocol.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"

static const char *TAG = "ESPNOW_RX";

/* MAC de la pantalla */
static const uint8_t display_mac[6] =
{
    0x20, 0x50, 0x0D, 0x11, 0xBE, 0xD0
};

/* Variables compartidas */
volatile uint8_t g_control_mode = 0;
volatile float g_manual_setpoint = 0.0f;

/*----------------------------------------------------------*/

static void recv_cb(const esp_now_recv_info_t *recv_info,
                    const uint8_t *data,
                    int len)
{
    if (len != sizeof(screen_command_t))
    {
        ESP_LOGW(TAG, "Paquete inválido (%d bytes)", len);
        return;
    }

    screen_command_t cmd;

    memcpy(&cmd, data, sizeof(cmd));

    g_control_mode = cmd.control_mode;
    g_manual_setpoint = cmd.manual_setpoint;

    ESP_LOGI(TAG,
             "CMD RX -> modo=%d  setpoint=%.2f",
             g_control_mode,
             g_manual_setpoint);
}

/*----------------------------------------------------------*/

esp_err_t espnow_receiver_init(void)
{
    esp_now_peer_info_t peer = {0};

    memcpy(peer.peer_addr,
           display_mac,
           ESP_NOW_ETH_ALEN);

    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);

    if (err != ESP_OK &&
        err != ESP_ERR_ESPNOW_EXIST)
    {
        return err;
    }

    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(recv_cb));

    ESP_LOGI(TAG, "Receptor ESP-NOW listo");

    return ESP_OK;
}