#include "espnow_display.h"

#include <string.h>

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ESPNOW_TX";

/* MAC de la pantalla */
static const uint8_t display_mac[6] =
{
    0x20, 0x50, 0x0D, 0x11, 0xBE, 0xD0
};

/* Variables de estado de conexión */
static bool s_pantalla_conectada = false;
static uint32_t s_ultimo_envio_ok = 0U;
static const uint32_t TIMEOUT_PANTALLA_MS = 500U;  /* 500ms como pide el proyecto */

/*----------------------------------------------------------*/

static void send_cb(const esp_now_send_info_t *tx_info,
                    esp_now_send_status_t status)
{
    uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
    
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_ultimo_envio_ok = ahora;
        if (!s_pantalla_conectada) {
            s_pantalla_conectada = true;
            ESP_LOGI(TAG, "✅ Pantalla CONECTADA");
        }
    } else {
        if (s_pantalla_conectada) {
            s_pantalla_conectada = false;
            ESP_LOGW(TAG, "⚠️ Pantalla DESCONECTADA (envío fallido)");
        }
    }
}

/*----------------------------------------------------------*/

esp_err_t espnow_display_init(void)
{
    esp_err_t err;

    err = esp_now_init();

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error inicializando ESP-NOW (%s)",
                 esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));

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
    
    s_pantalla_conectada = false;
    s_ultimo_envio_ok = (uint32_t)(esp_timer_get_time() / 1000ULL);

    return ESP_OK;
}

/*----------------------------------------------------------*/

esp_err_t espnow_display_send(const telemetry_packet_t *packet)
{
    esp_err_t ret = esp_now_send(
        display_mac,
        (const uint8_t *)packet,
        sizeof(telemetry_packet_t));
    
    /* Verificar timeout */
    uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (s_pantalla_conectada && (ahora - s_ultimo_envio_ok) > TIMEOUT_PANTALLA_MS) {
        s_pantalla_conectada = false;
        ESP_LOGW(TAG, "⚠️ Pantalla DESCONECTADA (timeout: %d ms)", TIMEOUT_PANTALLA_MS);
    }
    
    return ret;
}

/*----------------------------------------------------------*/

bool espnow_display_is_connected(void)
{
    return s_pantalla_conectada;
}