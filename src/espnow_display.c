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

/* ⭐ VARIABLES DE ESTADO ⭐ */
static bool s_pantalla_conectada = false;
static uint32_t s_ultimo_envio_ok = 0U;
static uint32_t s_ultimo_cambio_estado = 0U;
static uint32_t s_ultimo_intento_reconexion = 0U;
static const uint32_t TIMEOUT_PANTALLA_MS = 1000U;
static const uint32_t DEBOUNCE_MS = 500U;
static const uint32_t RECONEXION_INTERVALO_MS = 5000U;  /* Intentar reconectar cada 5 segundos */

/*----------------------------------------------------------*/

static void send_cb(const esp_now_send_info_t *tx_info,
                    esp_now_send_status_t status)
{
    uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
    
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_ultimo_envio_ok = ahora;
        ESP_LOGD(TAG, "ESP-NOW TX -> OK");
    } else {
        ESP_LOGD(TAG, "ESP-NOW TX -> FAIL");
    }
}

/*----------------------------------------------------------*/

/* ⭐ FUNCIÓN PARA RE-AGREGAR EL PEER ⭐ */
static void reconectar_pantalla(void)
{
    esp_now_peer_info_t peer = {0};

    memcpy(peer.peer_addr,
           display_mac,
           ESP_NOW_ETH_ALEN);

    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    /* Intentar eliminar el peer primero (si existe) */
    esp_now_del_peer(display_mac);
    
    /* Agregar el peer nuevamente */
    esp_err_t err = esp_now_add_peer(&peer);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✅ Pantalla reconectada como peer ESP-NOW");
        /* Reseteamos el timer de éxito para forzar nueva conexión */
        s_ultimo_envio_ok = (uint32_t)(esp_timer_get_time() / 1000ULL);
    } else {
        ESP_LOGW(TAG, "⚠️ Error al reconectar pantalla: %s", esp_err_to_name(err));
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
    
    /* Inicializar estado */
    s_pantalla_conectada = true;
    s_ultimo_envio_ok = (uint32_t)(esp_timer_get_time() / 1000ULL);
    s_ultimo_cambio_estado = 0U;
    s_ultimo_intento_reconexion = 0U;

    return ESP_OK;
}

/*----------------------------------------------------------*/

esp_err_t espnow_display_send(const telemetry_packet_t *packet)
{
    esp_err_t ret = esp_now_send(
        display_mac,
        (const uint8_t *)packet,
        sizeof(telemetry_packet_t));
    
    /* Verificar timeout y aplicar filtro anti-parpadeo */
    uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
    bool estado_real;
    
    /* Determinar estado real basado en el tiempo desde el último éxito */
    if ((ahora - s_ultimo_envio_ok) > TIMEOUT_PANTALLA_MS) {
        estado_real = false;
    } else {
        estado_real = true;
    }
    
    /* ⭐ APLICAR FILTRO DEBOUNCE ⭐ */
    if (estado_real != s_pantalla_conectada) {
        if ((ahora - s_ultimo_cambio_estado) > DEBOUNCE_MS) {
            s_pantalla_conectada = estado_real;
            s_ultimo_cambio_estado = ahora;
            
            if (estado_real) {
                ESP_LOGI(TAG, "✅ Pantalla CONECTADA (estable)");
            } else {
                ESP_LOGW(TAG, "⚠️ Pantalla DESCONECTADA (estable)");
                s_ultimo_intento_reconexion = 0U;  /* Resetear timer para reconexión */
            }
        }
    }
    
    /* ⭐⭐⭐ INTENTAR RECONEXIÓN SI ESTÁ DESCONECTADA ⭐⭐⭐ */
    if (!s_pantalla_conectada) {
        /* Intentar reconectar cada RECONEXION_INTERVALO_MS */
        if ((ahora - s_ultimo_intento_reconexion) > RECONEXION_INTERVALO_MS) {
            ESP_LOGI(TAG, "🔄 Intentando reconectar pantalla...");
            reconectar_pantalla();
            s_ultimo_intento_reconexion = ahora;
        }
    }
    
    return ret;
}

/*----------------------------------------------------------*/

bool espnow_display_is_connected(void)
{
    return s_pantalla_conectada;
}