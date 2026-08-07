/**
 * @file wifi_manager.c
 * @brief Implementación del gestor WiFi
 * 
 * @note Este es TU CÓDIGO EXISTENTE con pequeñas mejoras
 *       para añadir funcionalidades como obtener IP y reconectar.
 */

#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <string.h>

#define TAG "WIFI"

/* ──────────────────────────────────────────────────────────────
 *  VARIABLES GLOBALES
 * ────────────────────────────────────────────────────────────── */

EventGroupHandle_t s_wifi_event_group = NULL;
bool wifi_connected = false;
static int s_retry_num = 0;
static esp_netif_t *s_wifi_netif = NULL;

/* ──────────────────────────────────────────────────────────────
 *  MANEJADOR DE EVENTOS WiFi
 * ────────────────────────────────────────────────────────────── */

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "📡 Intentando conectar a WiFi...");
        
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "🔄 Reintentando conectar (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "❌ No se pudo conectar a WiFi después de %d intentos", WIFI_MAX_RETRY);
            wifi_connected = false;
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ WiFi conectado! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ──────────────────────────────────────────────────────────────
 *  INICIALIZACIÓN WiFi
 * ────────────────────────────────────────────────────────────── */

void wifi_init_sta(void)
{
    /* Crear grupo de eventos */
    s_wifi_event_group = xEventGroupCreate();

    /* Inicializar red y loop de eventos */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifi_netif = esp_netif_create_default_wifi_sta();

    /* Inicializar WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Registrar manejadores de eventos */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    /* Configurar WiFi (SSID y contraseña) */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold = {
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "📡 Conectando a WiFi: %s...", WIFI_SSID);

    /* Esperar conexión o fallo (bloqueante) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Conectado a WiFi correctamente");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "❌ Falló conexión WiFi");
    } else {
        ESP_LOGE(TAG, "❌ Error inesperado en conexión WiFi");
    }
}

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES ADICIONALES
 * ────────────────────────────────────────────────────────────── */

bool is_wifi_connected(void)
{
    return wifi_connected;
}

bool wifi_get_ip(char *ip_buffer, size_t buffer_size)
{
    if (!wifi_connected || ip_buffer == NULL || buffer_size < 16) {
        return false;
    }

    esp_netif_ip_info_t ip_info;
    if (s_wifi_netif != NULL && esp_netif_get_ip_info(s_wifi_netif, &ip_info) == ESP_OK) {
        snprintf(ip_buffer, buffer_size, IPSTR, IP2STR(&ip_info.ip));
        return true;
    }

    return false;
}

void wifi_disconnect(void)
{
    if (wifi_connected) {
        esp_wifi_disconnect();
        wifi_connected = false;
        ESP_LOGI(TAG, "📡 WiFi desconectado");
    }
}

bool wifi_reconnect(void)
{
    if (wifi_connected) {
        return true;  /* Ya está conectado */
    }

    s_retry_num = 0;
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "🔄 Reconectando a WiFi...");
        return true;
    }

    ESP_LOGE(TAG, "❌ Error al reconectar: %d", err);
    return false;
}