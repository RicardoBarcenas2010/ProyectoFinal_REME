/**
 * @file http_server.c
 * @brief Servidor HTTP minimalista - SOLO endpoint /angle
 * @details Recibe ángulos vía GET /angle?value=XX
 *          Sin HTML, sin página web, solo datos.
 */

#include "http_server.h"
#include "wifi_manager.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

#define TAG "HTTP"

/* ──────────────────────────────────────────────────────────────
 *  VARIABLES GLOBALES
 * ────────────────────────────────────────────────────────────── */

float ultimo_angulo = 0.0f;
int contador_peticiones = 0;

/* ──────────────────────────────────────────────────────────────
 *  MANEJADOR: GET /angle?value=XX
 * ────────────────────────────────────────────────────────────── */

static esp_err_t angle_get_handler(httpd_req_t *req)
{
    char param[32] = {0};
    char query[64] = {0};
    char response[64];
    
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    
    if (query_len > 1) {
        if (httpd_req_get_url_query_str(req, query, query_len) == ESP_OK) {
            if (httpd_query_key_value(query, "value", param, sizeof(param)) == ESP_OK) {
                ultimo_angulo = atof(param);
                contador_peticiones++;
                
                ESP_LOGI(TAG, "📐 Ángulo recibido #%d: %.2f°", contador_peticiones, ultimo_angulo);
                
                /* Respuesta simple en texto plano */
                snprintf(response, sizeof(response), "OK: %.2f", ultimo_angulo);
                httpd_resp_set_type(req, "text/plain");
                httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
        }
    }
    
    /* Error si falta el parámetro */
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_send(req, "ERROR: Falta 'value'", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────
 *  MANEJADOR: GET /status (opcional - solo para verificar)
 * ────────────────────────────────────────────────────────────── */

static esp_err_t status_get_handler(httpd_req_t *req)
{
    char response[64];
    snprintf(response, sizeof(response), "Ultimo angulo: %.2f, Peticiones: %d", 
             ultimo_angulo, contador_peticiones);
    
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────
 *  INICIAR SERVIDOR
 * ────────────────────────────────────────────────────────────── */

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        
        /* Endpoint principal: /angle?value=XX */
        httpd_uri_t angle_uri = {
            .uri       = "/angle",
            .method    = HTTP_GET,
            .handler   = angle_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &angle_uri);
        
        /* Endpoint opcional: /status */
        httpd_uri_t status_uri = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = status_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_uri);

        ESP_LOGI(TAG, "✅ Servidor HTTP iniciado (sin HTML)");
        ESP_LOGI(TAG, "📡 Endpoint: /angle?value=XX");

        /* Mostrar IP */
        char ip[16];
        if (wifi_get_ip(ip, sizeof(ip))) {
            ESP_LOGI(TAG, "📱 Envía ángulos a: http://%s/angle?value=30", ip);
        }

        return server;
    }

    ESP_LOGE(TAG, "❌ Error al iniciar servidor");
    return NULL;
}

/* ──────────────────────────────────────────────────────────────
 *  DETENER SERVIDOR
 * ────────────────────────────────────────────────────────────── */

void stop_webserver(httpd_handle_t server)
{
    if (server) {
        httpd_stop(server);
        ESP_LOGI(TAG, "🛑 Servidor HTTP detenido");
    }
}

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES DE ACCESO
 * ────────────────────────────────────────────────────────────── */

float get_ultimo_angulo(void)
{
    return ultimo_angulo;
}

int get_contador_peticiones(void)
{
    return contador_peticiones;
}