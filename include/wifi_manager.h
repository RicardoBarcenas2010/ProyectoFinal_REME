/**
 * @file wifi_manager.h
 * @brief Gestión de conexión WiFi para ESP32
 * @details Configuración y control de la conexión WiFi en modo STA (Station)
 * 
 * @note Este archivo es PARTE DE TU CÓDIGO EXISTENTE
 *       Se mantiene sin cambios para compatibilidad
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────────────────────────────────────────────
 *  CONFIGURACIÓN DE WiFi (AJUSTA SEGÚN TU RED)
 * ────────────────────────────────────────────────────────────── */

/** @brief SSID de la red WiFi (nombre de la red) */
//#define WIFI_SSID               "Galaxy S25 FE 8015"
#define WIFI_SSID               "MEGACABLE-2.4G-73A3"
/** @brief Contraseña de la red WiFi */
//#define WIFI_PASS               "Ricardo20"
#define WIFI_PASS               "BBzbusN3Xu"

/** @brief Bit de evento: WiFi conectado */
#define WIFI_CONNECTED_BIT      BIT0

/** @brief Bit de evento: Fallo de conexión */
#define WIFI_FAIL_BIT           BIT1

/** @brief Número máximo de intentos de reconexión */
#define WIFI_MAX_RETRY          5

/* ──────────────────────────────────────────────────────────────
 *  VARIABLES GLOBALES (extern)
 * ────────────────────────────────────────────────────────────── */

/**
 * @brief Grupo de eventos para sincronización WiFi
 * @note Usado para esperar conexión o fallo
 */
extern EventGroupHandle_t s_wifi_event_group;

/**
 * @brief Estado de conexión WiFi
 * @note true = conectado, false = desconectado
 */
extern bool wifi_connected;

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES PÚBLICAS
 * ────────────────────────────────────────────────────────────── */

/**
 * @brief Inicializa y conecta a la red WiFi en modo STA
 * @details Configura el WiFi, inicia la conexión y espera
 *          hasta que se conecte o falle.
 * 
 * @note Esta función es bloqueante. Espera hasta que
 *       se establezca la conexión o falle después de
 *       WIFI_MAX_RETRY intentos.
 * 
 * @code
 * // Ejemplo de uso:
 * void app_main(void) {
 *     wifi_init_sta();
 *     if (wifi_connected) {
 *         ESP_LOGI(TAG, "WiFi conectado correctamente");
 *     }
 * }
 * @endcode
 */
void wifi_init_sta(void);

/**
 * @brief Verifica si el WiFi está conectado
 * @return true si está conectado, false si no
 * 
 * @code
 * if (is_wifi_connected()) {
 *     // Enviar datos por WiFi
 * }
 * @endcode
 */
bool is_wifi_connected(void);

/**
 * @brief Obtiene la dirección IP asignada al ESP32
 * @param ip_buffer Buffer donde se almacenará la IP (formato string)
 * @param buffer_size Tamaño del buffer (mínimo 16 bytes para "xxx.xxx.xxx.xxx")
 * @return true si se obtuvo la IP, false si no
 * 
 * @code
 * char ip[16];
 * if (wifi_get_ip(ip, sizeof(ip))) {
 *     ESP_LOGI(TAG, "IP: %s", ip);
 * }
 * @endcode
 */
bool wifi_get_ip(char *ip_buffer, size_t buffer_size);

/**
 * @brief Desconecta el WiFi y libera recursos
 * @details Detiene la conexión WiFi y libera la memoria
 *          utilizada por la interfaz de red.
 */
void wifi_disconnect(void);

/**
 * @brief Reconoce el WiFi después de una desconexión
 * @return true si se inició la reconexión, false si no
 */
bool wifi_reconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */