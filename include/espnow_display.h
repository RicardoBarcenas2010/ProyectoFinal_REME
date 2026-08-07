#ifndef ESPNOW_DISPLAY_H
#define ESPNOW_DISPLAY_H

#include "esp_err.h"
#include "communication_protocol.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t espnow_display_init(void);

esp_err_t espnow_display_send(const telemetry_packet_t *packet);

/**
 * @brief Verifica si la pantalla está conectada
 * @return true si está conectada (recibe ESP-NOW OK), false si no
 */
bool espnow_display_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif