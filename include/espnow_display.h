#ifndef ESPNOW_DISPLAY_H
#define ESPNOW_DISPLAY_H

#include "esp_err.h"
#include "communication_protocol.h"
#include <stdbool.h>        /* ← AGREGAR ESTE INCLUDE */

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t espnow_display_init(void);

esp_err_t espnow_display_send(const telemetry_packet_t *packet);

/* ⭐ AGREGAR ESTA FUNCIÓN ⭐ */
bool espnow_display_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif