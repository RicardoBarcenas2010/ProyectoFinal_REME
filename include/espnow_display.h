#ifndef ESPNOW_DISPLAY_H
#define ESPNOW_DISPLAY_H

#include "esp_err.h"
#include "communication_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t espnow_display_init(void);

esp_err_t espnow_display_send(const telemetry_packet_t *packet);

#ifdef __cplusplus
}
#endif

#endif