#ifndef ESPNOW_RECEIVER_H
#define ESPNOW_RECEIVER_H

#include "esp_err.h"
#include <stdint.h>

extern volatile uint8_t g_control_mode;
extern volatile float g_manual_setpoint;

esp_err_t espnow_receiver_init(void);

#endif