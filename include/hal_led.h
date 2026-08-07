#ifndef HAL_LED_H
#define HAL_LED_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t hal_led_inicializar(void);
void hal_led_encender(void);
void hal_led_apagar(void);
void hal_led_parpadear(uint32_t intervalo_ms);
bool hal_led_esta_encendido(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_LED_H */