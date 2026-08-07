#ifndef HAL_POTENCIOMETRO_H
#define HAL_POTENCIOMETRO_H

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_SAMPLES             16

extern adc_oneshot_unit_handle_t adc_handle;

esp_err_t hal_potenciometro_inicializar(void);
int hal_potenciometro_leer_raw(void);
float hal_potenciometro_adc_to_angle(int adc_raw);
float hal_potenciometro_leer(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_POTENCIOMETRO_H */