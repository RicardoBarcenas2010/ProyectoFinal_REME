#ifndef HAL_PWM_BRUSHLESS_H
#define HAL_PWM_BRUSHLESS_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa los ESC (Electronic Speed Controllers)
 * @return ESP_OK si éxito, otro valor si error
 */
esp_err_t hal_esc_inicializar(void);

/**
 * @brief Controla el motor izquierdo (GPIO 13)
 * @param velocidad 0.0 a 100.0 (%)
 */
void hal_esc_motor_izquierdo(float velocidad);

/**
 * @brief Controla el motor derecho (GPIO 15)
 * @param velocidad 0.0 a 100.0 (%)
 */
void hal_esc_motor_derecho(float velocidad);

/**
 * @brief Detiene ambos motores (0%)
 */
void hal_esc_parar_motores(void);

/**
 * @brief Arma ambos ESC simultáneamente
 * @param percent_izq 0.0 a 100.0 (%)
 * @param percent_der 0.0 a 100.0 (%)
 */
void hal_esc_armar_simultaneo(float percent_izq, float percent_der);

#ifdef __cplusplus
}
#endif

#endif /* HAL_PWM_BRUSHLESS_H */