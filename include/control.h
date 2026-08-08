#ifndef CONTROL_H
#define CONTROL_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el controlador PID
 * @return ESP_OK si éxito, otro valor si error
 */
esp_err_t control_inicializar(void);

/**
 * @brief Actualiza los parámetros PID (función de reserva)
 * @param kp Ganancia proporcional
 * @param ki Ganancia integral
 * @param kd Ganancia derivativa
 */
void control_actualizar_pid(float kp, float ki, float kd);

/**
 * @brief Obtiene el setpoint actual
 * @return Setpoint en grados
 */
float control_obtener_setpoint(void);

/**
 * @brief Establece el modo automático
 * @param modo_auto true = automático, false = manual
 */
void control_set_modo_auto(bool modo_auto);

/* ⭐ FUNCIONES ELIMINADAS (ahora son estáticas en vTaskControl.c) ⭐ */
/* void control_fijar_setpoint(float setpoint); */
/* void control_resetear(void); */

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */