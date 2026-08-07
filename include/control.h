#ifndef CONTROL_H
#define CONTROL_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t control_inicializar(void);
void control_actualizar_pid(float kp, float ki, float kd);
float control_obtener_setpoint(void);
void control_fijar_setpoint(float setpoint);
void control_set_modo_auto(bool modo_auto);  /* ← 'auto' es palabra reservada en C++ */
void control_resetear(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */