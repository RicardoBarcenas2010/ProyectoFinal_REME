#ifndef HAL_DISPLAY_H
#define HAL_DISPLAY_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────────────────────────────────────────────
 *  EVENTOS TÁCTILES
 * ────────────────────────────────────────────────────────────── */

typedef struct {
    enum {
        TOUCH_BUTTON_SETPOINT,
        TOUCH_BUTTON_PID,
        TOUCH_BUTTON_MODO_AUTO,
        TOUCH_BUTTON_MODO_MANUAL,
        TOUCH_NONE
    } tipo;
    float valor;          /* Para setpoint */
    float kp, ki, kd;     /* Para PID */
} TouchEvent_t;

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES DE PANTALLA
 * ────────────────────────────────────────────────────────────── */

esp_err_t hal_display_inicializar(void);
void hal_display_mostrar_angulo_propio(float angulo);
void hal_display_mostrar_angulo_maestro(float angulo);
void hal_display_mostrar_pwm(float pwm_izq, float pwm_der);
void hal_display_mostrar_modo_seguro(bool activo);
void hal_display_mostrar_estado_tareas(void);
void hal_display_mostrar_mensaje(const char *msg);
esp_err_t hal_display_obtener_evento(TouchEvent_t *evento);
uint32_t hal_display_obtener_heartbeat(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_DISPLAY_H */