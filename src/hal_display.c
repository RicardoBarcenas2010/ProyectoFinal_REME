/**
 * @file hal_display.c
 * @brief Wrapper para la pantalla táctil.
 * 
 * Esta implementación usa un display dummy para pruebas sin hardware real.
 */

#include "hal_display.h"
#include "hardware.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "HAL_DISPLAY";
static bool s_display_inicializada = false;
static uint32_t s_heartbeat = 0U;

esp_err_t hal_display_inicializar(void)
{
    ESP_LOGW(TAG, "Display dummy (sin hardware)");
    s_display_inicializada = true;
    s_heartbeat = (uint32_t)(esp_timer_get_time() / 1000ULL);
    return ESP_OK;
}

void hal_display_mostrar_angulo_propio(float angulo)
{
    if (!s_display_inicializada) return;
    /* Log opcional */
}

void hal_display_mostrar_angulo_maestro(float angulo)
{
    if (!s_display_inicializada) return;
    /* Log opcional */
}

void hal_display_mostrar_pwm(float pwm_izq, float pwm_der)
{
    if (!s_display_inicializada) return;
    /* Log opcional */
}

void hal_display_mostrar_modo_seguro(bool activo)
{
    if (!s_display_inicializada) return;
    /* Log opcional */
}

void hal_display_mostrar_estado_tareas(void)
{
    if (!s_display_inicializada) return;
    /* Log opcional */
}

void hal_display_mostrar_mensaje(const char *msg)
{
    if (!s_display_inicializada) return;
    /* Log opcional */
}

esp_err_t hal_display_obtener_evento(TouchEvent_t *evento)
{
    if (!s_display_inicializada || evento == NULL) {
        return ESP_FAIL;
    }
    return ESP_ERR_NOT_FOUND;
}

uint32_t hal_display_obtener_heartbeat(void)
{
    s_heartbeat = (uint32_t)(esp_timer_get_time() / 1000ULL);
    return s_heartbeat;
}