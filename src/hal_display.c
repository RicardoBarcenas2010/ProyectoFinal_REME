/**
 * @file hal_display.c
 * @brief Wrapper para pantalla táctil (conecta con tu código)
 * 
 * ¡PEGA TU CÓDIGO DE PANTALLA AQUÍ!
 */

#include "hal_display.h"
#include "hardware.h"
#include "esp_log.h"
#include "esp_timer.h"

/* ============================================================
 *  AQUÍ DEBES INCLUIR TU CÓDIGO DE PANTALLA
 *  Ejemplo: #include "tft_ili9341.h"
 * ============================================================ */

static const char *TAG = "HAL_DISPLAY";
static bool s_display_inicializada = false;
static uint32_t s_heartbeat = 0U;

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES QUE DEBES IMPLEMENTAR CON TU CÓDIGO
 *  (Adapta estas funciones a tu librería de pantalla)
 * ────────────────────────────────────────────────────────────── */

/* ============================================================
 *  EJEMPLO: Implementación con ILI9341 (descomentar y adaptar)
 * ============================================================ */

/*
#include "tft_ili9341.h"

esp_err_t hal_display_inicializar(void)
{
    tft_init();
    tft_set_rotation(0);
    tft_fill_screen(COLOR_BLACK);
    s_display_inicializada = true;
    s_heartbeat = esp_timer_get_time() / 1000ULL;
    ESP_LOGI(TAG, "Pantalla ILI9341 inicializada");
    return ESP_OK;
}

void hal_display_mostrar_angulo_propio(float angulo)
{
    if (!s_display_inicializada) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "Propio: %.1f°", angulo);
    tft_draw_text(10, 20, buf, COLOR_WHITE, COLOR_BLACK);
}

void hal_display_mostrar_angulo_maestro(float angulo)
{
    if (!s_display_inicializada) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "Maestro: %.1f°", angulo);
    tft_draw_text(10, 50, buf, COLOR_YELLOW, COLOR_BLACK);
}

void hal_display_mostrar_pwm(float pwm_izq, float pwm_der)
{
    if (!s_display_inicializada) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "PWM: I=%.2f D=%.2f", pwm_izq, pwm_der);
    tft_draw_text(10, 80, buf, COLOR_CYAN, COLOR_BLACK);
}

void hal_display_mostrar_modo_seguro(bool activo)
{
    if (!s_display_inicializada) return;
    if (activo) {
        tft_draw_text(10, 110, "⚠️ MODO SEGURO", COLOR_RED, COLOR_BLACK);
    } else {
        tft_draw_text(10, 110, "✅ Modo Normal    ", COLOR_GREEN, COLOR_BLACK);
    }
}

void hal_display_mostrar_estado_tareas(void)
{
    if (!s_display_inicializada) return;
    tft_draw_text(10, 140, "Tareas: OK", COLOR_WHITE, COLOR_BLACK);
}

void hal_display_mostrar_mensaje(const char *msg)
{
    if (!s_display_inicializada) return;
    tft_draw_text(10, 170, msg, COLOR_WHITE, COLOR_BLACK);
}

esp_err_t hal_display_obtener_evento(TouchEvent_t *evento)
{
    if (!s_display_inicializada || evento == NULL) {
        return ESP_FAIL;
    }
    
    // Implementar lectura de touch
    // Ejemplo:
    // if (tft_touch_get(&x, &y)) {
    //     evento->tipo = TOUCH_BUTTON_SETPOINT;
    //     evento->valor = map_touch_to_angle(x, y);
    //     return ESP_OK;
    // }
    
    return ESP_ERR_NOT_FOUND;
}

uint32_t hal_display_obtener_heartbeat(void)
{
    s_heartbeat = esp_timer_get_time() / 1000ULL;
    return s_heartbeat;
}
*/

/* ──────────────────────────────────────────────────────────────
 *  IMPLEMENTACIÓN POR DEFECTO (sin pantalla real)
 *  - Útil para pruebas sin hardware
 * ────────────────────────────────────────────────────────────── */

esp_err_t hal_display_inicializar(void)
{
    ESP_LOGW(TAG, "⚠️ Usando display dummy (sin hardware)");
    s_display_inicializada = true;
    s_heartbeat = (uint32_t)(esp_timer_get_time() / 1000ULL);
    return ESP_OK;
}

void hal_display_mostrar_angulo_propio(float angulo)
{
    if (!s_display_inicializada) return;
    ESP_LOGI(TAG, "[DISPLAY] Ángulo propio: %.1f°", angulo);
}

void hal_display_mostrar_angulo_maestro(float angulo)
{
    if (!s_display_inicializada) return;
    ESP_LOGI(TAG, "[DISPLAY] Ángulo Maestro: %.1f°", angulo);
}

void hal_display_mostrar_pwm(float pwm_izq, float pwm_der)
{
    if (!s_display_inicializada) return;
    ESP_LOGI(TAG, "[DISPLAY] PWM: I=%.2f D=%.2f", pwm_izq, pwm_der);
}

void hal_display_mostrar_modo_seguro(bool activo)
{
    if (!s_display_inicializada) return;
    if (activo) {
        ESP_LOGW(TAG, "[DISPLAY] ⚠️ MODO SEGURO");
    } else {
        ESP_LOGI(TAG, "[DISPLAY] ✅ Modo Normal");
    }
}

void hal_display_mostrar_estado_tareas(void)
{
    if (!s_display_inicializada) return;
    ESP_LOGD(TAG, "[DISPLAY] Estado tareas: OK");
}

void hal_display_mostrar_mensaje(const char *msg)
{
    if (!s_display_inicializada) return;
    ESP_LOGI(TAG, "[DISPLAY] %s", msg);
}

esp_err_t hal_display_obtener_evento(TouchEvent_t *evento)
{
    if (!s_display_inicializada || evento == NULL) {
        return ESP_FAIL;
    }
    
    /* Simular evento táctil (para pruebas) */
    static int contador = 0;
    contador++;
    if (contador % 100 == 0) {
        evento->tipo = TOUCH_BUTTON_SETPOINT;
        evento->valor = 10.0f;  /* Simular setpoint de 10° */
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

uint32_t hal_display_obtener_heartbeat(void)
{
    s_heartbeat = (uint32_t)(esp_timer_get_time() / 1000ULL);
    return s_heartbeat;
}