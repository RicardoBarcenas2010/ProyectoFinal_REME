/**
 * @file vTaskWatchdogUI.c
 * @brief Watchdog para monitorear la comunicación con la pantalla táctil
 */

#include "tareas.h"
#include "hardware.h"
#include "hal_display.h"
#include "hal_led.h"          /* ← AGREGAR */
#include "esp_log.h"
#include "esp_timer.h"        /* ← AGREGAR */

static const char *TAG = "WATCHDOG";

/* ──────────────────────────────────────────────────────────────
 *  TAREA WATCHDOG UI
 * ────────────────────────────────────────────────────────────── */

void vTaskWatchdogUI(void *pvParameters)
{
    (void)pvParameters;

    uint32_t ultimo_heartbeat = 0U;
    uint32_t timeout_ms = 500U;

    ESP_LOGI(TAG, "🔄 Watchdog UI iniciado (timeout: %d ms)", timeout_ms);

    for (;;) {
        /*
        uint32_t heartbeat_actual = hal_display_obtener_heartbeat();

        if (heartbeat_actual == 0U) {
            ESP_LOGW(TAG, "⚠️ Heartbeat de pantalla = 0");
            vTaskDelay(pdMS_TO_TICKS(100U));
            continue;
        }

        uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t diff = ahora - ultimo_heartbeat;

        if (diff > timeout_ms && !esta_en_modo_seguro()) {
            ESP_LOGE(TAG, "❌ Falla de pantalla detectada (diff: %d ms)", diff);
            activar_modo_seguro(MOTIVO_PANTALLA);
            hal_led_parpadear(500U);
        }

        ultimo_heartbeat = heartbeat_actual;
    */
        vTaskDelay(pdMS_TO_TICKS(100U));
    }
}