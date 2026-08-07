/**
 * @file vTaskWatchdogUI.c
 * @brief Watchdog para monitorear la comunicación con la pantalla táctil
 */

#include "tareas.h"
#include "hardware.h"
#include "hal_display.h"
#include "hal_led.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "espnow_display.h"    /* ← AGREGAR */

static const char *TAG = "WATCHDOG";

void vTaskWatchdogUI(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "🔄 Watchdog UI iniciado");

    for (;;) {
        /* Verificar conexión de la pantalla */
        if (!espnow_display_is_connected()) {
            /* Si la pantalla está desconectada, encender LED */
            hal_led_encender();
            ESP_LOGW(TAG, "⚠️ Watchdog: Pantalla DESCONECTADA - LED encendido");
        } else {
            hal_led_apagar();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100U));
    }
}