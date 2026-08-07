/**
 * @file vTaskMonitor.c
 * @brief Tarea de monitoreo y telemetría
 */

#include "tareas.h"
#include "hardware.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MONITOR";

/* ──────────────────────────────────────────────────────────────
 *  TAREA DE MONITOREO
 * ────────────────────────────────────────────────────────────── */

void vTaskMonitor(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "🔄 Tarea de monitor iniciada");

    for (;;) {
        /* 1. Verificar uso de pila de la tarea actual */
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "📊 Stack libre: %d bytes", uxHighWaterMark);

        /* 2. Verificar estado de las colas */
        if (xColaSensorControl != NULL) {
            UBaseType_t uxItems = uxQueueMessagesWaiting(xColaSensorControl);
            ESP_LOGD(TAG, "Cola Sensor: %d items", uxItems);
        }

        if (xColaVisionControl != NULL) {
            UBaseType_t uxItems = uxQueueMessagesWaiting(xColaVisionControl);
            ESP_LOGD(TAG, "Cola Visión: %d items", uxItems);
        }

        if (xColaControlActuador != NULL) {
            UBaseType_t uxItems = uxQueueMessagesWaiting(xColaControlActuador);
            ESP_LOGD(TAG, "Cola Actuador: %d items", uxItems);
        }

        /* 3. Estado del modo seguro */
        if (esta_en_modo_seguro()) {
            ESP_LOGW(TAG, "⚠️ SISTEMA EN MODO SEGURO");
        }

        /* 4. Heap disponible */
        ESP_LOGI(TAG, "📊 Heap libre: %d bytes", esp_get_free_heap_size());

        vTaskDelay(pdMS_TO_TICKS(5000U));  /* Cada 5 segundos */
    }
}