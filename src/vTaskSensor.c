/**
 * @file vTaskSensor.c
 * @brief Lectura del sensor con filtro Kalman
 */

#include "tareas.h"
#include "hardware.h"
#include "hal_potenciometro.h"
#include "kalman.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "SENSOR";

/* Filtro Kalman */
static kalman_filter_t kalman;
static bool kalman_initialized = false;

void vTaskSensor(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    float initial_angle = hal_potenciometro_leer();
    kalman_init(&kalman, initial_angle, PERIODO_MUESTREO_MS / 1000.0f);
    kalman_initialized = true;

    ESP_LOGI(TAG, "📐 SENSOR INICIADO - Ángulo inicial: %.1f°", initial_angle);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PERIODO_MUESTREO_MS));

        int adc_raw = hal_potenciometro_leer_raw();
        float angle = hal_potenciometro_adc_to_angle(adc_raw);
        float filtered_angle = kalman_update(&kalman, angle, PERIODO_MUESTREO_MS / 1000.0f);

        /* ⭐ LOG CADA 500ms ⭐ */
        static uint32_t ultimo_log = 0U;
        uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if ((ahora - ultimo_log) >= 500U) {
            ESP_LOGI(TAG, "📐 ADC: %4d | Ángulo: %6.1f°", adc_raw, filtered_angle);
            ultimo_log = ahora;
        }

        SensorData_t datos;
        datos.angulo_actual = filtered_angle;
        datos.velocidad = 0.0f;
        datos.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        if (xColaSensorControl != NULL) {
            xQueueOverwrite(xColaSensorControl, &datos);
        }
    }
}