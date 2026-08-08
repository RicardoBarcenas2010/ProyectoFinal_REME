/**
 * @file vision.c
 * @brief Módulo de visión artificial para detección de ángulo.
 * 
 * Recibe ángulos vía HTTP desde el sistema Maestro y los envía al controlador
 * mediante una cola RTOS.
 */

#include "vision.h"
#include "http_server.h"
#include "tareas.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define TAG "VISION"
#define LED_PIN 2
#define BUZZER_PIN 12

/* ================================================================
 *  INICIALIZACIÓN
 * ================================================================ */
void init_vision(void)
{
    /* Configurar LED */
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_config);
    gpio_set_level(LED_PIN, 0);

    /* Configurar Buzzer */
    gpio_config_t buzzer_config = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&buzzer_config);
    gpio_set_level(BUZZER_PIN, 0);

    start_webserver();

    ESP_LOGI(TAG, "Módulo de visión inicializado");
}

/* ================================================================
 *  TAREA DE VISIÓN
 * ================================================================ */
void vision_task(void *pvParameters)
{
    float angulo_anterior = 0.0f;
    (void)pvParameters;

    ESP_LOGI(TAG, "Tarea de visión iniciada");

    while (1) {
        float angulo_actual = get_ultimo_angulo();
        int contador = get_contador_peticiones();

        if (angulo_actual != angulo_anterior) {
            ESP_LOGI(TAG, "Ángulo recibido #%d: %.2f°", contador, (double)angulo_actual);

            VisionData_t vision_data;
            vision_data.angulo_maestro = angulo_actual;
            vision_data.confianza = 1.0f;
            vision_data.deteccion_valida = true;
            vision_data.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

            if (xColaVisionControl != NULL) {
                xQueueOverwrite(xColaVisionControl, &vision_data);
            }

            /* LED para ángulos extremos */
            if ((angulo_actual > 45.0f) || (angulo_actual < -45.0f)) {
                gpio_set_level(LED_PIN, 1);
            } else {
                gpio_set_level(LED_PIN, 0);
            }

            /* Buzzer para ángulos muy extremos */
            if ((angulo_actual > 60.0f) || (angulo_actual < -60.0f)) {
                gpio_set_level(BUZZER_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(100U));
                gpio_set_level(BUZZER_PIN, 0);
            }

            angulo_anterior = angulo_actual;
        }

        vTaskDelay(pdMS_TO_TICKS(200U));
    }
}