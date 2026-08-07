/**
 * @file vision.c
 * @brief Módulo de visión - Recibe ángulo por HTTP (ADAPTADO)
 * 
 * Este es TU CÓDIGO ORIGINAL con añadido de envío a cola RTOS.
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

/* ──────────────────────────────────────────────────────────────
 *  INICIALIZACIÓN (TU CÓDIGO ORIGINAL)
 * ────────────────────────────────────────────────────────────── */

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
    
    /* Iniciar servidor HTTP */
    start_webserver();
    
    ESP_LOGI(TAG, "✅ Módulo de visión inicializado (HTTP Server)");
}

/* ──────────────────────────────────────────────────────────────
 *  TAREA DE VISIÓN (TU angle_control_task ADAPTADA)
 * ────────────────────────────────────────────────────────────── */

void vision_task(void *pvParameters)
{
    float angulo_anterior = 0.0f;
    float angulo_actual = 0.0f;
    int contador = 0;
    
    ESP_LOGI(TAG, "🔄 Tarea de visión iniciada (escuchando HTTP)");
    
    while (1) {
        /* Obtener el último ángulo recibido por HTTP */
        angulo_actual = get_ultimo_angulo();
        contador = get_contador_peticiones();
        
        /* Solo procesar si hay cambios */
        if (angulo_actual != angulo_anterior) {
            ESP_LOGI(TAG, "🎯 Ángulo recibido #%d: %.2f°", contador, angulo_actual);
            
            /* ============================================================
             *  🆕 ENVIAR ÁNGULO AL CONTROLADOR PID VÍA COLA RTOS
             * ============================================================ */
            VisionData_t vision_data;
            vision_data.angulo_maestro = angulo_actual;
            vision_data.confianza = 1.0f;
            vision_data.deteccion_valida = true;
            vision_data.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            
            if (xColaVisionControl != NULL) {
                xQueueOverwrite(xColaVisionControl, &vision_data);
                ESP_LOGD(TAG, "📤 Ángulo enviado a cola de control");
            }
            
            /* ============================================================
             *  TU CÓDIGO ORIGINAL - LED y Buzzer
             * ============================================================ */
            
            /* LED */
            if (angulo_actual > 45.0f || angulo_actual < -45.0f) {
                gpio_set_level(LED_PIN, 1);
                ESP_LOGI(TAG, "🔴 LED ENCENDIDO");
            } else {
                gpio_set_level(LED_PIN, 0);
                ESP_LOGI(TAG, "🟢 LED APAGADO");
            }
            
            /* Buzzer (ángulo extremo) */
            if (angulo_actual > 60.0f || angulo_actual < -60.0f) {
                gpio_set_level(BUZZER_PIN, 1);
                ESP_LOGI(TAG, "🔊 ALERTA! Ángulo extremo: %.1f°", angulo_actual);
                vTaskDelay(pdMS_TO_TICKS(100U));
                gpio_set_level(BUZZER_PIN, 0);
            }
            
            angulo_anterior = angulo_actual;
        }
        
        vTaskDelay(pdMS_TO_TICKS(200U));
    }
}

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES ADICIONALES
 * ────────────────────────────────────────────────────────────── */

float vision_obtener_ultimo_angulo(void)
{
    return get_ultimo_angulo();
}

int vision_obtener_contador(void)
{
    return get_contador_peticiones();
}