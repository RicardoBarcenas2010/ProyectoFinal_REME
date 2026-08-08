/**
 * @file vTaskActuador.c
 * @brief Control de motores brushless mediante PWM.
 * 
 * Esta tarea recibe los valores de PWM calculados por el controlador
 * y los aplica a los motores. Incluye secuencia de armado de ESC.
 */

#include "tareas.h"
#include "hardware.h"
#include "hal_pwm_brushless.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ACTUADOR";

/* Configuración de PWM */
#define PWM_BASE_IZQ            22.0f
#define PWM_BASE_DER            24.0f
#define PWM_MIN_IZQ             20.0f
#define PWM_MIN_DER             20.0f
#define PWM_MAX_IZQ             30.0f
#define PWM_MAX_DER             30.0f

#define ARMADO_PWM_0            0.0f
#define ARMADO_PWM_START        15.5f
#define ARMADO_PWM_BASE_IZQ     PWM_BASE_IZQ
#define ARMADO_PWM_BASE_DER     PWM_BASE_DER

#define TIEMPO_ARMADO_0_MS      3000
#define TIEMPO_ARMADO_START_MS  2000

/* ================================================================
 *  TAREA PRINCIPAL
 * ================================================================ */
void vTaskActuador(void *pvParameters)
{
    (void)pvParameters;

    ActuadorData_t datos;

    ESP_LOGI(TAG, "Actuador iniciado - IZQ:GPIO%d DER:GPIO%d",
             PIN_MOTOR_IZQ_PWM, PIN_MOTOR_DER_PWM);

    /* Secuencia de armado de ESC */
    ESP_LOGI(TAG, "Armando ESC...");
    
    ESP_LOGI(TAG, "Paso 1: 0%% por %d ms", TIEMPO_ARMADO_0_MS);
    hal_esc_armar_simultaneo(ARMADO_PWM_0, ARMADO_PWM_0);
    vTaskDelay(pdMS_TO_TICKS(TIEMPO_ARMADO_0_MS));

    ESP_LOGI(TAG, "Paso 2: %.1f%% por %d ms", ARMADO_PWM_START, TIEMPO_ARMADO_START_MS);
    hal_esc_armar_simultaneo(ARMADO_PWM_START, ARMADO_PWM_START);
    vTaskDelay(pdMS_TO_TICKS(TIEMPO_ARMADO_START_MS));

    ESP_LOGI(TAG, "Paso 3: señal de equilibrio");
    hal_esc_armar_simultaneo(ARMADO_PWM_BASE_IZQ, ARMADO_PWM_BASE_DER);
    vTaskDelay(pdMS_TO_TICKS(500U));

    ESP_LOGI(TAG, "ESC armados");

    /* Bucle principal */
    for (;;) {
        if (xQueueReceive(xColaControlActuador, &datos, portMAX_DELAY) == pdPASS) {
            float izq = datos.pwm_izquierdo;
            float der = datos.pwm_derecho;

            if (izq < PWM_MIN_IZQ) izq = PWM_MIN_IZQ;
            if (izq > PWM_MAX_IZQ) izq = PWM_MAX_IZQ;
            if (der < PWM_MIN_DER) der = PWM_MIN_DER;
            if (der > PWM_MAX_DER) der = PWM_MAX_DER;

            hal_esc_motor_izquierdo(izq);
            hal_esc_motor_derecho(der);

            static uint32_t ultimo_log = 0U;
            uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
            if ((ahora - ultimo_log) >= 2000U) {
                ESP_LOGI(TAG, "PWM: IZQ=%.1f%% DER=%.1f%%", izq, der);
                ultimo_log = ahora;
            }
        }
    }
}