/**
 * @file vTaskActuador.c
 * @brief Control de motores brushless - Pines: IZQ=13, DER=15
 *        CON ARMADO SIMULTÁNEO DE ESC
 */

#include "tareas.h"
#include "hardware.h"
#include "hal_pwm_brushless.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ACTUADOR";

/* ──────────────────────────────────────────────────────────────
 *  CONFIGURACIÓN PWM
 * ────────────────────────────────────────────────────────────── */

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

/* ──────────────────────────────────────────────────────────────
 *  FUNCIÓN PRINCIPAL DE LA TAREA
 * ────────────────────────────────────────────────────────────── */

void vTaskActuador(void *pvParameters)
{
    (void)pvParameters;

    ActuadorData_t datos;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🔧 ACTUADOR - PINES CORRECTOS:");
    ESP_LOGI(TAG, "   Motor IZQUIERDO: GPIO %d", PIN_MOTOR_IZQ_PWM);
    ESP_LOGI(TAG, "   Motor DERECHO:   GPIO %d", PIN_MOTOR_DER_PWM);
    ESP_LOGI(TAG, "========================================");

    /* ⭐⭐⭐ PRUEBA DE ARRANQUE DIRECTO - SIN ESPERAR ⭐⭐⭐ */
    ESP_LOGI(TAG, "🔧 PRUEBA DIRECTA: Motor IZQUIERDO al 20%% por 2 segundos");
    hal_esc_motor_izquierdo(20.0f);
    hal_esc_motor_derecho(0.0f);
    vTaskDelay(pdMS_TO_TICKS(2000U));
    hal_esc_parar_motores();
    vTaskDelay(pdMS_TO_TICKS(500U));

    ESP_LOGI(TAG, "🔧 PRUEBA DIRECTA: Motor DERECHO al 20%% por 2 segundos");
    hal_esc_motor_izquierdo(0.0f);
    hal_esc_motor_derecho(20.0f);
    vTaskDelay(pdMS_TO_TICKS(2000U));
    hal_esc_parar_motores();
    vTaskDelay(pdMS_TO_TICKS(500U));

    ESP_LOGI(TAG, "✅ PRUEBAS DIRECTAS COMPLETADAS");
    ESP_LOGI(TAG, "========================================");

    /* ─── 2. ARMADO SIMULTÁNEO DE ESC ─── */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🔧 INICIANDO ARMADO SIMULTÁNEO DE ESC");
    ESP_LOGI(TAG, "========================================");

    ESP_LOGI(TAG, "🔧 PASO 1: Enviando 0%% a AMBOS motores (%d ms)", TIEMPO_ARMADO_0_MS);
    hal_esc_armar_simultaneo(ARMADO_PWM_0, ARMADO_PWM_0);
    vTaskDelay(pdMS_TO_TICKS(TIEMPO_ARMADO_0_MS));
    ESP_LOGI(TAG, "✅ PASO 1 completado");

    ESP_LOGI(TAG, "🔧 PASO 2: Enviando %.1f%% a AMBOS motores (%d ms)", 
             ARMADO_PWM_START, TIEMPO_ARMADO_START_MS);
    hal_esc_armar_simultaneo(ARMADO_PWM_START, ARMADO_PWM_START);
    vTaskDelay(pdMS_TO_TICKS(TIEMPO_ARMADO_START_MS));
    ESP_LOGI(TAG, "✅ PASO 2 completado");

    ESP_LOGI(TAG, "🔧 PASO 3: Enviando señal de equilibrio");
    ESP_LOGI(TAG, "   IZQUIERDO: %.1f%% | DERECHO: %.1f%%", 
             ARMADO_PWM_BASE_IZQ, ARMADO_PWM_BASE_DER);
    hal_esc_armar_simultaneo(ARMADO_PWM_BASE_IZQ, ARMADO_PWM_BASE_DER);
    ESP_LOGI(TAG, "✅ PASO 3 completado");

    ESP_LOGI(TAG, "✅ ESC ARMADOS CORRECTAMENTE (SIMULTÁNEO)");
    ESP_LOGI(TAG, "========================================");

    /* ─── 3. PRUEBA DE MOTORES - VERIFICACIÓN VISUAL ─── */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🔧 PRUEBA DE MOTORES - VERIFICACIÓN VISUAL");
    ESP_LOGI(TAG, "========================================");

    ESP_LOGI(TAG, "🔧 PRUEBA 1: Motor IZQUIERDO (GPIO %d) al 25%% por 3 segundos", 
             PIN_MOTOR_IZQ_PWM);
    hal_esc_armar_simultaneo(25.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(3000U));
    hal_esc_armar_simultaneo(0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(1000U));

    ESP_LOGI(TAG, "🔧 PRUEBA 2: Motor DERECHO (GPIO %d) al 25%% por 3 segundos", 
             PIN_MOTOR_DER_PWM);
    hal_esc_armar_simultaneo(0.0f, 25.0f);
    vTaskDelay(pdMS_TO_TICKS(3000U));
    hal_esc_armar_simultaneo(0.0f, 0.0f);

    ESP_LOGI(TAG, "✅ PRUEBA COMPLETADA");
    ESP_LOGI(TAG, "========================================");

    /* Volver a la señal de equilibrio */
    ESP_LOGI(TAG, "🔧 Restaurando señal de equilibrio");
    hal_esc_armar_simultaneo(ARMADO_PWM_BASE_IZQ, ARMADO_PWM_BASE_DER);
    vTaskDelay(pdMS_TO_TICKS(500U));

    /* ─── 4. BUCLE PRINCIPAL ─── */
    ESP_LOGI(TAG, "🔄 Entrando en bucle principal de actuación");
    ESP_LOGI(TAG, "========================================");

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
                ESP_LOGI(TAG, "📊 PWM: IZQ=%.1f%% DER=%.1f%%", izq, der);
                ultimo_log = ahora;
            }
        }
    }
}