/**
 * @file hal_pwm_brushless.c
 * @brief Control de motores brushless vía PWM (LEDC)
 *        Pines: IZQ=GPIO13, DER=GPIO15
 */

#include "hal_pwm_brushless.h"
#include "hardware.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdint.h>

static const char *TAG = "HAL_PWM";

/* ──────────────────────────────────────────────────────────────
 *  CONFIGURACIÓN LEDC
 * ────────────────────────────────────────────────────────────── */

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RESOLUTION    LEDC_TIMER_14_BIT
#define LEDC_MAX_DUTY           16383UL

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES AUXILIARES
 * ────────────────────────────────────────────────────────────── */

static float clamp_float(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static uint32_t percent_to_duty(float percent)
{
    percent = clamp_float(percent, 0.0f, 100.0f);
    float pulse_us = ESC_MIN_PULSE_US + (percent / 100.0f) * (ESC_MAX_PULSE_US - ESC_MIN_PULSE_US);
    return (uint32_t)((pulse_us / ESC_PERIOD_US) * (float)LEDC_MAX_DUTY);
}

/* ──────────────────────────────────────────────────────────────
 *  INICIALIZACIÓN
 * ────────────────────────────────────────────────────────────── */

esp_err_t hal_esc_inicializar(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🔧 INICIALIZANDO ESC");
    ESP_LOGI(TAG, "   Motor IZQUIERDO: GPIO %d", PIN_MOTOR_IZQ_PWM);
    ESP_LOGI(TAG, "   Motor DERECHO:   GPIO %d", PIN_MOTOR_DER_PWM);
    ESP_LOGI(TAG, "========================================");

    /* Configurar timer LEDC */
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = ESC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    /* Configurar canal izquierdo (GPIO 13) */
    ledc_channel_config_t left_config = {
        .gpio_num = PIN_MOTOR_IZQ_PWM,      /* GPIO 13 */
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = percent_to_duty(0.0f),
        .hpoint = 0,
        .flags.output_invert = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_config));
    ESP_LOGI(TAG, "✅ Canal IZQUIERDO (GPIO %d) configurado", PIN_MOTOR_IZQ_PWM);

    /* Configurar canal derecho (GPIO 15) */
    ledc_channel_config_t right_config = {
        .gpio_num = PIN_MOTOR_DER_PWM,      /* GPIO 15 */
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = percent_to_duty(0.0f),
        .hpoint = 0,
        .flags.output_invert = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_config));
    ESP_LOGI(TAG, "✅ Canal DERECHO (GPIO %d) configurado", PIN_MOTOR_DER_PWM);

    /* Detener motores */
    hal_esc_parar_motores();

    ESP_LOGI(TAG, "✅ ESC inicializado correctamente");
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────
 *  CONTROL DE MOTORES
 * ────────────────────────────────────────────────────────────── */

void hal_esc_motor_izquierdo(float velocidad)
{
    uint32_t duty = percent_to_duty(velocidad);
    
    static uint32_t ultimo_log = 0U;
    uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((ahora - ultimo_log) >= 1000U) {
        float pulse_us = ESC_MIN_PULSE_US + (velocidad / 100.0f) * (ESC_MAX_PULSE_US - ESC_MIN_PULSE_US);
        ESP_LOGI(TAG, "🔧 IZQ(GPIO%d): %.1f%% → %.0f µs → duty %d", 
                 PIN_MOTOR_IZQ_PWM, velocidad, pulse_us, duty);
        ultimo_log = ahora;
    }
    
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_0);
}

void hal_esc_motor_derecho(float velocidad)
{
    uint32_t duty = percent_to_duty(velocidad);
    
    static uint32_t ultimo_log = 0U;
    uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((ahora - ultimo_log) >= 1000U) {
        float pulse_us = ESC_MIN_PULSE_US + (velocidad / 100.0f) * (ESC_MAX_PULSE_US - ESC_MIN_PULSE_US);
        ESP_LOGI(TAG, "🔧 DER(GPIO%d): %.1f%% → %.0f µs → duty %d", 
                 PIN_MOTOR_DER_PWM, velocidad, pulse_us, duty);
        ultimo_log = ahora;
    }
    
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_1);
}

void hal_esc_parar_motores(void)
{
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_0, percent_to_duty(0.0f));
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_0);
    
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_1, percent_to_duty(0.0f));
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_1);
    
    ESP_LOGI(TAG, "⏹ Motores detenidos (GPIO %d y %d)", PIN_MOTOR_IZQ_PWM, PIN_MOTOR_DER_PWM);
}

/* ⭐ NUEVA FUNCIÓN: ARMADO SIMULTÁNEO ⭐ */
void hal_esc_armar_simultaneo(float percent_izq, float percent_der)
{
    uint32_t duty_izq = percent_to_duty(percent_izq);
    uint32_t duty_der = percent_to_duty(percent_der);
    
    /* Escribir ambos PWM */
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_0, duty_izq);
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_1, duty_der);
    
    /* Actualizar ambos canales al mismo tiempo */
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_1);
    
    static uint32_t ultimo_log = 0U;
    uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((ahora - ultimo_log) >= 1000U) {
        ESP_LOGI(TAG, "🔧 SIMULTÁNEO: IZQ=%.1f%% DER=%.1f%%", percent_izq, percent_der);
        ultimo_log = ahora;
    }
}