/**
 * @file hal_led.c
 * @brief Control del LED de alerta
 */

#include "hal_led.h"
#include "hardware.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "HAL_LED";
static bool s_led_encendido = false;
static bool s_led_parpadeando = false;
static uint32_t s_ultimo_cambio = 0U;

/* ──────────────────────────────────────────────────────────────
 *  INICIALIZACIÓN
 * ────────────────────────────────────────────────────────────── */

esp_err_t hal_led_inicializar(void)
{
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << PIN_LED_ALERTA),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    gpio_set_level(PIN_LED_ALERTA, 0);
    s_led_encendido = false;
    s_led_parpadeando = false;

    ESP_LOGI(TAG, "LED inicializado en GPIO %d", PIN_LED_ALERTA);
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────────────
 *  CONTROL DEL LED
 * ────────────────────────────────────────────────────────────── */

void hal_led_encender(void)
{
    gpio_set_level(PIN_LED_ALERTA, 1);
    s_led_encendido = true;
    s_led_parpadeando = false;
}

void hal_led_apagar(void)
{
    gpio_set_level(PIN_LED_ALERTA, 0);
    s_led_encendido = false;
    s_led_parpadeando = false;
}

void hal_led_parpadear(uint32_t intervalo_ms)  /* ← FUNCIÓN QUE FALTA */
{
    s_led_parpadeando = true;
    
    uint32_t ahora = (uint32_t)(esp_timer_get_time() / 1000ULL);
    
    if ((ahora - s_ultimo_cambio) >= intervalo_ms) {
        if (s_led_encendido) {
            gpio_set_level(PIN_LED_ALERTA, 0);
            s_led_encendido = false;
        } else {
            gpio_set_level(PIN_LED_ALERTA, 1);
            s_led_encendido = true;
        }
        s_ultimo_cambio = ahora;
    }
}

bool hal_led_esta_encendido(void)
{
    return s_led_encendido;
}