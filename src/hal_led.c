/**
 * @file hal_led.c
 * @brief Control del LED de alerta.
 */

#include "hal_led.h"
#include "hardware.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "HAL_LED";
static bool s_led_encendido = false;
static bool s_led_parpadeando = false;

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

    ESP_LOGI(TAG, "LED en GPIO %d", PIN_LED_ALERTA);
    return ESP_OK;
}

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

bool hal_led_esta_encendido(void)
{
    return s_led_encendido;
}