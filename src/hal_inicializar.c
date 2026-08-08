/**
 * @file hal_inicializar.c
 * @brief Inicialización de todos los periféricos
 */

#include "hardware.h"
#include "hal_potenciometro.h"
#include "hal_pwm_brushless.h"
#include "hal_led.h"
#include "hal_display.h"
#include "esp_log.h"

static const char *TAG = "HAL";

esp_err_t hal_inicializar(void)
{
    esp_err_t ret = ESP_OK;

    ret = hal_potenciometro_inicializar();
    if (ret != ESP_OK) {
       // ESP_LOGE(TAG, "Error al inicializar potenciómetro");
        return ret;
    }
    //ESP_LOGI(TAG, "Potenciómetro inicializado");

    ret = hal_led_inicializar();
    if (ret != ESP_OK) {
       // ESP_LOGE(TAG, "Error al inicializar LED");
        return ret;
    }
    ESP_LOGI(TAG, "LED de alerta inicializado");

    ret = hal_display_inicializar();
    if (ret != ESP_OK) {
       // ESP_LOGW(TAG, "Pantalla táctil no disponible (continuando...)");
    } else {
       // ESP_LOGI(TAG, "Pantalla táctil inicializada");
    }

   // ESP_LOGI(TAG, "Hardware inicializado correctamente");
    return ESP_OK;
}

esp_err_t hal_motores_inicializar(void)
{
    esp_err_t ret = hal_esc_inicializar();
    if (ret != ESP_OK) {
       // ESP_LOGE(TAG, "Error al inicializar motores");
        return ret;
    }

    hal_esc_parar_motores();

    //ESP_LOGI(TAG, "Motores brushless inicializados");
    return ESP_OK;
}