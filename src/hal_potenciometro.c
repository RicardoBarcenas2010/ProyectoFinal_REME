/**
 * @file hal_potenciometro.c
 * @brief Lectura del potenciómetro con promedio y calibración
 */

#include "hal_potenciometro.h"
#include "hardware.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include <math.h>

static const char *TAG = "HAL_POT";
adc_oneshot_unit_handle_t adc_handle = NULL;

/* ================================================================
 *  CALIBRACIÓN ADC -> ÁNGULO
 * ================================================================ */
typedef struct {
    float angle_deg;
    float adc_value;
} calibration_point_t;

static const calibration_point_t angle_calibration[] = {
    {-40.0f, 2800.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    {-35.0f, 2730.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    {-30.0f, 2640.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    {-25.0f, 2540.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    {-20.0f, 2460.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    {-15.0f, 2362.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    {-10.0f, 2278.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    { -5.0f, 2197.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    {  0.0f, 2095.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    {  5.0f, 1981.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    { 10.0f,1900.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    { 15.0f, 1836.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    { 20.0f, 1745.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    { 25.0f, 1685.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    { 30.0f, 1608.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    { 35.0f, 1520.00f},  /* ← REEMPLAZA CON TU VALOR MEDIDO */
    { 40.0f, 1430.00f}   /* ← REEMPLAZA CON TU VALOR MEDIDO */
};

/* ================================================================
 *  FUNCIONES AUXILIARES
 * ================================================================ */

static float linear_interpolation(float x, float x1, float y1, float x2, float y2) {
    if (fabsf(x2 - x1) < 0.000001f) return y1;
    return y1 + ((x - x1) * (y2 - y1)) / (x2 - x1);
}

/* ================================================================
 *  INICIALIZACIÓN
 * ================================================================ */

esp_err_t hal_potenciometro_inicializar(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar ADC: %d", ret);
        return ret;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ret = adc_oneshot_config_channel(adc_handle, ADC_POT_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar canal: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "ADC inicializado en GPIO %d (Canal %d)", PIN_POTENCIOMETRO, ADC_POT_CHANNEL);
    return ESP_OK;
}

/* ================================================================
 *  LECTURA RAW (VALOR ADC PROMEDIADO)
 * ================================================================ */

int hal_potenciometro_leer_raw(void)
{
    int raw = 0;
    int valid_samples = 0;
    int32_t sum = 0;

    for (int sample = 0; sample < ADC_SAMPLES; sample++) {
        esp_err_t ret = adc_oneshot_read(adc_handle, ADC_POT_CHANNEL, &raw);
        if (ret == ESP_OK) {
            sum += raw;
            valid_samples++;
        }
    }

    if (valid_samples == 0) return 0;
    return (int)(sum / valid_samples);
}

/* ================================================================
 *  CONVERSIÓN ADC -> ÁNGULO
 * ================================================================ */

float hal_potenciometro_adc_to_angle(int adc_raw)
{
    const size_t count = sizeof(angle_calibration) / sizeof(angle_calibration[0]);
    const float adc = (float)adc_raw;

    if (adc >= angle_calibration[0].adc_value) {
        return linear_interpolation(adc,
            angle_calibration[0].adc_value, angle_calibration[0].angle_deg,
            angle_calibration[1].adc_value, angle_calibration[1].angle_deg);
    }

    if (adc <= angle_calibration[count - 1].adc_value) {
        return linear_interpolation(adc,
            angle_calibration[count - 2].adc_value, angle_calibration[count - 2].angle_deg,
            angle_calibration[count - 1].adc_value, angle_calibration[count - 1].angle_deg);
    }

    for (size_t idx = 0; idx < count - 1; idx++) {
        float adc_first = angle_calibration[idx].adc_value;
        float adc_second = angle_calibration[idx + 1].adc_value;
        if ((adc <= adc_first) && (adc >= adc_second)) {
            return linear_interpolation(adc,
                adc_first, angle_calibration[idx].angle_deg,
                adc_second, angle_calibration[idx + 1].angle_deg);
        }
    }
    return 0.0f;
}

/* ================================================================
 *  LECTURA COMPLETA (RAW + CONVERSIÓN)
 * ================================================================ */

float hal_potenciometro_leer(void)
{
    int adc_raw = hal_potenciometro_leer_raw();
    return hal_potenciometro_adc_to_angle(adc_raw);
}