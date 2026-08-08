/**
 * @file kalman.c
 * @brief Filtro de Kalman para suavizar la señal del potenciómetro.
 */

#include "kalman.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "KALMAN";

void kalman_init(kalman_filter_t *filter, float initial_angle_deg, float dt)
{
    if (filter == NULL) return;
    if (filter->initialized) {
        ESP_LOGW(TAG, "Filtro ya inicializado");
        return;
    }

    if ((isnan(initial_angle_deg) != 0) || (isinf(initial_angle_deg) != 0)) {
        initial_angle_deg = 0.0f;
    }

    if (dt < 0.0001f) dt = 0.01f;

    filter->x[0] = initial_angle_deg;
    filter->x[1] = 0.0f;
    filter->P[0][0] = 0.1f;
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    filter->P[1][1] = 0.01f;
    filter->A[0][0] = 1.0f;
    filter->A[0][1] = -dt;
    filter->A[1][0] = 0.0f;
    filter->A[1][1] = 1.0f;
    filter->B[0] = dt;
    filter->B[1] = 0.0f;
    filter->H[0] = 1.0f;
    filter->H[1] = 0.0f;
    filter->Q_angle_base = KALMAN_Q_ANGLE;
    filter->Q_bias_base = KALMAN_Q_BIAS;
    filter->Q[0][0] = filter->Q_angle_base;
    filter->Q[0][1] = 0.0f;
    filter->Q[1][0] = 0.0f;
    filter->Q[1][1] = filter->Q_bias_base;
    filter->R_measure_base = KALMAN_R_MEASURE;
    filter->R = filter->R_measure_base;
    filter->angle = initial_angle_deg;
    filter->angle_deg = initial_angle_deg;
    filter->bias = 0.0f;
    filter->bias_dps = 0.0f;
    filter->innovation = 0.0f;
    filter->innovation_limit = KALMAN_INNOVATION_LIMIT;
    filter->dt = dt;
    filter->dt_actual = dt;
    filter->init_counter = 0;
    filter->stable_init_done = false;
    filter->initialized = true;

    ESP_LOGI(TAG, "Kalman inicializado: %.2f°", (double)initial_angle_deg);
}

static void kalman_predict(kalman_filter_t *filter, float dt)
{
    if (filter == NULL || !filter->initialized) return;

    filter->A[0][1] = -dt;
    filter->B[0] = dt;
    filter->dt_actual = dt;

    float x0 = filter->x[0];
    float x1 = filter->x[1];
    filter->x[0] = x0 - dt * x1;
    filter->x[1] = x1;

    float P00 = filter->P[0][0];
    float P01 = filter->P[0][1];
    float P10 = filter->P[1][0];
    float P11 = filter->P[1][1];

    filter->P[0][0] = (P00 - dt * P10) - dt * (P10 - dt * P11) + filter->Q[0][0];
    filter->P[0][1] = (P01 - dt * P11) - dt * P11 + filter->Q[0][1];
    filter->P[1][0] = P10 - dt * P11 + filter->Q[1][0];
    filter->P[1][1] = P11 + filter->Q[1][1];
    filter->P[0][1] = filter->P[1][0];
}

static void kalman_update_measurement(kalman_filter_t *filter, float measurement)
{
    if (filter == NULL || !filter->initialized) return;

    float y = measurement - filter->x[0];
    filter->innovation = y;

    if (fabsf(y) > filter->innovation_limit) {
        float sign = (y > 0.0f) ? 1.0f : -1.0f;
        y = sign * filter->innovation_limit;
    }

    float P00 = filter->P[0][0];
    float P01 = filter->P[0][1];
    float P10 = filter->P[1][0];
    float P11 = filter->P[1][1];
    float S = P00 + filter->R;

    if (S < 1e-10f) S = 1e-10f;

    float K0 = P00 / S;
    float K1 = P10 / S;

    filter->x[0] += K0 * y;
    filter->x[1] += K1 * y;

    filter->P[0][0] = P00 - K0 * P00;
    filter->P[0][1] = P01 - K0 * P01;
    filter->P[1][0] = P10 - K1 * P00;
    filter->P[1][1] = P11 - K1 * P01;
    filter->P[0][1] = filter->P[1][0];
}

float kalman_update(kalman_filter_t *filter, float measurement, float dt)
{
    if (filter == NULL || !filter->initialized) return measurement;

    if ((isnan(measurement) != 0) || (isinf(measurement) != 0)) {
        measurement = filter->angle_deg;
    }

    if (dt < 0.0001f) dt = 0.01f;
    if (dt > 0.05f) dt = 0.01f;

    kalman_predict(filter, dt);
    kalman_update_measurement(filter, measurement);

    filter->angle = filter->x[0];
    filter->angle_deg = filter->x[0];
    filter->bias = filter->x[1];
    filter->bias_dps = filter->x[1];
    filter->dt = dt;

    while (filter->angle_deg > 180.0f) filter->angle_deg -= 360.0f;
    while (filter->angle_deg < -180.0f) filter->angle_deg += 360.0f;

    if (filter->init_counter < 100) {
        filter->init_counter++;
        if (filter->init_counter >= 100) {
            filter->stable_init_done = true;
            ESP_LOGI(TAG, "Kalman estabilizado");
        }
    }

    return filter->angle_deg;
}