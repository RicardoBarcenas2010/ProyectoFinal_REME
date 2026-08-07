#ifndef KALMAN_H
#define KALMAN_H

#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef KALMAN_Q_ANGLE
#define KALMAN_Q_ANGLE          0.001f
#endif

#ifndef KALMAN_Q_BIAS
#define KALMAN_Q_BIAS           0.003f
#endif

#ifndef KALMAN_R_MEASURE
#define KALMAN_R_MEASURE        0.03f
#endif

#ifndef KALMAN_INNOVATION_LIMIT
#define KALMAN_INNOVATION_LIMIT 5.0f
#endif

typedef struct {
    float x[2];
    float P[2][2];
    float A[2][2];
    float B[2];
    float H[2];
    float Q[2][2];
    float R;
    float Q_angle_base;
    float Q_bias_base;
    float R_measure_base;
    float angle;
    float angle_deg;
    float bias;
    float bias_dps;
    float innovation;
    float innovation_limit;
    bool initialized;
    float dt;
    float dt_actual;
    int init_counter;
    bool stable_init_done;
} kalman_filter_t;

void kalman_init(kalman_filter_t *filter, float initial_angle_deg, float dt);
float kalman_update(kalman_filter_t *filter, float measurement, float dt);
float kalman_get_angle_deg(kalman_filter_t *filter);
float kalman_get_bias_dps(kalman_filter_t *filter);
void kalman_reset(kalman_filter_t *filter);
bool kalman_is_initialized(kalman_filter_t *filter);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H */