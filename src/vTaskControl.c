/**
 * @file vTaskControl.c
 * @brief Controlador PID del sistema seguidor.
 * 
 * Esta tarea se ejecuta periódicamente para calcular la señal de control
 * basada en el error entre el ángulo deseado (setpoint) y el ángulo actual.
 * El setpoint puede provenir de la cámara o ser fijo (0°).
 * 
 * Si se pierde la comunicación con la pantalla, el sistema entra en modo seguro:
 * - Setpoint forzado a 0°
 * - LED de alerta encendido
 */

#include "tareas.h"
#include "hardware.h"
#include "control.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include "driver/uart.h"
#include "vision.h"
#include "communication_protocol.h"
#include "espnow_display.h"
#include "espnow_receiver.h"
#include "hal_led.h"

static const char *TAG = "CONTROL";

/* ================================================================
 *  CONFIGURACIÓN UART
 * ================================================================ */
#define UART_NUM                UART_NUM_0
#define RX_BUF_SIZE             64

/* ================================================================
 *  VARIABLES ESTÁTICAS DEL CONTROLADOR
 * ================================================================ */
static ControlPID_t s_ctrl;
static float s_filtered_angle = 0.0f;
static int64_t s_last_update_us = 0;

/* ================================================================
 *  ESTADO DE LA CÁMARA
 * ================================================================ */
static float s_ultimo_angulo_camara = 0.0f;
static uint32_t s_ultimo_timestamp_camara = 0U;
static bool s_tiene_datos_camara = false;

/* ================================================================
 *  PROTOTIPOS DE FUNCIONES ESTÁTICAS
 * ================================================================ */
static void leer_comando_teclado(void);
static float clamp_float(float value, float min, float max);
static float slew_limit(float target, float previous, float max_step);
static const char* mode_to_string(operating_mode_t mode);
static const char* state_to_string(control_state_t state);
static void control_fijar_setpoint(float setpoint);
static void control_resetear(void);
static operating_mode_t update_operating_mode(operating_mode_t current_mode, float reference_deg);
static void select_parameters(operating_mode_t mode, float *kp, float *ki, float *kd,
                              float *integral_max, float *output_max,
                              float *pwm_left_min, float *pwm_left_max,
                              float *pwm_right_min, float *pwm_right_max);
static float calculate_velocity(float current_angle, int64_t current_time_us,
                                float *angle_history, int64_t *time_history,
                                size_t *history_index, size_t *history_count);
static control_state_t control_pid_update(float angle_actual, float dt_s,
                                          float *pwm_left, float *pwm_right);

/* ================================================================
 *  LECTURA DE COMANDOS POR UART
 * ================================================================ */
static void leer_comando_teclado(void)
{
    uint8_t data[RX_BUF_SIZE] = {0};
    int bytes_available = 0;
    int len = 0;

    uart_get_buffered_data_len(UART_NUM, (size_t*)&bytes_available);

    if (bytes_available == 0) {
        return;
    }

    len = uart_read_bytes(UART_NUM, data, 1, pdMS_TO_TICKS(10));

    if (len > 0) {
        char comando = (char)data[0];

        switch (comando) {
            case 'r':
                control_resetear();
                ESP_LOGI(TAG, "PID reseteado");
                break;
            case 'h':
                ESP_LOGI(TAG, "Controles: 'r' reset, 'h' ayuda");
                break;
            default:
                break;
        }
    }
}

/* ================================================================
 *  FUNCIONES AUXILIARES
 * ================================================================ */
static float clamp_float(float value, float min, float max)
{
    float result = value;

    if (value < min) {
        result = min;
    } else if (value > max) {
        result = max;
    }

    return result;
}

static float slew_limit(float target, float previous, float max_step)
{
    float diff = target - previous;
    float result = previous;

    if (diff > max_step) {
        result = previous + max_step;
    } else if (diff < -max_step) {
        result = previous - max_step;
    } else {
        result = target;
    }

    return result;
}

static const char* mode_to_string(operating_mode_t mode)
{
    const char* result = "DESCONOCIDO";

    switch (mode) {
        case OPERATING_MODE_NORMAL:
            result = "NORMAL";
            break;
        case OPERATING_MODE_HIGH_POSITIVE:
            result = "ALTO_POS";
            break;
        case OPERATING_MODE_HIGH_NEGATIVE:
            result = "ALTO_NEG";
            break;
        default:
            break;
    }

    return result;
}

static const char* state_to_string(control_state_t state)
{
    const char* result = "DESCONOCIDO";

    switch (state) {
        case STATE_NORMAL:
            result = "NORMAL";
            break;
        case STATE_EMERGENCY_POSITIVE:
            result = "EMERG_POS";
            break;
        case STATE_EMERGENCY_NEGATIVE:
            result = "EMERG_NEG";
            break;
        default:
            break;
    }

    return result;
}

static void control_fijar_setpoint(float setpoint)
{
    s_ctrl.target_reference = setpoint;
}

static void control_resetear(void)
{
    s_ctrl.integral = 0.0f;
    s_ctrl.error_anterior = 0.0f;
    s_ctrl.ramped_reference = s_ctrl.target_reference;
    s_ctrl.filtered_velocity = 0.0f;
    s_ctrl.pwm_left_prev = PWM_LEFT_BASE;
    s_ctrl.pwm_right_prev = PWM_RIGHT_BASE;
    s_ctrl.state = STATE_NORMAL;
    s_ctrl.operating_mode = OPERATING_MODE_NORMAL;
    ESP_LOGI(TAG, "PID reseteado");
}

/* ================================================================
 *  SELECCIÓN DE MODO DE OPERACIÓN
 * ================================================================ */
static operating_mode_t update_operating_mode(
    operating_mode_t current_mode,
    float reference_deg)
{
    operating_mode_t new_mode = current_mode;

    switch (current_mode) {
        case OPERATING_MODE_NORMAL:
            if (reference_deg >= HIGH_MODE_ENTER_DEG) {
                new_mode = OPERATING_MODE_HIGH_POSITIVE;
            } else if (reference_deg <= -HIGH_MODE_ENTER_DEG) {
                new_mode = OPERATING_MODE_HIGH_NEGATIVE;
            } else {
                new_mode = OPERATING_MODE_NORMAL;
            }
            break;

        case OPERATING_MODE_HIGH_POSITIVE:
            if (reference_deg <= -HIGH_MODE_ENTER_DEG) {
                new_mode = OPERATING_MODE_HIGH_NEGATIVE;
            } else if (reference_deg <= HIGH_MODE_EXIT_DEG) {
                new_mode = OPERATING_MODE_NORMAL;
            } else {
                new_mode = OPERATING_MODE_HIGH_POSITIVE;
            }
            break;

        case OPERATING_MODE_HIGH_NEGATIVE:
            if (reference_deg >= HIGH_MODE_ENTER_DEG) {
                new_mode = OPERATING_MODE_HIGH_POSITIVE;
            } else if (reference_deg >= -HIGH_MODE_EXIT_DEG) {
                new_mode = OPERATING_MODE_NORMAL;
            } else {
                new_mode = OPERATING_MODE_HIGH_NEGATIVE;
            }
            break;

        default:
            new_mode = OPERATING_MODE_NORMAL;
            break;
    }

    return new_mode;
}

/* ================================================================
 *  SELECCIÓN DE PARÁMETROS PID SEGÚN MODO
 * ================================================================ */
static void select_parameters(
    operating_mode_t mode,
    float *kp, float *ki, float *kd,
    float *integral_max, float *output_max,
    float *pwm_left_min, float *pwm_left_max,
    float *pwm_right_min, float *pwm_right_max)
{
    /* Valores por defecto: modo normal */
    *kp = NORMAL_KP;
    *ki = NORMAL_KI;
    *kd = NORMAL_KD;
    *integral_max = NORMAL_INTEGRAL_MAX;
    *output_max = NORMAL_OUTPUT_MAX;
    *pwm_left_min = PWM_LEFT_MIN;
    *pwm_left_max = PWM_LEFT_MAX;
    *pwm_right_min = PWM_RIGHT_MIN;
    *pwm_right_max = PWM_RIGHT_MAX;

    switch (mode) {
        case OPERATING_MODE_HIGH_POSITIVE:
            *kp = HIGH_POSITIVE_KP;
            *ki = HIGH_POSITIVE_KI;
            *kd = HIGH_POSITIVE_KD;
            *integral_max = HIGH_POSITIVE_INTEGRAL_MAX;
            *output_max = HIGH_POSITIVE_OUTPUT_MAX;
            *pwm_left_min = HIGH_POSITIVE_PWM_LEFT_MIN;
            *pwm_right_max = HIGH_POSITIVE_PWM_RIGHT_MAX;
            break;

        case OPERATING_MODE_HIGH_NEGATIVE:
            *kp = HIGH_NEGATIVE_KP;
            *ki = HIGH_NEGATIVE_KI;
            *kd = HIGH_NEGATIVE_KD;
            *integral_max = HIGH_NEGATIVE_INTEGRAL_MAX;
            *output_max = HIGH_NEGATIVE_OUTPUT_MAX;
            *pwm_left_max = HIGH_NEGATIVE_PWM_LEFT_MAX;
            *pwm_right_min = HIGH_NEGATIVE_PWM_RIGHT_MIN;
            break;

        default:
            break;
    }
}

/* ================================================================
 *  CÁLCULO DE VELOCIDAD ANGULAR
 * ================================================================ */
static float calculate_velocity(
    float current_angle,
    int64_t current_time_us,
    float *angle_history,
    int64_t *time_history,
    size_t *history_index,
    size_t *history_count)
{
    float velocity = 0.0f;

    if (*history_count >= VELOCITY_WINDOW_SAMPLES) {
        float old_angle = angle_history[*history_index];
        int64_t old_time = time_history[*history_index];
        float dt = (float)(current_time_us - old_time) / 1000000.0f;

        if (dt > 0.0f) {
            velocity = (current_angle - old_angle) / dt;
        }
    }

    angle_history[*history_index] = current_angle;
    time_history[*history_index] = current_time_us;
    *history_index = (*history_index + 1) % VELOCITY_WINDOW_SAMPLES;

    if (*history_count < VELOCITY_WINDOW_SAMPLES) {
        (*history_count)++;
    }

    return velocity;
}

/* ================================================================
 *  FUNCIONES PÚBLICAS (declaradas en control.h)
 * ================================================================ */

esp_err_t control_inicializar(void)
{
    int64_t now = 0;
    size_t i = 0U;

    memset(&s_ctrl, 0, sizeof(s_ctrl));

    s_ctrl.operating_mode = OPERATING_MODE_NORMAL;
    s_ctrl.state = STATE_NORMAL;
    s_ctrl.ramped_reference = 0.0f;
    s_ctrl.target_reference = 0.0f;
    s_ctrl.filtered_velocity = 0.0f;
    s_ctrl.pwm_left_prev = PWM_LEFT_BASE;
    s_ctrl.pwm_right_prev = PWM_RIGHT_BASE;
    s_ctrl.initialized = true;

    now = esp_timer_get_time();
    for (i = 0U; i < VELOCITY_WINDOW_SAMPLES; i++) {
        s_ctrl.angle_history[i] = 0.0f;
        s_ctrl.time_history[i] = now;
    }

    s_last_update_us = now;

    ESP_LOGI(TAG, "Controlador PID inicializado");
    ESP_LOGI(TAG, "Modo normal: KP=%.2f KI=%.2f KD=%.2f",
             NORMAL_KP, NORMAL_KI, NORMAL_KD);

    return ESP_OK;
}

void control_actualizar_pid(float kp, float ki, float kd)
{
    (void)kp;
    (void)ki;
    (void)kd;
}

float control_obtener_setpoint(void)
{
    return s_ctrl.target_reference;
}

void control_set_modo_auto(bool modo_auto)
{
    (void)modo_auto;
}

operating_mode_t control_obtener_modo(void)
{
    return s_ctrl.operating_mode;
}

control_state_t control_obtener_estado(void)
{
    return s_ctrl.state;
}

float control_obtener_angulo_filtrado(void)
{
    return s_filtered_angle;
}

/* ================================================================
 *  ACTUALIZACIÓN DEL CONTROLADOR PID
 * ================================================================ */
static control_state_t control_pid_update(
    float angle_actual,
    float dt_s,
    float *pwm_left,
    float *pwm_right)
{
    control_state_t state_result = STATE_NORMAL;
    int64_t now = 0;
    float max_step = 0.0f;
    float diff = 0.0f;
    operating_mode_t new_mode = OPERATING_MODE_NORMAL;
    float raw_velocity = 0.0f;
    control_state_t new_state = STATE_NORMAL;
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    float integral_max = 0.0f, output_max = 0.0f;
    float pwm_l_min = 0.0f, pwm_l_max = 0.0f;
    float pwm_r_min = 0.0f, pwm_r_max = 0.0f;
    float error = 0.0f, effective_error = 0.0f;
    float integral_candidate = 0.0f;
    float provisional = 0.0f;
    bool sat_high = false;
    bool sat_low = false;
    float diff_output = 0.0f;
    float target_left = 0.0f;
    float target_right = 0.0f;

    if (!s_ctrl.initialized) {
        *pwm_left = 0.0f;
        *pwm_right = 0.0f;
        return STATE_NORMAL;
    }

    now = esp_timer_get_time();

    if ((angle_actual < SAFETY_ANGLE_MIN) || (angle_actual > SAFETY_ANGLE_MAX)) {
        ESP_LOGE(TAG, "Ángulo fuera de rango: %.1f°", angle_actual);
        *pwm_left = 0.0f;
        *pwm_right = 0.0f;
        return STATE_NORMAL;
    }

    /* Aplicar rampa al setpoint */
    max_step = REFERENCE_RAMP_DPS * dt_s;
    diff = s_ctrl.target_reference - s_ctrl.ramped_reference;

    if (diff > max_step) {
        s_ctrl.ramped_reference += max_step;
    } else if (diff < -max_step) {
        s_ctrl.ramped_reference -= max_step;
    } else {
        s_ctrl.ramped_reference = s_ctrl.target_reference;
    }

    /* Actualizar modo de operación */
    new_mode = update_operating_mode(s_ctrl.operating_mode, s_ctrl.ramped_reference);
    if (new_mode != s_ctrl.operating_mode) {
        ESP_LOGI(TAG, "Cambio de modo: %s -> %s",
                 mode_to_string(s_ctrl.operating_mode),
                 mode_to_string(new_mode));
        s_ctrl.operating_mode = new_mode;
    }

    /* Calcular velocidad angular */
    raw_velocity = calculate_velocity(
        angle_actual,
        now,
        s_ctrl.angle_history,
        s_ctrl.time_history,
        &s_ctrl.history_index,
        &s_ctrl.history_count
    );
    s_ctrl.filtered_velocity = (VELOCITY_FILTER_ALPHA * raw_velocity) +
                               ((1.0f - VELOCITY_FILTER_ALPHA) * s_ctrl.filtered_velocity);

    /* Verificar condición de emergencia */
    new_state = s_ctrl.state;

    if (angle_actual <= -EMERGENCY_ANGLE_DEG) {
        new_state = STATE_EMERGENCY_POSITIVE;
    } else if (angle_actual >= EMERGENCY_ANGLE_DEG) {
        new_state = STATE_EMERGENCY_NEGATIVE;
    } else if (new_state == STATE_EMERGENCY_POSITIVE) {
        if ((angle_actual >= -EMERGENCY_RELEASE_DEG) && (s_ctrl.filtered_velocity >= 0.0f)) {
            new_state = STATE_NORMAL;
        }
    } else if (new_state == STATE_EMERGENCY_NEGATIVE) {
        if ((angle_actual <= EMERGENCY_RELEASE_DEG) && (s_ctrl.filtered_velocity <= 0.0f)) {
            new_state = STATE_NORMAL;
        }
    } else {
        new_state = STATE_NORMAL;
    }

    if (new_state != s_ctrl.state) {
        ESP_LOGI(TAG, "Cambio de estado: %s -> %s (ángulo: %.1f°)",
                 state_to_string(s_ctrl.state),
                 state_to_string(new_state),
                 angle_actual);
        if (new_state != STATE_NORMAL) {
            s_ctrl.integral = 0.0f;
        }
        s_ctrl.state = new_state;
    }

    /* Seleccionar parámetros según modo */
    select_parameters(
        s_ctrl.operating_mode,
        &kp, &ki, &kd,
        &integral_max, &output_max,
        &pwm_l_min, &pwm_l_max,
        &pwm_r_min, &pwm_r_max
    );

    s_ctrl.active_kp = kp;
    s_ctrl.active_ki = ki;
    s_ctrl.active_kd = kd;
    s_ctrl.active_integral_max = integral_max;
    s_ctrl.active_output_max = output_max;

    /* Cálculo PID */
    error = s_ctrl.ramped_reference - angle_actual;
    effective_error = (fabsf(error) < DEADBAND_DEG) ? 0.0f : error;

    integral_candidate = s_ctrl.integral + (ki * effective_error * dt_s);
    integral_candidate = clamp_float(integral_candidate, -integral_max, integral_max);

    provisional = (kp * effective_error) + integral_candidate - (kd * s_ctrl.filtered_velocity);
    sat_high = provisional > output_max;
    sat_low = provisional < -output_max;

    /* Anti-windup */
    if ((!sat_high && !sat_low) ||
        (sat_high && (effective_error < 0.0f)) ||
        (sat_low && (effective_error > 0.0f))) {
        s_ctrl.integral = integral_candidate;
    }

    diff_output = (kp * effective_error) + s_ctrl.integral - (kd * s_ctrl.filtered_velocity);
    diff_output = clamp_float(diff_output, -output_max, output_max);

    /* Calcular PWM para cada motor */
    target_left = PWM_LEFT_BASE;
    target_right = PWM_RIGHT_BASE;

    if (diff_output > 0.0f) {
        target_left = PWM_LEFT_BASE + diff_output;
    } else if (diff_output < 0.0f) {
        target_right = PWM_RIGHT_BASE - diff_output;
    }

    /* Ajuste en modo emergencia */
    if (s_ctrl.state == STATE_EMERGENCY_POSITIVE) {
        target_left = PWM_LEFT_BASE + (EMERGENCY_DIFF_PERCENT * 0.5f);
        target_right = PWM_RIGHT_BASE - (EMERGENCY_DIFF_PERCENT * 0.5f);
    } else if (s_ctrl.state == STATE_EMERGENCY_NEGATIVE) {
        target_left = PWM_LEFT_BASE - (EMERGENCY_DIFF_PERCENT * 0.5f);
        target_right = PWM_RIGHT_BASE + (EMERGENCY_DIFF_PERCENT * 0.5f);
    }

    *pwm_left = slew_limit(target_left, s_ctrl.pwm_left_prev, PWM_SLEW_MAX_PER_CYCLE);
    *pwm_right = slew_limit(target_right, s_ctrl.pwm_right_prev, PWM_SLEW_MAX_PER_CYCLE);

    s_ctrl.pwm_left_prev = *pwm_left;
    s_ctrl.pwm_right_prev = *pwm_right;

    /* Aplicar límites de PWM */
    if (s_ctrl.state == STATE_NORMAL) {
        *pwm_left = clamp_float(*pwm_left, pwm_l_min, pwm_l_max);
        *pwm_right = clamp_float(*pwm_right, pwm_r_min, pwm_r_max);
    } else {
        *pwm_left = clamp_float(*pwm_left, PWM_LEFT_MIN, PWM_LEFT_MAX);
        *pwm_right = clamp_float(*pwm_right, PWM_RIGHT_MIN, PWM_RIGHT_MAX);
    }

    state_result = s_ctrl.state;

    return state_result;
}

/* ================================================================
 *  TAREA PRINCIPAL DEL CONTROLADOR
 * ================================================================ */
void vTaskControl(void *pvParameters)
{
    (void)pvParameters;

    SensorData_t sensor_data = {0};
    VisionData_t vision_data = {0};
    ActuadorData_t actuador_data = {0};

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t tiempo_sin_camara = 0U;
    const uint32_t TIMEOUT_CAMARA_MS = 1000U;
    bool estado_anterior_pantalla = false;

    s_last_update_us = esp_timer_get_time();

    control_fijar_setpoint(0.0f);

    ESP_LOGI(TAG, "Tarea de control iniciada");

    for (;;) {
        int64_t now_us = 0;
        float dt_s = 0.0f;
        uint32_t ahora_ms = 0U;
        bool pantalla_conectada = false;
        float setpoint_actual = 0.0f;
        bool camara_detectada = false;
        float pwm_left = 0.0f;
        float pwm_right = 0.0f;
        control_state_t state = STATE_NORMAL;
        static uint32_t ultimo_log = 0U;
        static uint32_t ultimo_envio = 0U;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PERIODO_MUESTREO_MS));

        now_us = esp_timer_get_time();
        dt_s = (float)(now_us - s_last_update_us) / 1000000.0f;
        s_last_update_us = now_us;
        ahora_ms = (uint32_t)(now_us / 1000ULL);

        leer_comando_teclado();

        /* ============================================================
         *  VERIFICAR CONEXIÓN CON LA PANTALLA
         * ============================================================ */
        pantalla_conectada = espnow_display_is_connected();

        /* Control del LED de alerta */
        if (pantalla_conectada) {
            hal_led_apagar();
        } else {
            hal_led_encender();
        }

        /* Modo seguro: pantalla desconectada */
        if (!pantalla_conectada) {
            if (s_tiene_datos_camara || (s_ctrl.target_reference != 0.0f)) {
                setpoint_actual = 0.0f;
                control_fijar_setpoint(setpoint_actual);
                s_tiene_datos_camara = false;

                if (estado_anterior_pantalla != pantalla_conectada) {
                    ESP_LOGW(TAG, "Modo seguro: pantalla desconectada -> setpoint 0°");
                }
            }

            estado_anterior_pantalla = pantalla_conectada;

            /* Leer sensor y calcular PWM */
            if (xQueuePeek(xColaSensorControl, &sensor_data, 0U) == pdPASS) {
                s_filtered_angle = (ANGLE_FILTER_ALPHA * sensor_data.angulo_actual) +
                                   ((1.0f - ANGLE_FILTER_ALPHA) * s_filtered_angle);
            }

            pwm_left = 0.0f;
            pwm_right = 0.0f;
            state = control_pid_update(s_filtered_angle, dt_s, &pwm_left, &pwm_right);

            actuador_data.pwm_izquierdo = pwm_left;
            actuador_data.pwm_derecho = pwm_right;
            xQueueOverwrite(xColaControlActuador, &actuador_data);

            if ((ahora_ms - ultimo_log) >= 500U) {
                float error = s_ctrl.ramped_reference - s_filtered_angle;
                ESP_LOGI(TAG, "Modo seguro Set:%.1f° Act:%.1f° Err:%.1f° | PWM L:%.1f%% R:%.1f%%",
                         s_ctrl.ramped_reference,
                         s_filtered_angle,
                         error,
                         pwm_left,
                         pwm_right);
                ultimo_log = ahora_ms;
            }

            continue;
        }

        /* ============================================================
         *  PANTALLA CONECTADA - OPERACIÓN NORMAL
         * ============================================================ */

        if (!estado_anterior_pantalla && pantalla_conectada) {
            ESP_LOGI(TAG, "Pantalla reconectada - modo normal");
        }
        estado_anterior_pantalla = pantalla_conectada;

        /* Seleccionar setpoint: cámara o valor fijo */
        camara_detectada = false;

        if (g_control_mode == CONTROL_MODE_MASTER) {
            if (xQueuePeek(xColaVisionControl, &vision_data, 0U) == pdPASS) {
                if (vision_data.deteccion_valida) {
                    camara_detectada = true;
                    s_ultimo_angulo_camara = vision_data.angulo_maestro;
                    s_ultimo_timestamp_camara = ahora_ms;
                    s_tiene_datos_camara = true;

                    setpoint_actual = vision_data.angulo_maestro;
                    control_fijar_setpoint(setpoint_actual);
                }
            }

            if (!camara_detectada && s_tiene_datos_camara) {
                tiempo_sin_camara = ahora_ms - s_ultimo_timestamp_camara;
                if (tiempo_sin_camara > TIMEOUT_CAMARA_MS) {
                    setpoint_actual = 0.0f;
                    control_fijar_setpoint(setpoint_actual);
                    s_tiene_datos_camara = false;
                }
            }
        } else {
            setpoint_actual = g_manual_setpoint;
            control_fijar_setpoint(setpoint_actual);
        }

        /* Leer ángulo del sensor */
        if (xQueuePeek(xColaSensorControl, &sensor_data, 0U) == pdPASS) {
            s_filtered_angle = (ANGLE_FILTER_ALPHA * sensor_data.angulo_actual) +
                               ((1.0f - ANGLE_FILTER_ALPHA) * s_filtered_angle);
        }

        /* Calcular PWM */
        pwm_left = 0.0f;
        pwm_right = 0.0f;
        state = control_pid_update(s_filtered_angle, dt_s, &pwm_left, &pwm_right);

        /* Enviar al actuador */
        actuador_data.pwm_izquierdo = pwm_left;
        actuador_data.pwm_derecho = pwm_right;
        xQueueOverwrite(xColaControlActuador, &actuador_data);

        /* Enviar telemetría */
        if ((ahora_ms - ultimo_envio) >= 500U) {
            telemetry_packet_t packet;
            packet.master_angle = s_ctrl.target_reference;
            packet.follower_angle = s_filtered_angle;
            packet.setpoint_angle = s_ctrl.ramped_reference;
            packet.pwm_left = pwm_left;
            packet.pwm_right = pwm_right;
            espnow_display_send(&packet);
            ultimo_envio = ahora_ms;
        }

        /* Log de depuración */
        if ((ahora_ms - ultimo_log) >= 200U) {
            float error = s_ctrl.ramped_reference - s_filtered_angle;
            ESP_LOGI(TAG, "Set:%.1f° Act:%.1f° Err:%.1f° Modo:%s | PWM L:%.1f%% R:%.1f%%",
                     s_ctrl.ramped_reference,
                     s_filtered_angle,
                     error,
                     mode_to_string(s_ctrl.operating_mode),
                     pwm_left,
                     pwm_right);
            ultimo_log = ahora_ms;
        }

        if (state != STATE_NORMAL) {
            if (!esta_en_modo_seguro()) {
                ESP_LOGW(TAG, "Emergencia activada: %s", state_to_string(state));
            }
        }
    }
}