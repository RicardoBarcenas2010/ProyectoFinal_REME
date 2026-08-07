/**
 * @file vTaskControl.c
 * @brief Controlador PID con cámara - ETAPA 4
 *        - Si hay datos de cámara: setpoint = ángulo de la cámara
 *        - Si NO hay datos de cámara: setpoint = 0° (fijo)
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

/* ──────────────────────────────────────────────────────────────
 *  CONFIGURACIÓN UART PARA TECLADO
 * ────────────────────────────────────────────────────────────── */
#define UART_NUM                UART_NUM_0
#define RX_BUF_SIZE             64

/* ──────────────────────────────────────────────────────────────
 *  INSTANCIA DEL CONTROLADOR
 * ────────────────────────────────────────────────────────────── */
static ControlPID_t s_ctrl;

/* ──────────────────────────────────────────────────────────────
 *  ÁNGULO FILTRADO
 * ────────────────────────────────────────────────────────────── */
static float s_filtered_angle = 0.0f;
static int64_t s_last_update_us = 0;

/* ──────────────────────────────────────────────────────────────
 *  CONTROL DE CÁMARA
 * ────────────────────────────────────────────────────────────── */
static float s_ultimo_angulo_camara = 0.0f;
static uint32_t s_ultimo_timestamp_camara = 0U;
static bool s_tiene_datos_camara = false;

/* ──────────────────────────────────────────────────────────────
 *  LECTURA DE TECLADO (SOLO PARA DIAGNÓSTICO)
 * ────────────────────────────────────────────────────────────── */
static void leer_comando_teclado(void)
{
    uint8_t data[RX_BUF_SIZE] = {0};
    
    int bytes_available = 0;
    uart_get_buffered_data_len(UART_NUM, (size_t*)&bytes_available);
    
    if (bytes_available == 0) {
        return;
    }
    
    int len = uart_read_bytes(UART_NUM, data, 1, pdMS_TO_TICKS(10));
    
    if (len > 0) {
        char comando = data[0];
        
        switch (comando) {
            case 'r':
                control_resetear();
                ESP_LOGI(TAG, "🔄 PID reseteado");
                break;
            case 'h':
                ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                ESP_LOGI(TAG, "📖 CONTROLES DE TECLADO (ETAPA 4):");
                ESP_LOGI(TAG, "   'r'  → Resetear PID");
                ESP_LOGI(TAG, "   'h'  → Mostrar ayuda");
                ESP_LOGI(TAG, "   La cámara controla el setpoint automáticamente");
                ESP_LOGI(TAG, "   Si no hay cámara → setpoint = 0°");
                ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                break;
            default:
                break;
        }
    }
}

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES AUXILIARES
 * ────────────────────────────────────────────────────────────── */
static float clamp_float(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static float slew_limit(float target, float previous, float max_step)
{
    float diff = target - previous;
    if (diff > max_step) diff = max_step;
    if (diff < -max_step) diff = -max_step;
    return previous + diff;
}

const char* mode_to_string(operating_mode_t mode)
{
    switch (mode) {
        case OPERATING_MODE_NORMAL: return "NORMAL";
        case OPERATING_MODE_HIGH_POSITIVE: return "ALTO_POS";
        case OPERATING_MODE_HIGH_NEGATIVE: return "ALTO_NEG";
        default: return "DESCONOCIDO";
    }
}

const char* state_to_string(control_state_t state)
{
    switch (state) {
        case STATE_NORMAL: return "NORMAL";
        case STATE_EMERGENCY_POSITIVE: return "EMERG_POS";
        case STATE_EMERGENCY_NEGATIVE: return "EMERG_NEG";
        default: return "DESCONOCIDO";
    }
}

/* ──────────────────────────────────────────────────────────────
 *  ACTUALIZACIÓN DEL MODO DE OPERACIÓN
 * ────────────────────────────────────────────────────────────── */
static operating_mode_t update_operating_mode(
    operating_mode_t current_mode,
    float reference_deg)
{
    switch (current_mode) {
        case OPERATING_MODE_NORMAL:
            if (reference_deg >= HIGH_MODE_ENTER_DEG)
                return OPERATING_MODE_HIGH_POSITIVE;
            if (reference_deg <= -HIGH_MODE_ENTER_DEG)
                return OPERATING_MODE_HIGH_NEGATIVE;
            return OPERATING_MODE_NORMAL;

        case OPERATING_MODE_HIGH_POSITIVE:
            if (reference_deg <= -HIGH_MODE_ENTER_DEG)
                return OPERATING_MODE_HIGH_NEGATIVE;
            if (reference_deg <= HIGH_MODE_EXIT_DEG)
                return OPERATING_MODE_NORMAL;
            return OPERATING_MODE_HIGH_POSITIVE;

        case OPERATING_MODE_HIGH_NEGATIVE:
            if (reference_deg >= HIGH_MODE_ENTER_DEG)
                return OPERATING_MODE_HIGH_POSITIVE;
            if (reference_deg >= -HIGH_MODE_EXIT_DEG)
                return OPERATING_MODE_NORMAL;
            return OPERATING_MODE_HIGH_NEGATIVE;

        default:
            return OPERATING_MODE_NORMAL;
    }
}

/* ──────────────────────────────────────────────────────────────
 *  SELECCIÓN DE PARÁMETROS SEGÚN MODO
 * ────────────────────────────────────────────────────────────── */
static void select_parameters(
    operating_mode_t mode,
    float *kp, float *ki, float *kd,
    float *integral_max, float *output_max,
    float *pwm_left_min, float *pwm_left_max,
    float *pwm_right_min, float *pwm_right_max)
{
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

/* ──────────────────────────────────────────────────────────────
 *  CÁLCULO DE VELOCIDAD POR VENTANA
 * ────────────────────────────────────────────────────────────── */
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

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES PÚBLICAS (control.h)
 * ────────────────────────────────────────────────────────────── */
esp_err_t control_inicializar(void)
{
    memset(&s_ctrl, 0, sizeof(s_ctrl));

    s_ctrl.operating_mode = OPERATING_MODE_NORMAL;
    s_ctrl.state = STATE_NORMAL;
    s_ctrl.ramped_reference = 0.0f;
    s_ctrl.target_reference = 0.0f;
    s_ctrl.filtered_velocity = 0.0f;
    s_ctrl.pwm_left_prev = PWM_LEFT_BASE;
    s_ctrl.pwm_right_prev = PWM_RIGHT_BASE;
    s_ctrl.initialized = true;

    int64_t now = esp_timer_get_time();
    for (size_t i = 0; i < VELOCITY_WINDOW_SAMPLES; i++) {
        s_ctrl.angle_history[i] = 0.0f;
        s_ctrl.time_history[i] = now;
    }

    s_last_update_us = now;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🔧 CONTROLADOR PID - ETAPA 4 (CON CÁMARA)");
    ESP_LOGI(TAG, "   Setpoint = 0° (fijo) si no hay cámara");
    ESP_LOGI(TAG, "   Setpoint = ángulo de cámara si hay datos");
    ESP_LOGI(TAG, "   Modo Normal: KP=%.2f KI=%.2f KD=%.2f", NORMAL_KP, NORMAL_KI, NORMAL_KD);
    ESP_LOGI(TAG, "   Modo Alto Pos (>24°): KP=%.2f", HIGH_POSITIVE_KP);
    ESP_LOGI(TAG, "   Modo Alto Neg (<-24°): KP=%.2f", HIGH_NEGATIVE_KP);
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}

void control_actualizar_pid(float kp, float ki, float kd)
{
    (void)kp; (void)ki; (void)kd;
}

float control_obtener_setpoint(void)
{
    return s_ctrl.target_reference;
}

void control_fijar_setpoint(float setpoint)
{
    s_ctrl.target_reference = setpoint;
}

void control_set_modo_auto(bool modo_auto)
{
    (void)modo_auto;
}

void control_resetear(void)
{
    s_ctrl.integral = 0.0f;
    s_ctrl.error_anterior = 0.0f;
    s_ctrl.ramped_reference = s_ctrl.target_reference;
    s_ctrl.filtered_velocity = 0.0f;
    s_ctrl.pwm_left_prev = PWM_LEFT_BASE;
    s_ctrl.pwm_right_prev = PWM_RIGHT_BASE;
    s_ctrl.state = STATE_NORMAL;
    s_ctrl.operating_mode = OPERATING_MODE_NORMAL;
    ESP_LOGI(TAG, "🔄 PID reseteado");
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

/* ──────────────────────────────────────────────────────────────
 *  ACTUALIZACIÓN DEL CONTROLADOR (IGUAL QUE ETAPA 3)
 * ────────────────────────────────────────────────────────────── */
static control_state_t control_pid_update(
    float angle_actual,
    float dt_s,
    float *pwm_left,
    float *pwm_right)
{
    if (!s_ctrl.initialized) {
        *pwm_left = 0.0f;
        *pwm_right = 0.0f;
        return STATE_NORMAL;
    }

    int64_t now = esp_timer_get_time();

    if (angle_actual < SAFETY_ANGLE_MIN || angle_actual > SAFETY_ANGLE_MAX) {
        ESP_LOGE(TAG, "⚠️ Ángulo fuera de rango seguro: %.1f°", angle_actual);
        *pwm_left = 0.0f;
        *pwm_right = 0.0f;
        return STATE_NORMAL;
    }

    float max_step = REFERENCE_RAMP_DPS * dt_s;
    float diff = s_ctrl.target_reference - s_ctrl.ramped_reference;
    if (diff > max_step) {
        s_ctrl.ramped_reference += max_step;
    } else if (diff < -max_step) {
        s_ctrl.ramped_reference -= max_step;
    } else {
        s_ctrl.ramped_reference = s_ctrl.target_reference;
    }

    operating_mode_t new_mode = update_operating_mode(
        s_ctrl.operating_mode,
        s_ctrl.ramped_reference
    );
    if (new_mode != s_ctrl.operating_mode) {
        ESP_LOGI(TAG, "🔀 Modo: %s → %s",
            mode_to_string(s_ctrl.operating_mode),
            mode_to_string(new_mode));
        s_ctrl.operating_mode = new_mode;
    }

    float raw_velocity = calculate_velocity(
        angle_actual,
        now,
        s_ctrl.angle_history,
        s_ctrl.time_history,
        &s_ctrl.history_index,
        &s_ctrl.history_count
    );
    s_ctrl.filtered_velocity = (VELOCITY_FILTER_ALPHA * raw_velocity) +
                              ((1.0f - VELOCITY_FILTER_ALPHA) * s_ctrl.filtered_velocity);

    control_state_t new_state = s_ctrl.state;
    if (angle_actual <= -EMERGENCY_ANGLE_DEG) {
        new_state = STATE_EMERGENCY_POSITIVE;
    } else if (angle_actual >= EMERGENCY_ANGLE_DEG) {
        new_state = STATE_EMERGENCY_NEGATIVE;
    } else if (new_state == STATE_EMERGENCY_POSITIVE) {
        if (angle_actual >= -EMERGENCY_RELEASE_DEG && s_ctrl.filtered_velocity >= 0.0f) {
            new_state = STATE_NORMAL;
        }
    } else if (new_state == STATE_EMERGENCY_NEGATIVE) {
        if (angle_actual <= EMERGENCY_RELEASE_DEG && s_ctrl.filtered_velocity <= 0.0f) {
            new_state = STATE_NORMAL;
        }
    } else {
        new_state = STATE_NORMAL;
    }

    if (new_state != s_ctrl.state) {
        ESP_LOGI(TAG, "🚨 Estado: %s → %s (ángulo: %.1f°)",
            state_to_string(s_ctrl.state),
            state_to_string(new_state),
            angle_actual);
        if (new_state != STATE_NORMAL) {
            s_ctrl.integral = 0.0f;
        }
        s_ctrl.state = new_state;
    }

    float kp, ki, kd, integral_max, output_max;
    float pwm_l_min, pwm_l_max, pwm_r_min, pwm_r_max;

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

    float error = s_ctrl.ramped_reference - angle_actual;
    float effective_error = (fabsf(error) < DEADBAND_DEG) ? 0.0f : error;

    float integral_candidate = s_ctrl.integral + (ki * effective_error * dt_s);
    integral_candidate = clamp_float(integral_candidate, -integral_max, integral_max);

    float provisional = (kp * effective_error) + integral_candidate - (kd * s_ctrl.filtered_velocity);
    bool sat_high = provisional > output_max;
    bool sat_low = provisional < -output_max;

    if ((!sat_high && !sat_low) || (sat_high && effective_error < 0.0f) || (sat_low && effective_error > 0.0f)) {
        s_ctrl.integral = integral_candidate;
    }

    float diff_output = (kp * effective_error) + s_ctrl.integral - (kd * s_ctrl.filtered_velocity);
    diff_output = clamp_float(diff_output, -output_max, output_max);

    float target_left = PWM_LEFT_BASE;
    float target_right = PWM_RIGHT_BASE;

    if (diff_output > 0.0f) {
        target_left = PWM_LEFT_BASE + diff_output;
    } else if (diff_output < 0.0f) {
        target_right = PWM_RIGHT_BASE - diff_output;
    }

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

    if (s_ctrl.state == STATE_NORMAL) {
        *pwm_left = clamp_float(*pwm_left, pwm_l_min, pwm_l_max);
        *pwm_right = clamp_float(*pwm_right, pwm_r_min, pwm_r_max);
    } else {
        *pwm_left = clamp_float(*pwm_left, PWM_LEFT_MIN, PWM_LEFT_MAX);
        *pwm_right = clamp_float(*pwm_right, PWM_RIGHT_MIN, PWM_RIGHT_MAX);
    }

    return s_ctrl.state;
}

/* ──────────────────────────────────────────────────────────────
 *  TAREA DE CONTROL - ETAPA 4 (CON CÁMARA)
 *  Estrategia: Setpoint = 0° fijo si no hay cámara
 *              Setpoint = ángulo de cámara si hay datos
 * ────────────────────────────────────────────────────────────── */
void vTaskControl(void *pvParameters)
{
    (void)pvParameters;

    SensorData_t sensor_data = {0};
    VisionData_t vision_data = {0};
    ActuadorData_t actuador_data = {0};

    TickType_t xLastWakeTime = xTaskGetTickCount();
    s_last_update_us = esp_timer_get_time();

    float setpoint_actual = 0.0f;
    control_fijar_setpoint(setpoint_actual);

    ESP_LOGI(TAG, "🔄 Tarea de control iniciada - ETAPA 4");
    ESP_LOGI(TAG, "📷 Setpoint desde cámara (si hay datos) o 0° (fijo)");
    ESP_LOGI(TAG, "📖 Presiona 'h' para ayuda");

    /* Tiempo desde la última detección de cámara */
    uint32_t tiempo_sin_camara = 0U;
    const uint32_t TIMEOUT_CAMARA_MS = 1000U;  /* 1 segundo sin cámara → setpoint = 0° */

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PERIODO_MUESTREO_MS));

        int64_t now_us = esp_timer_get_time();
        float dt_s = (float)(now_us - s_last_update_us) / 1000000.0f;
        s_last_update_us = now_us;
        uint32_t ahora_ms = (uint32_t)(now_us / 1000ULL);

        /* ⭐ LEER TECLADO (SOLO DIAGNÓSTICO) ⭐ */
        leer_comando_teclado();

         bool pantalla_conectada = espnow_display_is_connected();

        /* Control del LED */
        if (pantalla_conectada) {
            hal_led_apagar();   /* LED apagado = pantalla conectada */
        } else {
            hal_led_encender(); /* LED encendido = pantalla desconectada */
        }

        /* Si la pantalla está desconectada → forzar setpoint = 0° */
        if (!pantalla_conectada) {
            /* Forzar setpoint a 0° (la barra se va a 0°) */
            if (s_tiene_datos_camara || s_ctrl.target_reference != 0.0f) {
                setpoint_actual = 0.0f;
                control_fijar_setpoint(setpoint_actual);
                s_tiene_datos_camara = false;
                ESP_LOGW(TAG, "🔌 MODO SEGURO: Pantalla desconectada → Setpoint = 0°");
            }
            
            /* El PID sigue funcionando normalmente, pero con setpoint = 0° */
            /* NO reducimos el PWM al 50% - solo vamos a 0° */
        }

        /* ⭐⭐⭐ LÓGICA DE SETPOINT: CÁMARA O 0° FIJO ⭐⭐⭐ */
        bool camara_detectada = false;

        if (g_control_mode == CONTROL_MODE_MASTER)
        {
            if (xQueuePeek(xColaVisionControl, &vision_data, 0U) == pdPASS)
            {
                if (vision_data.deteccion_valida)
                {
                    camara_detectada = true;

                    s_ultimo_angulo_camara = vision_data.angulo_maestro;
                    s_ultimo_timestamp_camara = ahora_ms;
                    s_tiene_datos_camara = true;

                    setpoint_actual = vision_data.angulo_maestro;
                    control_fijar_setpoint(setpoint_actual);
                }
            }

            if (!camara_detectada && s_tiene_datos_camara)
            {
                tiempo_sin_camara = ahora_ms - s_ultimo_timestamp_camara;

                if (tiempo_sin_camara > TIMEOUT_CAMARA_MS)
                {
                    setpoint_actual = 0.0f;
                    control_fijar_setpoint(setpoint_actual);

                    s_tiene_datos_camara = false;
                }
            }
        }
        else
        {
            setpoint_actual = g_manual_setpoint;
            control_fijar_setpoint(setpoint_actual);
        }

        /* ⭐ LEER ÁNGULO DEL SENSOR (IGUAL QUE ETAPA 3) ⭐ */
        if (xQueuePeek(xColaSensorControl, &sensor_data, 0U) == pdPASS) {
            s_filtered_angle = (ANGLE_FILTER_ALPHA * sensor_data.angulo_actual) + 
                              ((1.0f - ANGLE_FILTER_ALPHA) * s_filtered_angle);
        }

        /* ⭐ CALCULAR PWM (EXACTAMENTE IGUAL QUE ETAPA 3) ⭐ */
        float pwm_left = 0.0f;
        float pwm_right = 0.0f;

        control_state_t state = control_pid_update(
            s_filtered_angle,
            dt_s,
            &pwm_left,
            &pwm_right
        );

        /* ⭐ ENVIAR AL ACTUADOR (IGUAL QUE ETAPA 3) ⭐ */
        actuador_data.pwm_izquierdo = pwm_left;
        actuador_data.pwm_derecho = pwm_right;
        xQueueOverwrite(xColaControlActuador, &actuador_data);

        /* ⭐ ENVIAR TELEMETRÍA ⭐ */
        static uint32_t ultimo_envio = 0U;
        if ((ahora_ms - ultimo_envio) >= 500U) {
            telemetry_packet_t packet;
            packet.master_angle = s_ctrl.target_reference;
            packet.follower_angle = s_filtered_angle;
            packet.setpoint_angle = s_ctrl.ramped_reference;
            packet.pwm_left = pwm_left;
            packet.pwm_right = pwm_right;

            ESP_LOGI("TX",
            "sizeof(packet) = %d",
            sizeof(telemetry_packet_t));

            espnow_display_send(&packet);
            ultimo_envio = ahora_ms;
        }

        /* ⭐ LOG CADA 200ms (IGUAL QUE ETAPA 3) ⭐ */
        static uint32_t ultimo_log = 0U;
        if ((ahora_ms - ultimo_log) >= 200U) {
            float error = s_ctrl.ramped_reference - s_filtered_angle;
            ESP_LOGI(TAG, "📊 %s Set:%.1f° Act:%.1f° Err:%+.1f° Modo:%s | PWM L:%5.1f%% R:%5.1f%%",
                     s_tiene_datos_camara ? "📷CAM" : "⏸FJO",
                     s_ctrl.ramped_reference,
                     s_filtered_angle,
                     error,
                     mode_to_string(s_ctrl.operating_mode),
                     pwm_left, pwm_right);
            ultimo_log = ahora_ms;
        }

        if (state != STATE_NORMAL) {
            if (!esta_en_modo_seguro()) {
                ESP_LOGW(TAG, "⚠️ Emergencia activada! Estado: %s", state_to_string(state));
            }
        }
    }
}