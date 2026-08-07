#ifndef HARDWARE_H
#define HARDWARE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────────────────────────────────────────────
 *  MOTIVOS DE MODO SEGURO
 * ────────────────────────────────────────────────────────────── */
#define MOTIVO_PANTALLA         1U
#define MOTIVO_SENSOR           2U
#define MOTIVO_STACK_OVERFLOW   3U
#define MOTIVO_MEMORIA          4U

/* ──────────────────────────────────────────────────────────────
 *  PINES GPIO - CONFIGURACIÓN CORRECTA
 *  Motor IZQUIERDO: GPIO 13
 *  Motor DERECHO:   GPIO 15
 * ────────────────────────────────────────────────────────────── */
#define PIN_POTENCIOMETRO       34
#define PIN_MOTOR_IZQ_PWM       13   /* GPIO 13 - Motor IZQUIERDO */
#define PIN_MOTOR_DER_PWM       15   /* GPIO 15 - Motor DERECHO */
#define PIN_LED_ALERTA          2

/* ──────────────────────────────────────────────────────────────
 *  CONFIGURACIÓN ADC
 * ────────────────────────────────────────────────────────────── */
#define ADC_POT_CHANNEL         6
#define ADC_POT_UNIT            1

/* ──────────────────────────────────────────────────────────────
 *  CONFIGURACIÓN ESC
 * ────────────────────────────────────────────────────────────── */
#define ESC_MIN_PULSE_US        1000.0f
#define ESC_MAX_PULSE_US        2000.0f
#define ESC_PERIOD_US           20000.0f
#define ESC_FREQUENCY_HZ        50

/* ──────────────────────────────────────────────────────────────
 *  PWM BASE - CON MÁS RANGO
 * ────────────────────────────────────────────────────────────── */
#define PWM_LEFT_BASE           22.50f
#define PWM_RIGHT_BASE          24.50f
#define PWM_LEFT_MIN            25.00f
#define PWM_LEFT_MAX            45.00f  /* ← 45.50 → 50.00 */
#define PWM_RIGHT_MIN           25.80f
#define PWM_RIGHT_MAX           45.00f  /* ← 45.50 → 50.00 */

/* ──────────────────────────────────────────────────────────────
 *  PARÁMETROS DE CONTROL
 * ────────────────────────────────────────────────────────────── */
#define PERIODO_MUESTREO_MS     10

/* ──────────────────────────────────────────────────────────────
 *  CONTROL PID - MODO NORMAL
 * ────────────────────────────────────────────────────────────── */
#define NORMAL_KP               1.0f
#define NORMAL_KI               0.75f
#define NORMAL_KD               0.3f
#define NORMAL_INTEGRAL_MAX     4.00f
#define NORMAL_OUTPUT_MAX       5.00f

/* ──────────────────────────────────────────────────────────────
 *  CONTROL PID - MODO ALTO POSITIVO (>24°)
 * ────────────────────────────────────────────────────────────── */
#define HIGH_POSITIVE_KP                0.5f   /* ← 1.30 → 1.20 */
#define HIGH_POSITIVE_KI                0.80f   /* ← 0.50 → 0.80 */
#define HIGH_POSITIVE_KD                0.08f
#define HIGH_POSITIVE_INTEGRAL_MAX      12.00f
#define HIGH_POSITIVE_OUTPUT_MAX        7.00f  /* ← 15.00 → 18.00 */
#define HIGH_POSITIVE_PWM_LEFT_BASE     22.00f
#define HIGH_POSITIVE_PWM_RIGHT_BASE    24.50f
#define HIGH_POSITIVE_PWM_LEFT_MIN      22.00f
#define HIGH_POSITIVE_PWM_RIGHT_MAX     50.00f  /* ← 45.00 → 50.00 */

/* ──────────────────────────────────────────────────────────────
 *  CONTROL PID - MODO ALTO NEGATIVO (<-24°)
 * ────────────────────────────────────────────────────────────── */
#define HIGH_NEGATIVE_KP                1.30f   /* ← 1.50 → 1.20 */
#define HIGH_NEGATIVE_KI                0.8f   /* ← 0.50 → 0.80 */
#define HIGH_NEGATIVE_KD                0.03f
#define HIGH_NEGATIVE_INTEGRAL_MAX      12.00f
#define HIGH_NEGATIVE_OUTPUT_MAX        7.00f  /* ← 15.00 → 18.00 */
#define HIGH_NEGATIVE_PWM_LEFT_BASE     22.50f
#define HIGH_NEGATIVE_PWM_RIGHT_BASE    24.50f
#define HIGH_NEGATIVE_PWM_LEFT_MAX      60.00f  /* ← 40.00 → 45.00 */
#define HIGH_NEGATIVE_PWM_RIGHT_MAX     60.00f  /* ← 40.00 → 50.00 */
#define HIGH_NEGATIVE_PWM_RIGHT_MIN     25.80f

/* ──────────────────────────────────────────────────────────────
 *  MÁQUINA DE ESTADOS POR RANGO - CORRECTO
 * ────────────────────────────────────────────────────────────── */
#define HIGH_MODE_ENTER_DEG     20.00f
#define HIGH_MODE_EXIT_DEG      18.00f

/* ──────────────────────────────────────────────────────────────
 *  EMERGENCIA - MÁS TOLERANTE
 * ────────────────────────────────────────────────────────────── */
#define EMERGENCY_ANGLE_DEG     55.00f  /* ← 50 → 55 */
#define EMERGENCY_RELEASE_DEG   48.00f  /* ← 45 → 48 */
#define EMERGENCY_DIFF_PERCENT  10.00f

/* ──────────────────────────────────────────────────────────────
 *  FILTROS Y RAMPA
 * ────────────────────────────────────────────────────────────── */
#define ANGLE_FILTER_ALPHA      0.40f
#define VELOCITY_FILTER_ALPHA   0.30f
#define VELOCITY_WINDOW_SAMPLES 3
#define REFERENCE_RAMP_DPS      5.00f
#define PWM_SLEW_MAX_PER_CYCLE  0.50f
#define DEADBAND_DEG            0.05f

/* ──────────────────────────────────────────────────────────────
 *  SEGURIDAD - MÁS TOLERANTE
 * ────────────────────────────────────────────────────────────── */
#define SAFETY_ANGLE_MIN        -50.0f  /* ← -55 → -60 */
#define SAFETY_ANGLE_MAX        50.0f   /* ← 55 → 60 */

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES
 * ────────────────────────────────────────────────────────────── */
esp_err_t hal_inicializar(void);
esp_err_t hal_motores_inicializar(void);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_H */