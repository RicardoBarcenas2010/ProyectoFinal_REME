/**
 * @file tareas.h
 * @brief Definiciones de tareas, prioridades, colas y estructuras
 * @details Contiene todas las declaraciones de tareas RTOS, prioridades,
 *          tamaños de pila, colas, semáforos y estructuras de datos
 *          compartidas entre tareas.
 */

#ifndef TAREAS_H
#define TAREAS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  DECLARACIONES DE TAREAS RTOS
 * ================================================================ */

void vTaskSensor(void *pvParameters);
void vTaskControl(void *pvParameters);
void vTaskActuador(void *pvParameters);
void vTaskUI(void *pvParameters);
void vTaskWatchdogUI(void *pvParameters);
void vTaskMonitor(void *pvParameters);

/* ================================================================
 *  PRIORIDADES DE LAS TAREAS
 * ================================================================ */

#define PRIORIDAD_SENSOR        (tskIDLE_PRIORITY + 4)
#define PRIORIDAD_CONTROL       (tskIDLE_PRIORITY + 4)
#define PRIORIDAD_ACTUADOR      (tskIDLE_PRIORITY + 3)
#define PRIORIDAD_VISION        (tskIDLE_PRIORITY + 3)
#define PRIORIDAD_UI            (tskIDLE_PRIORITY + 2)
#define PRIORIDAD_WATCHDOG_UI   (tskIDLE_PRIORITY + 5)
#define PRIORIDAD_MONITOR       (tskIDLE_PRIORITY + 1)

/* ================================================================
 *  TAMAÑOS DE PILA (STACK) EN BYTES
 * ================================================================ */

#define STACK_SENSOR            (2048U)
#define STACK_CONTROL           (4096U)
#define STACK_ACTUADOR          (2048U)
#define STACK_VISION            (4096U)
#define STACK_UI                (4096U)
#define STACK_WATCHDOG_UI       (1024U)
#define STACK_MONITOR           (1024U)

/* ================================================================
 *  COLAS RTOS (EXPORTADAS PARA USO GLOBAL)
 * ================================================================ */

extern QueueHandle_t xColaSensorControl;
extern QueueHandle_t xColaControlActuador;
extern QueueHandle_t xColaVisionControl;
extern QueueHandle_t xColaComandosUI;

/* ================================================================
 *  SEMÁFOROS (EXPORTADOS PARA USO GLOBAL)
 * ================================================================ */

extern SemaphoreHandle_t xSemMutexPantalla;
extern SemaphoreHandle_t xSemModoSeguro;

/* ================================================================
 *  ESTRUCTURAS DE DATOS PARA COLAS
 * ================================================================ */

typedef struct {
    float angulo_maestro;
    float confianza;
    bool deteccion_valida;
    uint32_t timestamp_ms;
} VisionData_t;

typedef struct {
    float angulo_actual;
    float velocidad;
    uint32_t timestamp_ms;
} SensorData_t;

typedef struct {
    float setpoint;
    float angulo_actual;
    float salida_pwm;
    float error;
    float integral;
} ControlData_t;

typedef struct {
    float pwm_izquierdo;
    float pwm_derecho;
} ActuadorData_t;

typedef struct {
    enum {
        COMANDO_SETPOINT,
        COMANDO_AJUSTE_PID,
        COMANDO_RESET,
        COMANDO_MODO_MANUAL,
        COMANDO_MODO_AUTO
    } tipo;

    union {
        float setpoint;
        struct {
            float kp;
            float ki;
            float kd;
        } pid;
    } datos;
} UICommand_t;

/* ================================================================
 *  ESTRUCTURAS DEL CONTROLADOR PID AVANZADO
 * ================================================================ */

typedef enum {
    OPERATING_MODE_NORMAL = 0,
    OPERATING_MODE_HIGH_POSITIVE,
    OPERATING_MODE_HIGH_NEGATIVE
} operating_mode_t;

typedef enum {
    STATE_NORMAL = 0,
    STATE_EMERGENCY_POSITIVE,
    STATE_EMERGENCY_NEGATIVE
} control_state_t;

typedef struct {
    operating_mode_t operating_mode;
    control_state_t state;
    float integral;
    float error_anterior;
    float pwm_left_prev;
    float pwm_right_prev;
    float filtered_velocity;
    float angle_history[3];
    int64_t time_history[3];
    size_t history_index;
    size_t history_count;
    float ramped_reference;
    float target_reference;
    float active_kp;
    float active_ki;
    float active_kd;
    float active_integral_max;
    float active_output_max;
    bool initialized;
} ControlPID_t;

/* ================================================================
 *  FUNCIONES GLOBALES (NO ESTATICAS)
 * ================================================================ */

BaseType_t crear_colas_y_semaforos(void);

/* ⭐ ELIMINADO: BaseType_t crear_tareas_segun_etapa(void) - ahora estática en main.c */

void activar_modo_seguro(uint8_t motivo);
void restaurar_desde_modo_seguro(void);
bool esta_en_modo_seguro(void);

operating_mode_t control_obtener_modo(void);
control_state_t control_obtener_estado(void);
float control_obtener_angulo_filtrado(void);

/* ⭐ ELIMINADO: const char* mode_to_string - ahora estática en vTaskControl.c */
/* ⭐ ELIMINADO: const char* state_to_string - ahora estática en vTaskControl.c */

#ifdef __cplusplus
}
#endif

#endif /* TAREAS_H */