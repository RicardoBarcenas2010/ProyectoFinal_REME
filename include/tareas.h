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

/**
 * @brief Tarea de lectura del sensor (potenciómetro con filtro Kalman)
 * @param pvParameters Parámetros de la tarea (no utilizados)
 */
void vTaskSensor(void *pvParameters);

/**
 * @brief Tarea de controlador en cascada (PID + modos de operación)
 * @param pvParameters Parámetros de la tarea (no utilizados)
 */
void vTaskControl(void *pvParameters);

/**
 * @brief Tarea de actuación (control de motores brushless)
 * @param pvParameters Parámetros de la tarea (no utilizados)
 */
void vTaskActuador(void *pvParameters);

/**
 * @brief Tarea de interfaz de usuario (pantalla táctil)
 * @param pvParameters Parámetros de la tarea (no utilizados)
 */
void vTaskUI(void *pvParameters);

/**
 * @brief Tarea de watchdog para monitoreo de la pantalla táctil
 * @param pvParameters Parámetros de la tarea (no utilizados)
 */
void vTaskWatchdogUI(void *pvParameters);

/**
 * @brief Tarea de monitoreo y telemetría del sistema
 * @param pvParameters Parámetros de la tarea (no utilizados)
 */
void vTaskMonitor(void *pvParameters);

/* ================================================================
 *  PRIORIDADES DE LAS TAREAS
 * ================================================================ */

/** Prioridad de vTaskSensor (alta) */
#define PRIORIDAD_SENSOR        (tskIDLE_PRIORITY + 4)

/** Prioridad de vTaskControl (alta) */
#define PRIORIDAD_CONTROL       (tskIDLE_PRIORITY + 4)

/** Prioridad de vTaskActuador (alta) */
#define PRIORIDAD_ACTUADOR      (tskIDLE_PRIORITY + 3)

/** Prioridad de vTaskVision (normal) */
#define PRIORIDAD_VISION        (tskIDLE_PRIORITY + 3)

/** Prioridad de vTaskUI (normal) */
#define PRIORIDAD_UI            (tskIDLE_PRIORITY + 2)

/** Prioridad de vTaskWatchdogUI (muy alta) */
#define PRIORIDAD_WATCHDOG_UI   (tskIDLE_PRIORITY + 5)

/** Prioridad de vTaskMonitor (baja) */
#define PRIORIDAD_MONITOR       (tskIDLE_PRIORITY + 1)

/* ================================================================
 *  TAMAÑOS DE PILA (STACK) EN BYTES
 * ================================================================ */

/** Stack para vTaskSensor */
#define STACK_SENSOR            (2048U)

/** Stack para vTaskControl */
#define STACK_CONTROL           (4096U)  /* Aumentado para el PID avanzado */

/** Stack para vTaskActuador */
#define STACK_ACTUADOR          (2048U)

/** Stack para vTaskVision */
#define STACK_VISION            (4096U)

/** Stack para vTaskUI */
#define STACK_UI                (4096U)

/** Stack para vTaskWatchdogUI */
#define STACK_WATCHDOG_UI       (1024U)

/** Stack para vTaskMonitor */
#define STACK_MONITOR           (1024U)

/* ================================================================
 *  COLAS RTOS (EXPORTADAS PARA USO GLOBAL)
 * ================================================================ */

/**
 * @brief Cola: Sensor → Control
 * @details Envía el ángulo filtrado del potenciómetro desde vTaskSensor a vTaskControl
 * @note Tamaño: 1 (overwrite)
 */
extern QueueHandle_t xColaSensorControl;

/**
 * @brief Cola: Control → Actuador
 * @details Envía la señal PWM calculada desde vTaskControl a vTaskActuador
 * @note Tamaño: 1 (overwrite)
 */
extern QueueHandle_t xColaControlActuador;

/**
 * @brief Cola: Visión → Control
 * @details Envía el ángulo del Maestro recibido por HTTP desde vTaskVision a vTaskControl
 * @note Tamaño: 1 (overwrite)
 */
extern QueueHandle_t xColaVisionControl;

/**
 * @brief Cola: UI → Control
 * @details Envía comandos desde la pantalla táctil (setpoint, PID, modos)
 * @note Tamaño: 10 (cola de comandos)
 */
extern QueueHandle_t xColaComandosUI;

/* ================================================================
 *  SEMÁFOROS (EXPORTADOS PARA USO GLOBAL)
 * ================================================================ */

/**
 * @brief Mutex para acceso exclusivo a la pantalla táctil
 * @details Protege el acceso a la pantalla entre vTaskUI y otras tareas
 */
extern SemaphoreHandle_t xSemMutexPantalla;

/**
 * @brief Semáforo binario para notificación de modo seguro
 * @details Se activa cuando se detecta una falla crítica
 */
extern SemaphoreHandle_t xSemModoSeguro;

/* ================================================================
 *  ESTRUCTURAS DE DATOS PARA COLAS
 * ================================================================ */

/**
 * @brief Datos de visión (ángulo del Maestro desde HTTP)
 */
typedef struct {
    float angulo_maestro;       /**< Ángulo recibido por HTTP (grados) */
    float confianza;            /**< Confianza de la detección (0.0-1.0) */
    bool deteccion_valida;      /**< true si la detección es válida */
    uint32_t timestamp_ms;      /**< Timestamp de recepción (ms) */
} VisionData_t;

/**
 * @brief Datos del sensor (ángulo del potenciómetro filtrado)
 */
typedef struct {
    float angulo_actual;        /**< Ángulo filtrado por Kalman (grados) */
    float velocidad;            /**< Velocidad angular estimada (grados/segundo) */
    uint32_t timestamp_ms;      /**< Timestamp de la lectura (ms) */
} SensorData_t;

/**
 * @brief Datos del controlador (señal PWM calculada)
 */
typedef struct {
    float setpoint;             /**< Setpoint actual (grados) */
    float angulo_actual;        /**< Ángulo actual (grados) */
    float salida_pwm;           /**< Señal de control (0.0-100.0) */
    float error;                /**< Error de seguimiento (grados) */
    float integral;             /**< Término integral acumulado */
} ControlData_t;

/**
 * @brief Datos de actuación (PWM para motores)
 */
typedef struct {
    float pwm_izquierdo;        /**< PWM motor izquierdo (0.0-100.0%) */
    float pwm_derecho;          /**< PWM motor derecho (0.0-100.0%) */
} ActuadorData_t;

/**
 * @brief Comandos desde la interfaz de usuario
 */
typedef struct {
    enum {
        COMANDO_SETPOINT,       /**< Cambiar setpoint manual */
        COMANDO_AJUSTE_PID,     /**< Ajustar parámetros PID */
        COMANDO_RESET,          /**< Resetear integral del PID */
        COMANDO_MODO_MANUAL,    /**< Activar modo manual */
        COMANDO_MODO_AUTO       /**< Activar modo automático (seguir Maestro) */
    } tipo;                     /**< Tipo de comando */

    union {
        float setpoint;         /**< Nuevo setpoint (grados) */
        struct {
            float kp;           /**< Ganancia proporcional */
            float ki;           /**< Ganancia integral */
            float kd;           /**< Ganancia derivativa */
        } pid;                  /**< Parámetros PID */
    } datos;                    /**< Datos del comando */
} UICommand_t;

/* ================================================================
 *  ESTRUCTURAS DEL CONTROLADOR PID AVANZADO
 * ================================================================ */

/**
 * @brief Modos de operación del controlador
 */
typedef enum {
    OPERATING_MODE_NORMAL = 0,
    OPERATING_MODE_HIGH_POSITIVE,
    OPERATING_MODE_HIGH_NEGATIVE
} operating_mode_t;

/**
 * @brief Estados del controlador (emergencia)
 */
typedef enum {
    STATE_NORMAL = 0,
    STATE_EMERGENCY_POSITIVE,
    STATE_EMERGENCY_NEGATIVE
} control_state_t;

/**
 * @brief Estructura del controlador PID avanzado
 */
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
 *  FUNCIONES GLOBALES
 * ================================================================ */

/**
 * @brief Crea todas las colas y semáforos del sistema
 * @return pdPASS si éxito, pdFAIL si error
 */
BaseType_t crear_colas_y_semaforos(void);

/**
 * @brief Crea las tareas según la etapa definida
 * @return pdPASS si éxito, pdFAIL si error
 */
BaseType_t crear_tareas_segun_etapa(void);

/**
 * @brief Activa el modo seguro por falla crítica
 * @param motivo Código del motivo (definido en hardware.h)
 */
void activar_modo_seguro(uint8_t motivo);

/**
 * @brief Restaura el sistema desde el modo seguro
 */
void restaurar_desde_modo_seguro(void);

/**
 * @brief Verifica si el sistema está en modo seguro
 * @return true si está en modo seguro, false en caso contrario
 */
bool esta_en_modo_seguro(void);

/**
 * @brief Obtiene el modo de operación actual del controlador
 */
operating_mode_t control_obtener_modo(void);

/**
 * @brief Obtiene el estado actual del controlador
 */
control_state_t control_obtener_estado(void);

/**
 * @brief Obtiene el ángulo filtrado actual
 */
float control_obtener_angulo_filtrado(void);

/**
 * @brief Convierte modo a string para debug
 */
const char* mode_to_string(operating_mode_t mode);

/**
 * @brief Convierte estado a string para debug
 */
const char* state_to_string(control_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* TAREAS_H */