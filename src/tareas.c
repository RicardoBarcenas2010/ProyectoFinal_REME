/**
 * @file tareas.c
 * @brief Implementación de colas, modo seguro y funciones globales
 */

#include "tareas.h"
#include "hardware.h"
#include "hal_led.h"
#include "esp_log.h"

static const char *TAG = "TAREAS";

/* ================================================================
 *  VARIABLES GLOBALES
 * ================================================================ */
QueueHandle_t xColaSensorControl = NULL;
QueueHandle_t xColaControlActuador = NULL;
QueueHandle_t xColaVisionControl = NULL;
QueueHandle_t xColaComandosUI = NULL;
SemaphoreHandle_t xSemMutexPantalla = NULL;
SemaphoreHandle_t xSemModoSeguro = NULL;

static volatile bool s_modo_seguro_activo = false;
static volatile uint8_t s_motivo_seguro = 0U;

/* ================================================================
 *  CREACIÓN DE COLAS Y SEMÁFOROS
 * ================================================================ */
BaseType_t crear_colas_y_semaforos(void)
{
    xColaSensorControl = xQueueCreate(1U, sizeof(SensorData_t));
    if (xColaSensorControl == NULL) {
        ESP_LOGE(TAG, "Fallo al crear xColaSensorControl");
        return pdFAIL;
    }

    xColaControlActuador = xQueueCreate(1U, sizeof(ActuadorData_t));
    if (xColaControlActuador == NULL) {
        ESP_LOGE(TAG, "Fallo al crear xColaControlActuador");
        return pdFAIL;
    }

    xColaVisionControl = xQueueCreate(1U, sizeof(VisionData_t));
    if (xColaVisionControl == NULL) {
        ESP_LOGE(TAG, "Fallo al crear xColaVisionControl");
        return pdFAIL;
    }

    xColaComandosUI = xQueueCreate(10U, sizeof(UICommand_t));
    if (xColaComandosUI == NULL) {
        ESP_LOGE(TAG, "Fallo al crear xColaComandosUI");
        return pdFAIL;
    }

    xSemMutexPantalla = xSemaphoreCreateMutex();
    if (xSemMutexPantalla == NULL) {
        ESP_LOGE(TAG, "Fallo al crear xSemMutexPantalla");
        return pdFAIL;
    }

    xSemModoSeguro = xSemaphoreCreateBinary();
    if (xSemModoSeguro == NULL) {
        ESP_LOGE(TAG, "Fallo al crear xSemModoSeguro");
        return pdFAIL;
    }

    ESP_LOGI(TAG, "Colas y semáforos creados");
    return pdPASS;
}

/* ================================================================
 *  MODO SEGURO
 * ================================================================ */
void activar_modo_seguro(uint8_t motivo)
{
    if (s_modo_seguro_activo) return;

    s_modo_seguro_activo = true;
    s_motivo_seguro = motivo;

    ESP_LOGW(TAG, "⚠️ MODO SEGURO activado - Motivo: %d", motivo);

    hal_led_encender();

    ActuadorData_t emergencia = { .pwm_izquierdo = 0.0f, .pwm_derecho = 0.0f };
    if (xColaControlActuador != NULL) {
        xQueueOverwrite(xColaControlActuador, &emergencia);
    }
}

void restaurar_desde_modo_seguro(void)
{
    if (!s_modo_seguro_activo) return;

    ESP_LOGW(TAG, "Restaurando desde modo seguro");
    s_modo_seguro_activo = false;
    s_motivo_seguro = 0U;
    hal_led_apagar();
}

bool esta_en_modo_seguro(void)
{
    return s_modo_seguro_activo;
}