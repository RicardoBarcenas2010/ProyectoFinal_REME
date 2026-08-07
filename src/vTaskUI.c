/**
 * @file vTaskUI.c
 * @brief Tarea de interfaz de usuario (pantalla táctil)
 */

#include "tareas.h"
#include "hardware.h"
#include "hal_display.h"
#include "esp_log.h"

static const char *TAG = "UI";

/* ──────────────────────────────────────────────────────────────
 *  TAREA DE UI
 * ────────────────────────────────────────────────────────────── */

void vTaskUI(void *pvParameters)
{
    (void)pvParameters;

    TouchEvent_t evento;
    SensorData_t sensor_data;
    VisionData_t vision_data;
    ActuadorData_t actuador_data;
    UICommand_t cmd;

    /* Inicializar pantalla */
    if (hal_display_inicializar() != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Pantalla no disponible, tarea suspendida");
        vTaskSuspend(NULL);
        return;
    }

    ESP_LOGI(TAG, "🔄 Tarea de UI iniciada");

    for (;;) {
        /* Tomar mutex para acceder a la pantalla */
        if (xSemaphoreTake(xSemMutexPantalla, pdMS_TO_TICKS(100U)) == pdTRUE) {

            /* ─── Leer datos del sensor ─── */
            if (xQueuePeek(xColaSensorControl, &sensor_data, 0U) == pdPASS) {
                hal_display_mostrar_angulo_propio(sensor_data.angulo_actual);
            }

            /* ─── Leer datos de visión ─── */
            if (xQueuePeek(xColaVisionControl, &vision_data, 0U) == pdPASS) {
                if (vision_data.deteccion_valida) {
                    hal_display_mostrar_angulo_maestro(vision_data.angulo_maestro);
                } else {
                    hal_display_mostrar_mensaje("Sin detección");
                }
            }

            /* ─── Leer datos del actuador ─── */
            if (xQueuePeek(xColaControlActuador, &actuador_data, 0U) == pdPASS) {
                hal_display_mostrar_pwm(actuador_data.pwm_izquierdo,
                                        actuador_data.pwm_derecho);
            }

            /* ─── Mostrar modo seguro ─── */
            hal_display_mostrar_modo_seguro(esta_en_modo_seguro());

            /* ─── Estado de tareas ─── */
            hal_display_mostrar_estado_tareas();

            /* ─── Procesar eventos táctiles ─── */
            while (hal_display_obtener_evento(&evento) == ESP_OK) {
                switch (evento.tipo) {
                    case TOUCH_BUTTON_SETPOINT:
                        cmd.tipo = COMANDO_SETPOINT;
                        cmd.datos.setpoint = evento.valor;
                        xQueueSend(xColaComandosUI, &cmd, 0U);
                        ESP_LOGI(TAG, "📝 Setpoint: %.1f°", evento.valor);
                        break;

                    case TOUCH_BUTTON_PID:
                        cmd.tipo = COMANDO_AJUSTE_PID;
                        cmd.datos.pid.kp = evento.kp;
                        cmd.datos.pid.ki = evento.ki;
                        cmd.datos.pid.kd = evento.kd;
                        xQueueSend(xColaComandosUI, &cmd, 0U);
                        ESP_LOGI(TAG, "📝 PID: Kp=%.2f Ki=%.2f Kd=%.2f",
                                 evento.kp, evento.ki, evento.kd);
                        break;

                    case TOUCH_BUTTON_MODO_AUTO:
                        cmd.tipo = COMANDO_MODO_AUTO;
                        xQueueSend(xColaComandosUI, &cmd, 0U);
                        ESP_LOGI(TAG, "🔄 Modo AUTO");
                        break;

                    case TOUCH_BUTTON_MODO_MANUAL:
                        cmd.tipo = COMANDO_MODO_MANUAL;
                        xQueueSend(xColaComandosUI, &cmd, 0U);
                        ESP_LOGI(TAG, "👆 Modo MANUAL");
                        break;

                    default:
                        break;
                }
            }

            xSemaphoreGive(xSemMutexPantalla);
        }

        vTaskDelay(pdMS_TO_TICKS(50U));  /* 20 Hz */
    }
}