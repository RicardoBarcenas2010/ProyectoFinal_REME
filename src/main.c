/**
 * @file main.c
 * @brief Sistema Seguidor - Equipo B
 * 
 * Punto de entrada del sistema. Inicializa hardware, RTOS y crea las tareas.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "wifi_manager.h"
#include "http_server.h"
#include "vision.h"
#include "tareas.h"
#include "hardware.h"
#include "control.h"
#include "espnow_display.h"
#include "espnow_receiver.h"

static const char *TAG = "MAIN";

/* ================================================================
 *  SELECCIÓN DE ETAPA DE PRUEBA
 * ================================================================ */
/* #define ETAPA_1 */
/* #define ETAPA_2 */
/* #define ETAPA_3 */
#define ETAPA_4
/* #define ETAPA_5 */

/* ================================================================
 *  CREACIÓN DE TAREAS SEGÚN ETAPA (ESTÁTICA - DECLARADA ANTES DE USARSE)
 * ================================================================ */
static BaseType_t crear_tareas_segun_etapa(void)
{
    BaseType_t xRet = pdPASS;

    #if defined(ETAPA_1) || defined(ETAPA_2) || defined(ETAPA_4) || defined(ETAPA_5)
        xRet = xTaskCreatePinnedToCore(
            vision_task,
            "Vision",
            STACK_VISION,
            NULL,
            PRIORIDAD_VISION,
            NULL,
            tskNO_AFFINITY
        );
        if (xRet != pdPASS) {
            ESP_LOGE(TAG, "Fallo al crear vision_task");
            return xRet;
        }
        ESP_LOGI(TAG, "vision_task creada");
    #endif

    #if defined(ETAPA_2) || defined(ETAPA_3) || defined(ETAPA_4) || defined(ETAPA_5)
        xRet = xTaskCreatePinnedToCore(
            vTaskSensor,
            "Sensor",
            STACK_SENSOR,
            NULL,
            PRIORIDAD_SENSOR,
            NULL,
            tskNO_AFFINITY
        );
        if (xRet != pdPASS) {
            ESP_LOGE(TAG, "Fallo al crear vTaskSensor");
            return xRet;
        }
        ESP_LOGI(TAG, "vTaskSensor creada");
    #endif

    #if defined(ETAPA_3) || defined(ETAPA_4) || defined(ETAPA_5)
        xRet = xTaskCreatePinnedToCore(
            vTaskControl,
            "Control",
            STACK_CONTROL,
            NULL,
            PRIORIDAD_CONTROL,
            NULL,
            tskNO_AFFINITY
        );
        if (xRet != pdPASS) {
            ESP_LOGE(TAG, "Fallo al crear vTaskControl");
            return xRet;
        }
        ESP_LOGI(TAG, "vTaskControl creada");
    #endif

    #if defined(ETAPA_3) || defined(ETAPA_4) || defined(ETAPA_5)
        xRet = xTaskCreatePinnedToCore(
            vTaskActuador,
            "Actuador",
            STACK_ACTUADOR,
            NULL,
            PRIORIDAD_ACTUADOR,
            NULL,
            tskNO_AFFINITY
        );
        if (xRet != pdPASS) {
            ESP_LOGE(TAG, "Fallo al crear vTaskActuador");
            return xRet;
        }
        ESP_LOGI(TAG, "vTaskActuador creada");
    #endif

    #if defined(ETAPA_4) || defined(ETAPA_5)
        xRet = xTaskCreatePinnedToCore(
            vTaskUI,
            "UI",
            STACK_UI,
            NULL,
            PRIORIDAD_UI,
            NULL,
            tskNO_AFFINITY
        );
        if (xRet != pdPASS) {
            ESP_LOGE(TAG, "Fallo al crear vTaskUI");
            return xRet;
        }
        ESP_LOGI(TAG, "vTaskUI creada");
    #endif

    #if defined(ETAPA_4) || defined(ETAPA_5)
        xRet = xTaskCreatePinnedToCore(
            vTaskWatchdogUI,
            "WatchdogUI",
            STACK_WATCHDOG_UI,
            NULL,
            PRIORIDAD_WATCHDOG_UI,
            NULL,
            tskNO_AFFINITY
        );
        if (xRet != pdPASS) {
            ESP_LOGE(TAG, "Fallo al crear vTaskWatchdogUI");
            return xRet;
        }
        ESP_LOGI(TAG, "vTaskWatchdogUI creada");
    #endif

    xRet = xTaskCreatePinnedToCore(
        vTaskMonitor,
        "Monitor",
        STACK_MONITOR,
        NULL,
        PRIORIDAD_MONITOR,
        NULL,
        tskNO_AFFINITY
    );
    if (xRet != pdPASS) {
        ESP_LOGE(TAG, "Fallo al crear vTaskMonitor");
        return xRet;
    }
    ESP_LOGI(TAG, "vTaskMonitor creada");

    return pdPASS;
}

/* ================================================================
 *  HOOKS DE SEGURIDAD
 * ================================================================ */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    ESP_LOGE(TAG, "Stack overflow en tarea: %s", pcTaskName);
    activar_modo_seguro(MOTIVO_STACK_OVERFLOW);
    while (1) { }
}

void vApplicationMallocFailedHook(void)
{
    ESP_LOGE(TAG, "Falla de memoria");
    activar_modo_seguro(MOTIVO_MEMORIA);
    while (1) { }
}

/* ================================================================
 *  FUNCIÓN PRINCIPAL
 * ================================================================ */

void app_main(void)
{
    esp_err_t ret = ESP_OK;
    uart_config_t uart_config;
    uint8_t channel = 0U;
    wifi_second_chan_t second = 0U;

    ESP_LOGI(TAG, "Sistema Seguidor - Equipo B");

    #ifdef ETAPA_1
        ESP_LOGI(TAG, "Etapa 1: WiFi + HTTP");
    #elif defined(ETAPA_2)
        ESP_LOGI(TAG, "Etapa 2: WiFi + HTTP + Sensor");
    #elif defined(ETAPA_3)
        ESP_LOGI(TAG, "Etapa 3: Sensor + Motores + PID");
    #elif defined(ETAPA_4)
        ESP_LOGI(TAG, "Etapa 4: WiFi + HTTP + Sensor + Motores + Pantalla");
    #elif defined(ETAPA_5)
        ESP_LOGI(TAG, "Etapa 5: Sistema completo");
    #else
        #error "Definir una etapa (1-5)"
    #endif

    /* Inicializar NVS */
    ret = nvs_flash_init();
    if ((ret == ESP_ERR_NVS_NO_FREE_PAGES) || (ret == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Inicializar UART para comandos por teclado */
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "UART inicializado");

    /* Inicializar WiFi */
    #if defined(ETAPA_1) || defined(ETAPA_2) || defined(ETAPA_4) || defined(ETAPA_5)
        wifi_init_sta();
        ESP_ERROR_CHECK(esp_wifi_get_channel(&channel, &second));
        ESP_LOGI(TAG, "Canal WiFi: %d", (int)channel);
        ESP_LOGI(TAG, "Inicializando ESP-NOW...");
        ESP_ERROR_CHECK(espnow_display_init());
        ESP_ERROR_CHECK(espnow_receiver_init());
    #endif

    /* Inicializar según etapa */
    #ifdef ETAPA_1
        ESP_LOGI(TAG, "Ejecutando etapa 1...");
        init_vision();
        xTaskCreate(vision_task, "vision_task", 4096, NULL, 6, NULL);
        ESP_LOGI(TAG, "Sistema iniciado");
        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        return;
    #endif

    /* Inicializar hardware */
    if (hal_inicializar() != ESP_OK) {
        ESP_LOGE(TAG, "Error en inicialización de hardware");
        return;
    }

    /* Inicializar motores */
    #if defined(ETAPA_3) || defined(ETAPA_4) || defined(ETAPA_5)
        if (hal_motores_inicializar() != ESP_OK) {
            ESP_LOGE(TAG, "Error en inicialización de motores");
            return;
        }
    #endif

    /* Inicializar controlador */
    #if defined(ETAPA_3) || defined(ETAPA_4) || defined(ETAPA_5)
        if (control_inicializar() != ESP_OK) {
            ESP_LOGE(TAG, "Error en inicialización del controlador");
            return;
        }
    #endif

    /* Crear colas y semáforos */
    if (crear_colas_y_semaforos() != pdPASS) {
        ESP_LOGE(TAG, "Error al crear colas y semáforos");
        return;
    }

    /* Inicializar visión */
    #if defined(ETAPA_1) || defined(ETAPA_2) || defined(ETAPA_4) || defined(ETAPA_5)
        init_vision();
    #endif

    /* Crear tareas */
    if (crear_tareas_segun_etapa() != pdPASS) {
        ESP_LOGE(TAG, "Error al crear tareas");
        return;
    }

    ESP_LOGI(TAG, "Sistema iniciado correctamente");

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}