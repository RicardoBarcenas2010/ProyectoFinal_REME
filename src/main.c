/**
 * @file main.c
 * @brief Sistema Seguidor - Equipo B
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

/* ─── TU CÓDIGO EXISTENTE ─── */
#include "wifi_manager.h"
#include "http_server.h"
#include "vision.h"

/* ─── NUEVO CÓDIGO RTOS ─── */
#include "tareas.h"
#include "hardware.h"
#include "control.h"

static const char *TAG = "MAIN";

/* ============================================================
 *  🔧 SWITCH DE PRUEBAS - ACTIVA ETAPA_3
 * ============================================================ */

// #define ETAPA_1
// #define ETAPA_2
//#define ETAPA_3   /* Sistema con sensor + motores + PID (sin cámara ni pantalla) */
#define ETAPA_4
//#define ETAPA_5

/* ============================================================
 *  HOOKS DE SEGURIDAD
 * ============================================================ */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    ESP_LOGE(TAG, "!!! STACK OVERFLOW en tarea: %s", pcTaskName);
    activar_modo_seguro(MOTIVO_STACK_OVERFLOW);
    while (1) { }
}

void vApplicationMallocFailedHook(void)
{
    ESP_LOGE(TAG, "!!! FALLA DE MEMORIA");
    activar_modo_seguro(MOTIVO_MEMORIA);
    while (1) { }
}

/* ============================================================
 *  FUNCIÓN PRINCIPAL
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "=== SISTEMA SEGUIDOR (Equipo B) ===");
    
    #ifdef ETAPA_1
        ESP_LOGI(TAG, "🔧 ETAPA 1: Solo WiFi + HTTP");
    #elif defined(ETAPA_2)
        ESP_LOGI(TAG, "🔧 ETAPA 2: WiFi + HTTP + Sensor");
    #elif defined(ETAPA_3)
        ESP_LOGI(TAG, "🔧 ETAPA 3: Sensor + Motores + PID (SIN cámara ni pantalla)");
    #elif defined(ETAPA_4)
        ESP_LOGI(TAG, "🔧 ETAPA 4: WiFi + HTTP + Sensor + Motores + Pantalla");
    #elif defined(ETAPA_5)
        ESP_LOGI(TAG, "🔧 ETAPA 5: Sistema COMPLETO con PID");
    #else
        #error "Debes definir una ETAPA (1-5)"
    #endif

    /* 1. Inicializar NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ⭐ INICIALIZAR UART PARA CONTROL POR TECLADO ⭐ */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "✅ UART inicializado para control por teclado");

    /* 2. Inicializar WiFi (SOLO si es necesario) */
    #if defined(ETAPA_1) || defined(ETAPA_2) || defined(ETAPA_4) || defined(ETAPA_5)
        wifi_init_sta();
        
        /* Obtener MAC solo si es necesario */
        // uint8_t mac[6];
        // esp_read_mac(mac, ESP_MAC_WIFI_STA);
        // ESP_LOGI(TAG, "📡 MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
        //          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    #endif

    /* 3. Inicializar según etapa */
    #ifdef ETAPA_1
        ESP_LOGI(TAG, "📡 Ejecutando tu código original...");
        init_vision();
        xTaskCreate(vision_task, "vision_task", 4096, NULL, 6, NULL);
        ESP_LOGI(TAG, "✅ Sistema ETAPA 1 iniciado");
        while (1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
        return;
    #endif

    /* ETAPA 2+ */
    if (hal_inicializar() != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar hardware");
        return;
    }

    /* ETAPA 3+ */
    #if defined(ETAPA_3) || defined(ETAPA_4) || defined(ETAPA_5)
        if (hal_motores_inicializar() != ESP_OK) {
            ESP_LOGE(TAG, "Error al inicializar motores");
            return;
        }
    #endif

    /* Inicializar controlador (ETAPA 3, 4, 5) */
    #if defined(ETAPA_3) || defined(ETAPA_4) || defined(ETAPA_5)
        if (control_inicializar() != ESP_OK) {
            ESP_LOGE(TAG, "Error al inicializar controlador");
            return;
        }
    #endif

    /* 4. Crear colas */
    if (crear_colas_y_semaforos() != pdPASS) {
        ESP_LOGE(TAG, "Error al crear colas/semáforos");
        return;
    }

    /* 5. Inicializar visión (SOLO si es necesario) */
    #if defined(ETAPA_1) || defined(ETAPA_2) || defined(ETAPA_4) || defined(ETAPA_5)
        init_vision();
    #endif

    /* 6. Crear tareas */
    if (crear_tareas_segun_etapa() != pdPASS) {
        ESP_LOGE(TAG, "Error al crear tareas");
        return;
    }

    ESP_LOGI(TAG, "✅ Sistema iniciado correctamente");
    ESP_LOGI(TAG, "📖 Presiona '+' para aumentar setpoint, '-' para disminuir");
    
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* ============================================================
 *  CREACIÓN DE TAREAS SEGÚN ETAPA
 * ============================================================ */

BaseType_t crear_tareas_segun_etapa(void)
{
    BaseType_t xRet = pdPASS;

    /* ─── Visión - SOLO ETAPAS con WiFi ─── */
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
        ESP_LOGI(TAG, "✅ vision_task creada");
    #endif

    /* ─── Sensor - ETAPA 2+ ─── */
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
        ESP_LOGI(TAG, "✅ vTaskSensor creada");
    #endif

    /* ─── Control - ETAPA 3+ ─── */
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
        ESP_LOGI(TAG, "✅ vTaskControl creada");
    #endif

    /* ─── Actuador - ETAPA 3+ ─── */
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
        ESP_LOGI(TAG, "✅ vTaskActuador creada");
    #endif

    /* ─── UI - SOLO ETAPA 4+ ─── */
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
        ESP_LOGI(TAG, "✅ vTaskUI creada");
    #endif

    /* ─── Watchdog - SOLO ETAPA 4+ ─── */
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
        ESP_LOGI(TAG, "✅ vTaskWatchdogUI creada");
    #endif

    /* ─── Monitor - SIEMPRE ─── */
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
    ESP_LOGI(TAG, "✅ vTaskMonitor creada");

    return pdPASS;
}