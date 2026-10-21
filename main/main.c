#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Motor_Controller.h"

static const char *TAG = "MAIN_APP";

// Callback para capturar errores críticos del lazo de control
static void motor_error_handler(motor_error_code_t err)
{
    switch (err) {
        case MOTOR_ERR_OVERSPEED:
            ESP_LOGE(TAG, "¡FALLA CRÍTICA! Sobrevelocidad detectada (MOTOR_ERR_OVERSPEED). Motor detenido.");
            break;
        case MOTOR_ERR_STALL:
            ESP_LOGE(TAG, "¡FALLA CRÍTICA! Motor atascado o sin lectura de encoder (MOTOR_ERR_STALL). Motor detenido.");
            break;
        default:
            ESP_LOGE(TAG, "¡FALLA CRÍTICA! Error desconocido (%d).", err);
            break;
    }
}

// Tarea opcional para monitorear la respuesta del PID por consola
static void motor_monitor_task(void *pvParameters)
{
    while (1) {
        // Formato amigable para Serial Plotter o lectura directa en terminal
        ESP_LOGI(TAG, "Velocidad actual: %.2f RPM", Get_Motor_RPM());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando prueba del controlador PID...");

    // 1. Registrar el callback de seguridad antes de arrancar
    motor_set_error_callback(motor_error_handler);

    // 2. Inicializar periféricos (PWM, PCNT y Timer de 15 ms)
    ESP_ERROR_CHECK(Motor_Controller_Init());

    // 3. Crear tarea de telemetría en segundo plano
    xTaskCreate(motor_monitor_task, "motor_monitor", 2048, NULL, 5, NULL);

    // --- SECUENCIA DE PRUEBA ---

    // Paso 1: Arranque suave a 60 RPM durante 8 segundos
    ESP_LOGI(TAG, "-> Setpoint: 60.0 RPM");
    Set_Motor_Setpoint(60.0f);
    vTaskDelay(pdMS_TO_TICKS(15000));

    // Paso 2: Cambio de escalón a 100 RPM durante 8 segundos
    ESP_LOGI(TAG, "-> Setpoint: 80.0 RPM");
    Set_Motor_Setpoint(80.0f);
    vTaskDelay(pdMS_TO_TICKS(15000));

    // Paso 3: Reducción suave a 40 RPM durante 6 segundos
    ESP_LOGI(TAG, "-> Setpoint: 40.0 RPM");
    Set_Motor_Setpoint(40.0f);
    vTaskDelay(pdMS_TO_TICKS(15000));

    // Paso 4: Apagado por rampa llevando el setpoint a 0 RPM
    ESP_LOGI(TAG, "-> Setpoint: 0.0 RPM (Desaceleración controlada)");
    Set_Motor_Setpoint(0.0f);
    vTaskDelay(pdMS_TO_TICKS(15000));

    // Paso 5: Confirmar parada total
    Emergency_Stop();
    ESP_LOGI(TAG, "Secuencia de prueba finalizada.");
}