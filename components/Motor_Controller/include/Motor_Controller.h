#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================= ERRORES DEL MOTOR =================
typedef enum {
    MOTOR_ERR_NONE = 0,
    MOTOR_ERR_OVERSPEED,       // Embalamiento / Sobrevelocidad sin control
    MOTOR_ERR_STALL            // Motor atascado o encoder desconectado (Duty alto, bajas RPM)
} motor_error_code_t;

typedef void (*motor_error_cb_t)(motor_error_code_t err);

// ================= API PÚBLICA =================

/**
 * @brief Inicializa periféricos (PCNT, PWM LEDC y Timer periódico del lazo de control).
 */
esp_err_t Motor_Controller_Init(void);

/**
 * @brief Establece el setpoint de velocidad en RPM (pasa por la rampa de aceleración).
 */
void Set_Motor_Setpoint(float new_setpoint);

/**
 * @brief Obtiene la velocidad actual filtrada en RPM.
 */
float Get_Motor_RPM(void);

/**
 * @brief Aplica una parada de emergencia inmediata, apagando el PWM y reseteando el PID.
 */
void Emergency_Stop(void);

/**
 * @brief Registra un callback para notificar errores críticos (Stall / Overspeed).
 */
void motor_set_error_callback(motor_error_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROLLER_H