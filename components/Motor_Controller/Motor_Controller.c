#include <stdio.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "Motor_Controller.h"

static const char *TAG = "Motor_Controller";

// ================= CONFIGURACIÓN DE HARDWARE Y MOTOR =================
#define ENCODER_GPIO           GPIO_NUM_22
#define PWM_GPIO               GPIO_NUM_16

#define PULSES_PER_REV         2.0f   // Pulsos del encoder por vuelta del eje primario
#define GEAR_RATIO             80.0f   // Relación de reducción de la caja de engranajes (1.0f si es directo)

// ================= CONFIGURACIÓN DEL LAZO DE CONTROL =================
#define TIMER_INTERVAL_US      15000   // Periodo de muestreo: 15 ms (66.6 Hz)
#define CONTROL_TS             0.015f  // Tiempo de muestreo en segundos (coincide con TIMER_INTERVAL_US)
#define RPM_MIN_THRESHOLD      15.0f   // Zona muerta: por debajo de estas RPM el motor se apaga
#define AVG_WINDOW             8       // Tamaño de ventana para el filtro de media móvil

// ================= CONFIGURACIÓN PWM (LEDC) =================
#define PWM_FREQ_HZ            10000
#define PWM_RES                LEDC_TIMER_10_BIT
#define PWM_MAX                1023     // Límite superior de ciclo útil (sobre 1023)

// ================= UMBRALES DE SEGURIDAD (WATCHDOG DE CONTROL) =================
#define OVERSPEED_MARGIN_RPM   30.0f   // Margen máximo permitido por encima del setpoint
#define STALL_DUTY_THRESHOLD   200     // Duty a partir del cual se exige respuesta del encoder
#define ERROR_CONFIRM_CYCLES   15      // 15 ciclos * 15 ms = 225 ms de persistencia para confirmar falla

// ================= ESTRUCTURAS DE CONTROL =================
typedef struct {
    float Ts;
    float slope;          // Aceleración máxima en RPM / s
    float sp_ramped;
} Ramp_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Ts;
    float e_k1;
    float y_k;
    float y_k1;
    float y_k2;
    float u_k;
    float u_k1;
    float u_min;
    float u_max;
} PID_t;

// ================= VARIABLES INTERNAS =================
static Ramp_t sp_ramp = {
    .Ts = CONTROL_TS,
    .slope = 100.0f,
    .sp_ramped = 0.0f
};

static PID_t pid = {
    .Kp = 3.5f,
    .Ki = 0.8f,
    .Kd = 0.03f,
    .Ts = CONTROL_TS,
    .u_min = 0.0f,
    .u_max = PWM_MAX
};

static volatile float rpm_setpoint = 0.0f;
static volatile float rpm_filtrado = 0.0f;
static float rpm_filtrado_prev = 0.0f;
static volatile int duty_A = 0;

// Filtro de media móvil
static float rpm_sum = 0.0f;
static float rpm_buffer[AVG_WINDOW] = {0};
static int idx = 0;
static int count = 0;

// Gestión de errores
static motor_error_cb_t s_motor_error_cb = NULL;
static bool s_system_blocked = false;
static uint16_t overspeed_counter = 0;
static uint16_t stall_counter = 0;

// Handles de periféricos
static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_chan = NULL;

// ================= FUNCIONES PRIVADAS =================

static void Reset_Control_States(void)
{
    pid.e_k1 = 0.0f;
    pid.y_k  = 0.0f;
    pid.y_k1 = 0.0f;
    pid.y_k2 = 0.0f;
    pid.u_k  = 0.0f;
    pid.u_k1 = 0.0f;
}

static void trigger_motor_fatal_error(motor_error_code_t err)
{
    s_system_blocked = true;
    Emergency_Stop();
    if (s_motor_error_cb) {
        s_motor_error_cb(err);
    }
}

static float Ramp_Update(Ramp_t *r, float sp_target)
{
    float max_step = r->slope * r->Ts;
    float delta = sp_target - r->sp_ramped;

    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;

    r->sp_ramped += delta;
    return r->sp_ramped;
}

static float PID_Update(PID_t *p, float setpoint, float measurement)
{
    p->y_k = measurement;
    float e = setpoint - p->y_k;

    // Anti-windup (congelamiento del integrador por saturación)
    bool freeze_I = ((p->u_k1 >= p->u_max && e > 0.0f) ||
                     (p->u_k1 <= p->u_min && e < 0.0f));

    // PID Incremental con derivada sobre la medición (evita derivative kick)
    float P = p->Kp * (e - p->e_k1);
    float I = freeze_I ? 0.0f : (p->Ki * p->Ts * e);
    float D = -p->Kd * (p->y_k - 2.0f * p->y_k1 + p->y_k2) / p->Ts;

    p->u_k = p->u_k1 + P + I + D;

    // Saturación de salida
    if (p->u_k > p->u_max) p->u_k = p->u_max;
    if (p->u_k < p->u_min) p->u_k = p->u_min;

    // Actualizar estados
    p->e_k1 = e;
    p->y_k2 = p->y_k1;
    p->y_k1 = p->y_k;
    p->u_k1 = p->u_k;

    return p->u_k;
}

static void rpm_timer_callback(void *arg)
{
    int pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &pulse_count);
    pcnt_unit_clear_count(pcnt_unit);

    float rpm = (pulse_count * 60.0f * 1e6f) /
                (PULSES_PER_REV * TIMER_INTERVAL_US * GEAR_RATIO);

    // Filtro de media móvil circular
    rpm_sum -= rpm_buffer[idx];
    rpm_buffer[idx] = rpm;
    rpm_sum += rpm;
    idx = (idx + 1) % AVG_WINDOW;
    if (count < AVG_WINDOW) count++;
    rpm_filtrado = (count == 0) ? rpm : (rpm_sum / count);

    if (s_system_blocked) {
        duty_A = 0;
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        return;
    }

    float sp_pid = Ramp_Update(&sp_ramp, rpm_setpoint);

    // Evaluación de zona muerta
    if (sp_pid < RPM_MIN_THRESHOLD) {
        duty_A = 0;
        Reset_Control_States();
    } else {
        duty_A = (int)PID_Update(&pid, sp_pid, rpm_filtrado);
    }

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_A);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    // Monitoreo de seguridad en tiempo real
    if (duty_A > 0) {
        bool is_decelerating = (rpm_filtrado < rpm_filtrado_prev);

        // 1. Detección de sobrevelocidad
        if ((rpm_setpoint > 0.0f) &&
            (rpm_filtrado > (rpm_setpoint + OVERSPEED_MARGIN_RPM)) &&
            (rpm_filtrado > (sp_pid + OVERSPEED_MARGIN_RPM)) &&
            !is_decelerating) {
            if (++overspeed_counter >= ERROR_CONFIRM_CYCLES) {
                trigger_motor_fatal_error(MOTOR_ERR_OVERSPEED);
            }
        } else {
            overspeed_counter = 0;
        }

        // 2. Detección de atascamiento (Stall) o desconexión de encoder
        if ((duty_A >= STALL_DUTY_THRESHOLD) && (rpm_filtrado < (sp_pid * 0.5f))) {
            if (++stall_counter >= ERROR_CONFIRM_CYCLES) {
                trigger_motor_fatal_error(MOTOR_ERR_STALL);
            }
        } else {
            stall_counter = 0;
        }
    } else {
        overspeed_counter = 0;
        stall_counter = 0;
    }

    rpm_filtrado_prev = rpm_filtrado;
}

static void pcnt_init(void)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit  = -32768,
    };
    pcnt_new_unit(&unit_config, &pcnt_unit);

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num  = ENCODER_GPIO,
        .level_gpio_num = -1,
    };
    pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);

    pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
}

static void pwm_init(void)
{
    ledc_timer_config_t timerConfig = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = PWM_RES,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timerConfig);

    ledc_channel_config_t channelConfig = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channelConfig);
}

// ================= API PÚBLICA =================

esp_err_t Motor_Controller_Init(void)
{
    pwm_init();
    pcnt_init();

    esp_timer_handle_t rpm_timer;
    const esp_timer_create_args_t timer_args = {
        .callback = &rpm_timer_callback,
        .name = "rpm_control_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &rpm_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(rpm_timer, TIMER_INTERVAL_US));

    ESP_LOGI(TAG, "Controlador PID de motor inicializado correctamente.");
    return ESP_OK;
}

void Set_Motor_Setpoint(float new_setpoint)
{
    if (s_system_blocked) {
        ESP_LOGW(TAG, "Setpoint rechazado: Sistema bloqueado por falla de seguridad.");
        return;
    }
    rpm_setpoint = new_setpoint;
}

float Get_Motor_RPM(void)
{
    return rpm_filtrado;
}

void Emergency_Stop(void)
{
    duty_A = 0;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    Reset_Control_States();
    sp_ramp.sp_ramped = 0.0f;
    rpm_setpoint = 0.0f;
}

void motor_set_error_callback(motor_error_cb_t cb)
{
    s_motor_error_cb = cb;
}