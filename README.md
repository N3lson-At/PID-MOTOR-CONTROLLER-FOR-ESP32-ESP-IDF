# ESP-IDF DC Motor PID Controller

Un componente modular, robusto y de grado industrial para el control de velocidad en lazo cerrado de motores DC utilizando microcontroladores ESP32 (ESP-IDF v5.x).

A diferencia de los enfoques académicos tradicionales, este componente integra características de seguridad de hardware en tiempo real (detección de *Stall* y sobrevelocidad) y un algoritmo PID diseñado para mitigar efectos indeseados en la planta.

---

## 🚀 Características Principales

* **PID Incremental con Anti-Windup:** Implementación digital en velocidad que congela el integrador ante la saturación de salida, evitando sobreimpulsos agresivos (*overshoot*).
* **Derivada sobre la Medición:** Calcula la acción derivativa utilizando la segunda diferencia del sensor (`y_k`, `y_k1`, `y_k2`), eliminando por completo el *derivative kick* (picos de corriente) cuando cambia el setpoint.
* **Rampa de Aceleración Suave (`Ramp_t`):** Limita dinámicamente la aceleración máxima permitida (en RPM/s) para proteger la mecánica del sistema y la electrónica de potencia.
* **Filtrado de Feedback por Media Móvil:** Ventana circular configurable para suavizar el ruido eléctrico y mecánico de las lecturas del encoder.
* **Periféricos Nativos de ESP-IDF:** 
  * Utiliza el driver **PCNT** (Pulse Counter) de la versión actual con filtro anti-glitches por hardware (1000 ns).
  * Emplea el controlador **LEDC** para la generación de PWM de alta frecuencia.
  * Ejecuta el lazo de control de forma determinista mediante **`esp_timer`** en segundo plano.
* **Watchdog de Seguridad Integrado:** Monitoreo en tiempo real de atascamiento (*Stall*) por falta de feedback y sobrevelocidad (*Overspeed*), con callbacks asíncronos para paradas de emergencia.

---

## 📂 Estructura del Repositorio
```text
.
├── CMakeLists.txt
├── README.md
├── include
│   └── Motor_Controller.h
├── Motor_Controller.c
└── examples
    └── main_test
        ├── CMakeLists.txt
        └── main.c
```
## ⚙️ Parámetros Configurables

En la parte superior de `Motor_Controller.c` encontrarás todas las macros y constantes para adaptar el controlador a tu motor, tu electrónica de potencia y tus requerimientos dinámicos:

### 1. Mapeo de Hardware y Mecánica
```c
#define ENCODER_GPIO           GPIO_NUM_22  // Pin conectado al canal de pulsos del encoder
#define PWM_GPIO               GPIO_NUM_4   // Pin de control PWM hacia el driver del motor
#define PULSES_PER_REV         2.0f        // Pulsos del encoder por vuelta del eje primario
#define GEAR_RATIO             80.0f        // Relación de reducción de la caja de engranajes (1.0f si es directo)
```
### 2. Dinámica del Lazo y Rampa
```c
#define TIMER_INTERVAL_US      15000        // Periodo de ejecución del lazo (15 ms / 66.6 Hz)
#define CONTROL_TS             0.015f       // Tiempo de muestreo en segundos (coincide con el timer)
#define RPM_MIN_THRESHOLD      15.0f        // Zona muerta: por debajo de este setpoint el motor se apaga
#define AVG_WINDOW             8            // Tamaño de la ventana circular para el filtro de media móvil (RPM)
```
### 3. PWM y Límites de Potencia
```c
#define PWM_FREQ_HZ            10000        // Frecuencia de la señal PWM (10 kHz)
#define PWM_RES                LEDC_TIMER_10_BIT // Resolución del temporizador LEDC (0 - 1023)
#define PWM_MAX                600          // Límite superior de ciclo útil permitido (saturación del PID)
```
### 4. Sintonización del PID y la Rampa
```c
// Estructura de la rampa de aceleración
static Ramp_t sp_ramp = {
    .Ts = CONTROL_TS,
    .slope = 100.0f,      // Aceleración máxima permitida en RPM / segundo
    .sp_ramped = 0.0f
};

// Constantes del controlador PID Incremental
static PID_t pid = {
    .Kp = 3.5f,           // Ganancia Proporcional
    .Ki = 0.8f,           // Ganancia Integral
    .Kd = 0.03f,          // Ganancia Derivativa
    .Ts = CONTROL_TS,
    .u_min = 0.0f,
    .u_max = PWM_MAX
};
```
### 5. Watchdog de Seguridad (Monitoreo de Fallas)
```c
#define OVERSPEED_MARGIN_RPM   30.0f        // Margen de tolerancia sobre el setpoint antes de detectar sobrevelocidad
#define STALL_DUTY_THRESHOLD   10           // Duty mínimo a partir del cual se exige respuesta del encoder
#define ERROR_CONFIRM_CYCLES   15           // Ciclos de persistencia (15 * 15ms = 225ms) para confirmar una falla
```
## 📜 Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo [LICENSE] para más detalles.
