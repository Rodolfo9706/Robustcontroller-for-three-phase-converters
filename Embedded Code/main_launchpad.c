#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "transforms_and_pll.h"
#include "robust_controller.h"

// Variables de Control y Transformaciones
TransformVars g_tfVars;
CurrentPLL g_pll;
ControllerParams g_ctrl;

// Offsets de sensores de corriente (Calibración a cero corriente)
float g_offset_ia = 2.5f; 
float g_offset_ib = 2.5f;
float g_offset_ic = 2.5f;
float g_sensor_gain = 10.0f; // Sensibilidad del ACS712 (A/V)

// Prototipo de la ISR
__interrupt void epwm1ISR(void);

// Modulación SVPWM de min-max injection
void Update_SVPWM(float palpha, float pbeta) {
    // Conversión de alpha-beta a marco trifásico equivalente
    float Va = palpha;
    float Vb = -0.5f * palpha + (sqrtf(3.0f) / 2.0f) * pbeta;
    float Vc = -0.5f * palpha - (sqrtf(3.0f) / 2.0f) * pbeta;

    // Inyección de secuencia cero (Space Vector PWM)
    float Vmax = Va > Vb ? (Va > Vc ? Va : Vc) : (Vb > Vc ? Vb : Vc);
    float Vmin = Va < Vb ? (Va < Vc ? Va : Vc) : (Vb < Vc ? Vb : Vc);
    float Vzero = -0.5f * (Vmax + Vmin);

    // Mapeo a Ciclos de Trabajo (Duty Cycles) [0.0, 1.0]
    float Da = 0.5f * (Va + Vzero + 1.0f);
    float Db = 0.5f * (Vb + Vzero + 1.0f);
    float Dc = 0.5f * (Vc + Vzero + 1.0f);

    // Carga de registros de PWM (Periodo = EPWM1_TIMER_TBPRD)
    uint16_t tbprd = EPWM_getTimeBasePeriod(EPWM1_BASE);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Da * tbprd));
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Db * tbprd));
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(Dc * tbprd));
}

void main(void) {
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // 1. Parámetros del Controlador (Tabla I / Tabla V del artículo)
    g_ctrl.Em = 12.0f;              // Voltaje pico bajo (Hardware prototipo)
    g_ctrl.R = 0.1f;
    g_ctrl.L = 0.002f;              // 2 mH
    g_ctrl.C = 0.0025f;             // 2500 uF
    g_ctrl.vod = 24.0f;             // Referencia de voltaje 24 V
    g_ctrl.omega = 2.0f * M_PI * 60.0f;
    
    // Ganancias de Tuning (Tabla V)
    g_ctrl.ks = 0.1f;
    g_ctrl.kq = 100.0f;
    g_ctrl.gamma = 0.0025f;
    g_ctrl.kappa = 0.008f;
    g_ctrl.lambda_f = 1500.0f;
    g_ctrl.v_bar = 1.0f;
    g_ctrl.ad = 1.013f;
    g_ctrl.aq = 0.55f;
    g_ctrl.g_min = 0.01f;
    g_ctrl.g_max = 0.025f;
    g_ctrl.ep_proj = 0.001f;
    g_ctrl.g_hat = 0.015f;
    g_ctrl.nu = 0.0f;
    g_ctrl.ts = 0.0001f;            // Frecuencia de muestreo 10 kHz

    // 2. Inicialización de PLL
    PLL_init(&g_pll, 10.0f, 200.0f, 60.0f, 0.0001f);

    // 3. Configuración de Interrupciones
    Interrupt_register(INT_EPWM1, &epwm1ISR);
    Interrupt_enable(INT_EPWM1);
    
    EINT; // Habilitar Interrupciones Globales
    ERTM;

    while(1) {
        // Bucle principal (tareas de baja prioridad)
    }
}

// Rutina de Interrupción ejecutada a 10 kHz (Sincronizada con EPWM)
__interrupt void epwm1ISR(void) {
    // 1. Lectura ADC (Convertida a Voltios)
    float adc_ia = (float)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0) * (3.0f / 4095.0f);
    float adc_ib = (float)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0) * (3.0f / 4095.0f);
    float adc_vo = (float)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0) * (3.0f / 4095.0f);

    // 2. Reconstrucción de Mediciones Sensadas (Acondicionamiento)
    g_tfVars.ia = (adc_ia - g_offset_ia) * g_sensor_gain;
    g_tfVars.ib = (adc_ib - g_offset_ib) * g_sensor_gain;
    g_tfVars.ic = -g_tfVars.ia - g_tfVars.ib; // Sistema trifásico 3 hilos
    
    float vo_measured = adc_vo * (10.0f); // Escalado del divisor resistivo (p. ej. 1:10)

    // 3. Transformaciones Directas
    Clarke_Transform(&g_tfVars);
    PLL_run(&g_pll, &g_tfVars);
    Park_Transform(&g_tfVars);

    // 4. Ejecución del Controlador Robusto
    Controller_run(&g_ctrl, vo_measured, g_tfVars.id, g_tfVars.iq, &g_tfVars.pd, &g_tfVars.pq);

    // 5. Transformación Inversa de Park
    Inv_Park_Transform(&g_tfVars);

    // 6. Actualización del SVPWM en los Comparadores PWM
    Update_SVPWM(g_tfVars.palpha, g_tfVars.pbeta);

    // Limpieza de bandera de interrupción EPWM
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}
