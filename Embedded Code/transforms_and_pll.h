#ifndef TRANSFORMS_AND_PLL_H
#define TRANSFORMS_AND_PLL_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Estructura para transformaciones Park/Clarke
typedef struct {
    float ia, ib, ic;       // Corrientes tridásicas medidas
    float ialpha, ibeta;   // Marco estacionario alpha-beta
    float id, iq;           // Marco rotatorio d-q
    float pd, pq;           // Entradas de control d-q
    float palpha, pbeta;   // Salidas de control alpha-beta
    float sin_th, cos_th;   // Seno y coseno del ángulo de la PLL
} TransformVars;

// Estructura para la SRF-PLL basada en Corriente
typedef struct {
    float kp;               // Ganancia proporcional PLL
    float ki;               // Ganancia integral PLL
    float integrator;       // Estado del integrador
    float phase;            // Ángulo estimado theta [0, 2pi]
    float w_base;           // Frecuencia base (2 * pi * 60)
    float w_out;            // Frecuencia calculada
    float ts;               // Tiempo de muestreo (1 / 10kHz = 0.0001s)
} CurrentPLL;

// Inicialización de la PLL
static inline void PLL_init(CurrentPLL *pll, float kp, float ki, float f_base, float ts) {
    pll->kp = kp;
    pll->ki = ki;
    pll->integrator = 0.0f;
    pll->phase = 0.0f;
    pll->w_base = 2.0f * M_PI * f_base;
    pll->w_out = pll->w_base;
    pll->ts = ts;
}

// Clarke: abc -> alpha/beta (Invariante en amplitud)
static inline void Clarke_Transform(TransformVars *v) {
    v->ialpha = v->ia;
    v->ibeta = (1.0f / sqrtf(3.0f)) * (v->ia + 2.0f * v->ib);
}

// Park: alpha/beta -> d/q
static inline void Park_Transform(TransformVars *v) {
    v->id =  v->ialpha * v->cos_th + v->ibeta * v->sin_th;
    v->iq = -v->ialpha * v->sin_th + v->ibeta * v->sin_th; // Orientado a corriente
}

// Park Inversa: d/q -> alpha/beta
static inline void Inv_Park_Transform(TransformVars *v) {
    v->palpha = v->pd * v->cos_th - v->pq * v->sin_th;
    v->pbeta  = v->pd * v->sin_th + v->pq * v->cos_th;
}

// Ejecución de SRF-PLL Normalizada en corriente
static inline void PLL_run(CurrentPLL *pll, TransformVars *v) {
    // Normalización de vector para mantener constante la ganancia a bajas corrientes
    float mag = sqrtf(v->ialpha * v->ialpha + v->ibeta * v->ibeta);
    float err = 0.0f;
    
    if (mag > 0.05f) { // Umbral mínimo de corriente (evita división por cero)
        float ialpha_norm = v->ialpha / mag;
        float ibeta_norm  = v->ibeta / mag;
        // Error de fase: sin(delta_theta) = -i_alpha*sin(theta) + i_beta*cos(theta)
        err = -ialpha_norm * v->sin_th + ibeta_norm * v->cos_th;
    }

    // PI Loop Filter
    pll->integrator += pll->ki * err * pll->ts;
    pll->w_out = pll->w_base + pll->kp * err + pll->integrator;

    // Integración de ángulo
    pll->phase += pll->w_out * pll->ts;
    if (pll->phase >= 2.0f * M_PI) pll->phase -= 2.0f * M_PI;
    if (pll->phase < 0.0f) pll->phase += 2.0f * M_PI;

    // Actualización de seno y coseno
    v->sin_th = sinf(pll->phase);
    v->cos_th = cosf(pll->phase);
}

#endif
