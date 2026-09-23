#ifndef ROBUST_CONTROLLER_H
#define ROBUST_CONTROLLER_H

#include <math.h>

typedef struct {
    // Parámetros Físicos del Sistema
    float Em;               // Voltaje pico de fase AC
    float R;                // Resistencia de línea
    float L;                // Inductancia de línea
    float C;                // Capacitancia del bus CD
    float vod;              // Referencia de voltaje del bus CD
    float omega;            // Frecuencia angular de la red
    
    // Parámetros de Control (Tuning)
    float ks, kq;           // Amortiguamiento del puerto
    float gamma;            // Ganancia de adaptación del estimador
    float kappa;            // Ganancia de acción proporcional filtrada
    float lambda_f;         // Ancho de banda del filtro
    float v_bar;            // Cota de saturación de la función tanh
    float ad, aq;           // Límites de saturación SVPWM (rectángulo S)
    
    // Proyección de Estimación [g_min, g_max]
    float g_min, g_max, ep_proj;
    
    // Estados Internos
    float g_hat;            // Estado del estimador g^
    float nu;               // Estado del filtro nu
    float g_r;              // Conductancia de referencia resultante
    float ts;               // Periodo de muestreo
} ControllerParams;

// Saturación simétrica tipo sat_a(u)
static inline float sat(float val, float limit) {
    if (val > limit) return limit;
    if (val < -limit) return -limit;
    return val;
}

// Proyección de actualización de conductancia Proj(g_hat, y)
static inline float Projection(float g_hat, float y, float g_min, float g_max, float ep) {
    float c_plus = 0.0f;
    float c_minus = 0.0f;

    if (g_hat > g_max) c_plus = (g_hat - g_max) / ep;
    if (c_plus > 1.0f) c_plus = 1.0f;

    if (g_min > g_hat) c_minus = (g_min - g_hat) / ep;
    if (c_minus > 1.0f) c_minus = 1.0f;

    float term_pos = (y > 0.0f) ? y : 0.0f;
    float term_neg = (y < 0.0f) ? y : 0.0f;

    return y - c_plus * term_pos - c_minus * term_neg;
}

// Mapeo del equilibrio I(g_r)
static inline float Compute_I_eq(ControllerParams *c, float g_r) {
    float term = c->Em * c->Em - (8.0f / 3.0f) * c->R * c->vod * c->vod * g_r;
    if (term < 0.0f) term = 0.0f;
    return (c->Em - sqrtf(term)) / (2.0f * c->R);
}

// Ejecución de la Ley de Control (Ecuación 16)
static inline void Controller_run(ControllerParams *c, float vo, float id, float iq, float *pd_out, float *pq_out) {
    float e3 = vo - c->vod; // Error de voltaje
    
    // 1. Actualización del filtro del error de voltaje: nu_dot = lambda_f * (e3 - nu)
    c->nu += c->lambda_f * (e3 - c->nu) * c->ts;
    
    // 2. Corrección proporcional saturada: psi(nu) = v_bar * tanh(nu / v_bar)
    float psi_nu = c->v_bar * tanhf(c->nu / c->v_bar);
    
    // 3. Adaptación del estimador de conductancia g_hat_dot con proyección
    float y_grad = -2.0f * c->gamma * c->vod * e3;
    float g_hat_dot = Projection(c->g_hat, y_grad, c->g_min, c->g_max, c->ep_proj);
    c->g_hat += g_hat_dot * c->ts;
    
    // Conductancia de referencia
    c->g_r = c->g_hat - c->kappa * psi_nu;

    // 4. Mapeo a corrientes y alimentaciones anticipadas (Feedforward)
    float I_ref = Compute_I_eq(c, c->g_r);
    float pd_eq = (2.0f * (c->Em - c->R * I_ref)) / c->vod;
    float pq_eq = (-2.0f * c->L * c->omega * I_ref) / c->vod;

    // 5. Errores de puerto
    float e1 = id - I_ref;
    float s_port = I_ref * e3 - c->vod * e1;

    // 6. Leyes de control puerto con saturación
    float ud = -(2.0f * c->ks / 3.0f) * s_port;
    float uq = (2.0f * c->kq / (3.0f * c->vod)) * iq;

    *pd_out = sat(pd_eq + ud, c->ad);
    *pq_out = sat(pq_eq + uq, c->aq);
}

#endif
