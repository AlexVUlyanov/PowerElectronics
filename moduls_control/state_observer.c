/**
  ******************************************************************************
  * @file    state_observer.c
  * @brief   Реализация наблюдателя состояния для оценки положения ротора
  ******************************************************************************
  */

#include "state_observer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Инициализация наблюдателя */
void Observer_Init(ObserverParams_t* params, ObserverState_t* state)
{
    /* Инициализация параметров по умолчанию */
    params->Ts = 0.001f;    /* 1 мс */
    params->R = 1.0f;       /* Ом */
    params->Ld = 0.001f;    /* Гн */
    params->Lq = 0.001f;    /* Гн */
    params->Ke = 0.1f;      /* В/рад/с */
    params->J = 0.001f;     /* кг*м^2 */
    params->B = 0.001f;     /* Н*м*с/рад */
    params->K1 = 100.0f;    /* Коэффициент 1 */
    params->K2 = 1000.0f;   /* Коэффициент 2 */

    /* Инициализация состояния */
    state->theta_hat = 0.0f;
    state->omega_hat = 0.0f;
    state->id_hat = 0.0f;
    state->iq_hat = 0.0f;
    state->ed_hat = 0.0f;
    state->eq_hat = 0.0f;
}

/* Обновление состояния наблюдателя */
void Observer_Update(ObserverParams_t* params, ObserverState_t* state, 
                    ObserverInputs_t* inputs)
{
    float Ts = params->Ts;
    float R = params->R;
    float Ld = params->Ld;
    float Lq = params->Lq;
    float Ke = params->Ke;
    float K1 = params->K1;
    float K2 = params->K2;

    /* Ошибки оценки тока */
    float id_error = inputs->id - state->id_hat;
    float iq_error = inputs->iq - state->iq_hat;

    /* Обновление оценки ЭДС */
    state->ed_hat += Ts * (K1 * id_error);
    state->eq_hat += Ts * (K1 * iq_error);

    /* Обновление оценки скорости */
    float omega_error = (state->eq_hat * cosf(state->theta_hat) - 
                        state->ed_hat * sinf(state->theta_hat)) / Ke;
    state->omega_hat += Ts * (K2 * omega_error);

    /* Обновление оценки угла */
    state->theta_hat += Ts * state->omega_hat;
    if (state->theta_hat > 2.0f * M_PI)
        state->theta_hat -= 2.0f * M_PI;
    if (state->theta_hat < 0.0f)
        state->theta_hat += 2.0f * M_PI;

    /* Обновление оценки тока */
    state->id_hat += Ts * ((inputs->vd - R * state->id_hat + 
                           state->omega_hat * Lq * state->iq_hat) / Ld);
    state->iq_hat += Ts * ((inputs->vq - R * state->iq_hat - 
                           state->omega_hat * Ld * state->id_hat - 
                           Ke * state->omega_hat) / Lq);
}

/* Получение оценки угла */
float Observer_GetAngle(ObserverState_t* state)
{
    return state->theta_hat;
}

/* Получение оценки скорости */
float Observer_GetSpeed(ObserverState_t* state)
{
    return state->omega_hat;
}

/* Настройка коэффициентов наблюдателя */
void Observer_SetGains(ObserverParams_t* params, float k1, float k2)
{
    params->K1 = k1;
    params->K2 = k2;
} 