/**
  ******************************************************************************
  * @file    state_observer.h
  * @brief   Наблюдатель состояния для оценки положения ротора
  ******************************************************************************
  */

#ifndef STATE_OBSERVER_H
#define STATE_OBSERVER_H

#include <stdint.h>

/* Структура параметров наблюдателя */
typedef struct {
    float Ts;           /* Период дискретизации */
    float R;            /* Сопротивление статора */
    float Ld;           /* Индуктивность по оси d */
    float Lq;           /* Индуктивность по оси q */
    float Ke;           /* Коэффициент ЭДС */
    float J;            /* Момент инерции */
    float B;            /* Коэффициент вязкого трения */
    float K1;           /* Коэффициент наблюдателя 1 */
    float K2;           /* Коэффициент наблюдателя 2 */
} ObserverParams_t;

/* Структура состояния наблюдателя */
typedef struct {
    float theta_hat;    /* Оценка угла */
    float omega_hat;    /* Оценка скорости */
    float id_hat;       /* Оценка тока по оси d */
    float iq_hat;       /* Оценка тока по оси q */
    float ed_hat;       /* Оценка ЭДС по оси d */
    float eq_hat;       /* Оценка ЭДС по оси q */
} ObserverState_t;

/* Структура входных сигналов */
typedef struct {
    float vd;           /* Напряжение по оси d */
    float vq;           /* Напряжение по оси q */
    float id;           /* Измеренный ток по оси d */
    float iq;           /* Измеренный ток по оси q */
} ObserverInputs_t;

/* Инициализация наблюдателя */
void Observer_Init(ObserverParams_t* params, ObserverState_t* state);

/* Обновление состояния наблюдателя */
void Observer_Update(ObserverParams_t* params, ObserverState_t* state, 
                    ObserverInputs_t* inputs);

/* Получение оценки угла */
float Observer_GetAngle(ObserverState_t* state);

/* Получение оценки скорости */
float Observer_GetSpeed(ObserverState_t* state);

/* Настройка коэффициентов наблюдателя */
void Observer_SetGains(ObserverParams_t* params, float k1, float k2);

#endif /* STATE_OBSERVER_H */ 