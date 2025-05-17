/**
  ******************************************************************************
  * @file    pid_controller.h
  * @brief   PID Controller module header file
  ******************************************************************************
  */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

/* Структура для хранения параметров PID-регулятора */
typedef struct {
    int16_t hKpGain;           /* Пропорциональный коэффициент */
    int16_t hKiGain;           /* Интегральный коэффициент */
    int16_t hKdGain;           /* Дифференциальный коэффициент */
    
    uint16_t hKpDivisorPOW2;   /* Делитель для Kp (степень двойки) */
    uint16_t hKiDivisorPOW2;   /* Делитель для Ki (степень двойки) */
    uint16_t hKdDivisorPOW2;   /* Делитель для Kd (степень двойки) */

  /*Kp = 0.5 (hKpGain = 1, hKpDivisorPOW2 = 1) */
    
    int32_t wIntegralTerm;     /* Интегральная составляющая */
    int32_t wPrevError;        /* Предыдущая ошибка */
    
    int32_t wLowerIntegralTermLimit;  /* Нижний предел интегральной составляющей */
    int32_t wUpperIntegralTermLimit;  /* Верхний предел интегральной составляющей */
    
    int16_t hLowerOutputLimit; /* Нижний предел выходного сигнала */
    int16_t hUpperOutputLimit; /* Верхний предел выходного сигнала */
    
    int16_t hDefKpGain;        /* Значение Kp по умолчанию */
    int16_t hDefKiGain;        /* Значение Ki по умолчанию */
    int16_t hDefKdGain;        /* Значение Kd по умолчанию */
} PID_Handle_t;

/* Функции инициализации и настройки */
void PID_HandleInit(PID_Handle_t *pHandle);
void PID_SetKP(PID_Handle_t *pHandle, int16_t hKpGain);
void PID_SetKI(PID_Handle_t *pHandle, int16_t hKiGain);
void PID_SetKD(PID_Handle_t *pHandle, int16_t hKdGain);

/* Функции установки делителей */
void PID_SetKPDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKpDivisorPOW2);
void PID_SetKIDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKiDivisorPOW2);
void PID_SetKDDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKdDivisorPOW2);

/* Функции установки ограничений */
void PID_SetLowerIntegralTermLimit(PID_Handle_t *pHandle, int32_t wLowerLimit);
void PID_SetUpperIntegralTermLimit(PID_Handle_t *pHandle, int32_t wUpperLimit);
void PID_SetLowerOutputLimit(PID_Handle_t *pHandle, int16_t hLowerLimit);
void PID_SetUpperOutputLimit(PID_Handle_t *pHandle, int16_t hUpperLimit);

/* Функции получения значений */
int16_t PID_GetKP(PID_Handle_t *pHandle);
int16_t PID_GetKI(PID_Handle_t *pHandle);
int16_t PID_GetKD(PID_Handle_t *pHandle);
int16_t PID_GetDefaultKP(PID_Handle_t *pHandle);
int16_t PID_GetDefaultKI(PID_Handle_t *pHandle);

/* Основные функции регулятора */
int16_t PI_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError);
int16_t PID_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError);

#endif /* PID_CONTROLLER_H */