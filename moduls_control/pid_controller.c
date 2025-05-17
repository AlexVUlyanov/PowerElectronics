/**
  ******************************************************************************
  * @file    pid_controller.c
  * @brief   PID Controller module implementation
  ******************************************************************************
  */

#include "pid_controller.h"

/* Инициализация PID-регулятора */
void PID_HandleInit(PID_Handle_t *pHandle)
{
    pHandle->wIntegralTerm = 0;
    pHandle->wPrevError = 0;
    
    pHandle->hKpGain = pHandle->hDefKpGain;
    pHandle->hKiGain = pHandle->hDefKiGain;
    pHandle->hKdGain = pHandle->hDefKdGain;
    
    pHandle->hKpDivisorPOW2 = 0;
    pHandle->hKiDivisorPOW2 = 0;
    pHandle->hKdDivisorPOW2 = 0;
    
    pHandle->wLowerIntegralTermLimit = -INT32_MAX;
    pHandle->wUpperIntegralTermLimit = INT32_MAX;
    
    pHandle->hLowerOutputLimit = -INT16_MAX;
    pHandle->hUpperOutputLimit = INT16_MAX;
}

/* Установка пропорционального коэффициента */
void PID_SetKP(PID_Handle_t *pHandle, int16_t hKpGain)
{
    pHandle->hKpGain = hKpGain;
}

/* Установка интегрального коэффициента */
void PID_SetKI(PID_Handle_t *pHandle, int16_t hKiGain)
{
    pHandle->hKiGain = hKiGain;
}

/* Установка дифференциального коэффициента */
void PID_SetKD(PID_Handle_t *pHandle, int16_t hKdGain)
{
    pHandle->hKdGain = hKdGain;
}

/* Установка делителя для Kp */
void PID_SetKPDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKpDivisorPOW2)
{
    pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
}

/* Установка делителя для Ki */
void PID_SetKIDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKiDivisorPOW2)
{
    pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
}

/* Установка делителя для Kd */
void PID_SetKDDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKdDivisorPOW2)
{
    pHandle->hKdDivisorPOW2 = hKdDivisorPOW2;
}

/* Установка нижнего предела интегральной составляющей */
void PID_SetLowerIntegralTermLimit(PID_Handle_t *pHandle, int32_t wLowerLimit)
{
    pHandle->wLowerIntegralTermLimit = wLowerLimit;
}

/* Установка верхнего предела интегральной составляющей */
void PID_SetUpperIntegralTermLimit(PID_Handle_t *pHandle, int32_t wUpperLimit)
{
    pHandle->wUpperIntegralTermLimit = wUpperLimit;
}

/* Установка нижнего предела выходного сигнала */
void PID_SetLowerOutputLimit(PID_Handle_t *pHandle, int16_t hLowerLimit)
{
    pHandle->hLowerOutputLimit = hLowerLimit;
}

/* Установка верхнего предела выходного сигнала */
void PID_SetUpperOutputLimit(PID_Handle_t *pHandle, int16_t hUpperLimit)
{
    pHandle->hUpperOutputLimit = hUpperLimit;
}

/* Получение значения Kp */
int16_t PID_GetKP(PID_Handle_t *pHandle)
{
    return pHandle->hKpGain;
}

/* Получение значения Ki */
int16_t PID_GetKI(PID_Handle_t *pHandle)
{
    return pHandle->hKiGain;
}

/* Получение значения Kd */
int16_t PID_GetKD(PID_Handle_t *pHandle)
{
    return pHandle->hKdGain;
}

/* Получение значения Kp по умолчанию */
int16_t PID_GetDefaultKP(PID_Handle_t *pHandle)
{
    return pHandle->hDefKpGain;
}

/* Получение значения Ki по умолчанию */
int16_t PID_GetDefaultKI(PID_Handle_t *pHandle)
{
    return pHandle->hDefKiGain;
}

/* PI-регулятор */
int16_t PI_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
    int32_t wProportional_Term;
    int32_t wIntegral_Term;
    int32_t wOutput_32;
    int16_t hOutput;
    
    /* Пропорциональный член */
    wProportional_Term = (int32_t)pHandle->hKpGain * wProcessVarError;
    wProportional_Term = wProportional_Term >> pHandle->hKpDivisorPOW2;
    
    /* Интегральный член */
    wIntegral_Term = (int32_t)pHandle->hKiGain * wProcessVarError;
    wIntegral_Term = wIntegral_Term >> pHandle->hKiDivisorPOW2;
    pHandle->wIntegralTerm += wIntegral_Term;
    
    /* Ограничение интегральной составляющей */
    if (pHandle->wIntegralTerm > pHandle->wUpperIntegralTermLimit)
    {
        pHandle->wIntegralTerm = pHandle->wUpperIntegralTermLimit;
    }
    else if (pHandle->wIntegralTerm < pHandle->wLowerIntegralTermLimit)
    {
        pHandle->wIntegralTerm = pHandle->wLowerIntegralTermLimit;
    }
    
    /* Вычисление выходного сигнала */
    wOutput_32 = wProportional_Term + pHandle->wIntegralTerm;
    
    /* Ограничение выходного сигнала */
    if (wOutput_32 > pHandle->hUpperOutputLimit)
    {
        hOutput = pHandle->hUpperOutputLimit;
    }
    else if (wOutput_32 < pHandle->hLowerOutputLimit)
    {
        hOutput = pHandle->hLowerOutputLimit;
    }
    else
    {
        hOutput = (int16_t)wOutput_32;
    }
    
    return hOutput;
}

/* PID-регулятор */
int16_t PID_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
    int32_t wProportional_Term;
    int32_t wIntegral_Term;
    int32_t wDerivative_Term;
    int32_t wOutput_32;
    int16_t hOutput;
    
    /* Пропорциональный член */
    wProportional_Term = (int32_t)pHandle->hKpGain * wProcessVarError;
    wProportional_Term = wProportional_Term >> pHandle->hKpDivisorPOW2;
    
    /* Интегральный член */
    wIntegral_Term = (int32_t)pHandle->hKiGain * wProcessVarError;
    wIntegral_Term = wIntegral_Term >> pHandle->hKiDivisorPOW2;
    pHandle->wIntegralTerm += wIntegral_Term;
    
    /* Ограничение интегральной составляющей */
    if (pHandle->wIntegralTerm > pHandle->wUpperIntegralTermLimit)
    {
        pHandle->wIntegralTerm = pHandle->wUpperIntegralTermLimit;
    }
    else if (pHandle->wIntegralTerm < pHandle->wLowerIntegralTermLimit)
    {
        pHandle->wIntegralTerm = pHandle->wLowerIntegralTermLimit;
    }
    
    /* Дифференциальный член */
    wDerivative_Term = (int32_t)pHandle->hKdGain * (wProcessVarError - pHandle->wPrevError);
    wDerivative_Term = wDerivative_Term >> pHandle->hKdDivisorPOW2;
    pHandle->wPrevError = wProcessVarError;
    
    /* Вычисление выходного сигнала */
    wOutput_32 = wProportional_Term + pHandle->wIntegralTerm + wDerivative_Term;
    
    /* Ограничение выходного сигнала */
    if (wOutput_32 > pHandle->hUpperOutputLimit)
    {
        hOutput = pHandle->hUpperOutputLimit;
    }
    else if (wOutput_32 < pHandle->hLowerOutputLimit)
    {
        hOutput = pHandle->hLowerOutputLimit;
    }
    else
    {
        hOutput = (int16_t)wOutput_32;
    }
    
    return hOutput;
}