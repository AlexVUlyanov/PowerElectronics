/**
  ******************************************************************************
  * @file    state_observer.h
  * @brief   ����������� ��������� ��� ������ ��������� ������
  ******************************************************************************
  */

#ifndef STATE_OBSERVER_H
#define STATE_OBSERVER_H

#include <stdint.h>

/* ��������� ���������� ����������� */
typedef struct {
    float Ts;           /* ������ ������������� */
    float R;            /* ������������� ������� */
    float Ld;           /* ������������� �� ��� d */
    float Lq;           /* ������������� �� ��� q */
    float Ke;           /* ����������� ��� */
    float J;            /* ������ ������� */
    float B;            /* ����������� ������� ������ */
    float K1;           /* ����������� ����������� 1 */
    float K2;           /* ����������� ����������� 2 */
} ObserverParams_t;

/* ��������� ��������� ����������� */
typedef struct {
    float theta_hat;    /* ������ ���� */
    float omega_hat;    /* ������ �������� */
    float id_hat;       /* ������ ���� �� ��� d */
    float iq_hat;       /* ������ ���� �� ��� q */
    float ed_hat;       /* ������ ��� �� ��� d */
    float eq_hat;       /* ������ ��� �� ��� q */
} ObserverState_t;

/* ��������� ������� �������� */
typedef struct {
    float vd;           /* ���������� �� ��� d */
    float vq;           /* ���������� �� ��� q */
    float id;           /* ���������� ��� �� ��� d */
    float iq;           /* ���������� ��� �� ��� q */
} ObserverInputs_t;

/* ������������� ����������� */
void Observer_Init(ObserverParams_t* params, ObserverState_t* state);

/* ���������� ��������� ����������� */
void Observer_Update(ObserverParams_t* params, ObserverState_t* state, 
                    ObserverInputs_t* inputs);

/* ��������� ������ ���� */
float Observer_GetAngle(ObserverState_t* state);

/* ��������� ������ �������� */
float Observer_GetSpeed(ObserverState_t* state);

/* ��������� ������������� ����������� */
void Observer_SetGains(ObserverParams_t* params, float k1, float k2);

#endif /* STATE_OBSERVER_H */ 