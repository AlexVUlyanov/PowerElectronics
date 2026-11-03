/**
 * Гибридное управление скоростью: разомкнутый I/f на малой скорости, наблюдатель STO+PLL выше.
 *
 * Зачем. Бездатчиковый наблюдатель восстанавливает угол по ПЭДС, а она пропорциональна скорости:
 * при 100 об/мин это ~0.5 В на фоне 15 В шины и падения R*I ~1 В — наблюдатель срывается.
 * Поэтому ниже APP_HYB_OPEN_RPM угол ротора не измеряется, а задаётся: интегрируется задание
 * скорости (32-битный аккумулятор, точность до долей об/мин), а по оси q виртуальной системы
 * координат течёт постоянный ток APP_OL_CURRENT_A. Ротор следует за вращающимся вектором тока,
 * как у шагового двигателя, в любую сторону и через ноль. Выше APP_HYB_CLOSE_RPM, когда
 * наблюдатель сошёлся, угол плавно сводится к оценке наблюдателя (APP_HYB_BLEND_MS), вектор тока
 * относительно ротора при этом не двигается, затем включается ПИ скорости и Id спадает к нулю.
 * Обратный переход (ниже APP_HYB_OPEN_RPM по оценке STO) мгновенный: виртуальная ось q ставится
 * на ось d ротора, ток уходит на неё (момента не создаёт), интегралы ПИ тока поворачиваются в новые
 * оси. Благодаря этому задание скорости непрерывно от -max до +max, реверс на ходу — обычная рампа
 * через ноль.
 *
 * Стыковка с MCSDK (всё через публичные API и USER CODE в mc_tasks.c):
 * - пуск: rev-up оставлен только как выравнивание (app_tuning.c); когда фаза 0 закончена,
 *   APP_HYB_PostMediumFrequency переводит машину состояний START -> RUN в разомкнутом контуре;
 * - RUN: APP_HYB_RunM1 вызывается из USER CODE MediumFrequencyTask M1 2 перед FOC_CalcCurrRef,
 *   ведёт задание, переключает датчик STC (VSS/STO), режим STC (момент/скорость) и Iqdref.d,
 *   а в разомкнутом контуре отменяет проверку достоверности наблюдателя;
 * - частота ШИМ: APP_HYB_HighFrequency из USER CODE HighFrequencyTask SINGLEDRIVE_3 интегрирует
 *   угол и пишет его в виртуальный датчик (VSS_SetElAngle), в сведении смешивает его с углом STO.
 * Режим момента пользователя (TORQUE) возможен только в замкнутом контуре; гибрид на это время
 * отключается (APP_HYB_EnterTorque / APP_HYB_LeaveTorque).
 */
#ifndef APP_HYBRID_H
#define APP_HYBRID_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  HYB_OFF = 0,    /* не RUN или режим момента пользователя */
  HYB_OPEN,       /* разомкнутый I/f: угол задаётся, ток фиксирован */
  HYB_TO_CLOSED,  /* сведение угла к наблюдателю */
  HYB_CLOSED      /* наблюдатель + ПИ скорости */
} hyb_mode_t;

void APP_HYB_Init(void);

/* Вызовы из контуров MC. */
void APP_HYB_PostMediumFrequency(void);        /* каждый 1 мс после TSK_MediumFrequencyTaskM1 */
bool APP_HYB_RunM1(bool observer_reliable);    /* RUN, перед FOC_CalcCurrRef; возвращает достоверность скорости */
void APP_HYB_HighFrequency(void);              /* каждый период ШИМ после расчёта наблюдателя */

/* Задание скорости. Внутри всё в милли-об/мин, задание непрерывно через ноль. */
void APP_HYB_SetRates(uint16_t accel_rpm_s, uint16_t brake_rpm_s);
void APP_HYB_GetRates(uint16_t *accel_rpm_s, uint16_t *brake_rpm_s);
/* rate_rpm_s < 0 — темпы ACCEL/BRAKE, 0 — скачок (в разомкнутом контуре ограничен APP_HYB_OL_MAX_RATE). */
void APP_HYB_SetGoalRpm(int16_t goal_rpm, int32_t rate_rpm_s);
void APP_HYB_HoldRef(void);                    /* цель := текущее задание */
int16_t APP_HYB_GetGoalRpm(void);
int16_t APP_HYB_GetRefRpm(void);
int16_t APP_HYB_GetRefUnit(void);
bool APP_HYB_RampDone(void);

hyb_mode_t APP_HYB_GetMode(void);
const char *APP_HYB_ModeName(hyb_mode_t mode);
bool APP_HYB_InTorqueMode(void);

/* Режим момента пользователя: разрешён только в RUN с замкнутым контуром. */
bool APP_HYB_EnterTorque(void);
void APP_HYB_LeaveTorque(void);

#endif /* APP_HYBRID_H */
