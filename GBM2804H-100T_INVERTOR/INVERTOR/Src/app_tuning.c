/**
 * Подстройка привода под стенд. См. Inc/app_tuning.h.
 */
#include "app_tuning.h"

#include "mc_config.h"
#include "parameters_conversion.h"
#include "pid_regulator.h"
#include "revup_ctrl.h"

/* Ток I/f в цифрах АЦП. Выше NOMINAL_CURRENT библиотека задание всё равно обрежет, здесь это делается явно. */
int16_t APP_TUNING_OpenLoopCurrentDigits(void)
{
  int16_t digits = (int16_t)((APP_OL_CURRENT_A * (float)CURRENT_CONV_FACTOR) + 0.5f);

  if (digits > (int16_t)NOMINAL_CURRENT)
  {
    digits = (int16_t)NOMINAL_CURRENT;
  }
  return digits;
}

/* Один раз из MC_APP_BootHook. Повторная генерация Workbench эти вызовы API не затирает. */
void APP_TUNING_Apply(void)
{
  int16_t digits = APP_TUNING_OpenLoopCurrentDigits();
  uint8_t phase;

  /* Rev-up сведён к выравниванию: фаза 0 — ток растёт до APP_OL_CURRENT_A при нулевой скорости.
   * Когда она закончится, app_hybrid переводит привод в RUN с разомкнутым контуром.
   * Фаза 1 — страховка на 1 с, до её конца дело не доходит; остальные пустые. */
  for (phase = 0u; phase < (uint8_t)RUC_MAX_PHASE_NUMBER; phase++)
  {
    uint16_t duration = 0u;

    if (phase == 0u)
    {
      duration = (uint16_t)APP_ALIGN_MS;
    }
    else if (phase == 1u)
    {
      duration = 1000u;
    }
    else
    {
      /* пустая фаза */
    }
    RUC_SetPhaseDurationms(&RevUpControlM1, phase, duration);
    RUC_SetPhaseFinalMecSpeedUnit(&RevUpControlM1, phase, 0);
    RUC_SetPhaseFinalTorque(&RevUpControlM1, phase, digits);
  }

  PID_SetKP(&PIDSpeedHandle_M1, (int16_t)APP_SPEED_KP);
  PID_SetKI(&PIDSpeedHandle_M1, (int16_t)APP_SPEED_KI);
}
