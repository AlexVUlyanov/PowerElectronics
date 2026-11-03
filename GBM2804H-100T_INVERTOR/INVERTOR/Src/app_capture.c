/**
 * Скоростной захват сигналов контура FOC. Описание протокола — в Inc/app_capture.h.
 */
#include "app_capture.h"
#include "app_hybrid.h"

#include <stddef.h>

#include "mc_config.h"
#include "mc_interface.h"
#include "parameters_conversion.h"
#include "speed_torq_ctrl.h"
#include "sto_pll_speed_pos_fdbk.h"

_Static_assert(offsetof(app_cap_ctrl_t, decimation) == 12u, "раскладка app_cap_ctrl — протокол с хостом");
_Static_assert(offsetof(app_cap_ctrl_t, arm) == 24u, "раскладка app_cap_ctrl — протокол с хостом");
_Static_assert(offsetof(app_cap_ctrl_t, channel) == 28u, "раскладка app_cap_ctrl — протокол с хостом");
_Static_assert(offsetof(app_cap_ctrl_t, pwm_hz) == 40u, "раскладка app_cap_ctrl — протокол с хостом");
_Static_assert(offsetof(app_cap_ctrl_t, trig_gap) == 44u, "раскладка app_cap_ctrl — протокол с хостом");
_Static_assert(sizeof(app_cap_ctrl_t) == 48u, "раскладка app_cap_ctrl — протокол с хостом");
_Static_assert((uint32_t)CAP_SRC_COUNT == 20u, "номера источников — протокол с хостом");
_Static_assert((APP_CAP_SAMPLES & (APP_CAP_SAMPLES - 1u)) == 0u, "APP_CAP_SAMPLES должен быть степенью двойки");

volatile app_cap_ctrl_t app_cap_ctrl =
{
  .magic = APP_CAP_MAGIC,
  .decimation = 1u,
  .samples = (uint16_t)APP_CAP_SAMPLES,
  .channels = (uint16_t)APP_CAP_CHANNELS,
  .pwm_hz = (uint32_t)TF_REGULATION_RATE,
  .channel = { (uint8_t)CAP_SRC_IA, (uint8_t)CAP_SRC_IB, (uint8_t)CAP_SRC_VALPHA, (uint8_t)CAP_SRC_VBETA,
               (uint8_t)CAP_SRC_EL_ANGLE, (uint8_t)CAP_SRC_STO_ANGLE, (uint8_t)CAP_SRC_BEMF_A,
               (uint8_t)CAP_SRC_BEMF_B },
};
volatile int16_t app_cap_buf[APP_CAP_SAMPLES][APP_CAP_CHANNELS];
volatile uint8_t app_cap_tag[APP_CAP_SAMPLES];

/* Настройки, снятые при взведении: хост может переписать app_cap_ctrl во время записи. */
static uint16_t decimation;
static uint16_t decim_left;
static uint16_t post_left;
static uint8_t trigger;
static uint8_t trig_arg;
static uint8_t channel[APP_CAP_CHANNELS];
static uint8_t prev_mode;
static uint8_t prev_state;
static int16_t prev_ref;
static volatile uint8_t trace_hold;

/* Одно значение выбранного источника в цифрах, как его хранит контур тока. Неизвестный номер даёт 0. */
static int16_t read_source(uint8_t source, const FOCVars_t *f)
{
  switch ((app_cap_src_t)source)
  {
    case CAP_SRC_IA:         return f->Iab.a;
    case CAP_SRC_IB:         return f->Iab.b;
    case CAP_SRC_VALPHA:     return f->Valphabeta.alpha;
    case CAP_SRC_VBETA:      return f->Valphabeta.beta;
    case CAP_SRC_VQ:         return f->Vqd.q;
    case CAP_SRC_VD:         return f->Vqd.d;
    case CAP_SRC_IQ:         return f->Iqd.q;
    case CAP_SRC_ID:         return f->Iqd.d;
    case CAP_SRC_IQREF:      return f->Iqdref.q;
    case CAP_SRC_IDREF:      return f->Iqdref.d;
    case CAP_SRC_EL_ANGLE:   return f->hElAngle;
    case CAP_SRC_STO_ANGLE:  return STO_PLL_M1._Super.hElAngle;
    case CAP_SRC_BEMF_A:     return STO_PLL_M1.hBemf_alfa_est;
    case CAP_SRC_BEMF_B:     return STO_PLL_M1.hBemf_beta_est;
    case CAP_SRC_STO_DPP:    return STO_PLL_M1._Super.InstantaneousElSpeedDpp;
    case CAP_SRC_STO_UNIT:   return STO_PLL_M1._Super.hAvrMecSpeedUnit;
    case CAP_SRC_SPEED_UNIT: return STC_GetSpeedSensor(pSTC[M1])->hAvrMecSpeedUnit;
    case CAP_SRC_REF_RPM:    return APP_HYB_GetRefRpm();
    case CAP_SRC_VBUS_D:     return (int16_t)BusVoltageSensor_M1._Super.AvBusVoltage_d;
    default:                 return 0;
  }
}

/* Снимает копию настроек хоста. Пока идёт запись, хост может переписать app_cap_ctrl — на этот прогон это уже не влияет. */
static void arm(uint8_t state, uint8_t mode)
{
  uint8_t i;

  decimation = (app_cap_ctrl.decimation == 0u) ? 1u : app_cap_ctrl.decimation;
  trigger = app_cap_ctrl.trigger;
  trig_arg = app_cap_ctrl.trig_arg;
  for (i = 0u; i < (uint8_t)APP_CAP_CHANNELS; i++)
  {
    channel[i] = app_cap_ctrl.channel[i];
  }
  post_left = (trigger == (uint8_t)CAP_TRIG_NOW) ? (uint16_t)APP_CAP_SAMPLES
                                                 : (uint16_t)(APP_CAP_SAMPLES - app_cap_ctrl.pretrigger);
  if ((app_cap_ctrl.pretrigger >= APP_CAP_SAMPLES) && (trigger != (uint8_t)CAP_TRIG_NOW))
  {
    post_left = 1u;
  }
  decim_left = 1u;
  prev_mode = mode;
  prev_state = ((trigger == (uint8_t)CAP_TRIG_STATE) && (trig_arg != 0xFFu)) ? 0xFFu : state;
  prev_ref = APP_HYB_GetRefRpm();
  app_cap_ctrl.trig_gap = 0u;
  app_cap_ctrl.write_index = 0u;
  app_cap_ctrl.filled = 0u;
  app_cap_ctrl.trig_index = 0u;
  trace_hold = 0u;
  app_cap_ctrl.status = (uint8_t)APP_CAP_ARMED;
  app_cap_ctrl.arm = 0u;
}

/* Условие триггера по копии настроек. CAP_TRIG_NOW срабатывает на первом же периоде после взвода. */
static bool triggered(uint8_t state, uint8_t mode)
{
  bool fire = false;

  switch ((app_cap_trig_t)trigger)
  {
    case CAP_TRIG_NOW:
      fire = true;
      break;
    case CAP_TRIG_MODE:
      fire = (mode != prev_mode) && ((trig_arg == 0xFFu) || (mode == trig_arg));
      break;
    case CAP_TRIG_STATE:
      fire = (state != prev_state) && ((trig_arg == 0xFFu) || (state == trig_arg));
      break;
    case CAP_TRIG_REF_SIGN:
    {
      int16_t ref = APP_HYB_GetRefRpm();

      fire = ((prev_ref > 0) && (ref <= 0)) || ((prev_ref < 0) && (ref >= 0));
      prev_ref = ref;
      break;
    }
    default:
      break;
  }
  prev_mode = mode;
  prev_state = state;
  return fire;
}

/* Конец периода ШИМ. Пока захват не взведён, функция только считает периоды и сразу выходит. */
void APP_CAP_HighFrequency(void)
{
  uint8_t status;
  uint8_t state;
  uint8_t mode;

  /* Счётчик периодов ШИМ. Растёт на каждом вызове контура, в том числе когда запись не взведена. */
  app_cap_ctrl.hf_count++;
  if ((app_cap_ctrl.arm == 0u) && (app_cap_ctrl.status != (uint8_t)APP_CAP_ARMED)
      && (app_cap_ctrl.status != (uint8_t)APP_CAP_TRIGGERED))
  {
    return;
  }
  state = (uint8_t)Mci[M1].State;
  mode = (uint8_t)APP_HYB_GetMode();
  if (app_cap_ctrl.arm != 0u)
  {
    arm(state, mode);
  }
  status = app_cap_ctrl.status;
  if ((status == (uint8_t)APP_CAP_ARMED) && triggered(state, mode))
  {
    status = (uint8_t)APP_CAP_TRIGGERED;
    app_cap_ctrl.trig_hf_count = app_cap_ctrl.hf_count;
    app_cap_ctrl.trig_index = app_cap_ctrl.write_index;
    app_cap_ctrl.vbus_d = BusVoltageSensor_M1._Super.AvBusVoltage_d;
    /* Сколько периодов ШИМ прошло с предыдущей записанной выборки до выборки триггера. */
    app_cap_ctrl.trig_gap = (uint16_t)(decimation - decim_left + 1u);
    app_cap_ctrl.status = status;
    decim_left = 1u;   /* выборка триггера пишется в этом же периоде */
  }
  decim_left--;
  if (decim_left != 0u)
  {
    return;
  }
  decim_left = decimation;
  {
    const FOCVars_t *f = Mci[M1].pFOCVars;
    uint16_t index = app_cap_ctrl.write_index;
    uint8_t i;

    for (i = 0u; i < (uint8_t)APP_CAP_CHANNELS; i++)
    {
      app_cap_buf[index][i] = read_source(channel[i], f);
    }
    /* Байт метки: младшие 5 бит — состояние MCI (до 20), старшие 3 — режим гибрида. */
    app_cap_tag[index] = (uint8_t)((state & 0x1Fu) | (uint8_t)((mode & 0x07u) << 5));
    app_cap_ctrl.write_index = (uint16_t)((index + 1u) & (APP_CAP_SAMPLES - 1u));
    if (app_cap_ctrl.filled < APP_CAP_SAMPLES)
    {
      app_cap_ctrl.filled++;
    }
  }
  if (status == (uint8_t)APP_CAP_TRIGGERED)
  {
    post_left--;
    if (post_left == 0u)
    {
      trace_hold = 1u;
      app_cap_ctrl.status = (uint8_t)APP_CAP_DONE;
    }
  }
}

/* Захват дописан и ждёт чтения. Трасса CLI стоит, чтобы её последняя миллисекунда совпала с концом окна. */
bool APP_CAP_TraceHold(void)
{
  return (trace_hold != 0u);
}

/* START снова разрешает трассу. Сам буфер захвата при этом не стирается: его забирает хост. */
void APP_CAP_ReleaseTraceHold(void)
{
  trace_hold = 0u;
}
