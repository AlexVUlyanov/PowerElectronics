/**
 * Гибридное управление скоростью: разомкнутый I/f внизу, наблюдатель STO+PLL вверху.
 * Описание и схема стыковки с MCSDK — в Inc/app_hybrid.h.
 *
 * Соглашение о системах координат в разомкнутом контуре такое же, как у rev-up MCSDK:
 * ток течёт по оси q виртуального датчика, ротор выстраивается d-осью вдоль вектора тока,
 * то есть отстаёт от виртуальной оси q на угол нагрузки. При смене направления ось q
 * меняет знак, поэтому виртуальный угол одновременно поворачивается на 180°, а знак тока — на
 * противоположный: вектор тока в статоре остаётся на месте, ротор ничего не замечает.
 */
#include "app_hybrid.h"
#include "app_tuning.h"

#include "stm32g4xx.h"

#include "mc_config.h"
#include "mc_interface.h"
#include "mc_math.h"
#include "parameters_conversion.h"
#include "pid_regulator.h"
#include "speed_torq_ctrl.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "virtual_speed_sensor.h"

/* Внутренняя единица задания — милли-об/мин: темп об/мин/с равен шагу за 1 мс. */
#define HYB_MRPM_PER_UNIT   (((int32_t)U_RPM * 1000) / (int32_t)SPEED_UNIT)
#define HYB_CLOSE_MRPM      ((int32_t)APP_HYB_CLOSE_RPM * 1000)
#define HYB_OL_MAX_MRPM     ((int32_t)APP_HYB_OL_MAX_RPM * 1000)
#define HYB_PLL_RESET_MRPM  ((int32_t)APP_HYB_PLL_RESET_RPM * 1000)
#define HYB_OPEN_UNIT       (((int32_t)APP_HYB_OPEN_RPM * (int32_t)SPEED_UNIT) / (int32_t)U_RPM)
#define HYB_OPEN_MIN_START_MRPM (((int32_t)APP_HYB_OPEN_RPM - 100) * 1000)
#define HYB_BLEND_STEPS     ((uint16_t)(((uint32_t)APP_HYB_BLEND_MS * TF_REGULATION_RATE) / 1000u))

/* Приращение электрического угла за период ШИМ на 1 милли-об/мин, формат Q16 dpp, хранится в Q24. */
#define HYB_W_Q24 ((int64_t)((((double)POLE_PAIR_NUM * 65536.0 * 65536.0 * 16777216.0) \
                              / (60000.0 * (double)TF_REGULATION_RATE)) + 0.5))

_Static_assert(APP_HYB_OPEN_RPM < APP_HYB_CLOSE_RPM, "гистерезис режимов: OPEN должен быть ниже CLOSE");
_Static_assert(APP_HYB_CLOSE_RPM > OBS_MINIMUM_SPEED_RPM, "наблюдатель считается сошедшимся только выше OBS_MINIMUM_SPEED_RPM");
_Static_assert(APP_HYB_OL_MAX_RPM > APP_HYB_CLOSE_RPM, "потолок разомкнутого контура должен быть выше порога замыкания");

#define HF_NONE  0u
#define HF_OPEN  1u
#define HF_BLEND 2u

/* Задание. */
static int32_t goal_mrpm = (int32_t)DEFAULT_TARGET_SPEED_RPM * 1000;
static int32_t ref_mrpm;
static int32_t rate_override = -1;   /* <0: ACCEL/BRAKE */
static uint16_t accel_rpm_s;
static uint16_t brake_rpm_s;

/* Состояние гибрида (контур 1 кГц). */
static hyb_mode_t mode = HYB_OFF;
static bool user_torque;
static int8_t dir = 1;               /* знак тока на виртуальной оси q = направление вращения */
static uint16_t phase_ms;            /* счётчик переходов */
static int16_t id_start;             /* начальное Id для линейного спада */
static qd_t i_rotor;                 /* вектор тока в осях наблюдателя, удерживается на время сведения */
static uint8_t open_debounce;
static uint16_t close_debounce;

/* Разделяется с контуром ШИМ. */
static volatile uint8_t hf_mode = HF_NONE;
static volatile uint8_t hf_reset_pll;
static volatile uint32_t ol_theta32;  /* электрический угол, старшие 16 бит — s16degree */
static volatile int32_t ol_w32;       /* приращение угла за период ШИМ, Q16 dpp */
static volatile uint16_t blend_left;

/* Запрет прерываний на несколько присваиваний. Контур 1 кГц и контур ШИМ пишут одни и те же угол, ток и датчик. */
static uint32_t cs_enter(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  return primask;
}

/* Возвращает PRIMASK, который был до cs_enter: если прерывания уже были закрыты, они остаются закрытыми. */
static void cs_exit(uint32_t primask)
{
  __set_PRIMASK(primask);
}

/* FOCVars в mc_tasks.c статический. Сюда он виден только через указатель в MCI. */
static FOCVars_t *focvars(void)
{
  return Mci[M1].pFOCVars;
}

/* Модуль задания в милли-об/мин. Сравнение порогов от знака не зависит. */
static int32_t i32_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

/* Старшие 16 бит накопителя — угол s16: полный оборот равен 65536, как в датчиках MCSDK. */
static int16_t hi16(uint32_t value)
{
  return (int16_t)(uint16_t)(value >> 16);
}

/* Милли-об/мин в единицу библиотеки (6 об/мин) с округлением к ближайшему. Этот квант остаётся только у STC. */
static int16_t mrpm_to_unit(int32_t mrpm)
{
  int32_t half = HYB_MRPM_PER_UNIT / 2;

  mrpm += (mrpm >= 0) ? half : -half;
  return (int16_t)(mrpm / HYB_MRPM_PER_UNIT);
}

/* Милли-об/мин в об/мин для терминала и захвата. Округление ±0.5 об/мин. */
static int16_t mrpm_to_rpm(int32_t mrpm)
{
  mrpm += (mrpm >= 0) ? 500 : -500;
  return (int16_t)(mrpm / 1000);
}

/* Приращение накопителя угла за один период ШИМ. Отрицательная скорость вычитается за счёт переноса uint32. */
static int32_t w_from_mrpm(int32_t mrpm)
{
  return (int32_t)(((int64_t)mrpm * HYB_W_Q24) / 16777216);
}

/* Скорость виртуального датчика нужна автомату MCSDK. Угол ему пишет контур ШИМ, не эта функция. */
static void set_vss_speed(void)
{
  VSS_SetMecAcceleration(&VirtualSpeedSensorM1, mrpm_to_unit(ref_mrpm), 0u);
}

/* Интеграл ПИ скорости ставится равным текущему Iq, чтобы в момент замыкания момент не прыгнул. */
static void preset_speed_pi(int16_t iq_digits)
{
#if (PID_SPEED_INTEGRAL_INIT_DIV == 0)
  (void)iq_digits;
  PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
  PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                      ((int32_t)iq_digits * (int32_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                      / PID_SPEED_INTEGRAL_INIT_DIV);
#endif
}

/* Шаг рампы задания. В разомкнутом контуре темп ограничен тем, за чем успевает ротор. */
static void ramp_step(bool open_loop)
{
  int32_t delta = goal_mrpm - ref_mrpm;

  if (delta != 0)
  {
    int32_t rate;

    if (rate_override >= 0)
    {
      rate = rate_override;
    }
    else
    {
      /* Разгон — цель дальше от нуля и с той же стороны. К нулю и через ноль берётся темп торможения. */
      bool same_side = (ref_mrpm == 0) || ((ref_mrpm > 0) == (goal_mrpm > 0));
      bool faster = same_side && (i32_abs(goal_mrpm) > i32_abs(ref_mrpm));

      rate = faster ? (int32_t)accel_rpm_s : (int32_t)brake_rpm_s;
    }
    if (open_loop && ((rate == 0) || (rate > (int32_t)APP_HYB_OL_MAX_RATE)))
    {
      rate = (int32_t)APP_HYB_OL_MAX_RATE;
    }
    if (rate == 0)
    {
      ref_mrpm = goal_mrpm;
    }
    else if (delta > rate)
    {
      ref_mrpm += rate;
    }
    else if (delta < -rate)
    {
      ref_mrpm -= rate;
    }
    else
    {
      ref_mrpm = goal_mrpm;
    }
  }
  if (open_loop)
  {
    if (ref_mrpm > HYB_OL_MAX_MRPM)
    {
      ref_mrpm = HYB_OL_MAX_MRPM;
    }
    else if (ref_mrpm < -HYB_OL_MAX_MRPM)
    {
      ref_mrpm = -HYB_OL_MAX_MRPM;
    }
    else
    {
      /* в пределах */
    }
  }
}

/* Смена направления в разомкнутом контуре: ось q меняет знак, угол — на 180°, вектор тока статора неподвижен. */
static void open_update_direction(void)
{
  int8_t want = dir;

  if (ref_mrpm > 0)
  {
    want = 1;
  }
  else if (ref_mrpm < 0)
  {
    want = -1;
  }
  else
  {
    /* на нуле направление не меняем */
  }
  if (want != dir)
  {
    int16_t current = (int16_t)(want * APP_TUNING_OpenLoopCurrentDigits());
    uint32_t primask = cs_enter();

    dir = want;
    ol_theta32 += 0x80000000u;
    VSS_SetElAngle(&VirtualSpeedSensorM1, hi16(ol_theta32));
    (void)STC_ExecRamp(pSTC[M1], current, 0u);
    focvars()->Iqdref.q = current;
    cs_exit(primask);
  }
}

/* Углы виртуального датчика и наблюдателя одним снимком (контур ШИМ двигает оба). */
static void read_angles(int16_t *vss_angle, int16_t *sto_angle)
{
  uint32_t primask = cs_enter();

  *vss_angle = SPD_GetElAngle(&VirtualSpeedSensorM1._Super);
  *sto_angle = SPD_GetElAngle(&STO_PLL_M1._Super);
  cs_exit(primask);
}

/* Вектор тока, заданный в виртуальной системе координат, в системе координат ротора (наблюдателя). */
static qd_t rotate_qd(qd_t vector, int16_t from_angle, int16_t to_angle)
{
  return MCM_Park(MCM_Rev_Park(vector, from_angle), to_angle);
}

/* Начало сведения. Вектор тока пересчитывается в оси наблюдателя; его q-составляющая — это момент,
 * который ротор сейчас реально получает, с неё стартует интеграл ПИ скорости. Пока угол сводится,
 * q ведёт ПИ скорости по оценке STO (ошибка угла наблюдателя в 10-15° даёт лишние 0.1-0.2 А момента,
 * без обратной связи за 100 мс ротор уходил на +150 об/мин), d удерживается. FOC_CalcCurrRef на это
 * время отключается (bDriveInput = EXTERNAL), Iqdref пишется здесь. */
static void start_to_closed(void)
{
  FOCVars_t *f = focvars();
  int16_t vss_angle;
  int16_t sto_angle;

  read_angles(&vss_angle, &sto_angle);
  i_rotor = rotate_qd(f->Iqdref, vss_angle, sto_angle);
  preset_speed_pi(i_rotor.q);
  f->bDriveInput = EXTERNAL;
  blend_left = HYB_BLEND_STEPS;
  hf_mode = HF_BLEND;
  mode = HYB_TO_CLOSED;
}

/* Разомкнутый I/f. Угол копится из задания, ток по оси q постоянный. Замыкание — только после выдержки сходимости. */
static void run_open(void)
{
  ramp_step(true);
  open_update_direction();
  set_vss_speed();
  ol_w32 = w_from_mrpm(ref_mrpm);
  hf_reset_pll = (uint8_t)(i32_abs(ref_mrpm) < HYB_PLL_RESET_MRPM);
  STO_SetDirection(&STO_PLL_M1, dir);
  if (i32_abs(ref_mrpm) >= HYB_CLOSE_MRPM)
  {
    int16_t forced = mrpm_to_unit(ref_mrpm);

    /* Сходимость проверяется по скорости; угол ФАПЧ дотягивается ещё несколько десятков мс,
     * поэтому держим разомкнутый контур, пока наблюдатель не продержится APP_HYB_CONVERGED_MS. */
    if (STO_PLL_IsObserverConverged(&STO_PLL_M1, &forced))
    {
      close_debounce++;
      if (close_debounce >= (uint16_t)APP_HYB_CONVERGED_MS)
      {
        start_to_closed();
      }
    }
    else
    {
      close_debounce = 0u;
    }
  }
  else
  {
    close_debounce = 0u;
  }
}

/* Сведение: угол виртуального датчика ведёт контур ШИМ к углу наблюдателя, здесь Iqdref
 * пересчитывается так, чтобы вектор тока относительно ротора (в осях наблюдателя) стоял на месте. */
static void run_to_closed(void)
{
  FOCVars_t *f = focvars();
  int16_t vss_angle;
  int16_t sto_angle;
  qd_t iqd;

  set_vss_speed();
  STO_SetDirection(&STO_PLL_M1, dir);
  if (blend_left != 0u)
  {
    int32_t error = (int32_t)mrpm_to_unit(ref_mrpm) - (int32_t)STO_PLL_M1._Super.hAvrMecSpeedUnit;
    uint32_t primask;

    i_rotor.q = PI_Controller(&PIDSpeedHandle_M1, error);
    read_angles(&vss_angle, &sto_angle);
    iqd = rotate_qd(i_rotor, sto_angle, vss_angle);
    primask = cs_enter();
    f->Iqdref = iqd;
    f->hTeref = iqd.q;
    cs_exit(primask);
    return;
  }
  /* Угол виртуального датчика совпал с углом наблюдателя: датчик и ПИ скорости — штатным путём STC.
   * Интеграл ПИ уже ведётся с начала сведения, разрыва момента нет. */
  hf_mode = HF_NONE;
  hf_reset_pll = 0u;
  STC_SetSpeedSensor(pSTC[M1], &STO_PLL_M1._Super);
  STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
  (void)STC_ExecRamp(pSTC[M1], mrpm_to_unit(ref_mrpm), 0u);
  f->Iqdref.d = i_rotor.d;
  f->bDriveInput = INTERNAL;
  id_start = i_rotor.d;
  phase_ms = 0u;
  open_debounce = 0u;
  mode = HYB_CLOSED;
}

/* Переход к разомкнутому контуру. Виртуальная ось q ставится на ось d ротора (угол наблюдателя),
 * ток сразу переводится на неё: ток по оси d ротора момента не создаёт, поэтому для ротора это
 * лишь снятие тормозного/тягового момента — за ним ротор успевает всегда, в отличие от плавного
 * поворота вектора при резком торможении (инерция гимбала ~2e-5 кг*м2). Интегралы ПИ тока
 * поворачиваются в новую систему координат, чтобы напряжение статора не прыгнуло. */
static void start_to_open(int16_t sto_speed_unit)
{
  FOCVars_t *f = focvars();
  int16_t angle_sto = SPD_GetElAngle(&STO_PLL_M1._Super);
  int16_t angle_ol;
  int16_t current;
  alphabeta_t vector;
  qd_t vqd;
  uint32_t primask;

  if (sto_speed_unit > 0)
  {
    dir = 1;
  }
  else if (sto_speed_unit < 0)
  {
    dir = -1;
  }
  else
  {
    dir = (ref_mrpm < 0) ? -1 : 1;
  }
  current = (int16_t)(dir * APP_TUNING_OpenLoopCurrentDigits());
  /* При резком торможении оценка скорости STO проваливается быстрее ротора (по трассе: 516 -> 324 об/мин
   * за 3 мс), поэтому стартовое задание не опускаем ниже порога размыкания минус запас. */
  ref_mrpm = (int32_t)sto_speed_unit * HYB_MRPM_PER_UNIT;
  if (i32_abs(ref_mrpm) < HYB_OPEN_MIN_START_MRPM)
  {
    ref_mrpm = (int32_t)dir * HYB_OPEN_MIN_START_MRPM;
  }
  /* 16384 — четверть оборота в s16. Ось q виртуального датчика встаёт на ось магнитов: ток по ней момента не даёт. */
  angle_ol = (int16_t)(angle_sto - (int16_t)(dir * 16384));
  vector = MCM_Rev_Park(f->Vqd, angle_sto);
  vqd = MCM_Park(vector, angle_ol);

  primask = cs_enter();
  ol_theta32 = ((uint32_t)(uint16_t)angle_ol) << 16;
  ol_w32 = w_from_mrpm(ref_mrpm);
  VSS_SetElAngle(&VirtualSpeedSensorM1, angle_ol);
  hf_mode = HF_OPEN;
  STC_SetControlMode(pSTC[M1], MCM_TORQUE_MODE);
  (void)STC_ExecRamp(pSTC[M1], current, 0u);
  f->Iqdref.q = current;
  f->Iqdref.d = 0;
  PID_SetIntegralTerm(&PIDIqHandle_M1, (int32_t)vqd.q * (int32_t)PID_GetKIDivisor(&PIDIqHandle_M1));
  PID_SetIntegralTerm(&PIDIdHandle_M1, (int32_t)vqd.d * (int32_t)PID_GetKIDivisor(&PIDIdHandle_M1));
  STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
  cs_exit(primask);

  set_vss_speed();
  hf_reset_pll = 0u;
  close_debounce = 0u;
  mode = HYB_OPEN;
}

/* Замкнутый контур: рампу ведём сами, в STC отдаём уже округлённую скорость. Ниже порога два такта подряд — размыкание. */
static void run_closed(void)
{
  FOCVars_t *f = focvars();
  int16_t sto_speed = STO_PLL_M1._Super.hAvrMecSpeedUnit;

  ramp_step(false);
  (void)STC_ExecRamp(pSTC[M1], mrpm_to_unit(ref_mrpm), 0u);
  if (phase_ms < (uint16_t)APP_HYB_ID_DECAY_MS)
  {
    phase_ms++;
    f->Iqdref.d = (int16_t)(((int32_t)id_start * (int32_t)((uint16_t)APP_HYB_ID_DECAY_MS - phase_ms))
                            / (int32_t)APP_HYB_ID_DECAY_MS);
  }
  if (i32_abs((int32_t)sto_speed) < HYB_OPEN_UNIT)
  {
    open_debounce++;
    if (open_debounce >= 2u)
    {
      start_to_open(sto_speed);
    }
  }
  else
  {
    open_debounce = 0u;
  }
}

/* RUN достигнут не через наше выравнивание (штатный rev-up MCSDK) или после режима момента:
 * датчик уже наблюдатель, замыкаем контур скорости на текущей скорости. */
static void enter_closed(void)
{
  FOCVars_t *f = focvars();

  if (Mci[M1].CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED)
  {
    MCI_ExecBufferedCommands(&Mci[M1]);
  }
  ref_mrpm = (int32_t)STO_PLL_M1._Super.hAvrMecSpeedUnit * HYB_MRPM_PER_UNIT;
  hf_mode = HF_NONE;
  hf_reset_pll = 0u;
  f->bDriveInput = INTERNAL;
  STC_SetSpeedSensor(pSTC[M1], &STO_PLL_M1._Super);
  STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
  preset_speed_pi(f->Iqdref.q);
  (void)STC_ExecRamp(pSTC[M1], mrpm_to_unit(ref_mrpm), 0u);
  id_start = f->Iqdref.d;
  phase_ms = 0u;
  open_debounce = 0u;
  mode = HYB_CLOSED;
}

/* Выравнивание закончено: START -> RUN в разомкнутом контуре на нулевой скорости. */
static void handover_from_start(void)
{
  FOCVars_t *f = focvars();
  int16_t angle = SPD_GetElAngle(&VirtualSpeedSensorM1._Super);
  int16_t current;
  uint32_t primask;

  /* Отложенная команда MCI исполнилась бы в RUN и переключила бы STC. Исполняем её сейчас и перекрываем. */
  MCI_ExecBufferedCommands(&Mci[M1]);
  dir = (f->Iqdref.q < 0) ? -1 : 1;
  current = (int16_t)(dir * APP_TUNING_OpenLoopCurrentDigits());

  primask = cs_enter();
  ol_theta32 = ((uint32_t)(uint16_t)angle) << 16;
  ol_w32 = 0;
  VSS_SetElAngle(&VirtualSpeedSensorM1, angle);
  hf_mode = HF_OPEN;
  STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
  STC_SetControlMode(pSTC[M1], MCM_TORQUE_MODE);
  (void)STC_ExecRamp(pSTC[M1], current, 0u);
  f->Iqdref.q = current;
  f->Iqdref.d = 0;
  f->bDriveInput = INTERNAL;
  cs_exit(primask);

  ref_mrpm = 0;
  user_torque = false;
  hf_reset_pll = 1u;
  open_debounce = 0u;
  close_debounce = 0u;
  set_vss_speed();
  mode = HYB_OPEN;
  Mci[M1].State = RUN;
}

static void reset_to_off(void)
{
  hf_mode = HF_NONE;
  hf_reset_pll = 0u;
  user_torque = false;
  ref_mrpm = 0;
  mode = HYB_OFF;
}

/* После сброса темпы нулевые: в замкнутом контуре это скачок, в разомкнутом гибрид подставит свой предел. */
void APP_HYB_Init(void)
{
  accel_rpm_s = 0u;
  brake_rpm_s = 0u;
  reset_to_off();
}

/* Вызывается после автомата MCSDK. Как только выравнивание (фаза 0) кончилось, START становится RUN
 * в разомкнутом контуре. До наблюдателя на нулевой скорости дело не доходит, SWITCH_OVER не используется. */
void APP_HYB_PostMediumFrequency(void)
{
  MCI_State_t state = Mci[M1].State;

  if (state == START)
  {
    if (RevUpControlM1.bStageCnt >= 1u)
    {
      handover_from_start();
    }
  }
  else if (state != RUN)
  {
    if (mode != HYB_OFF)
    {
      reset_to_off();
    }
  }
  else
  {
    /* RUN: работа идёт в APP_HYB_RunM1 */
  }
}

/* Такт 1 кГц в RUN. В разомкнутом контуре и на сведении недостоверность наблюдателя аварией не считается. */
bool APP_HYB_RunM1(bool observer_reliable)
{
  if (user_torque)
  {
    return observer_reliable;
  }
  switch (mode)
  {
    case HYB_OPEN:
      run_open();
      break;
    case HYB_TO_CLOSED:
      run_to_closed();
      break;
    case HYB_CLOSED:
      run_closed();
      break;
    default:
      enter_closed();
      break;
  }
  return (mode == HYB_CLOSED) ? observer_reliable : true;
}

/* Каждый период ШИМ после наблюдателя. В OPEN пишет угол накопителя, в BLEND сводит его к углу STO. */
void APP_HYB_HighFrequency(void)
{
  uint8_t current_mode = hf_mode;

  if (current_mode == HF_NONE)
  {
    return;
  }
  if (hf_reset_pll != 0u)
  {
    STO_ResetPLL(&STO_PLL_M1);
  }
  ol_theta32 += (uint32_t)ol_w32;
  if (current_mode == HF_OPEN)
  {
    VSS_SetElAngle(&VirtualSpeedSensorM1, hi16(ol_theta32));
  }
  else
  {
    int16_t sto = SPD_GetElAngle(&STO_PLL_M1._Super);
    /* Кратчайшая разность углов в s16. Доля left/N линейно уводит виртуальный угол от накопителя к STO. */
    int16_t diff = (int16_t)(sto - hi16(ol_theta32));
    uint16_t left = blend_left;
    int32_t correction = ((int32_t)diff * (int32_t)left) / (int32_t)HYB_BLEND_STEPS;

    VSS_SetElAngle(&VirtualSpeedSensorM1, (int16_t)(sto - (int16_t)correction));
    if (left != 0u)
    {
      blend_left = (uint16_t)(left - 1u);
    }
  }
}

/* Темпы ACCEL и BRAKE, об/мин/с. Ноль означает скачок, кроме разомкнутого контура. */
void APP_HYB_SetRates(uint16_t accel, uint16_t brake)
{
  accel_rpm_s = accel;
  brake_rpm_s = brake;
}

/* Текущие темпы для строки STATUS. */
void APP_HYB_GetRates(uint16_t *accel, uint16_t *brake)
{
  *accel = accel_rpm_s;
  *brake = brake_rpm_s;
}

/* Новая цель. rate_rpm_s < 0 — пользоваться ACCEL/BRAKE, 0 — скачок, больше нуля — темп только этого перехода. */
void APP_HYB_SetGoalRpm(int16_t goal_rpm, int32_t rate_rpm_s)
{
  goal_mrpm = (int32_t)goal_rpm * 1000;
  rate_override = rate_rpm_s;
}

/* STOPRAMP: цель становится текущим заданием, рампа останавливается на этом такте. */
void APP_HYB_HoldRef(void)
{
  goal_mrpm = ref_mrpm;
}

/* Цель, которую запомнил SPEED. Она не равна заданию, пока рампа не дошла. */
int16_t APP_HYB_GetGoalRpm(void)
{
  return mrpm_to_rpm(goal_mrpm);
}

/* Задание, которое гибрид уже выдал на этот миллисекундный такт. */
int16_t APP_HYB_GetRefRpm(void)
{
  return mrpm_to_rpm(ref_mrpm);
}

/* То же задание в единицах MCSDK, для сравнения с оценкой скорости. */
int16_t APP_HYB_GetRefUnit(void)
{
  return mrpm_to_unit(ref_mrpm);
}

/* Рампа дошла: задание совпало с целью. STOPRAMP в этом случае отвечает ERR. */
bool APP_HYB_RampDone(void)
{
  return (ref_mrpm == goal_mrpm);
}

/* Режим гибрида для STATUS, TRACE и тега захвата. */
hyb_mode_t APP_HYB_GetMode(void)
{
  return mode;
}

/* Короткое имя режима. Прочерк — гибрид выключен (не RUN или режим момента). */
const char *APP_HYB_ModeName(hyb_mode_t value)
{
  switch (value)
  {
    case HYB_OPEN:
      return "OPEN";
    case HYB_TO_CLOSED:
      return "OPEN>CLOSED";
    case HYB_CLOSED:
      return "CLOSED";
    default:
      return "-";
  }
}

/* Пользователь в режиме TORQUE. Пока это так, гибрид не трогает задание скорости. */
bool APP_HYB_InTorqueMode(void)
{
  return user_torque;
}

/* Отдать момент пользователю можно только в RUN и только когда угол уже ведёт наблюдатель. */
bool APP_HYB_EnterTorque(void)
{
  if ((Mci[M1].State != RUN) || (mode != HYB_CLOSED))
  {
    return false;
  }
  user_torque = true;
  mode = HYB_OFF;
  hf_mode = HF_NONE;
  focvars()->Iqdref.d = 0;
  return true;
}

/* Возврат в режим скорости без скачка: цель и интеграл ПИ берутся из текущего состояния. */
void APP_HYB_LeaveTorque(void)
{
  if (!user_torque)
  {
    return;
  }
  user_torque = false;
  if (Mci[M1].State == RUN)
  {
    enter_closed();
  }
  else
  {
    mode = HYB_OFF;
  }
}
