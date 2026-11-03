/**
 * Скоростной захват сигналов контура FOC («осциллограф» в прошивке).
 *
 * Каждый период ШИМ (30 кГц) или каждый N-й (прореживание) в кольцевой буфер пишутся 8 выбранных
 * сигналов int16 и байт состояния. Управление и чтение — только по SWD (STM32_Programmer_CLI в режиме
 * HOTPLUG, скрипты matlab/stand_*.m): хост пишет настройки в app_cap_ctrl, затем arm = 1;
 * контур ШИМ начинает запись, ждёт триггер, дописывает (samples - pretrigger) выборок и ставит
 * status = APP_CAP_DONE. Буфер не перезаписывается до следующего arm.
 *
 * Пока захват завершён и не перевзведён, трасса CLI (TRACE, 1 мс) тоже стоит: её последняя
 * выборка совпадает с концом захвата с точностью до 1 мс, по ней хост совмещает медленные сигналы.
 *
 * Раскладка app_cap_ctrl фиксирована (проверяется _Static_assert), хост читает её одним блоком.
 */
#ifndef APP_CAPTURE_H
#define APP_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#define APP_CAP_SAMPLES   512u
#define APP_CAP_CHANNELS  8u
#define APP_CAP_MAGIC     0x31504143u   /* "CAP1" */

/* Источники сигналов для app_cap_ctrl.channel[]. Номера — часть протокола с хостом, не менять. */
typedef enum
{
  CAP_SRC_NONE = 0,
  CAP_SRC_IA,           /* ток фазы U, цифры (FOCVars.Iab.a) */
  CAP_SRC_IB,           /* ток фазы V, цифры */
  CAP_SRC_VALPHA,       /* задание напряжения alpha, цифры (32767 = Vbus/sqrt3) */
  CAP_SRC_VBETA,        /* задание напряжения beta (ось beta MCSDK, знак обратный стандартному) */
  CAP_SRC_VQ,
  CAP_SRC_VD,
  CAP_SRC_IQ,
  CAP_SRC_ID,
  CAP_SRC_IQREF,
  CAP_SRC_IDREF,
  CAP_SRC_EL_ANGLE,     /* угол, по которому работает FOC (VSS или STO), s16degree */
  CAP_SRC_STO_ANGLE,    /* угол наблюдателя STO */
  CAP_SRC_BEMF_A,       /* оценка ПЭДС alpha наблюдателя, цифры */
  CAP_SRC_BEMF_B,
  CAP_SRC_STO_DPP,      /* мгновенная скорость ФАПЧ, dpp */
  CAP_SRC_STO_UNIT,     /* средняя скорость STO, 0.1 Гц */
  CAP_SRC_SPEED_UNIT,   /* средняя скорость активного датчика STC, 0.1 Гц */
  CAP_SRC_REF_RPM,      /* задание гибрида, об/мин */
  CAP_SRC_VBUS_D,       /* напряжение шины, цифры uint16 (65536 = 3.3 В / делитель) */
  CAP_SRC_COUNT
} app_cap_src_t;

typedef enum
{
  CAP_TRIG_NOW = 0,     /* сразу, предзапись не используется */
  CAP_TRIG_MODE,        /* режим гибрида стал trig_arg (0xFF — любая смена) */
  CAP_TRIG_STATE,       /* состояние MCI стало trig_arg (0xFF — любая смена); уже равное trig_arg
                           на момент взвода тоже срабатывает: в IDLE контур ШИМ может не вызываться */
  CAP_TRIG_REF_SIGN     /* задание скорости гибрида сменило знак (прошло через ноль) */
} app_cap_trig_t;

typedef enum
{
  APP_CAP_IDLE = 0,
  APP_CAP_ARMED,        /* запись идёт, триггер ещё не сработал */
  APP_CAP_TRIGGERED,    /* дописывается послетриггерная часть */
  APP_CAP_DONE
} app_cap_status_t;

typedef struct
{
  uint32_t magic;          /* +0  APP_CAP_MAGIC */
  uint32_t hf_count;       /* +4  число периодов ШИМ с загрузки */
  uint32_t trig_hf_count;  /* +8  hf_count в момент триггера */
  uint16_t decimation;     /* +12 запись каждого N-го периода ШИМ, 1..65535 (хост) */
  uint16_t pretrigger;     /* +14 выборок до триггера (хост) */
  uint16_t write_index;    /* +16 следующая ячейка записи */
  uint16_t filled;         /* +18 сколько ячеек заполнено (до APP_CAP_SAMPLES) */
  uint16_t trig_index;     /* +20 ячейка выборки, на которой сработал триггер */
  uint16_t vbus_d;         /* +22 напряжение шины в момент триггера, цифры */
  uint8_t arm;             /* +24 хост пишет 1, контур ШИМ сбрасывает в 0 */
  uint8_t status;          /* +25 app_cap_status_t */
  uint8_t trigger;         /* +26 app_cap_trig_t (хост) */
  uint8_t trig_arg;        /* +27 (хост) */
  uint8_t channel[APP_CAP_CHANNELS]; /* +28 app_cap_src_t (хост) */
  uint16_t samples;        /* +36 APP_CAP_SAMPLES */
  uint16_t channels;       /* +38 APP_CAP_CHANNELS */
  uint32_t pwm_hz;         /* +40 частота вызова контура FOC */
  uint16_t trig_gap;       /* +44 периодов ШИМ между последней выборкой до триггера и выборкой триггера */
  uint16_t reserved;       /* +46 */
} app_cap_ctrl_t;

extern volatile app_cap_ctrl_t app_cap_ctrl;
/* Выборки: [ячейка][канал]. Байт состояния: биты 0..4 — MCI_State_t, биты 5..7 — hyb_mode_t. */
extern volatile int16_t app_cap_buf[APP_CAP_SAMPLES][APP_CAP_CHANNELS];
extern volatile uint8_t app_cap_tag[APP_CAP_SAMPLES];

void APP_CAP_HighFrequency(void);   /* каждый период ШИМ, в конце TSK_HighFrequencyTask */
bool APP_CAP_TraceHold(void);       /* захват завершён: трасса CLI не пишется */
void APP_CAP_ReleaseTraceHold(void);

#endif /* APP_CAPTURE_H */
