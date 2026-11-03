/**
 * Текстовый терминал привода на USART2, 115200 8N1.
 * Приём и передача по прерыванию, без DMA и без ASPEP/MCP.
 */
#include "app_cli.h"
#include "app_capture.h"
#include "app_hybrid.h"
#include "app_tuning.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "mc_api.h"
#include "mc_app_hooks.h"
#include "mc_stm_types.h"
#include "motorcontrol.h"
#include "parameters_conversion.h"
#include "stm32g4xx_ll_usart.h"

#define CLI_USART USART2

#define CLI_LINE_SIZE 72u
#define CLI_RX_SIZE   128u
#define CLI_TX_SIZE   4096u
#define CLI_RX_MASK   (CLI_RX_SIZE - 1u)
#define CLI_TX_MASK   (CLI_TX_SIZE - 1u)

_Static_assert((CLI_RX_SIZE & CLI_RX_MASK) == 0u, "CLI_RX_SIZE должен быть степенью двойки");
_Static_assert((CLI_TX_SIZE & CLI_TX_MASK) == 0u, "CLI_TX_SIZE должен быть степенью двойки");

static volatile uint8_t cli_ready;
static volatile uint8_t rx_dropped;
static volatile uint8_t rx_buf[CLI_RX_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t tx_buf[CLI_TX_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;

static char line[CLI_LINE_SIZE];
static uint8_t line_len;
static uint8_t line_overflow;
static uint8_t last_was_cr;
static char help_text[2560];

/* Задание скорости и рампы ведёт Src/app_hybrid.c. Буферные команды MCI остались только у TORQUE. */
static uint8_t cmd_watch;     /* ждём, пока MC исполнит буферную команду TORQUE */
static uint8_t tx_dropped;
static MCI_State_t prev_state = ICLWAIT;

/* «Чёрный ящик»: последние CLI_TRACE_SIZE мс, замораживается при аварии. Команда TRACE печатает. */
#define CLI_TRACE_SIZE 512u
_Static_assert((CLI_TRACE_SIZE & (CLI_TRACE_SIZE - 1u)) == 0u, "CLI_TRACE_SIZE должен быть степенью двойки");

typedef struct
{
  int16_t speed_unit;   /* скорость по активному датчику (VSS в разомкнутом контуре, STO в замкнутом) */
  int16_t ref_rpm;      /* задание скорости гибрида, об/мин */
  int16_t iq;           /* измеренный Iq, цифры */
  int16_t iq_ref;       /* задание Iq, цифры */
  int16_t id;           /* измеренный Id, цифры */
  int16_t sto_unit;     /* оценка STO независимо от выбранного датчика */
  int16_t bemf_a;       /* оценка ПЭДС alpha, цифры */
  int16_t bemf_b;       /* оценка ПЭДС beta, цифры */
  uint8_t state;        /* MCI_State_t */
  uint8_t flags;        /* бит0 скорость достоверна, бит1 ПЭДС согласована, бит2 наблюдатель сошёлся,
                           биты 4..6 — режим гибрида (hyb_mode_t) */
} trace_sample_t;

/* 8×int16 + 2 байта. matlab/stand_read_capture.m читает выборку как 18 байт. */
_Static_assert(sizeof(trace_sample_t) == 18u, "раскладка выборки TRACE — протокол с хостом");

/* Заголовок кольца. Хост читает все 4 байта с &cli_trace_meta за одно подключение:
 * байты 0..1 — head, байт 2 — frozen, байт 3 — wrapped. Отдельные static-переменные
 * компилятор не обязан класть подряд, из-за этого признак оборота кольца читался не оттуда. */
typedef struct
{
  uint16_t head;     /* следующий индекс записи */
  uint8_t frozen;    /* 1 после аварии: буфер не обновляется */
  uint8_t wrapped;   /* буфер заполнен хотя бы раз */
} cli_trace_meta_t;

_Static_assert(sizeof(cli_trace_meta_t) == 4u, "раскладка заголовка TRACE — протокол с хостом");
_Static_assert(offsetof(cli_trace_meta_t, frozen) == 2u, "байт 2 — заморозка аварией");
_Static_assert(offsetof(cli_trace_meta_t, wrapped) == 3u, "байт 3 — кольцо уже заполнялось");

static trace_sample_t trace_buf[CLI_TRACE_SIZE];
static cli_trace_meta_t cli_trace_meta;
#define trace_head    (cli_trace_meta.head)
#define trace_frozen  (cli_trace_meta.frozen)
#define trace_wrapped (cli_trace_meta.wrapped)
static uint16_t trace_dump_left; /* сколько строк ещё печатать */
static uint16_t trace_dump_index;

/* Свободное место в кольце передачи. Один байт всегда оставляем пустым: так полный буфер не путается с пустым. */
static uint16_t tx_free(void)
{
  return (uint16_t)((tx_tail - tx_head - 1u) & CLI_TX_MASK);
}

/* Кладёт байт в кольцо передачи. 0 — буфер полон, байт потерян. Пишет только контур 1 кГц. */
static int tx_push(uint8_t byte)
{
  uint16_t next = (uint16_t)((tx_head + 1u) & CLI_TX_MASK);

  if (next == tx_tail)
  {
    return 0;
  }
  tx_buf[tx_head] = byte;
  tx_head = next;
  return 1;
}

/* Забирает байт для USART. Вызывается только из прерывания передачи. */
static int tx_pop(uint8_t *byte)
{
  if (tx_head == tx_tail)
  {
    return 0;
  }
  *byte = tx_buf[tx_tail];
  tx_tail = (uint16_t)((tx_tail + 1u) & CLI_TX_MASK);
  return 1;
}

/* Разрешает прерывание TXE, если в кольце ещё есть байты. */
static void tx_kick(void)
{
  if (tx_head != tx_tail)
  {
    LL_USART_EnableIT_TXE(CLI_USART);
  }
}

/* Строка в кольцо передачи. При переполнении ставит tx_dropped и обрывает хвост, промпт не блокируется. */
static void cli_puts(const char *text)
{
  while (*text != '\0')
  {
    if (tx_push((uint8_t)*text) == 0)
    {
      tx_dropped = 1u;
      break;
    }
    text++;
  }
  tx_kick();
}

/* Байт из прерывания приёма. 0 — кольцо полно, байт потерян (потом ERR «потеряны байты»). */
static int rx_push(uint8_t byte)
{
  uint16_t next = (uint16_t)((rx_head + 1u) & CLI_RX_MASK);

  if (next == rx_tail)
  {
    rx_dropped = 1u;
    return 0;
  }
  rx_buf[rx_head] = byte;
  rx_head = next;
  return 1;
}

/* Байт в разбор строки. Вызывается только из контура 1 кГц, не из прерывания. */
static int rx_pop(uint8_t *byte)
{
  if (rx_head == rx_tail)
  {
    return 0;
  }
  *byte = rx_buf[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1u) & CLI_RX_MASK);
  return 1;
}

/* Пропуск пробелов и табуляций. На конце строки возвращает указатель на '\0'. */
static const char *skip_space(const char *text)
{
  while ((*text == ' ') || (*text == '\t'))
  {
    text++;
  }
  return text;
}

/* Следующее слово в верхнем регистре. Разделители — пробел и табуляция, латиница приводится к A..Z. */
static void next_token(const char **text, char *out, size_t out_size)
{
  const char *cursor = skip_space(*text);
  size_t index = 0u;

  while ((*cursor != '\0') && (*cursor != ' ') && (*cursor != '\t') && ((index + 1u) < out_size))
  {
    char symbol = *cursor++;

    if ((symbol >= 'a') && (symbol <= 'z'))
    {
      symbol = (char)(symbol - ('a' - 'A'));
    }
    out[index++] = symbol;
  }
  out[index] = '\0';
  *text = cursor;
}

/* Сравнение слов, которые next_token уже привёл к верхнему регистру. */
static int same_token(const char *left, const char *right)
{
  return (strcmp(left, right) == 0);
}

/* Целое со знаком. Курсор остаётся на первом неразобранном символе. 0 — нет цифр или переполнение int32. */
static int parse_i32(const char **text, int32_t *value)
{
  const char *cursor = *text;
  int sign = 1;
  int32_t number = 0;

  if (*cursor == '+')
  {
    cursor++;
  }
  else if (*cursor == '-')
  {
    sign = -1;
    cursor++;
  }
  if ((*cursor < '0') || (*cursor > '9'))
  {
    return 0;
  }
  while ((*cursor >= '0') && (*cursor <= '9'))
  {
    int digit = (int)(*cursor - '0');

    if (number > ((2147483647 - digit) / 10))
    {
      return 0;
    }
    number = (number * 10) + digit;
    cursor++;
  }
  *value = number * sign;
  *text = cursor;
  return 1;
}

/* Дробное вида «-1.25». Нужна хотя бы одна цифра, экспонента не принимается. */
static int parse_f32(const char **text, float *value)
{
  const char *cursor = *text;
  float number = 0.0f;
  float place = 0.1f;
  int sign = 1;
  int any = 0;

  if (*cursor == '+')
  {
    cursor++;
  }
  else if (*cursor == '-')
  {
    sign = -1;
    cursor++;
  }
  while ((*cursor >= '0') && (*cursor <= '9'))
  {
    number = (number * 10.0f) + (float)(*cursor - '0');
    cursor++;
    any = 1;
  }
  if (*cursor == '.')
  {
    cursor++;
    while ((*cursor >= '0') && (*cursor <= '9'))
    {
      number += (float)(*cursor - '0') * place;
      place *= 0.1f;
      cursor++;
      any = 1;
    }
  }
  if (any == 0)
  {
    return 0;
  }
  *value = number * (float)sign;
  *text = cursor;
  return 1;
}

/* Неотрицательное целое 0..65535, и токен должен состоять только из него. Для миллисекунд и темпов. */
static int parse_ms(const char *token, uint16_t *duration_ms)
{
  const char *cursor = token;
  int32_t value;

  if ((parse_i32(&cursor, &value) == 0) || (*cursor != '\0') || (value < 0) || (value > 65535))
  {
    return 0;
  }
  *duration_ms = (uint16_t)value;
  return 1;
}

/* Предел |Iq| команды TORQUE, А. Это NOMINAL_CURRENT в амперах, не оценка момента. */
static float torque_limit_a(void)
{
  return (float)NOMINAL_CURRENT / (float)CURRENT_CONV_FACTOR;
}

/* Амперы в «-1.348». printf %f на newlib тянет много кода, поэтому три знака считаются вручную. */
static void format_milli(char *out, size_t out_size, float amperes)
{
  int32_t milli = (int32_t)((amperes * 1000.0f) + ((amperes >= 0.0f) ? 0.5f : -0.5f));
  int negative = 0;

  if (milli < 0)
  {
    negative = 1;
    milli = -milli;
  }
  (void)snprintf(out, out_size, "%s%ld.%03ld",
                 (negative != 0) ? "-" : "",
                 (long)(milli / 1000),
                 (long)(milli % 1000));
}

/* Вольты в строку с двумя знаками после запятой. */
static void format_volt(char *out, size_t out_size, float volts)
{
  int32_t centi = (int32_t)((volts * 100.0f) + ((volts >= 0.0f) ? 0.5f : -0.5f));
  int negative = 0;

  if (centi < 0)
  {
    negative = 1;
    centi = -centi;
  }
  (void)snprintf(out, out_size, "%s%ld.%02ld",
                 (negative != 0) ? "-" : "",
                 (long)(centi / 100),
                 (long)(centi % 100));
}

/* Напряжение шины, В: цифры АЦП умножаются на 3.3 В и делятся на коэффициент делителя и на 65536. */
static float vbus_volt(void)
{
  return ((float)VBS_GetAvBusVoltage_d(&BusVoltageSensor_M1._Super)
          * (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR)) / 65536.0f;
}

/* Момент СПМ: Te = 1.5 * p * λ * Iq. λ из Ke, Вrms межфазное на 1000 об/мин. */
static float torque_nm(float iq_amp)
{
  float lambda = ((MOTOR_VOLTAGE_CONSTANT * (float)SQRT_2) / (float)SQRT_3) * 60.0f
                 / ((float)POLE_PAIR_NUM * 1000.0f * 6.2831853f);

  return (1.5f * (float)POLE_PAIR_NUM * lambda * iq_amp);
}

/* Короткое имя состояния автомата MCSDK для STATUS и TRACE. */
static const char *state_name(MCI_State_t state)
{
  switch (state)
  {
    case ICLWAIT:
      return "ICLWAIT";
    case IDLE:
      return "IDLE";
    case ALIGNMENT:
      return "ALIGNMENT";
    case CHARGE_BOOT_CAP:
      return "CHARGE_BOOT";
    case OFFSET_CALIB:
      return "OFFSET_CALIB";
    case START:
      return "START";
    case SWITCH_OVER:
      return "SWITCH_OVER";
    case RUN:
      return "RUN";
    case STOP:
      return "STOP";
    case FAULT_NOW:
      return "FAULT_NOW";
    case FAULT_OVER:
      return "FAULT_OVER";
    case WAIT_STOP_MOTOR:
      return "WAIT_STOP";
    default:
      return "UNKNOWN";
  }
}

/* Дописывает одно число по формату. При нехватке места возвращает конец буфера, строка остаётся завершённой. */
static char *append_fmt(char *cursor, char *end, const char *fmt, long value)
{
  int written;
  size_t room;

  if ((cursor + 1) >= end)
  {
    return cursor;
  }
  room = (size_t)(end - cursor);
  written = snprintf(cursor, room, fmt, value);
  if ((written < 0) || ((size_t)written >= room))
  {
    return end - 1;
  }
  return cursor + written;
}

/* Дописывает текст, не выходя за end. Всегда оставляет завершающий нуль. */
static char *append_text(char *cursor, char *end, const char *text)
{
  while ((*text != '\0') && ((cursor + 1) < end))
  {
    *cursor++ = *text++;
  }
  *cursor = '\0';
  return cursor;
}

/* Имена установленных битов аварии через запятую. Незнакомая маска печатается как UNKNOWN. */
static char *append_faults(char *cursor, char *end, uint16_t faults)
{
  static const struct
  {
    uint16_t bit;
    const char *name;
  } table[] =
  {
    { MC_DURATION, "DURATION" },
    { MC_OVER_VOLT, "OV" },
    { MC_UNDER_VOLT, "UV" },
    { MC_OVER_TEMP, "OT" },
    { MC_START_UP, "STARTUP" },
    { MC_SPEED_FDBK, "SPEEDFBK" },
    { MC_BREAK_IN, "OCP" },
    { MC_SW_ERROR, "SW" }
  };
  unsigned index;
  int any = 0;

  if (faults == MC_NO_FAULTS)
  {
    return append_text(cursor, end, "none");
  }
  for (index = 0u; index < (sizeof(table) / sizeof(table[0])); index++)
  {
    if ((faults & table[index].bit) != 0u)
    {
      if (any != 0)
      {
        cursor = append_text(cursor, end, ",");
      }
      cursor = append_text(cursor, end, table[index].name);
      any = 1;
    }
  }
  if (any == 0)
  {
    cursor = append_text(cursor, end, "UNKNOWN");
  }
  return cursor;
}

/* Единица скорости библиотеки (0.1 Гц, то есть 6 об/мин) в об/мин. */
static int32_t to_rpm(int16_t speed_unit)
{
  return ((int32_t)speed_unit * (int32_t)U_RPM) / (int32_t)SPEED_UNIT;
}

/* Модуль int32. Для темпа SPEED и проверки предела момента. */
static int32_t i32_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

/* Знак: −1, 0 или +1. Нуль не считается сменой направления. */
static int sign_of(int32_t value)
{
  return (value > 0) - (value < 0);
}

/* Одна выборка чёрного ящика, раз в 1 мс. При аварии и после готового захвата запись стоит. */
static void trace_sample(MCI_State_t state)
{
  trace_sample_t *sample;
  qd_t iqd;
  uint8_t flags = 0u;

  if ((trace_frozen != 0u) || APP_CAP_TraceHold())
  {
    return;
  }
  sample = &trace_buf[trace_head];
  iqd = MC_GetIqdMotor1();
  sample->speed_unit = MC_GetMecSpeedAverageMotor1();
  sample->ref_rpm = APP_HYB_GetRefRpm();
  sample->iq = iqd.q;
  sample->id = iqd.d;
  sample->iq_ref = MC_GetIqdrefMotor1().q;
  sample->sto_unit = STO_PLL_M1._Super.hAvrMecSpeedUnit;
  sample->bemf_a = STO_PLL_M1.hBemf_alfa_est;
  sample->bemf_b = STO_PLL_M1.hBemf_beta_est;
  sample->state = (uint8_t)state;
  if (STO_PLL_M1.IsSpeedReliable)
  {
    flags |= 1u;
  }
  if (STO_PLL_M1.IsBemfConsistent)
  {
    flags |= 2u;
  }
  if (STO_PLL_M1.IsAlgorithmConverged)
  {
    flags |= 4u;
  }
  flags |= (uint8_t)(((uint8_t)APP_HYB_GetMode() & 7u) << 4);
  sample->flags = flags;
  trace_head = (uint16_t)((trace_head + 1u) & (CLI_TRACE_SIZE - 1u));
  if (trace_head == 0u)
  {
    trace_wrapped = 1u;
  }
}

/* Печать трассы по одной строке за вызов, чтобы не переполнить буфер передачи. */
static void trace_dump_poll(void)
{
  char text[160];
  const trace_sample_t *sample;

  while ((trace_dump_left > 0u) && (tx_free() > (uint16_t)sizeof(text)))
  {
    sample = &trace_buf[trace_dump_index];
    (void)snprintf(text, sizeof(text), "%ld %s %s w=%ld sto=%ld ref=%d iq=%d iqr=%d id=%d e=%d,%d f=%u\r\n",
                   -((long)trace_dump_left - 1L),
                   state_name((MCI_State_t)sample->state),
                   APP_HYB_ModeName((hyb_mode_t)((sample->flags >> 4) & 7u)),
                   (long)to_rpm(sample->speed_unit),
                   (long)to_rpm(sample->sto_unit),
                   (int)sample->ref_rpm,
                   (int)sample->iq, (int)sample->iq_ref, (int)sample->id,
                   (int)sample->bemf_a, (int)sample->bemf_b,
                   (unsigned)(sample->flags & 7u));
    cli_puts(text);
    trace_dump_index = (uint16_t)((trace_dump_index + 1u) & (CLI_TRACE_SIZE - 1u));
    trace_dump_left--;
    if (trace_dump_left == 0u)
    {
      cli_puts("> ");
    }
  }
}

/* TRACE: заголовок и запуск построчной печати. Сами строки уходят из trace_dump_poll, не отсюда. */
static void cmd_trace(void)
{
  char text[192];
  uint16_t count = (trace_wrapped != 0u) ? CLI_TRACE_SIZE : trace_head;

  if (count == 0u)
  {
    cli_puts("ERR трасса пуста\r\n");
    return;
  }
  (void)snprintf(text, sizeof(text),
                 "TRACE %u мс, %s. Столбцы: t[мс] STATE LOOP w sto ref[об/мин] iq iqr id e=ПЭДС a,b f\r\n",
                 (unsigned)count,
                 (trace_frozen != 0u) ? "заморожена аварией"
                 : (APP_CAP_TraceHold() ? "стоит до START: захват уже прочитан" : "идёт запись"));
  cli_puts(text);
  cli_puts("f: бит0 скорость достоверна, бит1 ПЭДС согласована, бит2 наблюдатель сошёлся\r\n");
  trace_dump_index = (trace_wrapped != 0u) ? trace_head : 0u;
  trace_dump_left = count;
}

/* Вызывается раз в 1 мс. Команду из буфера MC_GetCommandStateMotor1 читать можно один раз: чтение сбрасывает результат. */
static void command_poll(void)
{
  MCI_State_t state = MC_GetSTMStateMotor1();

  trace_sample(state);
  if (state != prev_state)
  {
    prev_state = state;
    if ((state == FAULT_NOW) || (state == FAULT_OVER))
    {
      cmd_watch = 0u;
      trace_frozen = 1u;
    }
    else if (state == START)
    {
      /* Пуск (в том числе кнопкой): после выравнивания гибрид сам поведёт задание к цели. */
      cmd_watch = 0u;
      trace_frozen = 0u;
      APP_CAP_ReleaseTraceHold();
    }
    else
    {
      /* Остальные переходы ничего не меняют. */
    }
  }
  if ((state != RUN) || (cmd_watch == 0u))
  {
    return;
  }
  {
    MCI_CommandState_t command = MC_GetCommandStateMotor1();

    if (command == MCI_COMMAND_NOT_ALREADY_EXECUTED)
    {
      return;
    }
    cmd_watch = 0u;
    if (command == MCI_COMMAND_EXECUTED_UNSUCCESFULLY)
    {
      cli_puts("\r\nERR задание момента отклонено\r\n> ");
    }
  }
}

/* STATUS: состояние, контур, скорости, токи фаз, момент по Iq, темпы и аварии. */
static void cmd_status(void)
{
  char message[512];
  char iq[16];
  char id[16];
  char iq_ref[16];
  char iu[16];
  char iv[16];
  char iw[16];
  char vbus[16];
  char torque[16];
  char *cursor = message;
  char *end = message + sizeof(message);
  uint16_t accel;
  uint16_t brake;
  qd_f_t measured = MC_GetIqdMotor1_F();
  qd_f_t reference = MC_GetIqdrefMotor1_F();
  ab_f_t phase = MC_GetIabMotor1_F();
  MC_ControlMode_t mode = MC_GetControlModeMotor1();
  const char *mode_name = "OTHER";

  if (mode == MCM_SPEED_MODE)
  {
    mode_name = "SPEED";
  }
  else if (mode == MCM_TORQUE_MODE)
  {
    mode_name = "TORQUE";
  }
  format_milli(iq, sizeof(iq), measured.q);
  format_milli(id, sizeof(id), measured.d);
  format_milli(iq_ref, sizeof(iq_ref), reference.q);
  format_milli(iu, sizeof(iu), phase.a);
  format_milli(iv, sizeof(iv), phase.b);
  /* Iw = −(Iu+Iv): сумма фазных токов без нулевого провода равна нулю. */
  format_milli(iw, sizeof(iw), -(phase.a + phase.b));
  format_volt(vbus, sizeof(vbus), vbus_volt());
  format_milli(torque, sizeof(torque), torque_nm(measured.q));
  message[0] = '\0';
  cursor = append_text(cursor, end, "STATE=");
  cursor = append_text(cursor, end, state_name(MC_GetSTMStateMotor1()));
  cursor = append_text(cursor, end, " MODE=");
  cursor = append_text(cursor, end, mode_name);
  cursor = append_text(cursor, end, " LOOP=");
  cursor = append_text(cursor, end, APP_HYB_ModeName(APP_HYB_GetMode()));
  cursor = append_text(cursor, end, " SPEED=");
  cursor = append_fmt(cursor, end, "%ld", (long)to_rpm(MC_GetMecSpeedAverageMotor1()));
  cursor = append_text(cursor, end, " rpm STO=");
  cursor = append_fmt(cursor, end, "%ld", (long)to_rpm(STO_PLL_M1._Super.hAvrMecSpeedUnit));
  cursor = append_text(cursor, end, " rpm REF=");
  if (mode == MCM_TORQUE_MODE)
  {
    cursor = append_text(cursor, end, "n/a");
  }
  else
  {
    cursor = append_fmt(cursor, end, "%ld", (long)APP_HYB_GetRefRpm());
    cursor = append_text(cursor, end, " rpm");
  }
  cursor = append_text(cursor, end, " GOAL=");
  cursor = append_fmt(cursor, end, "%ld", (long)APP_HYB_GetGoalRpm());
  cursor = append_text(cursor, end, " rpm Vbus=");
  cursor = append_text(cursor, end, vbus);
  cursor = append_text(cursor, end, " V\r\nIq=");
  cursor = append_text(cursor, end, iq);
  cursor = append_text(cursor, end, " A Id=");
  cursor = append_text(cursor, end, id);
  cursor = append_text(cursor, end, " A IqRef=");
  cursor = append_text(cursor, end, iq_ref);
  cursor = append_text(cursor, end, " A Iu=");
  cursor = append_text(cursor, end, iu);
  cursor = append_text(cursor, end, " A Iv=");
  cursor = append_text(cursor, end, iv);
  cursor = append_text(cursor, end, " A Iw=");
  cursor = append_text(cursor, end, iw);
  cursor = append_text(cursor, end, " A T=");
  cursor = append_text(cursor, end, torque);
  APP_HYB_GetRates(&accel, &brake);
  cursor = append_text(cursor, end, " Nm\r\nACCEL=");
  cursor = append_fmt(cursor, end, "%u", (unsigned)accel);
  cursor = append_text(cursor, end, " BRAKE=");
  cursor = append_fmt(cursor, end, "%u", (unsigned)brake);
  cursor = append_text(cursor, end, " об/мин/с FAULT=");
  cursor = append_faults(cursor, end, MC_GetCurrentFaultsMotor1());
  cursor = append_text(cursor, end, " OCCURRED=");
  (void)append_faults(cursor, end, MC_GetOccurredFaultsMotor1());
  cli_puts(message);
  cli_puts("\r\n");
}

/* Текст темпа: 0 об/мин/с в ответе называется «сразу», в разомкнутом контуре гибрид всё равно ограничит его. */
static void format_rate(char *out, size_t out_size, uint16_t rate)
{
  if (rate == 0u)
  {
    (void)snprintf(out, out_size, "сразу");
  }
  else
  {
    (void)snprintf(out, out_size, "%u об/мин/с", (unsigned)rate);
  }
}

/* SPEED запоминает цель. Вне RUN гибрид дойдёт до неё после выравнивания, цель между STOP и START не сбрасывается. */
static void cmd_speed(const char *args)
{
  int32_t rpm;
  uint16_t duration_ms = 0u;
  char token[16];
  char reply[160];
  char text[24];
  char *cursor;
  char *end;
  int explicit_ms = 0;
  int32_t rate = -1;
  int32_t from_rpm;
  int deferred;

  if (parse_i32(&args, &rpm) == 0)
  {
    cli_puts("ERR формат: SPEED <об/мин> [мс]\r\n");
    return;
  }
  next_token(&args, token, sizeof(token));
  if (token[0] != '\0')
  {
    if (parse_ms(token, &duration_ms) == 0)
    {
      cli_puts("ERR время перехода: 0..65535 мс\r\n");
      return;
    }
    explicit_ms = 1;
    next_token(&args, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
  }
  if ((rpm > (int32_t)MAX_APPLICATION_SPEED_RPM) || (rpm < -(int32_t)MAX_APPLICATION_SPEED_RPM))
  {
    (void)snprintf(reply, sizeof(reply), "ERR скорость вне %d..%d об/мин\r\n",
                   -(int)MAX_APPLICATION_SPEED_RPM, (int)MAX_APPLICATION_SPEED_RPM);
    cli_puts(reply);
    return;
  }
  deferred = (MC_GetSTMStateMotor1() != RUN);
  if (APP_HYB_InTorqueMode())
  {
    /* Безударный переход TORQUE -> SPEED: задание = текущая скорость, интеграл ПИ = текущий Iq. */
    APP_HYB_LeaveTorque();
  }
  from_rpm = (deferred != 0) ? 0 : (int32_t)APP_HYB_GetRefRpm();
  if (explicit_ms != 0)
  {
    /* Явное время перехода. В гибриде темп 0 — это скачок, а не «очень медленно»:
     * целочисленное деление давало 0 при переходе медленнее 1 об/мин/с. */
    int32_t delta = i32_abs(rpm - from_rpm);

    if ((duration_ms == 0u) || (delta == 0))
    {
      rate = 0;
    }
    else
    {
      rate = (delta * 1000) / (int32_t)duration_ms;
      if (rate == 0)
      {
        rate = 1;
      }
    }
  }
  cmd_watch = 0u;
  APP_HYB_SetGoalRpm((int16_t)rpm, rate);

  cursor = reply;
  end = reply + sizeof(reply);
  reply[0] = '\0';
  cursor = append_text(cursor, end, "OK SPEED ");
  cursor = append_fmt(cursor, end, "%ld", (long)rpm);
  cursor = append_text(cursor, end, " rpm");
  if (explicit_ms != 0)
  {
    cursor = append_fmt(cursor, end, ", %ld ms", (long)duration_ms);
  }
  else if (from_rpm != rpm)
  {
    uint16_t accel;
    uint16_t brake;
    int reversing = (from_rpm != 0) && (rpm != 0) && (sign_of(rpm) != sign_of(from_rpm));
    int accelerating = (i32_abs(rpm) > i32_abs(from_rpm)) && (reversing == 0);

    APP_HYB_GetRates(&accel, &brake);
    if (reversing != 0)
    {
      format_rate(text, sizeof(text), brake);
      cursor = append_text(cursor, end, ", реверс: торможение ");
      cursor = append_text(cursor, end, text);
      format_rate(text, sizeof(text), accel);
      cursor = append_text(cursor, end, ", разгон ");
      cursor = append_text(cursor, end, text);
    }
    else
    {
      format_rate(text, sizeof(text), (accelerating != 0) ? accel : brake);
      cursor = append_text(cursor, end, (accelerating != 0) ? ", разгон " : ", торможение ");
      cursor = append_text(cursor, end, text);
    }
  }
  else
  {
    /* Цель уже совпадает с заданием. */
  }
  if (deferred != 0)
  {
    cursor = append_text(cursor, end, ", после пуска");
  }
  (void)append_text(cursor, end, "\r\n");
  cli_puts(reply);
  if ((rpm != 0) && (i32_abs(rpm) < (int32_t)APP_HYB_CLOSE_RPM))
  {
    (void)snprintf(reply, sizeof(reply),
                   "INFO ниже %d об/мин разомкнутый контур: ток %d.%02d А, темп до %u об/мин/с\r\n",
                   (int)APP_HYB_CLOSE_RPM,
                   (int)APP_OL_CURRENT_A, (int)((APP_OL_CURRENT_A * 100.0f) + 0.5f) % 100,
                   (unsigned)APP_HYB_OL_MAX_RATE);
    cli_puts(reply);
  }
}

/* REVERSE меняет знак цели. Рампа идёт через ноль: сначала темп BRAKE, после нуля — ACCEL. */
static void cmd_reverse(const char *args)
{
  char token[16];
  char reply[96];
  int16_t goal = APP_HYB_GetGoalRpm();

  next_token(&args, token, sizeof(token));
  if (token[0] != '\0')
  {
    cli_puts("ERR лишние аргументы\r\n");
    return;
  }
  if (goal == 0)
  {
    cli_puts("ERR цель 0 об/мин, реверсировать нечего\r\n");
    return;
  }
  if (APP_HYB_InTorqueMode())
  {
    APP_HYB_LeaveTorque();
  }
  cmd_watch = 0u;
  APP_HYB_SetGoalRpm((int16_t)-goal, -1);
  (void)snprintf(reply, sizeof(reply), "OK REVERSE %d -> %d rpm%s\r\n",
                 (int)goal, (int)-goal, (MC_GetSTMStateMotor1() != RUN) ? ", после пуска" : "");
  cli_puts(reply);
}

/* ACCEL или BRAKE. Без числа показывает темп, с числом записывает его. Неверный аргумент темп не меняет. */
static void cmd_rate(const char *name, int is_accel, const char *args)
{
  char token[16];
  char reply[80];
  uint16_t value;
  uint16_t accel;
  uint16_t brake;

  APP_HYB_GetRates(&accel, &brake);
  next_token(&args, token, sizeof(token));
  if (token[0] == '\0')
  {
    (void)snprintf(reply, sizeof(reply), "%s %u об/мин/с\r\n", name, (unsigned)((is_accel != 0) ? accel : brake));
    cli_puts(reply);
    return;
  }
  if (parse_ms(token, &value) == 0)
  {
    (void)snprintf(reply, sizeof(reply), "ERR формат: %s <об/мин/с>, 0..65535\r\n", name);
    cli_puts(reply);
    return;
  }
  next_token(&args, token, sizeof(token));
  if (token[0] != '\0')
  {
    cli_puts("ERR лишние аргументы\r\n");
    return;
  }
  if (is_accel != 0)
  {
    accel = value;
  }
  else
  {
    brake = value;
  }
  APP_HYB_SetRates(accel, brake);
  (void)snprintf(reply, sizeof(reply), "OK %s %u об/мин/с\r\n", name, (unsigned)value);
  cli_puts(reply);
}

/* TORQUE только из замкнутого контура. Предел проверяется в цифрах АЦП, до передачи команды в MCSDK. */
static void cmd_torque(const char *args)
{
  char token[24];
  uint16_t duration_ms = 0u;
  float limit_a = torque_limit_a();
  float amperes = 0.0f;
  int32_t digits;
  int use_limit = 0;
  int negative = 0;
  int direction;

  next_token(&args, token, sizeof(token));
  if (token[0] == '\0')
  {
    cli_puts("ERR формат: TORQUE <А> [мс] | TORQUE MAX [мс] | TORQUE -MAX [мс]\r\n");
    return;
  }
  if (same_token(token, "MAX"))
  {
    use_limit = 1;
  }
  else if (same_token(token, "-MAX"))
  {
    use_limit = 1;
    negative = 1;
  }
  else
  {
    const char *cursor = token;

    if ((parse_f32(&cursor, &amperes) == 0) || (*cursor != '\0'))
    {
      cli_puts("ERR формат: TORQUE <А> [мс] | TORQUE MAX [мс] | TORQUE -MAX [мс]\r\n");
      return;
    }
  }
  if (use_limit != 0)
  {
    digits = (negative != 0) ? -(int32_t)NOMINAL_CURRENT : (int32_t)NOMINAL_CURRENT;
  }
  else
  {
    float scaled;

    if ((amperes > (2.0f * limit_a)) || (amperes < -(2.0f * limit_a)))
    {
      digits = (int32_t)NOMINAL_CURRENT + 1;
    }
    else
    {
      scaled = amperes * (float)CURRENT_CONV_FACTOR;
      digits = (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
    }
  }
  if (i32_abs(digits) > (int32_t)NOMINAL_CURRENT)
  {
    char range[48];
    char limit_text[16];

    format_milli(limit_text, sizeof(limit_text), limit_a);
    (void)snprintf(range, sizeof(range), "ERR момент вне ±%s А\r\n", limit_text);
    cli_puts(range);
    return;
  }
  next_token(&args, token, sizeof(token));
  if (token[0] != '\0')
  {
    if (parse_ms(token, &duration_ms) == 0)
    {
      cli_puts("ERR разгон: 0..65535 мс\r\n");
      return;
    }
    next_token(&args, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
  }
  direction = sign_of((int32_t)STO_PLL_M1._Super.hAvrMecSpeedUnit);
  if (!APP_HYB_InTorqueMode() && !APP_HYB_EnterTorque())
  {
    char reply[128];

    (void)snprintf(reply, sizeof(reply),
                   "ERR TORQUE только в RUN с замкнутым контуром (LOOP=CLOSED, выше %d об/мин)\r\n",
                   (int)APP_HYB_CLOSE_RPM);
    cli_puts(reply);
    return;
  }
  cmd_watch = 1u;
  MC_ProgramTorqueRampMotor1((int16_t)digits, duration_ms);
  {
    char reply[96];
    char amp_text[16];

    format_milli(amp_text, sizeof(amp_text), (float)digits / (float)CURRENT_CONV_FACTOR);
    (void)snprintf(reply, sizeof(reply), "OK TORQUE %s A, %u ms\r\n", amp_text, (unsigned)duration_ms);
    cli_puts(reply);
  }
  if (digits != 0)
  {
    cli_puts("WARN скорость не регулируется: без нагрузки ротор разгоняется. Выход: SPEED или STOP\r\n");
    if ((direction != 0) && (sign_of(digits) != direction))
    {
      cli_puts("WARN момент против вращения: ниже порога наблюдателя возможна авария SPEEDFBK\r\n");
    }
  }
}

/* Разбор одной строки. Пустая строка ничего не делает. Неизвестная команда отвечает ERR и не меняет привод. */
static void cmd_execute(char *command)
{
  const char *cursor = command;
  char token[24];

  if (rx_dropped != 0u)
  {
    rx_dropped = 0u;
    cli_puts("ERR потеряны байты, повторите команду\r\n");
  }
  if (tx_dropped != 0u)
  {
    tx_dropped = 0u;
    cli_puts("WARN часть вывода потеряна (переполнение буфера передачи)\r\n");
  }
  next_token(&cursor, token, sizeof(token));
  if (token[0] == '\0')
  {
    return;
  }
  if (same_token(token, "HELP") || same_token(token, "?"))
  {
    next_token(&cursor, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
    cli_puts(help_text);
    return;
  }
  if (same_token(token, "START"))
  {
    next_token(&cursor, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
    if (MC_StartMotor1())
    {
      char reply[96];

      (void)snprintf(reply, sizeof(reply), "OK START, выравнивание %u мс, затем к %d rpm\r\n",
                     (unsigned)APP_ALIGN_MS, (int)APP_HYB_GetGoalRpm());
      cli_puts(reply);
    }
    else if ((MC_GetSTMStateMotor1() == FAULT_NOW) || (MC_GetSTMStateMotor1() == FAULT_OVER))
    {
      cli_puts("ERR авария, сначала ACK\r\n");
    }
    else
    {
      cli_puts("ERR пуск только из IDLE\r\n");
    }
    return;
  }
  if (same_token(token, "STOP"))
  {
    next_token(&cursor, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
    if (MC_StopMotor1())
    {
      cmd_watch = 0u;
      cli_puts("OK STOP\r\n");
    }
    else
    {
      cli_puts("ERR стоп не принят (IDLE или авария)\r\n");
    }
    return;
  }
  if (same_token(token, "ACK"))
  {
    next_token(&cursor, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
    if (MC_AcknowledgeFaultMotor1())
    {
      cli_puts("OK ACK\r\n");
    }
    else
    {
      cli_puts("ERR ACK только в FAULT_OVER, когда текущая авария уже снята\r\n");
    }
    return;
  }
  if (same_token(token, "ACCEL"))
  {
    cmd_rate("ACCEL", 1, skip_space(cursor));
    return;
  }
  if (same_token(token, "BRAKE"))
  {
    cmd_rate("BRAKE", 0, skip_space(cursor));
    return;
  }
  if (same_token(token, "SPEED"))
  {
    cmd_speed(skip_space(cursor));
    return;
  }
  if (same_token(token, "REVERSE") || same_token(token, "REV"))
  {
    cmd_reverse(skip_space(cursor));
    return;
  }
  if (same_token(token, "TORQUE"))
  {
    cmd_torque(skip_space(cursor));
    return;
  }
  if (same_token(token, "STOPRAMP"))
  {
    next_token(&cursor, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
    if (MC_GetSTMStateMotor1() != RUN)
    {
      cli_puts("ERR STOPRAMP только в RUN\r\n");
    }
    else if (APP_HYB_InTorqueMode())
    {
      if ((cmd_watch == 0u) && MC_HasRampCompletedMotor1())
      {
        cli_puts("ERR нет активной рампы\r\n");
      }
      else
      {
        if (cmd_watch != 0u)
        {
          /* Команда ещё не исполнена: заменяем её удержанием, иначе она запустит рампу заново. */
          MC_ProgramTorqueRampMotor1(MC_GetTerefMotor1(), 0u);
        }
        MC_StopRampMotor1();
        cli_puts("OK STOPRAMP\r\n");
      }
    }
    else if (APP_HYB_RampDone())
    {
      cli_puts("ERR нет активной рампы\r\n");
    }
    else
    {
      char reply[64];

      APP_HYB_HoldRef();
      (void)snprintf(reply, sizeof(reply), "OK STOPRAMP, задание %d rpm\r\n", (int)APP_HYB_GetGoalRpm());
      cli_puts(reply);
    }
    return;
  }
  if (same_token(token, "STATUS"))
  {
    next_token(&cursor, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
    cmd_status();
    return;
  }
  if (same_token(token, "TRACE"))
  {
    next_token(&cursor, token, sizeof(token));
    if (token[0] != '\0')
    {
      cli_puts("ERR лишние аргументы\r\n");
      return;
    }
    cmd_trace();
    return;
  }
  cli_puts("ERR неизвестная команда. Отправьте HELP\r\n");
}

/* Один байт строки: CR/LF исполняют команду, Backspace стирает символ, остальное печатается эхом. */
static void cli_on_byte(uint8_t byte)
{
  if ((byte == '\n') && (last_was_cr != 0u))
  {
    last_was_cr = 0u;
    return;
  }
  last_was_cr = (uint8_t)(byte == '\r');
  if ((byte == '\r') || (byte == '\n'))
  {
    cli_puts("\r\n");
    if (line_overflow != 0u)
    {
      line_overflow = 0u;
      line_len = 0u;
      cli_puts("ERR слишком длинная строка\r\n> ");
      return;
    }
    line[line_len] = '\0';
    if (line_len > 0u)
    {
      cmd_execute(line);
    }
    line_len = 0u;
    cli_puts("> ");
    return;
  }
  if (line_overflow != 0u)
  {
    return;
  }
  if ((byte == '\b') || (byte == 0x7Fu))
  {
    if (line_len > 0u)
    {
      line_len--;
      cli_puts("\b \b");
    }
    return;
  }
  if ((byte < 0x20u) || (byte > 0x7Eu))
  {
    return;
  }
  if ((uint8_t)(line_len + 1u) >= CLI_LINE_SIZE)
  {
    line_overflow = 1u;
    return;
  }
  line[line_len++] = (char)byte;
  {
    char echo[2];

    echo[0] = (char)byte;
    echo[1] = '\0';
    cli_puts(echo);
  }
}

/* HELP собирается один раз при старте: в текст подставляются пределы скорости и тока этого двигателя. */
static void build_help(void)
{
  int limit_ma = (int)((1000.0f * torque_limit_a()) + 0.5f);

  (void)snprintf(help_text, sizeof(help_text),
                 "Команды:\r\n"
                 "\r\n"
                 "HELP\r\n"
                 "  список команд\r\n"
                 "?\r\n"
                 "  то же, что HELP\r\n"
                 "START\r\n"
                 "  пуск из IDLE, без аварии\r\n"
                 "  выравнивание %u мс, затем рампа к цели SPEED\r\n"
                 "STOP\r\n"
                 "  снять ШИМ\r\n"
                 "  отказ в IDLE и при аварии\r\n"
                 "ACK\r\n"
                 "  сброс аварии, только FAULT_OVER\r\n"
                 "ACCEL [об/мин/с]\r\n"
                 "  темп разгона, 0..65535, 0 - сразу\r\n"
                 "  без числа - показать темп\r\n"
                 "BRAKE [об/мин/с]\r\n"
                 "  темп торможения, 0..65535, 0 - сразу\r\n"
                 "  без числа - показать темп\r\n"
                 "SPEED <об/мин> [мс]\r\n"
                 "  режим скорости, %d..%d, через ноль\r\n"
                 "  без мс - темп ACCEL и BRAKE\r\n"
                 "  мс - время одного перехода\r\n"
                 "  вне RUN применится после пуска\r\n"
                 "  ниже %d об/мин - разомкнутый контур\r\n"
                 "  (угол задаётся, ток %d.%02d А, темп до %u об/мин/с)\r\n"
                 "  выше - наблюдатель, шаг %d об/мин\r\n"
                 "REVERSE\r\n"
                 "  цель := -цель, реверс на ходу через ноль\r\n"
                 "TORQUE <А> [мс]\r\n"
                 "  режим момента, ток Iq\r\n"
                 "  только в RUN при LOOP=CLOSED\r\n"
                 "  предел +-%d.%03d А\r\n"
                 "  мс - время выхода на ток\r\n"
                 "  скорость не регулируется\r\n"
                 "  без нагрузки ротор разгоняется\r\n"
                 "  выход: SPEED или STOP\r\n"
                 "TORQUE MAX [мс]\r\n"
                 "  положительный предел Iq\r\n"
                 "TORQUE -MAX [мс]\r\n"
                 "  отрицательный предел Iq\r\n"
                 "STOPRAMP\r\n"
                 "  зафиксировать задание, только RUN\r\n"
                 "STATUS\r\n"
                 "  STATE MODE LOOP SPEED STO REF GOAL Vbus\r\n"
                 "  LOOP: OPEN - разомкнутый, CLOSED - наблюдатель\r\n"
                 "  STO - оценка наблюдателя (внизу недостоверна)\r\n"
                 "  Iq Id IqRef Iu Iv Iw\r\n"
                 "  T - момент по Iq, Нм\r\n"
                 "  ACCEL BRAKE FAULT OCCURRED\r\n"
                 "TRACE\r\n"
                 "  последние 512 мс: состояние, контур, скорость,\r\n"
                 "  задание, Iq, IqRef, Id, флаги наблюдателя\r\n"
                 "  при аварии запись замирает до START\r\n",
                 (unsigned)APP_ALIGN_MS,
                 -(int)MAX_APPLICATION_SPEED_RPM,
                 (int)MAX_APPLICATION_SPEED_RPM,
                 (int)APP_HYB_CLOSE_RPM,
                 (int)APP_OL_CURRENT_A,
                 (int)((APP_OL_CURRENT_A * 100.0f) + 0.5f) % 100,
                 (unsigned)APP_HYB_OL_MAX_RATE,
                 (int)(U_RPM / SPEED_UNIT),
                 limit_ma / 1000,
                 limit_ma % 1000);
}
/* Прерывание USART2. Только кольца приёма и передачи: разбор команд здесь нельзя, он не умещается в бюджет прерывания. */
void APP_CLI_UartIrq(void)
{
  if (LL_USART_IsActiveFlag_RXNE(CLI_USART) != 0u)
  {
    uint8_t byte = LL_USART_ReceiveData8(CLI_USART);

    if (cli_ready != 0u)
    {
      (void)rx_push(byte);
    }
  }
  if ((LL_USART_IsEnabledIT_TXE(CLI_USART) != 0u) && (LL_USART_IsActiveFlag_TXE(CLI_USART) != 0u))
  {
    uint8_t byte;

    if (tx_pop(&byte) != 0)
    {
      LL_USART_TransmitData8(CLI_USART, byte);
    }
    else
    {
      LL_USART_DisableIT_TXE(CLI_USART);
    }
  }
  /* Флаги ошибок кадра сбрасываем, иначе прерывание USART будет входить снова и снова. */
  if (LL_USART_IsActiveFlag_ORE(CLI_USART) != 0u)
  {
    LL_USART_ClearFlag_ORE(CLI_USART);
  }
  if (LL_USART_IsActiveFlag_FE(CLI_USART) != 0u)
  {
    LL_USART_ClearFlag_FE(CLI_USART);
  }
  if (LL_USART_IsActiveFlag_NE(CLI_USART) != 0u)
  {
    LL_USART_ClearFlag_NE(CLI_USART);
  }
  if (LL_USART_IsActiveFlag_IDLE(CLI_USART) != 0u)
  {
    LL_USART_ClearFlag_IDLE(CLI_USART);
  }
  if (LL_USART_IsActiveFlag_TC(CLI_USART) != 0u)
  {
    LL_USART_ClearFlag_TC(CLI_USART);
  }
}

/* Конец MCboot: подстройка стенда, гибрид, цель по умолчанию и приветствие в терминал. */
void MC_APP_BootHook(void)
{
  int limit_ma;

  APP_TUNING_Apply();
  APP_HYB_Init();
  APP_HYB_SetGoalRpm((int16_t)DEFAULT_TARGET_SPEED_RPM, -1);
  build_help();
  cli_ready = 1u;
  LL_USART_EnableIT_RXNE(CLI_USART);
  limit_ma = (int)((1000.0f * torque_limit_a()) + 0.5f);
  cli_puts("INVERTOR CLI, 115200 8N1. HELP — список команд.\r\n");
  {
    char banner[80];

    (void)snprintf(banner, sizeof(banner),
                   "Скорость %d..%d об/мин, момент до ±%d.%03d А.\r\n> ",
                   -(int)MAX_APPLICATION_SPEED_RPM,
                   (int)MAX_APPLICATION_SPEED_RPM,
                   limit_ma / 1000,
                   limit_ma % 1000);
    cli_puts(banner);
  }
}

/* Конец такта 1 кГц. Сначала переход START→RUN, затем трасса и не больше 64 байт команды, чтобы такт не затянулся. */
void MC_APP_PostMediumFrequencyHook_M1(void)
{
  unsigned count = 0u;
  uint8_t byte;

  APP_HYB_PostMediumFrequency();
  command_poll();
  trace_dump_poll();
  while ((count < 64u) && (rx_pop(&byte) != 0))
  {
    cli_on_byte(byte);
    count++;
  }
}
