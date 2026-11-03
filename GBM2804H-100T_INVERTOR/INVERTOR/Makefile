##############################################################################
# INVERTOR - STM32G431RB + MCSDK 6.1.2 (P-NUCLEO-IHM03)
# Standalone build that mirrors STM32CubeIDE/.cproject (sources, defines, flags).
#
#   make                 - Debug build (default)
#   make CONFIG=Release  - Release build
#   make clean
#   make flash           - program via ST-LINK (STM32_Programmer_CLI)
#
# Toolchain is taken from PATH; tools/env.ps1 adds the STM32CubeIDE bundled tools.
##############################################################################

TARGET  := INVERTOR
CONFIG  ?= Debug
BUILD   := build/$(CONFIG)

PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size
PROG    ?= STM32_Programmer_CLI

MCSDK   := MCSDK_v6.1.2-Full/MotorControl/MCSDK/MCLib
HAL     := Drivers/STM32G4xx_HAL_Driver/Src

# ---------------------------------------------------------------- sources ---
APP_SRC := \
  Src/main.c \
  Src/app_capture.c \
  Src/app_cli.c \
  Src/app_hybrid.c \
  Src/app_tuning.c \
  Src/mc_api.c \
  Src/mc_app_hooks.c \
  Src/mc_config.c \
  Src/mc_interface.c \
  Src/mc_math.c \
  Src/mc_parameters.c \
  Src/mc_tasks.c \
  Src/motorcontrol.c \
  Src/pwm_curr_fdbk.c \
  Src/regular_conversion_manager.c \
  Src/stm32g4xx_hal_msp.c \
  Src/stm32g4xx_it.c \
  Src/stm32g4xx_mc_it.c \
  Src/system_stm32g4xx.c \
  STM32CubeIDE/Application/User/syscalls.c \
  STM32CubeIDE/Application/User/sysmem.c

HAL_SRC := $(addprefix $(HAL)/, \
  stm32g4xx_hal.c \
  stm32g4xx_hal_adc.c \
  stm32g4xx_hal_adc_ex.c \
  stm32g4xx_hal_cordic.c \
  stm32g4xx_hal_cortex.c \
  stm32g4xx_hal_exti.c \
  stm32g4xx_hal_flash.c \
  stm32g4xx_hal_flash_ex.c \
  stm32g4xx_hal_flash_ramfunc.c \
  stm32g4xx_hal_gpio.c \
  stm32g4xx_hal_pwr.c \
  stm32g4xx_hal_pwr_ex.c \
  stm32g4xx_hal_rcc.c \
  stm32g4xx_hal_rcc_ex.c \
  stm32g4xx_hal_tim.c \
  stm32g4xx_hal_tim_ex.c \
  stm32g4xx_hal_uart.c \
  stm32g4xx_hal_uart_ex.c \
  stm32g4xx_ll_adc.c)

MC_SRC := $(addprefix $(MCSDK)/Any/Src/, \
  bus_voltage_sensor.c \
  circle_limitation.c \
  ntc_temperature_sensor.c \
  open_loop.c \
  pid_regulator.c \
  pqd_motor_power_measurement.c \
  pwm_common.c \
  r_divider_bus_voltage_sensor.c \
  ramp_ext_mngr.c \
  revup_ctrl.c \
  speed_pos_fdbk.c \
  speed_torq_ctrl.c \
  sto_pll_speed_pos_fdbk.c \
  virtual_speed_sensor.c) \
  $(MCSDK)/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c

C_SRC   := $(APP_SRC) $(HAL_SRC) $(MC_SRC)
ASM_SRC := STM32CubeIDE/Application/Startup/startup_stm32g431rbtx.s
LDSCRIPT := STM32CubeIDE/STM32G431RBTX_FLASH.ld

# ------------------------------------------------------------------ flags ---
INCLUDES := \
  -IInc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
  -I$(MCSDK)/Any/Inc \
  -I$(MCSDK)/G4xx/Inc \
  -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
  -IDrivers/CMSIS/Include \
  -IDrivers/CMSIS/DSP/Include

DEFINES := -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx

MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

ifeq ($(CONFIG),Debug)
  DEFINES += -DDEBUG
  DBG := -g3
else
  DBG := -g0
endif

OPT ?= -Ofast

CFLAGS  := $(MCU) -std=gnu11 $(DBG) $(OPT) $(DEFINES) $(INCLUDES) \
           -ffunction-sections -fdata-sections -Wall -fstack-usage \
           --specs=nano.specs -MMD -MP
ASFLAGS := $(MCU) $(DBG) $(DEFINES) --specs=nano.specs -MMD -MP
LDFLAGS := $(MCU) -T$(LDSCRIPT) --specs=nosys.specs --specs=nano.specs \
           -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--gc-sections -static \
           -Wl,--print-memory-usage \
           -Wl,--start-group -lc -lm -Wl,--end-group

# ------------------------------------------------------------------ rules ---
OBJS := $(addprefix $(BUILD)/obj/, $(C_SRC:.c=.o) $(ASM_SRC:.s=.o))

ELF := $(BUILD)/$(TARGET).elf
HEX := $(BUILD)/$(TARGET).hex
BIN := $(BUILD)/$(TARGET).bin

.PHONY: all clean flash size
all: $(ELF) $(HEX) $(BIN) size

$(BUILD)/obj/%.o: %.c Makefile
	@mkdir -p $(@D)
	@echo CC $<
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/obj/%.o: %.s Makefile
	@mkdir -p $(@D)
	@echo AS $<
	@$(AS) $(ASFLAGS) -c $< -o $@

$(ELF): $(OBJS) $(LDSCRIPT)
	@echo LD $@
	@$(CC) $(OBJS) $(LDFLAGS) -o $@

$(HEX): $(ELF)
	@$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	@$(OBJCOPY) -O binary $< $@

size: $(ELF)
	@$(SIZE) $<

flash: $(ELF)
	$(PROG) -c port=SWD mode=UR reset=HWrst -w $(ELF) -v -rst

clean:
	rm -rf build

-include $(OBJS:.o=.d)
