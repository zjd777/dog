# Explicit sources; unused legacy LED/key/beeper modules are excluded.
C_SOURCES := \
  Core/Src/main.c \
  Core/Src/gpio.c \
  Core/Src/dma.c \
  Core/Src/spi.c \
  Core/Src/stm32h7xx_it.c \
  Core/Src/stm32h7xx_hal_msp.c \
  Core/Src/usart.c \
  Core/Src/tim.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c \
  Core/Src/system_stm32h7xx.c \
  Device/BMI088/BMI088driver.c \
  Device/BMI088/BMI088Middleware.c \
  Device/BMI088/imu.c \
  BSP/bsp_uart.c \
  BSP/bsp_sbus.c \
  my/ANO_TC.c \
  my/action.c \
  my/dog.c \
  my/fsm.c \
  my/delay_DWT.c \
  my/pos_ctrl.c \
  my/walk.c \
  Device/remote_control/obstacle_course.c\
  build_support/runtime.c

CXX_SOURCES := \
  my/control_task.cpp \
  my/motor_task.cpp \
  Device/unitree/UART.cpp \
  Device/unitree/motor_YS.cpp \
  Device/unitree/math_support.cpp \
  Device/remote_control/remote_control.cpp

ASM_SOURCES := build_support/startup_stm32h723xx.s
