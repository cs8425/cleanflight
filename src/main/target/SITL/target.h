/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "SITL"

//#define USBD_PRODUCT_STRING "SITL"

#define FLASH_SIZE 2048

#define ACC
#define USE_FAKE_ACC

#define GYRO
#define USE_FAKE_GYRO

#define MAG
#define USE_FAKE_MAG

#define BARO
#define USE_FAKE_BARO

#define USABLE_TIMER_CHANNEL_COUNT 0

#define USE_UART1
//#define UART1_RX_PIN PA10
//#define UART1_TX_PIN PA9

#define USE_UART2
#define UART2_RX_PIN PD6
#define UART2_TX_PIN PD5

#define USE_UART3
#define UART3_RX_PIN PD9
#define UART3_TX_PIN PD8

#define USE_UART4
#define UART4_RX_PIN PC11
#define UART4_TX_PIN PC10

#define USE_UART5
#define UART5_RX_PIN PD2
#define UART5_TX_PIN PC12

#define USE_UART6
#define UART6_RX_PIN PC7
#define UART6_TX_PIN PC6

#define USE_UART7
#define UART7_RX_PIN PE7
#define UART7_TX_PIN PE8

#define USE_UART8
#define UART8_RX_PIN PE0
#define UART8_TX_PIN PE1

//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT 8

#undef USE_ADC
#undef USE_VCP
#undef USE_PPM
#undef USE_PWM
#undef SERIAL_RX
#undef USE_SERIALRX_CRSF
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS
#undef LED_STRIP
#undef TELEMETRY_FRSKY
#undef TELEMETRY_HOTT
#undef TELEMETRY_SMARTPORT
#undef USE_RESOURCE_MGMT
#undef CMS
#undef TELEMETRY_CRSF
#undef TELEMETRY_IBUS
#undef TELEMETRY_JETIEXBUS
#undef TELEMETRY_SRXL
#undef USE_SERIALRX_JETIEXBUS
#undef VTX_COMMON
#undef VTX_CONTROL
#undef VTX_SMARTAUDIO
#undef VTX_TRAMP



# define DEFIO_PORT_USED_COUNT 0
# define DEFIO_PORT_USED_LIST /* empty */
# define DEFIO_PORT_OFFSET_LIST /* empty */

#define LED_STRIP_TIMER 1
#define SOFTSERIAL_1_TIMER 2
#define SOFTSERIAL_2_TIMER 3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff


#define USE_PARAMETER_GROUPS

#define U_ID_0 0
#define U_ID_1 1
#define U_ID_2 2

#define WS2811_DMA_TC_FLAG (void *)1
#define WS2811_DMA_HANDLER_IDENTIFER 0

#include <stdint.h>
#include <stddef.h>

uint32_t SystemCoreClock;

/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */

#ifdef __cplusplus
  #define     __I     volatile                /*!< defines 'read only' permissions      */
#else
  #define     __I     volatile const          /*!< defines 'read only' permissions      */
#endif
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

typedef enum
{
    Mode_TEST = 0x0,
    Mode_Out_PP = 0x10
} GPIO_Mode;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {TEST_IRQ = 0 } IRQn_Type;
typedef enum {
    EXTI_Trigger_Rising = 0x08,
    EXTI_Trigger_Falling = 0x0C,
    EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

typedef struct
{
  uint32_t IDR;
  uint32_t ODR;
  uint32_t BSRR;
  uint32_t BRR;
} GPIO_TypeDef;

#define GPIOA_BASE (0x0000)

typedef struct
{
    void* test;
} TIM_TypeDef;

typedef struct
{
    void* test;
} TIM_OCInitTypeDef;

typedef struct {
    void* test;
} DMA_TypeDef;

typedef struct {
    void* test;
} DMA_Channel_TypeDef;

uint8_t DMA_GetFlagStatus(void *);
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState );
void DMA_ClearFlag(uint32_t);

typedef struct
{
    void* test;
} SPI_TypeDef;

typedef struct
{
    void* test;
} USART_TypeDef;

typedef struct
{
    void* test;
} I2C_TypeDef;

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;

/*
typedef enum
{
    Mode_TEST = 0x0,
    Mode_Out_PP = 0x10
} GPIO_Mode;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {TEST_IRQ = 0 } IRQn_Type;
typedef enum {
    EXTI_Trigger_Rising = 0x08,
    EXTI_Trigger_Falling = 0x0C,
    EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

typedef struct
{
  __IO uint16_t CR1;
  __IO uint16_t CR2;
  __IO uint16_t SMCR;
  __IO uint16_t DIER;
  __IO uint16_t SR;
  __IO uint16_t EGR;
  __IO uint16_t CCMR1;
  __IO uint16_t CCMR2;
  __IO uint16_t CCER;
  __IO uint16_t CNT;
  __IO uint16_t PSC;
  __IO uint16_t ARR;
  __IO uint16_t RCR;
  __IO uint16_t CCR1;
  __IO uint16_t CCR2;
  __IO uint16_t CCR3;
  __IO uint16_t CCR4;
  __IO uint16_t BDTR;
  __IO uint16_t DCR;
  __IO uint16_t DMAR;
} TIM_TypeDef;

typedef struct
{
  uint16_t TIM_OCMode;
} TIM_OCInitTypeDef;
#define TIM_OCPolarity_Low     0
#define TIM_OCPolarity_High    1
#define TIM_OCMode_Timing      2
#define TIM_OutputState_Enable 4
#define TIM_OCMode_Inactive    8

typedef struct
{
  uint16_t TIM_Prescaler;
  uint16_t TIM_CounterMode;
  uint16_t TIM_Period;
  uint16_t TIM_ClockDivision;
  uint8_t TIM_RepetitionCounter;
} TIM_TimeBaseInitTypeDef;

#define TIM_DMA_CC1                        ((uint16_t)0x0200)
#define TIM_DMA_CC2                        ((uint16_t)0x0400)
#define TIM_DMA_CC3                        ((uint16_t)0x0800)
#define TIM_DMA_CC4                        ((uint16_t)0x1000)
#define TIM_Channel_1                      ((uint16_t)0x0000)
#define TIM_Channel_2                      ((uint16_t)0x0004)
#define TIM_Channel_3                      ((uint16_t)0x0008)
#define TIM_Channel_4                      ((uint16_t)0x000C)

typedef struct {
  __IO uint32_t ISR;
  __IO uint32_t IFCR;
} DMA_TypeDef;

typedef struct {
  __IO uint32_t CCR;
  __IO uint32_t CNDTR;
  __IO uint32_t CPAR;
  __IO uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;
  uint32_t DMA_MemoryBaseAddr;
  uint32_t DMA_DIR;
  uint32_t DMA_BufferSize;
  uint32_t DMA_PeripheralInc;
  uint32_t DMA_MemoryInc;
  uint32_t DMA_PeripheralDataSize;
  uint32_t DMA_MemoryDataSize;
  uint32_t DMA_Mode;
  uint32_t DMA_Priority;
  uint32_t DMA_M2M;
}DMA_InitTypeDef;

#define DMA_IT_TC 1

uint8_t DMA_GetFlagStatus(void *);
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState );
void DMA_ClearFlag(uint32_t);



typedef struct
{
  __IO uint16_t CR1;
  __IO uint16_t CR2;
  __IO uint16_t SR;
  __IO uint16_t DR;
  __IO uint16_t CRCPR;
  __IO uint16_t RXCRCR;
  __IO uint16_t TXCRCR;
  __IO uint16_t I2SCFGR;
  __IO uint16_t I2SPR;
} SPI_TypeDef;



typedef struct
{
  __IO uint16_t SR;
  __IO uint16_t DR;
  __IO uint16_t BRR;
  __IO uint16_t CR1;
  __IO uint16_t CR2;
  __IO uint16_t CR3;
  __IO uint16_t GTPR;
} USART_TypeDef;
#define USART_Parity_No                      ((uint16_t)0x0000)
#define USART_Parity_Even                    ((uint16_t)0x0400)
#define USART_Parity_Odd                     ((uint16_t)0x0600) 



typedef struct
{
  __IO uint16_t CR1;
  __IO uint16_t CR2;
  __IO uint16_t OAR1;
  __IO uint16_t OAR2;
  __IO uint16_t DR;
  __IO uint16_t SR1;
  __IO uint16_t SR2;
  __IO uint16_t CCR;
  __IO uint16_t TRISE;
} I2C_TypeDef;
*/


