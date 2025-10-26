/**
 * @file    UART_STM32F4xx.h
 * @author  Deadline039
 * @brief   Chip Support Package of UART on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 * @note    Generate Automatically.
 */

#ifndef __UART_STM32F4xx_H
#define __UART_STM32F4xx_H

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*****************************************************************************
 * @defgroup UART Public Marco. 
 * @{
 */

#define UART_INIT_OK         0
#define UART_INIT_FAIL       1
#define UART_INIT_DMA_FAIL   2
#define UART_INIT_MEM_FAIL   3
#define UART_INITED          4

#define UART_DEINIT_OK       0
#define UART_DEINIT_FAIL     1
#define UART_DEINIT_DMA_FAIL 2
#define UART_NO_INIT         3

/**
 * @}
 */

/* clang-format off */


/*****************************************************************************
 * @defgroup USART1 Functions
 * @{
 */

#if USART1_ENABLE

#if (USART1_TX_ID == 0)
#elif (USART1_TX_ID == 1)
#  define USART1_TX_GPIO_AF GPIO_AF7_USART1
#elif (USART1_TX_ID == 2)
#  define USART1_TX_GPIO_AF GPIO_AF7_USART1
#elif (USART1_TX_ID == 3)
#  define USART1_TX_GPIO_AF GPIO_AF7_USART1
#  if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PA15 only can be configured as USART1 TX on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART1_TX_ID */

#if (USART1_RX_ID == 0)
#elif (USART1_RX_ID == 1)
#  define USART1_RX_GPIO_AF GPIO_AF7_USART1
#elif (USART1_RX_ID == 2)
#  define USART1_RX_GPIO_AF GPIO_AF7_USART1
#elif (USART1_RX_ID == 3)
#  define USART1_RX_GPIO_AF GPIO_AF7_USART1
#  if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PB3 only can be configured as USART1 RX on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART1_RX_ID */

#if (USART1_CTS_ID == 0)
#elif (USART1_CTS_ID == 1)
#  define USART1_CTS_GPIO_AF GPIO_AF7_USART1
#endif  /* USART1_CTS_ID */

#if (USART1_RTS_ID == 0)
#elif (USART1_RTS_ID == 1)
#  define USART1_RTS_GPIO_AF GPIO_AF7_USART1
#endif  /* USART1_RTS_ID */

extern UART_HandleTypeDef usart1_handle;

uint8_t usart1_init(uint32_t baud_rate);
uint8_t usart1_deinit(void); 

#  if USART1_RX_DMA
#    define USART1_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(USART1_RX_DMA_NUMBER, USART1_RX_DMA_STREAM)
#    define USART1_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(USART1_RX_DMA_NUMBER, USART1_RX_DMA_STREAM)
#  endif /* USART1_RX_DMA */

#  if USART1_TX_DMA
#    define USART1_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(USART1_TX_DMA_NUMBER, USART1_TX_DMA_STREAM)
#    define USART1_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(USART1_TX_DMA_NUMBER, USART1_TX_DMA_STREAM)
#  endif /* USART1_TX_DMA */

#endif /* USART1_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup USART2 Functions
 * @{
 */

#if USART2_ENABLE

#if (USART2_TX_ID == 0)
#elif (USART2_TX_ID == 1)
#  define USART2_TX_GPIO_AF GPIO_AF7_USART2
#elif (USART2_TX_ID == 2)
#  define USART2_TX_GPIO_AF GPIO_AF7_USART2
#  if (defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PD5 can not be configured as USART2 TX on STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART2_TX_ID */

#if (USART2_RX_ID == 0)
#elif (USART2_RX_ID == 1)
#  define USART2_RX_GPIO_AF GPIO_AF7_USART2
#elif (USART2_RX_ID == 2)
#  define USART2_RX_GPIO_AF GPIO_AF7_USART2
#  if (defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PD6 can not be configured as USART2 RX on STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART2_RX_ID */

#if (USART2_CTS_ID == 0)
#elif (USART2_CTS_ID == 1)
#  define USART2_CTS_GPIO_AF GPIO_AF7_USART2
#elif (USART2_CTS_ID == 2)
#  define USART2_CTS_GPIO_AF GPIO_AF7_USART2
#  if (defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PD3 can not be configured as USART2 CTS on STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART2_CTS_ID */

#if (USART2_RTS_ID == 0)
#elif (USART2_RTS_ID == 1)
#  define USART2_RTS_GPIO_AF GPIO_AF7_USART2
#elif (USART2_RTS_ID == 2)
#  define USART2_RTS_GPIO_AF GPIO_AF7_USART2
#  if (defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PD4 can not be configured as USART2 RTS on STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART2_RTS_ID */

extern UART_HandleTypeDef usart2_handle;

uint8_t usart2_init(uint32_t baud_rate);
uint8_t usart2_deinit(void); 

#  if USART2_RX_DMA
#    if (USART2_RX_DMA_NUMBER == 1) && (USART2_RX_DMA_STREAM == 7) && (USART2_RX_DMA_CHANNEL == 6)

#     if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#       error "This USART2 DMA RX configuration only valid on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#     endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#    endif /* (USART2_RX_DMA_NUMBER == 1) && (USART2_RX_DMA_STREAM == 7) && (USART2_RX_DMA_CHANNEL == 6) */

#    define USART2_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(USART2_RX_DMA_NUMBER, USART2_RX_DMA_STREAM)
#    define USART2_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(USART2_RX_DMA_NUMBER, USART2_RX_DMA_STREAM)
#  endif /* USART2_RX_DMA */

#  if USART2_TX_DMA
#    define USART2_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(USART2_TX_DMA_NUMBER, USART2_TX_DMA_STREAM)
#    define USART2_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(USART2_TX_DMA_NUMBER, USART2_TX_DMA_STREAM)
#  endif /* USART2_TX_DMA */

#endif /* USART2_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup USART3 Functions
 * @{
 */

#if USART3_ENABLE

#if (USART3_TX_ID == 0)
#elif (USART3_TX_ID == 1)
#  define USART3_TX_GPIO_AF GPIO_AF7_USART3
#elif (USART3_TX_ID == 2)
#  define USART3_TX_GPIO_AF GPIO_AF7_USART3
#elif (USART3_TX_ID == 3)
#  define USART3_TX_GPIO_AF GPIO_AF7_USART3
#endif  /* USART3_TX_ID */

#if (USART3_RX_ID == 0)
#elif (USART3_RX_ID == 1)
#  define USART3_RX_GPIO_AF GPIO_AF7_USART3
#elif (USART3_RX_ID == 2)
#  define USART3_RX_GPIO_AF GPIO_AF7_USART3
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PC5 can not be configured as USART3 RX on STM32F479xx, STM32F437xx, STM32F405xx, STM32F415xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (USART3_RX_ID == 3)
#  define USART3_RX_GPIO_AF GPIO_AF7_USART3
#elif (USART3_RX_ID == 4)
#  define USART3_RX_GPIO_AF GPIO_AF7_USART3
#endif  /* USART3_RX_ID */

#if (USART3_CTS_ID == 0)
#elif (USART3_CTS_ID == 1)
#  define USART3_CTS_GPIO_AF GPIO_AF8_USART3
#elif (USART3_CTS_ID == 2)
#  define USART3_CTS_GPIO_AF GPIO_AF7_USART3
#endif  /* USART3_CTS_ID */

#if (USART3_RTS_ID == 0)
#elif (USART3_RTS_ID == 1)
#  define USART3_RTS_GPIO_AF GPIO_AF7_USART3
#elif (USART3_RTS_ID == 2)
#  define USART3_RTS_GPIO_AF GPIO_AF7_USART3
#endif  /* USART3_RTS_ID */

extern UART_HandleTypeDef usart3_handle;

uint8_t usart3_init(uint32_t baud_rate);
uint8_t usart3_deinit(void); 

#  if USART3_RX_DMA
#    define USART3_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(USART3_RX_DMA_NUMBER, USART3_RX_DMA_STREAM)
#    define USART3_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(USART3_RX_DMA_NUMBER, USART3_RX_DMA_STREAM)
#  endif /* USART3_RX_DMA */

#  if USART3_TX_DMA
#    define USART3_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(USART3_TX_DMA_NUMBER, USART3_TX_DMA_STREAM)
#    define USART3_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(USART3_TX_DMA_NUMBER, USART3_TX_DMA_STREAM)
#  endif /* USART3_TX_DMA */

#endif /* USART3_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup UART4 Functions
 * @{
 */

#if UART4_ENABLE

#if (UART4_TX_ID == 0)
#elif (UART4_TX_ID == 1)
#  define UART4_TX_GPIO_AF GPIO_AF8_UART4
#elif (UART4_TX_ID == 2)
#  define UART4_TX_GPIO_AF GPIO_AF11_UART4
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PA12 can not be configured as UART4 TX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART4_TX_ID == 3)
#  define UART4_TX_GPIO_AF GPIO_AF11_UART4
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PD1 can not be configured as UART4 TX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART4_TX_ID == 4)
#  define UART4_TX_GPIO_AF GPIO_AF8_UART4
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PD10 can not be configured as UART4 TX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART4_TX_ID == 5)
#  define UART4_TX_GPIO_AF GPIO_AF8_UART4
#  if (defined(STM32F423xx) || defined(STM32F413xx))
#    error "PC10 can not be configured as UART4 TX on STM32F423xx, STM32F413xx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F413xx)) */
#endif  /* UART4_TX_ID */

#if (UART4_RX_ID == 0)
#elif (UART4_RX_ID == 1)
#  define UART4_RX_GPIO_AF GPIO_AF8_UART4
#elif (UART4_RX_ID == 2)
#  define UART4_RX_GPIO_AF GPIO_AF11_UART4
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PA11 can not be configured as UART4 RX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART4_RX_ID == 3)
#  define UART4_RX_GPIO_AF GPIO_AF8_UART4
#elif (UART4_RX_ID == 4)
#  define UART4_RX_GPIO_AF GPIO_AF11_UART4
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PD0 can not be configured as UART4 RX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#endif  /* UART4_RX_ID */

#if (UART4_CTS_ID == 0)
#elif (UART4_CTS_ID == 1)
#  define UART4_CTS_GPIO_AF GPIO_AF8_UART4
#  if !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PB0 only can be configured as UART4 CTS on STM32F411xE, STM32F446xx, STM32F401xE, STM32F412Rx, STM32F412Cx, STM32F401xC, STM32F412Zx, STM32F410Tx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* UART4_CTS_ID */

#if (UART4_RTS_ID == 0)
#elif (UART4_RTS_ID == 1)
#  define UART4_RTS_GPIO_AF GPIO_AF8_UART4
#  if !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PA15 only can be configured as UART4 RTS on STM32F411xE, STM32F446xx, STM32F401xE, STM32F412Rx, STM32F412Cx, STM32F401xC, STM32F412Zx, STM32F410Tx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* UART4_RTS_ID */

extern UART_HandleTypeDef uart4_handle;

uint8_t uart4_init(uint32_t baud_rate);
uint8_t uart4_deinit(void); 

#  if UART4_RX_DMA
#    define UART4_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART4_RX_DMA_NUMBER, UART4_RX_DMA_STREAM)
#    define UART4_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART4_RX_DMA_NUMBER, UART4_RX_DMA_STREAM)
#  endif /* UART4_RX_DMA */

#  if UART4_TX_DMA
#    define UART4_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART4_TX_DMA_NUMBER, UART4_TX_DMA_STREAM)
#    define UART4_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART4_TX_DMA_NUMBER, UART4_TX_DMA_STREAM)
#  endif /* UART4_TX_DMA */

#endif /* UART4_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup UART5 Functions
 * @{
 */

#if UART5_ENABLE

#if (UART5_TX_ID == 0)
#elif (UART5_TX_ID == 1)
#  define UART5_TX_GPIO_AF GPIO_AF11_UART5
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PB6 can not be configured as UART5 TX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART5_TX_ID == 2)
#  define UART5_TX_GPIO_AF GPIO_AF11_UART5
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PB9 can not be configured as UART5 TX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART5_TX_ID == 3)
#  define UART5_TX_GPIO_AF GPIO_AF11_UART5
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PB13 can not be configured as UART5 TX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART5_TX_ID == 4)
#  define UART5_TX_GPIO_AF GPIO_AF8_UART5
#elif (UART5_TX_ID == 5)
#  define UART5_TX_GPIO_AF GPIO_AF8_UART5
#  if !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE8 only can be configured as UART5 TX on STM32F411xE, STM32F446xx, STM32F401xE, STM32F412Rx, STM32F412Cx, STM32F401xC, STM32F412Zx, STM32F410Tx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* UART5_TX_ID */

#if (UART5_RX_ID == 0)
#elif (UART5_RX_ID == 1)
#  define UART5_RX_GPIO_AF GPIO_AF11_UART5
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PB5 can not be configured as UART5 RX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART5_RX_ID == 2)
#  define UART5_RX_GPIO_AF GPIO_AF11_UART5
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PB8 can not be configured as UART5 RX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART5_RX_ID == 3)
#  define UART5_RX_GPIO_AF GPIO_AF11_UART5
#  if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PB12 can not be configured as UART5 RX on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (UART5_RX_ID == 4)
#  define UART5_RX_GPIO_AF GPIO_AF8_UART5
#elif (UART5_RX_ID == 5)
#  define UART5_RX_GPIO_AF GPIO_AF8_UART5
#  if !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE7 only can be configured as UART5 RX on STM32F411xE, STM32F446xx, STM32F401xE, STM32F412Rx, STM32F412Cx, STM32F401xC, STM32F412Zx, STM32F410Tx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* UART5_RX_ID */

#if (UART5_CTS_ID == 0)
#elif (UART5_CTS_ID == 1)
#  define UART5_CTS_GPIO_AF GPIO_AF7_UART5
#  if !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PC9 only can be configured as UART5 CTS on STM32F411xE, STM32F446xx, STM32F401xE, STM32F412Rx, STM32F412Cx, STM32F401xC, STM32F412Zx, STM32F410Tx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* UART5_CTS_ID */

#if (UART5_RTS_ID == 0)
#elif (UART5_RTS_ID == 1)
#  define UART5_RTS_GPIO_AF GPIO_AF7_UART5
#  if !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PC8 only can be configured as UART5 RTS on STM32F411xE, STM32F446xx, STM32F401xE, STM32F412Rx, STM32F412Cx, STM32F401xC, STM32F412Zx, STM32F410Tx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* UART5_RTS_ID */

extern UART_HandleTypeDef uart5_handle;

uint8_t uart5_init(uint32_t baud_rate);
uint8_t uart5_deinit(void); 

#  if UART5_RX_DMA
#    define UART5_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART5_RX_DMA_NUMBER, UART5_RX_DMA_STREAM)
#    define UART5_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART5_RX_DMA_NUMBER, UART5_RX_DMA_STREAM)
#  endif /* UART5_RX_DMA */

#  if UART5_TX_DMA
#    if (UART5_TX_DMA_NUMBER == 1) && (UART5_TX_DMA_STREAM == 7) && (UART5_TX_DMA_CHANNEL == 8)

#     if (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#       error "This UART5 DMA TX configuration invalid on STM32F479xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F437xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#     endif  /* (defined(STM32F479xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F437xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#    endif /* (UART5_TX_DMA_NUMBER == 1) && (UART5_TX_DMA_STREAM == 7) && (UART5_TX_DMA_CHANNEL == 8) */

#    if (UART5_TX_DMA_NUMBER == 1) && (UART5_TX_DMA_STREAM == 7) && (UART5_TX_DMA_CHANNEL == 4)

#     if (defined(STM32F423xx) || defined(STM32F413xx))
#       error "This UART5 DMA TX configuration invalid on STM32F423xx, STM32F413xx! "
#     endif  /* (defined(STM32F423xx) || defined(STM32F413xx)) */
#    endif /* (UART5_TX_DMA_NUMBER == 1) && (UART5_TX_DMA_STREAM == 7) && (UART5_TX_DMA_CHANNEL == 4) */

#    define UART5_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART5_TX_DMA_NUMBER, UART5_TX_DMA_STREAM)
#    define UART5_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART5_TX_DMA_NUMBER, UART5_TX_DMA_STREAM)
#  endif /* UART5_TX_DMA */

#endif /* UART5_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup USART6 Functions
 * @{
 */

#if USART6_ENABLE

#if (USART6_TX_ID == 0)
#elif (USART6_TX_ID == 1)
#  define USART6_TX_GPIO_AF GPIO_AF8_USART6
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F446xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PA11 can not be configured as USART6 TX on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F446xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F446xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (USART6_TX_ID == 2)
#  define USART6_TX_GPIO_AF GPIO_AF8_USART6
#elif (USART6_TX_ID == 3)
#  define USART6_TX_GPIO_AF GPIO_AF8_USART6
#  if (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PG14 can not be configured as USART6 TX on STM32F411xE, STM32F401xE, STM32F401xC, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART6_TX_ID */

#if (USART6_RX_ID == 0)
#elif (USART6_RX_ID == 1)
#  define USART6_RX_GPIO_AF GPIO_AF8_USART6
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F446xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PA12 can not be configured as USART6 RX on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F446xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F446xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (USART6_RX_ID == 2)
#  define USART6_RX_GPIO_AF GPIO_AF8_USART6
#elif (USART6_RX_ID == 3)
#  define USART6_RX_GPIO_AF GPIO_AF8_USART6
#  if (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PG9 can not be configured as USART6 RX on STM32F411xE, STM32F401xE, STM32F401xC, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART6_RX_ID */

#if (USART6_CTS_ID == 0)
#elif (USART6_CTS_ID == 1)
#  define USART6_CTS_GPIO_AF GPIO_AF8_USART6
#  if (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PG13 can not be configured as USART6 CTS on STM32F411xE, STM32F401xE, STM32F401xC, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (USART6_CTS_ID == 2)
#  define USART6_CTS_GPIO_AF GPIO_AF8_USART6
#  if (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PG15 can not be configured as USART6 CTS on STM32F411xE, STM32F401xE, STM32F401xC, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART6_CTS_ID */

#if (USART6_RTS_ID == 0)
#elif (USART6_RTS_ID == 1)
#  define USART6_RTS_GPIO_AF GPIO_AF8_USART6
#  if (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PG8 can not be configured as USART6 RTS on STM32F411xE, STM32F401xE, STM32F401xC, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (USART6_RTS_ID == 2)
#  define USART6_RTS_GPIO_AF GPIO_AF8_USART6
#  if (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PG12 can not be configured as USART6 RTS on STM32F411xE, STM32F401xE, STM32F401xC, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* USART6_RTS_ID */

extern UART_HandleTypeDef usart6_handle;

uint8_t usart6_init(uint32_t baud_rate);
uint8_t usart6_deinit(void); 

#  if USART6_RX_DMA
#    define USART6_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(USART6_RX_DMA_NUMBER, USART6_RX_DMA_STREAM)
#    define USART6_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(USART6_RX_DMA_NUMBER, USART6_RX_DMA_STREAM)
#  endif /* USART6_RX_DMA */

#  if USART6_TX_DMA
#    define USART6_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(USART6_TX_DMA_NUMBER, USART6_TX_DMA_STREAM)
#    define USART6_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(USART6_TX_DMA_NUMBER, USART6_TX_DMA_STREAM)
#  endif /* USART6_TX_DMA */

#endif /* USART6_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup UART7 Functions
 * @{
 */

#if UART7_ENABLE

#if (UART7_TX_ID == 0)
#elif (UART7_TX_ID == 1)
#  define UART7_TX_GPIO_AF GPIO_AF8_UART7
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PA15 can not be configured as UART7 TX on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (UART7_TX_ID == 2)
#  define UART7_TX_GPIO_AF GPIO_AF8_UART7
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PB4 can not be configured as UART7 TX on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (UART7_TX_ID == 3)
#  define UART7_TX_GPIO_AF GPIO_AF8_UART7
#elif (UART7_TX_ID == 4)
#  define UART7_TX_GPIO_AF GPIO_AF8_UART7
#endif  /* UART7_TX_ID */

#if (UART7_RX_ID == 0)
#elif (UART7_RX_ID == 1)
#  define UART7_RX_GPIO_AF GPIO_AF8_UART7
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PA8 can not be configured as UART7 RX on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (UART7_RX_ID == 2)
#  define UART7_RX_GPIO_AF GPIO_AF8_UART7
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PB3 can not be configured as UART7 RX on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (UART7_RX_ID == 3)
#  define UART7_RX_GPIO_AF GPIO_AF8_UART7
#elif (UART7_RX_ID == 4)
#  define UART7_RX_GPIO_AF GPIO_AF8_UART7
#endif  /* UART7_RX_ID */

extern UART_HandleTypeDef uart7_handle;

uint8_t uart7_init(uint32_t baud_rate);
uint8_t uart7_deinit(void); 

#  if UART7_RX_DMA
#    define UART7_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART7_RX_DMA_NUMBER, UART7_RX_DMA_STREAM)
#    define UART7_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART7_RX_DMA_NUMBER, UART7_RX_DMA_STREAM)
#  endif /* UART7_RX_DMA */

#  if UART7_TX_DMA
#    define UART7_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART7_TX_DMA_NUMBER, UART7_TX_DMA_STREAM)
#    define UART7_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART7_TX_DMA_NUMBER, UART7_TX_DMA_STREAM)
#  endif /* UART7_TX_DMA */

#endif /* UART7_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup UART8 Functions
 * @{
 */

#if UART8_ENABLE

#if (UART8_TX_ID == 0)
#elif (UART8_TX_ID == 1)
#  define UART8_TX_GPIO_AF GPIO_AF8_UART8
#elif (UART8_TX_ID == 2)
#  define UART8_TX_GPIO_AF GPIO_AF8_UART8
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PF9 can not be configured as UART8 TX on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#endif  /* UART8_TX_ID */

#if (UART8_RX_ID == 0)
#elif (UART8_RX_ID == 1)
#  define UART8_RX_GPIO_AF GPIO_AF8_UART8
#elif (UART8_RX_ID == 2)
#  define UART8_RX_GPIO_AF GPIO_AF8_UART8
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PF8 can not be configured as UART8 RX on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#endif  /* UART8_RX_ID */

extern UART_HandleTypeDef uart8_handle;

uint8_t uart8_init(uint32_t baud_rate);
uint8_t uart8_deinit(void); 

#  if UART8_RX_DMA
#    if (UART8_RX_DMA_NUMBER == 1) && (UART8_RX_DMA_STREAM == 0) && (UART8_RX_DMA_CHANNEL == 5)

#     if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#       error "This UART8 DMA RX configuration invalid on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#     endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#    endif /* (UART8_RX_DMA_NUMBER == 1) && (UART8_RX_DMA_STREAM == 0) && (UART8_RX_DMA_CHANNEL == 5) */

#    define UART8_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART8_RX_DMA_NUMBER, UART8_RX_DMA_STREAM)
#    define UART8_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART8_RX_DMA_NUMBER, UART8_RX_DMA_STREAM)
#  endif /* UART8_RX_DMA */

#  if UART8_TX_DMA
#    if (UART8_TX_DMA_NUMBER == 1) && (UART8_TX_DMA_STREAM == 0) && (UART8_TX_DMA_CHANNEL == 5)

#     if (defined(STM32F423xx) || defined(STM32F413xx))
#       error "This UART8 DMA TX configuration invalid on STM32F423xx, STM32F413xx! "
#     endif  /* (defined(STM32F423xx) || defined(STM32F413xx)) */
#    endif /* (UART8_TX_DMA_NUMBER == 1) && (UART8_TX_DMA_STREAM == 0) && (UART8_TX_DMA_CHANNEL == 5) */

#    define UART8_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART8_TX_DMA_NUMBER, UART8_TX_DMA_STREAM)
#    define UART8_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART8_TX_DMA_NUMBER, UART8_TX_DMA_STREAM)
#  endif /* UART8_TX_DMA */

#endif /* UART8_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup UART9 Functions
 * @{
 */

#if UART9_ENABLE

#if (UART9_TX_ID == 0)
#elif (UART9_TX_ID == 1)
#  define UART9_TX_GPIO_AF GPIO_AF11_UART9
#elif (UART9_TX_ID == 2)
#  define UART9_TX_GPIO_AF GPIO_AF11_UART9
#endif  /* UART9_TX_ID */

#if (UART9_RX_ID == 0)
#elif (UART9_RX_ID == 1)
#  define UART9_RX_GPIO_AF GPIO_AF11_UART9
#elif (UART9_RX_ID == 2)
#  define UART9_RX_GPIO_AF GPIO_AF11_UART9
#endif  /* UART9_RX_ID */

extern UART_HandleTypeDef uart9_handle;

uint8_t uart9_init(uint32_t baud_rate);
uint8_t uart9_deinit(void); 

#  if UART9_RX_DMA
#    define UART9_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART9_RX_DMA_NUMBER, UART9_RX_DMA_STREAM)
#    define UART9_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART9_RX_DMA_NUMBER, UART9_RX_DMA_STREAM)
#  endif /* UART9_RX_DMA */

#  if UART9_TX_DMA
#    define UART9_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART9_TX_DMA_NUMBER, UART9_TX_DMA_STREAM)
#    define UART9_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART9_TX_DMA_NUMBER, UART9_TX_DMA_STREAM)
#  endif /* UART9_TX_DMA */

#endif /* UART9_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup UART10 Functions
 * @{
 */

#if UART10_ENABLE

#if (UART10_TX_ID == 0)
#elif (UART10_TX_ID == 1)
#  define UART10_TX_GPIO_AF GPIO_AF11_UART10
#elif (UART10_TX_ID == 2)
#  define UART10_TX_GPIO_AF GPIO_AF11_UART10
#endif  /* UART10_TX_ID */

#if (UART10_RX_ID == 0)
#elif (UART10_RX_ID == 1)
#  define UART10_RX_GPIO_AF GPIO_AF11_UART10
#elif (UART10_RX_ID == 2)
#  define UART10_RX_GPIO_AF GPIO_AF11_UART10
#endif  /* UART10_RX_ID */

extern UART_HandleTypeDef uart10_handle;

uint8_t uart10_init(uint32_t baud_rate);
uint8_t uart10_deinit(void); 

#  if UART10_RX_DMA
#    define UART10_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART10_RX_DMA_NUMBER, UART10_RX_DMA_STREAM)
#    define UART10_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART10_RX_DMA_NUMBER, UART10_RX_DMA_STREAM)
#  endif /* UART10_RX_DMA */

#  if UART10_TX_DMA
#    define UART10_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(UART10_TX_DMA_NUMBER, UART10_TX_DMA_STREAM)
#    define UART10_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(UART10_TX_DMA_NUMBER, UART10_TX_DMA_STREAM)
#  endif /* UART10_TX_DMA */

#endif /* UART10_ENABLE */

/**
 * @}
 */


/* clang-format on */

/*****************************************************************************
 * @defgroup Public uart function.
 * @{
 */

int uart_printf(UART_HandleTypeDef *huart, const char *__format, ...);
int uart_scanf(UART_HandleTypeDef *huart, const char *__format, ...);

uint32_t uart_dmarx_read(UART_HandleTypeDef *huart, void *buf, size_t len);
uint8_t uart_dmarx_resize_fifo(UART_HandleTypeDef *huart, uint32_t buf_size,
                               uint32_t fifo_size);
uint32_t uart_dmarx_get_buf_size(UART_HandleTypeDef *huart);
uint32_t uart_dmarx_get_fifo_size(UART_HandleTypeDef *huart);

uint32_t uart_dmatx_write(UART_HandleTypeDef *huart, const void *data,
                          size_t len);
uint32_t uart_dmatx_send(UART_HandleTypeDef *huart);
uint8_t uart_dmatx_resize_buf(UART_HandleTypeDef *huart, uint32_t size);
uint32_t uart_damtx_get_buf_szie(UART_HandleTypeDef *huart);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __UART_STM32F4xx_H */
