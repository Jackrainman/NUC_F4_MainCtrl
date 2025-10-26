/**
 * @file    I2C_STM32F4xx.h
 * @author  Deadline039
 * @brief   Chip Support Package of I2C on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 * @note    Generate Automatically.
 */

#ifndef __I2C_STM32F4xx_H
#define __I2C_STM32F4xx_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*****************************************************************************
 * @defgroup I2C Public Marco. 
 * @{
 */

#define I2C_INIT_OK         0
#define I2C_INIT_FAIL       1
#define I2C_INIT_DMA_FAIL   2
#define I2C_INITED          3

#define I2C_DEINIT_OK       0
#define I2C_DEINIT_FAIL     1
#define I2C_DEINIT_DMA_FAIL 2
#define I2C_NO_INIT         3

/**
 * @}
 */
 
/* clang-format off */


/*****************************************************************************
 * @defgroup I2C1 Functions
 * @{
 */

#if I2C1_ENABLE

#if (I2C1_SCL_ID == 0)
#  define I2C1_SCL_GPIO_AF GPIO_AF4_I2C1
#elif (I2C1_SCL_ID == 1)
#  define I2C1_SCL_GPIO_AF GPIO_AF4_I2C1
#elif (I2C1_SCL_ID == 2)
#  define I2C1_SCL_GPIO_AF GPIO_AF4_I2C1
#  if !(defined(STM32F446xx))
#    error "PC6 only can be configured as I2C1 SCL on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#elif (I2C1_SCL_ID == 3)
#  define I2C1_SCL_GPIO_AF GPIO_AF4_I2C1
#  if !(defined(STM32F446xx))
#    error "PD12 only can be configured as I2C1 SCL on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#elif (I2C1_SCL_ID == 4)
#  define I2C1_SCL_GPIO_AF GPIO_AF4_I2C1
#  if !(defined(STM32F446xx))
#    error "PD14 only can be configured as I2C1 SCL on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#elif (I2C1_SCL_ID == 5)
#  define I2C1_SCL_GPIO_AF GPIO_AF4_I2C1
#  if !(defined(STM32F446xx))
#    error "PF14 only can be configured as I2C1 SCL on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#endif  /* I2C1_SCL_ID */

#if (I2C1_SDA_ID == 0)
#  define I2C1_SDA_GPIO_AF GPIO_AF4_I2C1
#elif (I2C1_SDA_ID == 1)
#  define I2C1_SDA_GPIO_AF GPIO_AF4_I2C1
#elif (I2C1_SDA_ID == 2)
#  define I2C1_SDA_GPIO_AF GPIO_AF4_I2C1
#  if !(defined(STM32F446xx))
#    error "PC7 only can be configured as I2C1 SDA on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#elif (I2C1_SDA_ID == 3)
#  define I2C1_SDA_GPIO_AF GPIO_AF4_I2C1
#  if !(defined(STM32F446xx))
#    error "PD13 only can be configured as I2C1 SDA on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#elif (I2C1_SDA_ID == 4)
#  define I2C1_SDA_GPIO_AF GPIO_AF4_I2C1
#  if !(defined(STM32F446xx))
#    error "PD15 only can be configured as I2C1 SDA on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#elif (I2C1_SDA_ID == 5)
#  define I2C1_SDA_GPIO_AF GPIO_AF4_I2C1
#  if !(defined(STM32F446xx))
#    error "PF15 only can be configured as I2C1 SDA on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#endif  /* I2C1_SDA_ID */

extern I2C_HandleTypeDef i2c1_handle;

uint8_t i2c1_init(uint32_t clock_speed, uint32_t address,
                  uint32_t address_mode);
uint8_t i2c1_deinit(void); 

#  if I2C1_RX_DMA
#    if (I2C1_RX_DMA_NUMBER == 1) && (I2C1_RX_DMA_STREAM == 2) && (I2C1_RX_DMA_CHANNEL == 2)

#     if !(defined(STM32F446xx))
#       error "This I2C1 DMA RX configuration only valid on STM32F446xx! "
#     endif  /* !(defined(STM32F446xx)) */
#    endif /* (I2C1_RX_DMA_NUMBER == 1) && (I2C1_RX_DMA_STREAM == 2) && (I2C1_RX_DMA_CHANNEL == 2) */

#    define I2C1_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(I2C1_RX_DMA_NUMBER, I2C1_RX_DMA_STREAM)
#    define I2C1_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(I2C1_RX_DMA_NUMBER, I2C1_RX_DMA_STREAM)
#  endif /* I2C1_RX_DMA */

#  if I2C1_TX_DMA
#    if (I2C1_TX_DMA_NUMBER == 1) && (I2C1_TX_DMA_STREAM == 1) && (I2C1_TX_DMA_CHANNEL == 0)

#     if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#       error "This I2C1 DMA TX configuration only valid on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#     endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#    endif /* (I2C1_TX_DMA_NUMBER == 1) && (I2C1_TX_DMA_STREAM == 1) && (I2C1_TX_DMA_CHANNEL == 0) */

#    if (I2C1_TX_DMA_NUMBER == 1) && (I2C1_TX_DMA_STREAM == 5) && (I2C1_TX_DMA_CHANNEL == 2)

#     if !(defined(STM32F446xx))
#       error "This I2C1 DMA TX configuration only valid on STM32F446xx! "
#     endif  /* !(defined(STM32F446xx)) */
#    endif /* (I2C1_TX_DMA_NUMBER == 1) && (I2C1_TX_DMA_STREAM == 5) && (I2C1_TX_DMA_CHANNEL == 2) */

#    define I2C1_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(I2C1_TX_DMA_NUMBER, I2C1_TX_DMA_STREAM)
#    define I2C1_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(I2C1_TX_DMA_NUMBER, I2C1_TX_DMA_STREAM)
#  endif /* I2C1_TX_DMA */

#endif /* I2C1_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup I2C2 Functions
 * @{
 */

#if I2C2_ENABLE

#if (I2C2_SCL_ID == 0)
#  define I2C2_SCL_GPIO_AF GPIO_AF4_I2C2
#elif (I2C2_SCL_ID == 1)
#  define I2C2_SCL_GPIO_AF GPIO_AF4_I2C2
#  if (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PF1 can not be configured as I2C2 SCL on STM32F411xE, STM32F401xE, STM32F401xC, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (I2C2_SCL_ID == 2)
#  define I2C2_SCL_GPIO_AF GPIO_AF4_I2C2
#  if !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PH4 only can be configured as I2C2 SCL on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#endif  /* I2C2_SCL_ID */

#if (I2C2_SDA_ID == 0)
#  define I2C2_SDA_GPIO_AF GPIO_AF9_I2C2
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PB3 can not be configured as I2C2 SDA on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (I2C2_SDA_ID == 1)
#  define I2C2_SDA_GPIO_AF GPIO_AF4_I2C2
#  if (defined(STM32F401xE))
#    error "PB11 can not be configured as I2C2 SDA on STM32F401xE! "
#  endif  /* (defined(STM32F401xE)) */
#elif (I2C2_SDA_ID == 2)
#  define I2C2_SDA_GPIO_AF GPIO_AF9_I2C2
#  if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PB9 only can be configured as I2C2 SDA on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (I2C2_SDA_ID == 3)
#  define I2C2_SDA_GPIO_AF GPIO_AF4_I2C2
#  if (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PF0 can not be configured as I2C2 SDA on STM32F411xE, STM32F401xE, STM32F401xC, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (I2C2_SDA_ID == 4)
#  define I2C2_SDA_GPIO_AF GPIO_AF4_I2C2
#  if !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PH5 only can be configured as I2C2 SDA on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (I2C2_SDA_ID == 5)
#  define I2C2_SDA_GPIO_AF GPIO_AF4_I2C2
#  if !(defined(STM32F446xx))
#    error "PC12 only can be configured as I2C2 SDA on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#endif  /* I2C2_SDA_ID */

extern I2C_HandleTypeDef i2c2_handle;

uint8_t i2c2_init(uint32_t clock_speed, uint32_t address,
                  uint32_t address_mode);
uint8_t i2c2_deinit(void); 

#  if I2C2_RX_DMA
#    define I2C2_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(I2C2_RX_DMA_NUMBER, I2C2_RX_DMA_STREAM)
#    define I2C2_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(I2C2_RX_DMA_NUMBER, I2C2_RX_DMA_STREAM)
#  endif /* I2C2_RX_DMA */

#  if I2C2_TX_DMA
#    define I2C2_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(I2C2_TX_DMA_NUMBER, I2C2_TX_DMA_STREAM)
#    define I2C2_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(I2C2_TX_DMA_NUMBER, I2C2_TX_DMA_STREAM)
#  endif /* I2C2_TX_DMA */

#endif /* I2C2_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup I2C3 Functions
 * @{
 */

#if I2C3_ENABLE

#if (I2C3_SCL_ID == 0)
#  define I2C3_SCL_GPIO_AF GPIO_AF4_I2C3
#elif (I2C3_SCL_ID == 1)
#  define I2C3_SCL_GPIO_AF GPIO_AF4_I2C3
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PH7 can not be configured as I2C3 SCL on STM32F411xE, STM32F423xx, STM32F446xx, STM32F401xE, STM32F412Rx, STM32F412Cx, STM32F401xC, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* I2C3_SCL_ID */

#if (I2C3_SDA_ID == 0)
#  define I2C3_SDA_GPIO_AF GPIO_AF9_I2C3
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PB4 can not be configured as I2C3 SDA on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (I2C3_SDA_ID == 1)
#  define I2C3_SDA_GPIO_AF GPIO_AF4_I2C3
#elif (I2C3_SDA_ID == 2)
#  define I2C3_SDA_GPIO_AF GPIO_AF9_I2C3
#  if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PB8 only can be configured as I2C3 SDA on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (I2C3_SDA_ID == 3)
#  define I2C3_SDA_GPIO_AF GPIO_AF4_I2C3
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PH8 can not be configured as I2C3 SDA on STM32F411xE, STM32F423xx, STM32F446xx, STM32F401xE, STM32F412Rx, STM32F412Cx, STM32F401xC, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F401xE) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F401xC) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* I2C3_SDA_ID */

extern I2C_HandleTypeDef i2c3_handle;

uint8_t i2c3_init(uint32_t clock_speed, uint32_t address,
                  uint32_t address_mode);
uint8_t i2c3_deinit(void); 

#  if I2C3_RX_DMA
#    if (I2C3_RX_DMA_NUMBER == 1) && (I2C3_RX_DMA_STREAM == 1) && (I2C3_RX_DMA_CHANNEL == 1)

#     if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#       error "This I2C3 DMA RX configuration invalid on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#     endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#    endif /* (I2C3_RX_DMA_NUMBER == 1) && (I2C3_RX_DMA_STREAM == 1) && (I2C3_RX_DMA_CHANNEL == 1) */

#    define I2C3_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(I2C3_RX_DMA_NUMBER, I2C3_RX_DMA_STREAM)
#    define I2C3_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(I2C3_RX_DMA_NUMBER, I2C3_RX_DMA_STREAM)
#  endif /* I2C3_RX_DMA */

#  if I2C3_TX_DMA
#    if (I2C3_TX_DMA_NUMBER == 1) && (I2C3_TX_DMA_STREAM == 5) && (I2C3_TX_DMA_CHANNEL == 6)

#     if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F446xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#       error "This I2C3 DMA TX configuration invalid on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F446xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#     endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F446xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#    endif /* (I2C3_TX_DMA_NUMBER == 1) && (I2C3_TX_DMA_STREAM == 5) && (I2C3_TX_DMA_CHANNEL == 6) */

#    define I2C3_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(I2C3_TX_DMA_NUMBER, I2C3_TX_DMA_STREAM)
#    define I2C3_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(I2C3_TX_DMA_NUMBER, I2C3_TX_DMA_STREAM)
#  endif /* I2C3_TX_DMA */

#endif /* I2C3_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup I2C4 Functions
 * @{
 */

#if I2C4_ENABLE

#if (I2C4_SCL_ID == 0)
#  define I2C4_SCL_GPIO_AF GPIO_AF4_I2C4
#elif (I2C4_SCL_ID == 1)
#  define I2C4_SCL_GPIO_AF GPIO_AF9_I2C4
#elif (I2C4_SCL_ID == 2)
#  define I2C4_SCL_GPIO_AF GPIO_AF4_I2C4
#elif (I2C4_SCL_ID == 3)
#  define I2C4_SCL_GPIO_AF GPIO_AF4_I2C4
#endif  /* I2C4_SCL_ID */

#if (I2C4_SDA_ID == 0)
#  define I2C4_SDA_GPIO_AF GPIO_AF4_I2C4
#elif (I2C4_SDA_ID == 1)
#  define I2C4_SDA_GPIO_AF GPIO_AF4_I2C4
#elif (I2C4_SDA_ID == 2)
#  define I2C4_SDA_GPIO_AF GPIO_AF4_I2C4
#elif (I2C4_SDA_ID == 3)
#  define I2C4_SDA_GPIO_AF GPIO_AF4_I2C4
#endif  /* I2C4_SDA_ID */

extern I2C_HandleTypeDef i2c4_handle;

uint8_t i2c4_init(uint32_t clock_speed, uint32_t address,
                  uint32_t address_mode);
uint8_t i2c4_deinit(void); 

#  if I2C4_RX_DMA
#    define I2C4_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(I2C4_RX_DMA_NUMBER, I2C4_RX_DMA_STREAM)
#    define I2C4_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(I2C4_RX_DMA_NUMBER, I2C4_RX_DMA_STREAM)
#  endif /* I2C4_RX_DMA */

#  if I2C4_TX_DMA
#    define I2C4_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(I2C4_TX_DMA_NUMBER, I2C4_TX_DMA_STREAM)
#    define I2C4_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(I2C4_TX_DMA_NUMBER, I2C4_TX_DMA_STREAM)
#  endif /* I2C4_TX_DMA */

#endif /* I2C4_ENABLE */

/**
 * @}
 */

    
/* clang-format on */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __I2C_STM32F4xx_H */  
