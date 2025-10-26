/**
 * @file    SPI_STM32F4xx.h
 * @author  Deadline039
 * @brief   Chip Support Package of SPI and QSPI on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 * @note    Generate Automatically.
 */

#ifndef __SPI_STM32F4xx_H
#define __SPI_STM32F4xx_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*****************************************************************************
 * @defgroup SPI Public Marcos and Types. 
 * @{
 */

#define SPI_INIT_OK         0
#define SPI_INIT_FAIL       1
#define SPI_INIT_DMA_FAIL   2
#define SPI_INITED          3

#define SPI_DEINIT_OK       0
#define SPI_DEINIT_FAIL     1
#define SPI_DEINIT_DMA_FAIL 2
#define SPI_NO_INIT         3

#define SPI_RW_TIMEOUT      1000

/**
 * @brief SPI Clock Mode select.
 */
typedef enum {
    SPI_CLK_MODE0, /*!< Mode 0: CPOL=0; CPHA=0 */
    SPI_CLK_MODE1, /*!< Mode 1: CPOL=0; CPHA=1 */
    SPI_CLK_MODE2, /*!< Mode 2: CPOL=1; CPHA=0 */
    SPI_CLK_MODE3  /*!< Mode 3: CPOL=1; CPHA=1 */
} spi_clk_mode_t;

/**
 * @}
 */

/* clang-format off */


/*****************************************************************************
 * @defgroup SPI1 Functions
 * @{
 */

#if SPI1_ENABLE

#if (SPI1_SCK_ID == 0)
#  define SPI1_SCK_GPIO_AF GPIO_AF5_SPI1
#elif (SPI1_SCK_ID == 1)
#  define SPI1_SCK_GPIO_AF GPIO_AF5_SPI1
#endif  /* SPI1_SCK_ID */

#if (SPI1_MISO_ID == 0)
#elif (SPI1_MISO_ID == 1)
#  define SPI1_MISO_GPIO_AF GPIO_AF5_SPI1
#elif (SPI1_MISO_ID == 2)
#  define SPI1_MISO_GPIO_AF GPIO_AF5_SPI1
#endif  /* SPI1_MISO_ID */

#if (SPI1_MOSI_ID == 0)
#elif (SPI1_MOSI_ID == 1)
#  define SPI1_MOSI_GPIO_AF GPIO_AF5_SPI1
#elif (SPI1_MOSI_ID == 2)
#  define SPI1_MOSI_GPIO_AF GPIO_AF5_SPI1
#endif  /* SPI1_MOSI_ID */

#if (SPI1_NSS_ID == 0)
#elif (SPI1_NSS_ID == 1)
#  define SPI1_NSS_GPIO_AF GPIO_AF5_SPI1
#elif (SPI1_NSS_ID == 2)
#  define SPI1_NSS_GPIO_AF GPIO_AF5_SPI1
#endif  /* SPI1_NSS_ID */

extern SPI_HandleTypeDef spi1_handle;

uint8_t spi1_init(uint32_t mode, spi_clk_mode_t clk_mode, uint32_t data_size,
                  uint32_t first_bit);
uint8_t spi1_deinit(void); 

#  if SPI1_RX_DMA
#    define SPI1_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI1_RX_DMA_NUMBER, SPI1_RX_DMA_STREAM)
#    define SPI1_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI1_RX_DMA_NUMBER, SPI1_RX_DMA_STREAM)
#  endif /* SPI1_RX_DMA */

#  if SPI1_TX_DMA
#    if (SPI1_TX_DMA_NUMBER == 2) && (SPI1_TX_DMA_STREAM == 2) && (SPI1_TX_DMA_CHANNEL == 2)

#     if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#       error "This SPI1 DMA TX configuration only valid on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#     endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#    endif /* (SPI1_TX_DMA_NUMBER == 2) && (SPI1_TX_DMA_STREAM == 2) && (SPI1_TX_DMA_CHANNEL == 2) */

#    define SPI1_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI1_TX_DMA_NUMBER, SPI1_TX_DMA_STREAM)
#    define SPI1_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI1_TX_DMA_NUMBER, SPI1_TX_DMA_STREAM)
#  endif /* SPI1_TX_DMA */

#endif /* SPI1_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup SPI2 Functions
 * @{
 */

#if SPI2_ENABLE

#if (SPI2_SCK_ID == 0)
#  define SPI2_SCK_GPIO_AF GPIO_AF5_SPI2
#elif (SPI2_SCK_ID == 1)
#  define SPI2_SCK_GPIO_AF GPIO_AF5_SPI2
#elif (SPI2_SCK_ID == 2)
#  define SPI2_SCK_GPIO_AF GPIO_AF5_SPI2
#  if (defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx))
#    error "PD3 can not be configured as SPI2 SCK on STM32F410Rx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F417xx, STM32F410Cx, STM32F410Tx! "
#  endif  /* (defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx)) */
#elif (SPI2_SCK_ID == 3)
#  define SPI2_SCK_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PC7 only can be configured as SPI2 SCK on STM32F411xE, STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI2_SCK_ID == 4)
#  define SPI2_SCK_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F423xx) || defined(STM32F479xx) || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F413xx))
#    error "PA9 only can be configured as SPI2 SCK on STM32F423xx, STM32F479xx, STM32F446xx, STM32F469xx, STM32F413xx! "
#  endif  /* !(defined(STM32F423xx) || defined(STM32F479xx) || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F413xx)) */
#elif (SPI2_SCK_ID == 5)
#  define SPI2_SCK_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PI1 only can be configured as SPI2 SCK on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#endif  /* SPI2_SCK_ID */

#if (SPI2_MISO_ID == 0)
#elif (SPI2_MISO_ID == 1)
#  define SPI2_MISO_GPIO_AF GPIO_AF5_SPI2
#elif (SPI2_MISO_ID == 2)
#  define SPI2_MISO_GPIO_AF GPIO_AF5_SPI2
#elif (SPI2_MISO_ID == 3)
#  define SPI2_MISO_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F423xx) || defined(STM32F413xx))
#    error "PA12 only can be configured as SPI2 MISO on STM32F423xx, STM32F413xx! "
#  endif  /* !(defined(STM32F423xx) || defined(STM32F413xx)) */
#elif (SPI2_MISO_ID == 4)
#  define SPI2_MISO_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PI2 only can be configured as SPI2 MISO on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#endif  /* SPI2_MISO_ID */

#if (SPI2_MOSI_ID == 0)
#elif (SPI2_MOSI_ID == 1)
#  define SPI2_MOSI_GPIO_AF GPIO_AF5_SPI2
#elif (SPI2_MOSI_ID == 2)
#  define SPI2_MOSI_GPIO_AF GPIO_AF5_SPI2
#elif (SPI2_MOSI_ID == 3)
#  define SPI2_MOSI_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F423xx) || defined(STM32F413xx))
#    error "PA10 only can be configured as SPI2 MOSI on STM32F423xx, STM32F413xx! "
#  endif  /* !(defined(STM32F423xx) || defined(STM32F413xx)) */
#elif (SPI2_MOSI_ID == 4)
#  define SPI2_MOSI_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PI3 only can be configured as SPI2 MOSI on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (SPI2_MOSI_ID == 5)
#  define SPI2_MOSI_GPIO_AF GPIO_AF7_SPI2
#  if !(defined(STM32F446xx) || defined(STM32F479xx) || defined(STM32F469xx))
#    error "PC1 only can be configured as SPI2 MOSI on STM32F446xx, STM32F479xx, STM32F469xx! "
#  endif  /* !(defined(STM32F446xx) || defined(STM32F479xx) || defined(STM32F469xx)) */
#endif  /* SPI2_MOSI_ID */

#if (SPI2_NSS_ID == 0)
#elif (SPI2_NSS_ID == 1)
#  define SPI2_NSS_GPIO_AF GPIO_AF5_SPI2
#elif (SPI2_NSS_ID == 2)
#  define SPI2_NSS_GPIO_AF GPIO_AF5_SPI2
#elif (SPI2_NSS_ID == 3)
#  define SPI2_NSS_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F423xx) || defined(STM32F413xx))
#    error "PA11 only can be configured as SPI2 NSS on STM32F423xx, STM32F413xx! "
#  endif  /* !(defined(STM32F423xx) || defined(STM32F413xx)) */
#elif (SPI2_NSS_ID == 4)
#  define SPI2_NSS_GPIO_AF GPIO_AF5_SPI2
#  if !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PI0 only can be configured as SPI2 NSS on STM32F479xx, STM32F437xx, STM32F415xx, STM32F405xx, STM32F407xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* !(defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (SPI2_NSS_ID == 5)
#  define SPI2_NSS_GPIO_AF GPIO_AF7_SPI2
#  if !(defined(STM32F446xx))
#    error "PB4 only can be configured as SPI2 NSS on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#elif (SPI2_NSS_ID == 6)
#  define SPI2_NSS_GPIO_AF GPIO_AF7_SPI2
#  if !(defined(STM32F446xx))
#    error "PD1 only can be configured as SPI2 NSS on STM32F446xx! "
#  endif  /* !(defined(STM32F446xx)) */
#endif  /* SPI2_NSS_ID */

extern SPI_HandleTypeDef spi2_handle;

uint8_t spi2_init(uint32_t mode, spi_clk_mode_t clk_mode, uint32_t data_size,
                  uint32_t first_bit);
uint8_t spi2_deinit(void); 

#  if SPI2_RX_DMA
#    define SPI2_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI2_RX_DMA_NUMBER, SPI2_RX_DMA_STREAM)
#    define SPI2_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI2_RX_DMA_NUMBER, SPI2_RX_DMA_STREAM)
#  endif /* SPI2_RX_DMA */

#  if SPI2_TX_DMA
#    define SPI2_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI2_TX_DMA_NUMBER, SPI2_TX_DMA_STREAM)
#    define SPI2_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI2_TX_DMA_NUMBER, SPI2_TX_DMA_STREAM)
#  endif /* SPI2_TX_DMA */

#endif /* SPI2_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup SPI3 Functions
 * @{
 */

#if SPI3_ENABLE

#if (SPI3_SCK_ID == 0)
#  define SPI3_SCK_GPIO_AF GPIO_AF6_SPI3
#elif (SPI3_SCK_ID == 1)
#  define SPI3_SCK_GPIO_AF GPIO_AF6_SPI3
#elif (SPI3_SCK_ID == 2)
#  define SPI3_SCK_GPIO_AF GPIO_AF7_SPI3
#  if !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PB12 only can be configured as SPI3 SCK on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* SPI3_SCK_ID */

#if (SPI3_MISO_ID == 0)
#elif (SPI3_MISO_ID == 1)
#  define SPI3_MISO_GPIO_AF GPIO_AF6_SPI3
#elif (SPI3_MISO_ID == 2)
#  define SPI3_MISO_GPIO_AF GPIO_AF6_SPI3
#endif  /* SPI3_MISO_ID */

#if (SPI3_MOSI_ID == 0)
#elif (SPI3_MOSI_ID == 1)
#  define SPI3_MOSI_GPIO_AF GPIO_AF6_SPI3
#elif (SPI3_MOSI_ID == 2)
#  define SPI3_MOSI_GPIO_AF GPIO_AF6_SPI3
#elif (SPI3_MOSI_ID == 3)
#  define SPI3_MOSI_GPIO_AF GPIO_AF5_SPI3
#  if (defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F405xx) || defined(STM32F415xx))
#    error "PD6 can not be configured as SPI3 MOSI on STM32F407xx, STM32F417xx, STM32F405xx, STM32F415xx! "
#  endif  /* (defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F405xx) || defined(STM32F415xx)) */
#elif (SPI3_MOSI_ID == 4)
#  define SPI3_MOSI_GPIO_AF GPIO_AF7_SPI3
#  if !(defined(STM32F410Tx) || defined(STM32F446xx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PB0 only can be configured as SPI3 MOSI on STM32F410Tx, STM32F446xx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F410Tx) || defined(STM32F446xx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI3_MOSI_ID == 5)
#  define SPI3_MOSI_GPIO_AF GPIO_AF7_SPI3
#  if !(defined(STM32F410Tx) || defined(STM32F446xx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PB2 only can be configured as SPI3 MOSI on STM32F410Tx, STM32F446xx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F410Tx) || defined(STM32F446xx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI3_MOSI_ID == 6)
#  define SPI3_MOSI_GPIO_AF GPIO_AF5_SPI3
#  if !(defined(STM32F410Tx) || defined(STM32F446xx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PC1 only can be configured as SPI3 MOSI on STM32F410Tx, STM32F446xx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F410Tx) || defined(STM32F446xx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI3_MOSI_ID == 7)
#  define SPI3_MOSI_GPIO_AF GPIO_AF6_SPI3
#  if !(defined(STM32F410Tx) || defined(STM32F446xx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PD0 only can be configured as SPI3 MOSI on STM32F410Tx, STM32F446xx, STM32F410Cx, STM32F410Rx! "
#  endif  /* !(defined(STM32F410Tx) || defined(STM32F446xx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* SPI3_MOSI_ID */

#if (SPI3_NSS_ID == 0)
#elif (SPI3_NSS_ID == 1)
#  define SPI3_NSS_GPIO_AF GPIO_AF6_SPI3
#elif (SPI3_NSS_ID == 2)
#  define SPI3_NSS_GPIO_AF GPIO_AF6_SPI3
#endif  /* SPI3_NSS_ID */

extern SPI_HandleTypeDef spi3_handle;

uint8_t spi3_init(uint32_t mode, spi_clk_mode_t clk_mode, uint32_t data_size,
                  uint32_t first_bit);
uint8_t spi3_deinit(void); 

#  if SPI3_RX_DMA
#    define SPI3_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI3_RX_DMA_NUMBER, SPI3_RX_DMA_STREAM)
#    define SPI3_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI3_RX_DMA_NUMBER, SPI3_RX_DMA_STREAM)
#  endif /* SPI3_RX_DMA */

#  if SPI3_TX_DMA
#    define SPI3_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI3_TX_DMA_NUMBER, SPI3_TX_DMA_STREAM)
#    define SPI3_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI3_TX_DMA_NUMBER, SPI3_TX_DMA_STREAM)
#  endif /* SPI3_TX_DMA */

#endif /* SPI3_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup SPI4 Functions
 * @{
 */

#if SPI4_ENABLE

#if (SPI4_SCK_ID == 0)
#  define SPI4_SCK_GPIO_AF GPIO_AF5_SPI4
#elif (SPI4_SCK_ID == 1)
#  define SPI4_SCK_GPIO_AF GPIO_AF5_SPI4
#elif (SPI4_SCK_ID == 2)
#  define SPI4_SCK_GPIO_AF GPIO_AF6_SPI4
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC))
#    error "PB13 can not be configured as SPI4 SCK on STM32F479xx, STM32F437xx, STM32F446xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F401xE, STM32F427xx, STM32F401xC! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC)) */
#elif (SPI4_SCK_ID == 3)
#  define SPI4_SCK_GPIO_AF GPIO_AF6_SPI4
#  if !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx))
#    error "PG11 only can be configured as SPI4 SCK on STM32F410Rx, STM32F415xx, STM32F405xx, STM32F446xx, STM32F407xx, STM32F417xx, STM32F410Cx, STM32F410Tx! "
#  endif  /* !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx)) */
#endif  /* SPI4_SCK_ID */

#if (SPI4_MISO_ID == 0)
#elif (SPI4_MISO_ID == 1)
#  define SPI4_MISO_GPIO_AF GPIO_AF5_SPI4
#elif (SPI4_MISO_ID == 2)
#  define SPI4_MISO_GPIO_AF GPIO_AF5_SPI4
#elif (SPI4_MISO_ID == 3)
#  define SPI4_MISO_GPIO_AF GPIO_AF6_SPI4
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC))
#    error "PA11 can not be configured as SPI4 MISO on STM32F479xx, STM32F437xx, STM32F446xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F401xE, STM32F427xx, STM32F401xC! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC)) */
#elif (SPI4_MISO_ID == 4)
#  define SPI4_MISO_GPIO_AF GPIO_AF5_SPI4
#  if !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx))
#    error "PD0 only can be configured as SPI4 MISO on STM32F410Rx, STM32F415xx, STM32F405xx, STM32F446xx, STM32F407xx, STM32F417xx, STM32F410Cx, STM32F410Tx! "
#  endif  /* !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx)) */
#elif (SPI4_MISO_ID == 5)
#  define SPI4_MISO_GPIO_AF GPIO_AF6_SPI4
#  if !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx))
#    error "PG12 only can be configured as SPI4 MISO on STM32F410Rx, STM32F415xx, STM32F405xx, STM32F446xx, STM32F407xx, STM32F417xx, STM32F410Cx, STM32F410Tx! "
#  endif  /* !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx)) */
#endif  /* SPI4_MISO_ID */

#if (SPI4_MOSI_ID == 0)
#elif (SPI4_MOSI_ID == 1)
#  define SPI4_MOSI_GPIO_AF GPIO_AF5_SPI4
#elif (SPI4_MOSI_ID == 2)
#  define SPI4_MOSI_GPIO_AF GPIO_AF5_SPI4
#elif (SPI4_MOSI_ID == 3)
#  define SPI4_MOSI_GPIO_AF GPIO_AF5_SPI4
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC))
#    error "PA1 can not be configured as SPI4 MOSI on STM32F479xx, STM32F437xx, STM32F446xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F401xE, STM32F427xx, STM32F401xC! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC)) */
#elif (SPI4_MOSI_ID == 4)
#  define SPI4_MOSI_GPIO_AF GPIO_AF6_SPI4
#  if !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx))
#    error "PG13 only can be configured as SPI4 MOSI on STM32F410Rx, STM32F415xx, STM32F405xx, STM32F446xx, STM32F407xx, STM32F417xx, STM32F410Cx, STM32F410Tx! "
#  endif  /* !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx)) */
#endif  /* SPI4_MOSI_ID */

#if (SPI4_NSS_ID == 0)
#elif (SPI4_NSS_ID == 1)
#  define SPI4_NSS_GPIO_AF GPIO_AF5_SPI4
#elif (SPI4_NSS_ID == 2)
#  define SPI4_NSS_GPIO_AF GPIO_AF5_SPI4
#elif (SPI4_NSS_ID == 3)
#  define SPI4_NSS_GPIO_AF GPIO_AF6_SPI4
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC))
#    error "PB12 can not be configured as SPI4 NSS on STM32F479xx, STM32F437xx, STM32F446xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F401xE, STM32F427xx, STM32F401xC! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC)) */
#elif (SPI4_NSS_ID == 4)
#  define SPI4_NSS_GPIO_AF GPIO_AF6_SPI4
#  if !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx))
#    error "PG14 only can be configured as SPI4 NSS on STM32F410Rx, STM32F415xx, STM32F405xx, STM32F446xx, STM32F407xx, STM32F417xx, STM32F410Cx, STM32F410Tx! "
#  endif  /* !(defined(STM32F410Rx) || defined(STM32F415xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F410Cx) || defined(STM32F410Tx)) */
#endif  /* SPI4_NSS_ID */

extern SPI_HandleTypeDef spi4_handle;

uint8_t spi4_init(uint32_t mode, spi_clk_mode_t clk_mode, uint32_t data_size,
                  uint32_t first_bit);
uint8_t spi4_deinit(void); 

#  if SPI4_RX_DMA
#    if (SPI4_RX_DMA_NUMBER == 2) && (SPI4_RX_DMA_STREAM == 4) && (SPI4_RX_DMA_CHANNEL == 4)

#     if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC))
#       error "This SPI4 DMA RX configuration invalid on STM32F479xx, STM32F437xx, STM32F446xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F401xE, STM32F427xx, STM32F401xC! "
#     endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F446xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F401xE) || defined(STM32F427xx) || defined(STM32F401xC)) */
#    endif /* (SPI4_RX_DMA_NUMBER == 2) && (SPI4_RX_DMA_STREAM == 4) && (SPI4_RX_DMA_CHANNEL == 4) */

#    define SPI4_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI4_RX_DMA_NUMBER, SPI4_RX_DMA_STREAM)
#    define SPI4_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI4_RX_DMA_NUMBER, SPI4_RX_DMA_STREAM)
#  endif /* SPI4_RX_DMA */

#  if SPI4_TX_DMA
#    define SPI4_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI4_TX_DMA_NUMBER, SPI4_TX_DMA_STREAM)
#    define SPI4_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI4_TX_DMA_NUMBER, SPI4_TX_DMA_STREAM)
#  endif /* SPI4_TX_DMA */

#endif /* SPI4_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup SPI5 Functions
 * @{
 */

#if SPI5_ENABLE

#if (SPI5_SCK_ID == 0)
#  define SPI5_SCK_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PB0 can not be configured as SPI5 SCK on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (SPI5_SCK_ID == 1)
#  define SPI5_SCK_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE2 can not be configured as SPI5 SCK on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_SCK_ID == 2)
#  define SPI5_SCK_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE12 can not be configured as SPI5 SCK on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_SCK_ID == 3)
#  define SPI5_SCK_GPIO_AF GPIO_AF5_SPI5
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PF7 can not be configured as SPI5 SCK on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_SCK_ID == 4)
#  define SPI5_SCK_GPIO_AF GPIO_AF5_SPI5
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PH6 can not be configured as SPI5 SCK on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* SPI5_SCK_ID */

#if (SPI5_MISO_ID == 0)
#elif (SPI5_MISO_ID == 1)
#  define SPI5_MISO_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PA12 can not be configured as SPI5 MISO on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (SPI5_MISO_ID == 2)
#  define SPI5_MISO_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE5 can not be configured as SPI5 MISO on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_MISO_ID == 3)
#  define SPI5_MISO_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE13 can not be configured as SPI5 MISO on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_MISO_ID == 4)
#  define SPI5_MISO_GPIO_AF GPIO_AF5_SPI5
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PF8 can not be configured as SPI5 MISO on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_MISO_ID == 5)
#  define SPI5_MISO_GPIO_AF GPIO_AF5_SPI5
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PH7 can not be configured as SPI5 MISO on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* SPI5_MISO_ID */

#if (SPI5_MOSI_ID == 0)
#elif (SPI5_MOSI_ID == 1)
#  define SPI5_MOSI_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PA10 can not be configured as SPI5 MOSI on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (SPI5_MOSI_ID == 2)
#  define SPI5_MOSI_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PB8 can not be configured as SPI5 MOSI on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (SPI5_MOSI_ID == 3)
#  define SPI5_MOSI_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE6 can not be configured as SPI5 MOSI on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_MOSI_ID == 4)
#  define SPI5_MOSI_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE14 can not be configured as SPI5 MOSI on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_MOSI_ID == 5)
#  define SPI5_MOSI_GPIO_AF GPIO_AF5_SPI5
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PF9 can not be configured as SPI5 MOSI on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_MOSI_ID == 6)
#  define SPI5_MOSI_GPIO_AF GPIO_AF5_SPI5
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PF11 can not be configured as SPI5 MOSI on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* SPI5_MOSI_ID */

#if (SPI5_NSS_ID == 0)
#elif (SPI5_NSS_ID == 1)
#  define SPI5_NSS_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#    error "PB1 can not be configured as SPI5 NSS on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#elif (SPI5_NSS_ID == 2)
#  define SPI5_NSS_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE4 can not be configured as SPI5 NSS on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_NSS_ID == 3)
#  define SPI5_NSS_GPIO_AF GPIO_AF6_SPI5
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PE11 can not be configured as SPI5 NSS on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx, STM32F410Tx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_NSS_ID == 4)
#  define SPI5_NSS_GPIO_AF GPIO_AF5_SPI5
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PF6 can not be configured as SPI5 NSS on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#elif (SPI5_NSS_ID == 5)
#  define SPI5_NSS_GPIO_AF GPIO_AF5_SPI5
#  if (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx))
#    error "PH5 can not be configured as SPI5 NSS on STM32F411xE, STM32F423xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F410Tx, STM32F413xx, STM32F412Vx, STM32F410Cx, STM32F410Rx! "
#  endif  /* (defined(STM32F411xE) || defined(STM32F423xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F410Tx) || defined(STM32F413xx) || defined(STM32F412Vx) || defined(STM32F410Cx) || defined(STM32F410Rx)) */
#endif  /* SPI5_NSS_ID */

extern SPI_HandleTypeDef spi5_handle;

uint8_t spi5_init(uint32_t mode, spi_clk_mode_t clk_mode, uint32_t data_size,
                  uint32_t first_bit);
uint8_t spi5_deinit(void); 

#  if SPI5_RX_DMA
#    define SPI5_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI5_RX_DMA_NUMBER, SPI5_RX_DMA_STREAM)
#    define SPI5_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI5_RX_DMA_NUMBER, SPI5_RX_DMA_STREAM)
#  endif /* SPI5_RX_DMA */

#  if SPI5_TX_DMA
#    if (SPI5_TX_DMA_NUMBER == 2) && (SPI5_TX_DMA_STREAM == 5) && (SPI5_TX_DMA_CHANNEL == 5)

#     if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx))
#       error "This SPI5 DMA TX configuration invalid on STM32F479xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx, STM32F427xx! "
#     endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F427xx)) */
#    endif /* (SPI5_TX_DMA_NUMBER == 2) && (SPI5_TX_DMA_STREAM == 5) && (SPI5_TX_DMA_CHANNEL == 5) */

#    define SPI5_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI5_TX_DMA_NUMBER, SPI5_TX_DMA_STREAM)
#    define SPI5_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI5_TX_DMA_NUMBER, SPI5_TX_DMA_STREAM)
#  endif /* SPI5_TX_DMA */

#endif /* SPI5_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup SPI6 Functions
 * @{
 */

#if SPI6_ENABLE

#if (SPI6_SCK_ID == 0)
#  define SPI6_SCK_GPIO_AF GPIO_AF5_SPI6
#endif  /* SPI6_SCK_ID */

#if (SPI6_MISO_ID == 0)
#elif (SPI6_MISO_ID == 1)
#  define SPI6_MISO_GPIO_AF GPIO_AF5_SPI6
#endif  /* SPI6_MISO_ID */

#if (SPI6_MOSI_ID == 0)
#elif (SPI6_MOSI_ID == 1)
#  define SPI6_MOSI_GPIO_AF GPIO_AF5_SPI6
#endif  /* SPI6_MOSI_ID */

#if (SPI6_NSS_ID == 0)
#elif (SPI6_NSS_ID == 1)
#  define SPI6_NSS_GPIO_AF GPIO_AF5_SPI6
#endif  /* SPI6_NSS_ID */

extern SPI_HandleTypeDef spi6_handle;

uint8_t spi6_init(uint32_t mode, spi_clk_mode_t clk_mode, uint32_t data_size,
                  uint32_t first_bit);
uint8_t spi6_deinit(void); 

#  if SPI6_RX_DMA
#    define SPI6_RX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI6_RX_DMA_NUMBER, SPI6_RX_DMA_STREAM)
#    define SPI6_RX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI6_RX_DMA_NUMBER, SPI6_RX_DMA_STREAM)
#  endif /* SPI6_RX_DMA */

#  if SPI6_TX_DMA
#    define SPI6_TX_DMA_IRQn                                                  \
      CSP_DMA_STREAM_IRQn(SPI6_TX_DMA_NUMBER, SPI6_TX_DMA_STREAM)
#    define SPI6_TX_DMA_IRQHandler                                             \
      CSP_DMA_STREAM_IRQ(SPI6_TX_DMA_NUMBER, SPI6_TX_DMA_STREAM)
#  endif /* SPI6_TX_DMA */

#endif /* SPI6_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup SPI Public Functions
 * @{
 */

uint8_t spi_rw_one_byte(SPI_HandleTypeDef *hspi, uint8_t byte);
uint16_t spi_rw_two_byte(SPI_HandleTypeDef *hspi, uint16_t tx_data);

uint8_t spi_change_speed(SPI_HandleTypeDef *hspi, uint8_t speed);

/**
 * @}
 */
 
/* clang-format off */

#if QUADSPI_ENABLE

#if (QUADSPI_CLK_ID == 0)
#  define QUADSPI_CLK_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PB1 can not be configured as QUADSPI CLK on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_CLK_ID == 1)
#  define QUADSPI_CLK_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F479xx) || defined(STM32F469xx))
#    error "PB2 can not be configured as QUADSPI CLK on STM32F479xx, STM32F469xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F469xx)) */
#elif (QUADSPI_CLK_ID == 2)
#  define QUADSPI_CLK_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F479xx) || defined(STM32F469xx))
#    error "PD3 can not be configured as QUADSPI CLK on STM32F479xx, STM32F469xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F469xx)) */
#elif (QUADSPI_CLK_ID == 3)
#  define QUADSPI_CLK_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PA7 can not be configured as QUADSPI CLK on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#elif (QUADSPI_CLK_ID == 4)
#  define QUADSPI_CLK_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PF10 can not be configured as QUADSPI CLK on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* QUADSPI_CLK_ID */

#if QUADSPI_BANK1_ENABLE

#if (QUADSPI_BK1_IO0_ID == 0)
#elif (QUADSPI_BK1_IO0_ID == 1)
#  define QUADSPI_BK1_IO0_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PA6 can not be configured as QUADSPI BK1_IO0 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK1_IO0_ID == 2)
#  define QUADSPI_BK1_IO0_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO0_ID == 3)
#  define QUADSPI_BK1_IO0_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO0_ID == 4)
#  define QUADSPI_BK1_IO0_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK1_IO0_ID == 5)
#  define QUADSPI_BK1_IO0_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK1_IO0_ID == 6)
#  define QUADSPI_BK1_IO0_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PH2 can not be configured as QUADSPI BK1_IO0 on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* QUADSPI_BK1_IO0_ID */

#if (QUADSPI_BK1_IO1_ID == 0)
#elif (QUADSPI_BK1_IO1_ID == 1)
#  define QUADSPI_BK1_IO1_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PA7 can not be configured as QUADSPI BK1_IO1 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK1_IO1_ID == 2)
#  define QUADSPI_BK1_IO1_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO1_ID == 3)
#  define QUADSPI_BK1_IO1_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO1_ID == 4)
#  define QUADSPI_BK1_IO1_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK1_IO1_ID == 5)
#  define QUADSPI_BK1_IO1_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK1_IO1_ID == 6)
#  define QUADSPI_BK1_IO1_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PH3 can not be configured as QUADSPI BK1_IO1 on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* QUADSPI_BK1_IO1_ID */

#if (QUADSPI_BK1_IO2_ID == 0)
#elif (QUADSPI_BK1_IO2_ID == 1)
#  define QUADSPI_BK1_IO2_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PC4 can not be configured as QUADSPI BK1_IO2 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK1_IO2_ID == 2)
#  define QUADSPI_BK1_IO2_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PC8 can not be configured as QUADSPI BK1_IO2 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK1_IO2_ID == 3)
#  define QUADSPI_BK1_IO2_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO2_ID == 4)
#  define QUADSPI_BK1_IO2_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK1_IO2_ID == 5)
#  define QUADSPI_BK1_IO2_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO2_ID == 6)
#  define QUADSPI_BK1_IO2_GPIO_AF GPIO_AF9_QUADSPI
#endif  /* QUADSPI_BK1_IO2_ID */

#if (QUADSPI_BK1_IO3_ID == 0)
#elif (QUADSPI_BK1_IO3_ID == 1)
#  define QUADSPI_BK1_IO3_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO3_ID == 2)
#  define QUADSPI_BK1_IO3_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PC5 can not be configured as QUADSPI BK1_IO3 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK1_IO3_ID == 3)
#  define QUADSPI_BK1_IO3_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO3_ID == 4)
#  define QUADSPI_BK1_IO3_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK1_IO3_ID == 5)
#  define QUADSPI_BK1_IO3_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_IO3_ID == 6)
#  define QUADSPI_BK1_IO3_GPIO_AF GPIO_AF9_QUADSPI
#endif  /* QUADSPI_BK1_IO3_ID */

#if (QUADSPI_BK1_NCS_ID == 0)
#elif (QUADSPI_BK1_NCS_ID == 1)
#  define QUADSPI_BK1_NCS_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK1_NCS_ID == 2)
#  define QUADSPI_BK1_NCS_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK1_NCS_ID == 3)
#  define QUADSPI_BK1_NCS_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F479xx) || defined(STM32F469xx))
#    error "PG6 can not be configured as QUADSPI BK1_NCS on STM32F479xx, STM32F469xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F469xx)) */
#elif (QUADSPI_BK1_NCS_ID == 4)
#  define QUADSPI_BK1_NCS_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PB10 can not be configured as QUADSPI BK1_NCS on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* QUADSPI_BK1_NCS_ID */

#endif /* QUADSPI_BANK1_ENABLE */

#if QUADSPI_BANK2_ENABLE

#if (QUADSPI_BK2_IO0_ID == 0)
#elif (QUADSPI_BK2_IO0_ID == 1)
#  define QUADSPI_BK2_IO0_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PA6 can not be configured as QUADSPI BK2_IO0 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK2_IO0_ID == 2)
#  define QUADSPI_BK2_IO0_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO0_ID == 3)
#  define QUADSPI_BK2_IO0_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO0_ID == 4)
#  define QUADSPI_BK2_IO0_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK2_IO0_ID == 5)
#  define QUADSPI_BK2_IO0_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK2_IO0_ID == 6)
#  define QUADSPI_BK2_IO0_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PH2 can not be configured as QUADSPI BK2_IO0 on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* QUADSPI_BK2_IO0_ID */

#if (QUADSPI_BK2_IO1_ID == 0)
#elif (QUADSPI_BK2_IO1_ID == 1)
#  define QUADSPI_BK2_IO1_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PA7 can not be configured as QUADSPI BK2_IO1 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK2_IO1_ID == 2)
#  define QUADSPI_BK2_IO1_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO1_ID == 3)
#  define QUADSPI_BK2_IO1_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO1_ID == 4)
#  define QUADSPI_BK2_IO1_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK2_IO1_ID == 5)
#  define QUADSPI_BK2_IO1_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK2_IO1_ID == 6)
#  define QUADSPI_BK2_IO1_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PH3 can not be configured as QUADSPI BK2_IO1 on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* QUADSPI_BK2_IO1_ID */

#if (QUADSPI_BK2_IO2_ID == 0)
#elif (QUADSPI_BK2_IO2_ID == 1)
#  define QUADSPI_BK2_IO2_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PC4 can not be configured as QUADSPI BK2_IO2 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK2_IO2_ID == 2)
#  define QUADSPI_BK2_IO2_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PC8 can not be configured as QUADSPI BK2_IO2 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK2_IO2_ID == 3)
#  define QUADSPI_BK2_IO2_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO2_ID == 4)
#  define QUADSPI_BK2_IO2_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK2_IO2_ID == 5)
#  define QUADSPI_BK2_IO2_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO2_ID == 6)
#  define QUADSPI_BK2_IO2_GPIO_AF GPIO_AF9_QUADSPI
#endif  /* QUADSPI_BK2_IO2_ID */

#if (QUADSPI_BK2_IO3_ID == 0)
#elif (QUADSPI_BK2_IO3_ID == 1)
#  define QUADSPI_BK2_IO3_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO3_ID == 2)
#  define QUADSPI_BK2_IO3_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx))
#    error "PC5 can not be configured as QUADSPI BK2_IO3 on STM32F469xx, STM32F479xx, STM32F446xx! "
#  endif  /* (defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F446xx)) */
#elif (QUADSPI_BK2_IO3_ID == 3)
#  define QUADSPI_BK2_IO3_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO3_ID == 4)
#  define QUADSPI_BK2_IO3_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK2_IO3_ID == 5)
#  define QUADSPI_BK2_IO3_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_IO3_ID == 6)
#  define QUADSPI_BK2_IO3_GPIO_AF GPIO_AF9_QUADSPI
#endif  /* QUADSPI_BK2_IO3_ID */

#if (QUADSPI_BK2_NCS_ID == 0)
#elif (QUADSPI_BK2_NCS_ID == 1)
#  define QUADSPI_BK2_NCS_GPIO_AF GPIO_AF10_QUADSPI
#elif (QUADSPI_BK2_NCS_ID == 2)
#  define QUADSPI_BK2_NCS_GPIO_AF GPIO_AF9_QUADSPI
#elif (QUADSPI_BK2_NCS_ID == 3)
#  define QUADSPI_BK2_NCS_GPIO_AF GPIO_AF10_QUADSPI
#  if (defined(STM32F479xx) || defined(STM32F469xx))
#    error "PG6 can not be configured as QUADSPI BK2_NCS on STM32F479xx, STM32F469xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F469xx)) */
#elif (QUADSPI_BK2_NCS_ID == 4)
#  define QUADSPI_BK2_NCS_GPIO_AF GPIO_AF9_QUADSPI
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PB10 can not be configured as QUADSPI BK2_NCS on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* QUADSPI_BK2_NCS_ID */

#endif /* QUADSPI_BANK2_ENABLE */

extern QSPI_HandleTypeDef qspi_handle;

uint8_t qspi_init(uint32_t clock_mode, uint32_t flash_size);

#  if QUADSPI_DMA
#    define QUADSPI_DMA_IRQn                                                  \
      CSP_DMA_CHANNEL_IRQn(QUADSPI_DMA_NUMBER, QUADSPI_DMA_CHANNEL)
#    define QUADSPI_DMA_IRQHandler                                             \
      CSP_DMA_CHANNEL_IRQ(QUADSPI_DMA_NUMBER, QUADSPI_DMA_CHANNEL)
#  endif /* QUADSPI_DMA */

/* clang-format on */

/*****************************************************************************
 * @defgroup QSPI Public Functions
 * @{
 */

#define QSPI_INIT_OK            0
#define QSPI_INIT_FAIL          1
#define QSPI_INIT_DMA_FAIL      2
#define QSPI_INITED             3

#define QSPI_DEINIT_OK          0
#define QSPI_DEINIT_FAIL        1
#define QSPI_DEINIT_DMA_FAIL    2
#define QSPI_NO_INIT            3

#define QSPI_RW_TIMEOUT         1000

uint8_t qspi_send_cmd(uint8_t cmd, uint32_t address, uint8_t mode,
                      uint8_t dummy_cycles);
uint8_t qspi_transmit(uint8_t *data, uint32_t len);
uint8_t qspi_receive(uint8_t *data, uint32_t len);

#endif /* QUADSPI_ENABLE */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SPI_STM32F4xx_H */

