/**
 * @file    CAN_STM32F4xx.h
 * @author  Deadline039
 * @brief   Chip Support Package of CAN on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 * @note    The calculation formula of CAN rate reference to NXP
 *          Application Note: CAN Bit Timing Requirements (AN1798)
 *          https://www.nxp.com/docs/en/application-note/AN1798.pdf
 *
 *          This code is generate automatically.
 */

#ifndef __CAN_STM32F4xx_H
#define __CAN_STM32F4xx_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*****************************************************************************
 * @defgroup Public Marco of CAN.
 * @{
 */

#define CAN_INIT_OK             0
#define CAN_INIT_RATE_ERR       1
#define CAN_INIT_FILTER_FAIL    2
#define CAN_INIT_FAIL           3
#define CAN_INIT_START_FAIL     4
#define CAN_INIT_NOTIFY_FAIL    5
#define CAN_INITED              6

#define CAN_DEINIT_OK           0
#define CAN_DEINIT_FAIL         1
#define CAN_NO_INIT             2

/* Wait for can tx mailbox empty times. */
#define CAN_SEND_TIMEOUT        100

/**
 * @}
 */


/*****************************************************************************
 * @defgroup Public type and variables of CAN.
 * @{
 */

/**
 * @brief Select which CAN will be used.
 */
typedef enum {
    can1_selected = 0U, /*!< Select CAN1 */
    can2_selected,      /*!< Select CAN2 */
    can3_selected       /*!< Select CAN3 */
} can_selected_t;

/**
 * @}
 */
 
/* clang-format off */

/*****************************************************************************
 * @defgroup CAN1 Functions
 * @{
 */

#if CAN1_ENABLE

#if (CAN1_TX_ID == 0)
#  define CAN1_TX_GPIO_AF GPIO_AF9_CAN1
#elif (CAN1_TX_ID == 1)
#  define CAN1_TX_GPIO_AF GPIO_AF8_CAN1
#elif (CAN1_TX_ID == 2)
#  define CAN1_TX_GPIO_AF GPIO_AF9_CAN1
#elif (CAN1_TX_ID == 3)
#  define CAN1_TX_GPIO_AF GPIO_AF9_CAN1
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PG1 can not be configured as CAN1 TX on STM32F479xx, STM32F437xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (CAN1_TX_ID == 4)
#  define CAN1_TX_GPIO_AF GPIO_AF9_CAN1
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PH13 can not be configured as CAN1 TX on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* CAN1_TX_ID */

#if (CAN1_RX_ID == 0)
#  define CAN1_RX_GPIO_AF GPIO_AF9_CAN1
#elif (CAN1_RX_ID == 1)
#  define CAN1_RX_GPIO_AF GPIO_AF8_CAN1
#elif (CAN1_RX_ID == 2)
#  define CAN1_RX_GPIO_AF GPIO_AF9_CAN1
#elif (CAN1_RX_ID == 3)
#  define CAN1_RX_GPIO_AF GPIO_AF9_CAN1
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PG0 can not be configured as CAN1 RX on STM32F479xx, STM32F437xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#elif (CAN1_RX_ID == 4)
#  define CAN1_RX_GPIO_AF GPIO_AF9_CAN1
#  if (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx))
#    error "PI9 can not be configured as CAN1 RX on STM32F423xx, STM32F446xx, STM32F412Rx, STM32F412Cx, STM32F412Zx, STM32F413xx, STM32F412Vx! "
#  endif  /* (defined(STM32F423xx) || defined(STM32F446xx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F412Vx)) */
#endif  /* CAN1_RX_ID */

extern CAN_HandleTypeDef can1_handle;

uint8_t can1_init(uint32_t baud_rate, uint32_t prop_delay);
uint8_t can1_deinit(void); 

#endif /* CAN1_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup CAN2 Functions
 * @{
 */

#if CAN2_ENABLE

#if (CAN2_TX_ID == 0)
#  define CAN2_TX_GPIO_AF GPIO_AF9_CAN2
#elif (CAN2_TX_ID == 1)
#  define CAN2_TX_GPIO_AF GPIO_AF9_CAN2
#elif (CAN2_TX_ID == 2)
#  define CAN2_TX_GPIO_AF GPIO_AF9_CAN2
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PG12 can not be configured as CAN2 TX on STM32F479xx, STM32F437xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#endif  /* CAN2_TX_ID */

#if (CAN2_RX_ID == 0)
#  define CAN2_RX_GPIO_AF GPIO_AF9_CAN2
#elif (CAN2_RX_ID == 1)
#  define CAN2_RX_GPIO_AF GPIO_AF9_CAN2
#elif (CAN2_RX_ID == 2)
#  define CAN2_RX_GPIO_AF GPIO_AF9_CAN2
#  if (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx))
#    error "PG11 can not be configured as CAN2 RX on STM32F479xx, STM32F437xx, STM32F405xx, STM32F446xx, STM32F415xx, STM32F407xx, STM32F439xx, STM32F429xx, STM32F469xx, STM32F417xx, STM32F427xx! "
#  endif  /* (defined(STM32F479xx) || defined(STM32F437xx) || defined(STM32F405xx) || defined(STM32F446xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F439xx) || defined(STM32F429xx) || defined(STM32F469xx) || defined(STM32F417xx) || defined(STM32F427xx)) */
#endif  /* CAN2_RX_ID */

extern CAN_HandleTypeDef can2_handle;

uint8_t can2_init(uint32_t baud_rate, uint32_t prop_delay);
uint8_t can2_deinit(void); 

#endif /* CAN2_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup CAN3 Functions
 * @{
 */

#if CAN3_ENABLE

#if (CAN3_TX_ID == 0)
#  define CAN3_TX_GPIO_AF GPIO_AF11_CAN3
#elif (CAN3_TX_ID == 1)
#  define CAN3_TX_GPIO_AF GPIO_AF11_CAN3
#endif  /* CAN3_TX_ID */

#if (CAN3_RX_ID == 0)
#  define CAN3_RX_GPIO_AF GPIO_AF11_CAN3
#elif (CAN3_RX_ID == 1)
#  define CAN3_RX_GPIO_AF GPIO_AF11_CAN3
#endif  /* CAN3_RX_ID */

extern CAN_HandleTypeDef can3_handle;

uint8_t can3_init(uint32_t baud_rate, uint32_t prop_delay);
uint8_t can3_deinit(void); 

#endif /* CAN3_ENABLE */

/**
 * @}
 */

/* clang-format on */

/*****************************************************************************
 * @defgroup Public functions of CAN.
 * @{
 */

uint8_t can_rate_calc(uint32_t baud_rate, uint32_t prop_delay,
                      uint32_t base_freq, uint32_t *prescale, uint32_t *tsjw,
                      uint32_t *tseg1, uint32_t *tseg2);

uint8_t can_send_message(can_selected_t can_selected, uint32_t can_ide,
                         uint32_t id, uint8_t len, const uint8_t *msg);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CAN_STM32F4xx_H */
