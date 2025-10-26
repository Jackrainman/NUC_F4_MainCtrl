/**
 * @file    CAN_STM32F4xx.c
 * @author  Deadline039
 * @brief   Chip Support Package of CAN on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 */

#include <CSP_Config.h>

#include "CAN_STM32F4xx.h"

#include <math.h>


/*****************************************************************************
 * @defgroup CAN1 Functions.
 * @{
 */

#if CAN1_ENABLE

CAN_HandleTypeDef can1_handle = {.Instance = CAN1,
                                 .Init = {.Mode = CAN_MODE_NORMAL,
                                          .TimeTriggeredMode = DISABLE,
                                          .AutoBusOff = DISABLE,
                                          .AutoWakeUp = DISABLE,
                                          .AutoRetransmission = ENABLE,
                                          .ReceiveFifoLocked = DISABLE,
                                          .TransmitFifoPriority = DISABLE}};

/**
 * @brief CAN1 initialization
 *
 * @param baud_rate Baud rate. Unit: Kbps.
 * @param prop_delay The propagation delay of bus, include cable and can
 *                   transceiver. Unit: ns.
 * @return CAN init status.
 *  @retval - 0: `CAN_INIT_OK`:           Success.
 *  @retval - 1: `CAN_INIT_RATE_ERR`:     Can not satisfied this baudrate in this
 *                                       condition.
 *  @retval - 2: `CAN_INIT_FILTER_FAIL`:  CAN Filter init failed.
 *  @retval - 3: `CAN_INIT_FAIL`:         CAN hardware init failed.
 *  @retval - 4: `CAN_INIT_START_FAIL`:   CAN start failed.
 *  @retval - 5: `CAN_INIT_NOTIFY_FAIL`:  Enable CAN receive notify failed.
 *  @retval - 6: `CAN_INITED`:            This can is inited.
 */
uint8_t can1_init(uint32_t baud_rate, uint32_t prop_delay) {
    if (HAL_CAN_GetState(&can1_handle) != HAL_CAN_STATE_RESET) {
        return CAN_INITED;
    }

    uint32_t prescale, tbs1, tbs2, tsjw;
    if (can_rate_calc(baud_rate * 1000, prop_delay, HAL_RCC_GetPCLK1Freq(),
                      &prescale, &tsjw, &tbs1, &tbs2) == 1) {
        return CAN_INIT_RATE_ERR;
    }

    can1_handle.Init.Prescaler = prescale;
    can1_handle.Init.TimeSeg1 = (tbs1 - 1) << CAN_BTR_TS1_Pos;
    can1_handle.Init.TimeSeg2 = (tbs2 - 1) << CAN_BTR_TS2_Pos;
    can1_handle.Init.SyncJumpWidth = (tsjw - 1) << CAN_BTR_SJW_Pos;

    if (HAL_CAN_Init(&can1_handle) != HAL_OK) {
        return CAN_INIT_FAIL;
    }

    CAN_FilterTypeDef can_filter_config;

    can_filter_config.FilterBank = 0;
    can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_config.FilterIdHigh = 0x0000;
    can_filter_config.FilterIdLow = 0x0000;
    can_filter_config.FilterMaskIdHigh = 0x0000;
    can_filter_config.FilterMaskIdLow = 0x0000;
    can_filter_config.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_config.SlaveStartFilterBank = 0;

#if CAN1_RX0_IT_ENABLE
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    if (HAL_CAN_ConfigFilter(&can1_handle, &can_filter_config) != HAL_OK) {
        return CAN_INIT_FILTER_FAIL;
    }
    if (HAL_CAN_ActivateNotification(&can1_handle,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return CAN_INIT_NOTIFY_FAIL;
    }
#endif /* CAN1_RX0_IT_ENABLE */

#if CAN1_RX1_IT_ENABLE
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    if (HAL_CAN_ConfigFilter(&can1_handle, &can_filter_config) != HAL_OK) {
        return CAN_INIT_FILTER_FAIL;
    }
    if (HAL_CAN_ActivateNotification(&can1_handle,
                                     CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        return CAN_INIT_NOTIFY_FAIL;
    }
#endif /* CAN1_RX1_IT_ENABLE */

    if (HAL_CAN_Start(&can1_handle) != HAL_OK) {
        return CAN_INIT_START_FAIL;
    }

    return CAN_INIT_OK;
}

#if CAN1_TX_IT_ENABLE

/**
 * @brief CAN1 TX ISR.
 *
 */
void CAN1_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1_handle);
}

#endif /* CAN1_TX_IT_ENABLE */

#if CAN1_RX0_IT_ENABLE

/**
 * @brief CAN1 RX FIFO0 ISR.
 *
 */
void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1_handle);
}

#endif /* CAN1_RX0_IT_ENABLE */

#if CAN1_RX1_IT_ENABLE

/**
 * @brief CAN1 RX FIFO1 ISR.
 *
 */
void CAN1_RX1_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1_handle);
}

#endif /* CAN1_RX1_IT_ENABLE */

#if CAN1_SCE_IT_ENABLE

/**
 * @brief CAN1 SCE ISR.
 *
 */
void CAN1_SCE_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can1_handle);
}

#endif /* CAN1_SCE_IT_ENABLE */

/**
 * @brief CAN1 deinitialization.
 *
 * @return Deinit status.
 *  @retval - 0: `CAN_DEINIT_OK`:   Success.
 *  @retval - 1: `CAN_DEINIT_FAIL`: CAN hardware deinit failed.
 *  @retval - 2: `CAN_NO_INIT`:     This can is no init.
 */
uint8_t can1_deinit(void) {
    if (HAL_CAN_GetState(&can1_handle) == HAL_CAN_STATE_RESET) {
        return CAN_NO_INIT;
    }

    if (HAL_CAN_Stop(&can1_handle) != HAL_OK) {
        return CAN_DEINIT_FAIL;
    }

    if (HAL_CAN_DeInit(&can1_handle) != HAL_OK) {
        return CAN_DEINIT_FAIL;
    }

    return CAN_DEINIT_OK;
}

#endif /* CAN1_ENABLE */

/**
 * @}
 */



/*****************************************************************************
 * @defgroup CAN2 Functions.
 * @{
 */

#if CAN2_ENABLE

CAN_HandleTypeDef can2_handle = {.Instance = CAN2,
                                 .Init = {.Mode = CAN_MODE_NORMAL,
                                          .TimeTriggeredMode = DISABLE,
                                          .AutoBusOff = DISABLE,
                                          .AutoWakeUp = DISABLE,
                                          .AutoRetransmission = ENABLE,
                                          .ReceiveFifoLocked = DISABLE,
                                          .TransmitFifoPriority = DISABLE}};

/**
 * @brief CAN2 initialization
 *
 * @param baud_rate Baud rate. Unit: Kbps.
 * @param prop_delay The propagation delay of bus, include cable and can
 *                   transceiver. Unit: ns.
 * @return CAN init status.
 *  @retval - 0: `CAN_INIT_OK`:           Success.
 *  @retval - 1: `CAN_INIT_RATE_ERR`:     Can not satisfied this baudrate in this
 *                                       condition.
 *  @retval - 2: `CAN_INIT_FILTER_FAIL`:  CAN Filter init failed.
 *  @retval - 3: `CAN_INIT_FAIL`:         CAN hardware init failed.
 *  @retval - 4: `CAN_INIT_START_FAIL`:   CAN start failed.
 *  @retval - 5: `CAN_INIT_NOTIFY_FAIL`:  Enable CAN receive notify failed.
 *  @retval - 6: `CAN_INITED`:            This can is inited.
 */
uint8_t can2_init(uint32_t baud_rate, uint32_t prop_delay) {
    if (HAL_CAN_GetState(&can2_handle) != HAL_CAN_STATE_RESET) {
        return CAN_INITED;
    }

    uint32_t prescale, tbs1, tbs2, tsjw;
    if (can_rate_calc(baud_rate * 1000, prop_delay, HAL_RCC_GetPCLK1Freq(),
                      &prescale, &tsjw, &tbs1, &tbs2) == 1) {
        return CAN_INIT_RATE_ERR;
    }

    can2_handle.Init.Prescaler = prescale;
    can2_handle.Init.TimeSeg1 = (tbs1 - 1) << CAN_BTR_TS1_Pos;
    can2_handle.Init.TimeSeg2 = (tbs2 - 1) << CAN_BTR_TS2_Pos;
    can2_handle.Init.SyncJumpWidth = (tsjw - 1) << CAN_BTR_SJW_Pos;

    if (HAL_CAN_Init(&can2_handle) != HAL_OK) {
        return CAN_INIT_FAIL;
    }

    CAN_FilterTypeDef can_filter_config;

    can_filter_config.FilterBank = 0;
    can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_config.FilterIdHigh = 0x0000;
    can_filter_config.FilterIdLow = 0x0000;
    can_filter_config.FilterMaskIdHigh = 0x0000;
    can_filter_config.FilterMaskIdLow = 0x0000;
    can_filter_config.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_config.SlaveStartFilterBank = 0;

#if CAN2_RX0_IT_ENABLE
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    if (HAL_CAN_ConfigFilter(&can2_handle, &can_filter_config) != HAL_OK) {
        return CAN_INIT_FILTER_FAIL;
    }
    if (HAL_CAN_ActivateNotification(&can2_handle,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return CAN_INIT_NOTIFY_FAIL;
    }
#endif /* CAN2_RX0_IT_ENABLE */

#if CAN2_RX1_IT_ENABLE
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    if (HAL_CAN_ConfigFilter(&can2_handle, &can_filter_config) != HAL_OK) {
        return CAN_INIT_FILTER_FAIL;
    }
    if (HAL_CAN_ActivateNotification(&can2_handle,
                                     CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        return CAN_INIT_NOTIFY_FAIL;
    }
#endif /* CAN2_RX1_IT_ENABLE */

    if (HAL_CAN_Start(&can2_handle) != HAL_OK) {
        return CAN_INIT_START_FAIL;
    }

    return CAN_INIT_OK;
}

#if CAN2_TX_IT_ENABLE

/**
 * @brief CAN2 TX ISR.
 *
 */
void CAN2_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2_handle);
}

#endif /* CAN2_TX_IT_ENABLE */

#if CAN2_RX0_IT_ENABLE

/**
 * @brief CAN2 RX FIFO0 ISR.
 *
 */
void CAN2_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2_handle);
}

#endif /* CAN2_RX0_IT_ENABLE */

#if CAN2_RX1_IT_ENABLE

/**
 * @brief CAN2 RX FIFO1 ISR.
 *
 */
void CAN2_RX1_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2_handle);
}

#endif /* CAN2_RX1_IT_ENABLE */

#if CAN2_SCE_IT_ENABLE

/**
 * @brief CAN2 SCE ISR.
 *
 */
void CAN2_SCE_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can2_handle);
}

#endif /* CAN2_SCE_IT_ENABLE */

/**
 * @brief CAN2 deinitialization.
 *
 * @return Deinit status.
 *  @retval - 0: `CAN_DEINIT_OK`:   Success.
 *  @retval - 1: `CAN_DEINIT_FAIL`: CAN hardware deinit failed.
 *  @retval - 2: `CAN_NO_INIT`:     This can is no init.
 */
uint8_t can2_deinit(void) {
    if (HAL_CAN_GetState(&can2_handle) == HAL_CAN_STATE_RESET) {
        return CAN_NO_INIT;
    }

    if (HAL_CAN_Stop(&can2_handle) != HAL_OK) {
        return CAN_DEINIT_FAIL;
    }

    if (HAL_CAN_DeInit(&can2_handle) != HAL_OK) {
        return CAN_DEINIT_FAIL;
    }

    return CAN_DEINIT_OK;
}

#endif /* CAN2_ENABLE */

/**
 * @}
 */



/*****************************************************************************
 * @defgroup CAN3 Functions.
 * @{
 */

#if CAN3_ENABLE

CAN_HandleTypeDef can3_handle = {.Instance = CAN3,
                                 .Init = {.Mode = CAN_MODE_NORMAL,
                                          .TimeTriggeredMode = DISABLE,
                                          .AutoBusOff = DISABLE,
                                          .AutoWakeUp = DISABLE,
                                          .AutoRetransmission = ENABLE,
                                          .ReceiveFifoLocked = DISABLE,
                                          .TransmitFifoPriority = DISABLE}};

/**
 * @brief CAN3 initialization
 *
 * @param baud_rate Baud rate. Unit: Kbps.
 * @param prop_delay The propagation delay of bus, include cable and can
 *                   transceiver. Unit: ns.
 * @return CAN init status.
 *  @retval - 0: `CAN_INIT_OK`:           Success.
 *  @retval - 1: `CAN_INIT_RATE_ERR`:     Can not satisfied this baudrate in this
 *                                       condition.
 *  @retval - 2: `CAN_INIT_FILTER_FAIL`:  CAN Filter init failed.
 *  @retval - 3: `CAN_INIT_FAIL`:         CAN hardware init failed.
 *  @retval - 4: `CAN_INIT_START_FAIL`:   CAN start failed.
 *  @retval - 5: `CAN_INIT_NOTIFY_FAIL`:  Enable CAN receive notify failed.
 *  @retval - 6: `CAN_INITED`:            This can is inited.
 */
uint8_t can3_init(uint32_t baud_rate, uint32_t prop_delay) {
    if (HAL_CAN_GetState(&can3_handle) != HAL_CAN_STATE_RESET) {
        return CAN_INITED;
    }

    uint32_t prescale, tbs1, tbs2, tsjw;
    if (can_rate_calc(baud_rate * 1000, prop_delay, HAL_RCC_GetPCLK1Freq(),
                      &prescale, &tsjw, &tbs1, &tbs2) == 1) {
        return CAN_INIT_RATE_ERR;
    }

    can3_handle.Init.Prescaler = prescale;
    can3_handle.Init.TimeSeg1 = (tbs1 - 1) << CAN_BTR_TS1_Pos;
    can3_handle.Init.TimeSeg2 = (tbs2 - 1) << CAN_BTR_TS2_Pos;
    can3_handle.Init.SyncJumpWidth = (tsjw - 1) << CAN_BTR_SJW_Pos;

    if (HAL_CAN_Init(&can3_handle) != HAL_OK) {
        return CAN_INIT_FAIL;
    }

    CAN_FilterTypeDef can_filter_config;

    can_filter_config.FilterBank = 0;
    can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_config.FilterIdHigh = 0x0000;
    can_filter_config.FilterIdLow = 0x0000;
    can_filter_config.FilterMaskIdHigh = 0x0000;
    can_filter_config.FilterMaskIdLow = 0x0000;
    can_filter_config.FilterActivation = CAN_FILTER_ENABLE;
    can_filter_config.SlaveStartFilterBank = 0;

#if CAN3_RX0_IT_ENABLE
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    if (HAL_CAN_ConfigFilter(&can3_handle, &can_filter_config) != HAL_OK) {
        return CAN_INIT_FILTER_FAIL;
    }
    if (HAL_CAN_ActivateNotification(&can3_handle,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return CAN_INIT_NOTIFY_FAIL;
    }
#endif /* CAN3_RX0_IT_ENABLE */

#if CAN3_RX1_IT_ENABLE
    can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    if (HAL_CAN_ConfigFilter(&can3_handle, &can_filter_config) != HAL_OK) {
        return CAN_INIT_FILTER_FAIL;
    }
    if (HAL_CAN_ActivateNotification(&can3_handle,
                                     CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        return CAN_INIT_NOTIFY_FAIL;
    }
#endif /* CAN3_RX1_IT_ENABLE */

    if (HAL_CAN_Start(&can3_handle) != HAL_OK) {
        return CAN_INIT_START_FAIL;
    }

    return CAN_INIT_OK;
}

#if CAN3_TX_IT_ENABLE

/**
 * @brief CAN3 TX ISR.
 *
 */
void CAN3_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can3_handle);
}

#endif /* CAN3_TX_IT_ENABLE */

#if CAN3_RX0_IT_ENABLE

/**
 * @brief CAN3 RX FIFO0 ISR.
 *
 */
void CAN3_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can3_handle);
}

#endif /* CAN3_RX0_IT_ENABLE */

#if CAN3_RX1_IT_ENABLE

/**
 * @brief CAN3 RX FIFO1 ISR.
 *
 */
void CAN3_RX1_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can3_handle);
}

#endif /* CAN3_RX1_IT_ENABLE */

#if CAN3_SCE_IT_ENABLE

/**
 * @brief CAN3 SCE ISR.
 *
 */
void CAN3_SCE_IRQHandler(void) {
    HAL_CAN_IRQHandler(&can3_handle);
}

#endif /* CAN3_SCE_IT_ENABLE */

/**
 * @brief CAN3 deinitialization.
 *
 * @return Deinit status.
 *  @retval - 0: `CAN_DEINIT_OK`:   Success.
 *  @retval - 1: `CAN_DEINIT_FAIL`: CAN hardware deinit failed.
 *  @retval - 2: `CAN_NO_INIT`:     This can is no init.
 */
uint8_t can3_deinit(void) {
    if (HAL_CAN_GetState(&can3_handle) == HAL_CAN_STATE_RESET) {
        return CAN_NO_INIT;
    }

    if (HAL_CAN_Stop(&can3_handle) != HAL_OK) {
        return CAN_DEINIT_FAIL;
    }

    if (HAL_CAN_DeInit(&can3_handle) != HAL_OK) {
        return CAN_DEINIT_FAIL;
    }

    return CAN_DEINIT_OK;
}

#endif /* CAN3_ENABLE */

/**
 * @}
 */



/*****************************************************************************
 * @defgroup Public functions of CAN.
 * @{
 */

/**
 * @brief CAN Low level initialization
 *
 * @param hcan The handle of CAN
 * @note If you want to use dual CAN, you must initialize both CAN GPIO at the
 *       same time, otherwise there will be a `Wait initialisation acknowledge`
 *       error.
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan) {
    GPIO_InitTypeDef gpio_init_struct = {.Mode = GPIO_MODE_AF_PP,
                                         .Pull = GPIO_PULLUP,
                                         .Speed = GPIO_SPEED_FREQ_VERY_HIGH};
#if CAN3_ENABLE
    if (hcan->Instance == CAN3) {

        CSP_GPIO_CLK_ENABLE(CAN3_TX_PORT);
        gpio_init_struct.Pin = CAN3_TX_PIN;
        gpio_init_struct.Alternate = CAN3_TX_GPIO_AF;
        HAL_GPIO_Init(CSP_GPIO_PORT(CAN3_TX_PORT), &gpio_init_struct);

        CSP_GPIO_CLK_ENABLE(CAN3_RX_PORT);
        gpio_init_struct.Pin = CAN3_RX_PIN;
        gpio_init_struct.Alternate = CAN3_RX_GPIO_AF;
        HAL_GPIO_Init(CSP_GPIO_PORT(CAN3_RX_PORT), &gpio_init_struct);
        __HAL_RCC_CAN3_CLK_ENABLE();

#if CAN3_TX_IT_ENABLE
        HAL_NVIC_SetPriority(CAN3_TX_IRQn, CAN3_TX_IT_PRIORITY, CAN3_TX_IT_SUB);
        HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
#endif /* CAN3_TX_IT_ENABLE */

#if CAN3_RX0_IT_ENABLE
        HAL_NVIC_SetPriority(CAN3_RX0_IRQn, CAN3_RX0_IT_PRIORITY,
                             CAN3_RX0_IT_SUB);
        HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
#endif /* CAN3_RX0_IT_ENABLE */

#if CAN3_RX1_IT_ENABLE
        HAL_NVIC_SetPriority(CAN3_RX1_IRQn, CAN3_RX1_IT_PRIORITY,
                             CAN3_RX1_IT_SUB);
        HAL_NVIC_EnableIRQ(CAN3_RX1_IRQn);
#endif /* CAN3_RX1_IT_ENABLE */

#if CAN3_SCE_IT_ENABLE
        HAL_NVIC_SetPriority(CAN3_SCE_IRQn, CAN3_SCE_IT_PRIORITY,
                             CAN3_SCE_IT_SUB);
        HAL_NVIC_EnableIRQ(CAN3_SCE_IRQn);
#endif /* CAN3_SCE_IT_ENABLE */

        return;
    }
#endif /* CAN3_ENABLE */

#if CAN1_ENABLE
    __HAL_RCC_CAN1_CLK_ENABLE();

    CSP_GPIO_CLK_ENABLE(CAN1_TX_PORT);
    gpio_init_struct.Pin = CAN1_TX_PIN;
    gpio_init_struct.Alternate = CAN1_TX_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(CAN1_TX_PORT), &gpio_init_struct);

    CSP_GPIO_CLK_ENABLE(CAN1_RX_PORT);
    gpio_init_struct.Pin = CAN1_RX_PIN;
    gpio_init_struct.Alternate = CAN1_RX_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(CAN1_RX_PORT), &gpio_init_struct);

#if CAN1_TX_IT_ENABLE
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, CAN1_TX_IT_PRIORITY, CAN1_TX_IT_SUB);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
#endif /* CAN1_TX_IT_ENABLE */

#if CAN1_RX0_IT_ENABLE
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, CAN1_RX0_IT_PRIORITY, CAN1_RX0_IT_SUB);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
#endif /* CAN1_RX0_IT_ENABLE */

#if CAN1_RX1_IT_ENABLE
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, CAN1_RX1_IT_PRIORITY, CAN1_RX1_IT_SUB);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
#endif /* CAN1_RX1_IT_ENABLE */

#if CAN1_SCE_IT_ENABLE
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, CAN1_SCE_IT_PRIORITY, CAN1_SCE_IT_SUB);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
#endif /* CAN1_SCE_IT_ENABLE */

#endif /* CAN1_ENABLE */

#if CAN2_ENABLE
    __HAL_RCC_CAN2_CLK_ENABLE();

    CSP_GPIO_CLK_ENABLE(CAN2_TX_PORT);
    gpio_init_struct.Pin = CAN2_TX_PIN;
    gpio_init_struct.Alternate = CAN2_TX_GPIO_AF;

    HAL_GPIO_Init(CSP_GPIO_PORT(CAN2_TX_PORT), &gpio_init_struct);

    CSP_GPIO_CLK_ENABLE(CAN2_RX_PORT);
    gpio_init_struct.Pin = CAN2_RX_PIN;
    gpio_init_struct.Alternate = CAN2_RX_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(CAN2_RX_PORT), &gpio_init_struct);

#if CAN2_TX_IT_ENABLE
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, CAN2_TX_IT_PRIORITY, CAN2_TX_IT_SUB);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
#endif /* CAN2_TX_IT_ENABLE */

#if CAN2_RX0_IT_ENABLE
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, CAN2_RX0_IT_PRIORITY, CAN2_RX0_IT_SUB);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
#endif /* CAN2_RX0_IT_ENABLE */

#if CAN2_RX1_IT_ENABLE
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, CAN2_RX1_IT_PRIORITY, CAN2_RX1_IT_SUB);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
#endif /* CAN2_RX1_IT_ENABLE */

#if CAN2_SCE_IT_ENABLE
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, CAN2_SCE_IT_PRIORITY, CAN2_SCE_IT_SUB);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
#endif /* CAN2_SCE_IT_ENABLE */

#endif /* CAN2_ENABLE */
}

/**
 * @brief CAN Low level deinitialization
 *
 * @param hcan The handle of CAN
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan) {

#if CAN1_ENABLE
    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_DISABLE();

        HAL_GPIO_DeInit(CSP_GPIO_PORT(CAN1_RX_PORT), CAN1_RX_PIN);
        HAL_GPIO_DeInit(CSP_GPIO_PORT(CAN1_TX_PORT), CAN1_TX_PIN);

#if CAN1_TX_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
#endif /* CAN1_TX_IT_ENABLE */

#if CAN1_RX0_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
#endif /* CAN1_RX0_IT_ENABLE */

#if CAN1_RX1_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
#endif /* CAN1_RX1_IT_ENABLE */

#if CAN1_SCE_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
#endif /* CAN1_SCE_IT_ENABLE */
    }
#endif /* CAN1_ENABLE */

#if CAN2_ENABLE
    if (hcan->Instance == CAN2) {
        __HAL_RCC_CAN2_CLK_DISABLE();

        HAL_GPIO_DeInit(CSP_GPIO_PORT(CAN2_RX_PORT), CAN2_RX_PIN);
        HAL_GPIO_DeInit(CSP_GPIO_PORT(CAN2_TX_PORT), CAN2_TX_PIN);

#if CAN2_TX_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
#endif /* CAN2_TX_IT_ENABLE */

#if CAN2_RX0_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
#endif /* CAN2_RX0_IT_ENABLE */

#if CAN2_RX1_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
#endif /* CAN2_RX1_IT_ENABLE */

#if CAN2_SCE_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);
#endif /* CAN2_SCE_IT_ENABLE */
    }
#endif /* CAN2_ENABLE */

#if CAN3_ENABLE
    if (hcan->Instance == CAN3) {
        __HAL_RCC_CAN3_CLK_DISABLE();

        HAL_GPIO_DeInit(CSP_GPIO_PORT(CAN3_RX_PORT), CAN3_RX_PIN);
        HAL_GPIO_DeInit(CSP_GPIO_PORT(CAN3_TX_PORT), CAN3_TX_PIN);

#if CAN3_TX_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN3_TX_IRQn);
#endif /* CAN3_TX_IT_ENABLE */

#if CAN3_RX0_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN3_RX0_IRQn);
#endif /* CAN3_RX0_IT_ENABLE */

#if CAN3_RX1_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN3_RX1_IRQn);
#endif /* CAN3_RX1_IT_ENABLE */

#if CAN3_SCE_IT_ENABLE
        HAL_NVIC_DisableIRQ(CAN3_SCE_IRQn);
#endif /* CAN3_SCE_IT_ENABLE */
    }
#endif /* CAN3_ENABLE */

}

/**
 * @brief Calculate parameters of specific CAN Classic baudrate.
 *
 * @param[in] baud_rate CAN band rate. Unit: bps.
 * @param[in] prop_delay The propagation delay of bus, include cable and can
 *                       transceiver. Unit: ns.
 * @param[in] base_freq Base frequency of peripherals. Unit: Hz.
 * @param[out] prescale The prescale of `base_freq`.
 * @param[out] tsjw Syncronisation Jump Width
 * @param[out] tseg1 Time of segment 1.
 * @param[out] tseg2 Time of segment 2.
 * @return Calculate status.
 *  @retval - 0: No error;
 *  @retval - 1: Can not satisfied this baudrate in this condition.
 */
uint8_t can_rate_calc(uint32_t baud_rate, uint32_t prop_delay,
                      uint32_t base_freq, uint32_t *prescale, uint32_t *tsjw,
                      uint32_t *tseg1, uint32_t *tseg2) {

    /*## step 1: determine minimum permissible time for the prop_seg
                 segment. #################################################*/
    uint32_t t_prop_seg = 2 * prop_delay;

    /*## step 2: choose can system clock frequency. #######################*/
    uint32_t f_div;
    *prescale = 0;

    while (1) {
        ++(*prescale);
        f_div = base_freq / (*prescale);

        if (f_div / baud_rate > 25) {
            continue;
        }

        if (f_div / baud_rate < 8) {
            /* cannot divide this baudrate. */
            return 1;
        }

        if (f_div % baud_rate != 0) {
            continue;
        } else {
            break;
        }
    }

    /*## step 3: calculate prop_seg duration. ##########################*/
    uint32_t tq_per_bit = f_div / baud_rate;
    uint32_t prop_seg =
        (uint32_t)ceilf((float)(t_prop_seg * f_div) / 1000000000.0f);

    /*## step 4: determine phase_seg1, phase_seg2 */
    uint32_t tq_remain = tq_per_bit - prop_seg - 1;
    if (tq_remain < 3) {
        /* frequency division is from small to large, which means that if
         * the condition is met, the frequency division coefficient must be
         * the highest. if even the highest frequency cannot meet this
         * condition, then there is no other way. */
        return 1;
    }

    if (tq_remain == 3) {
        /* if the remaining number is equal to 3, than phase_seg1 = 1 and
         * phase_seg2 = 2 */
        *tseg1 = 1;
        *tseg2 = 2;
    } else {
        /* if the remain number is an odd number greater than 3, than add
         * one to the prop_seg (also subtract one to the remain number)*/
        tq_remain = (tq_remain % 2 == 0) ? (tq_remain) : (tq_remain - 1);

        /* divide the remaining number by to and assign the result to
         * phase_seg1 and phase_seg2. */
        *tseg1 = tq_remain / 2;
        *tseg2 = tq_remain - *tseg1;
    }

    *tseg1 += prop_seg;

    if (*tseg2 > 8) {
        /* STM32F4 CAN BS2 range: 1-8 */
        *tseg2 = 8;
        *tseg1 = tq_remain - *tseg2;
    }
    if (*tseg1 > 16) {
        /* STM32F4 CAN BS1 range: 1-16 */
        return 1;
    }

    /*## step 5: determine trjw (also called tsjw) #########################*/
    /* rjw is the smaller of 4 and phase_seg1. */
    *tsjw = (*tseg1 > 4) ? (4) : (*tseg1);

    /*## skip step 6: calculate required oscillator. #######################*/
    return 0;
}

/**
 * @brief Get the CAN handle with specified CAN.
 *
 * @param can_selected Specific which can will to get.
 * @return The handle of CAN. return NULL which the CAN doesn't exist.
 */
CAN_HandleTypeDef *can_get_handle(can_selected_t can_selected) {
    switch (can_selected) {

#if CAN1_ENABLE
        case can1_selected:
            return &can1_handle;
#endif /* CAN1_ENABLE */

#if CAN2_ENABLE
        case can2_selected:
            return &can2_handle;
#endif /* CAN2_ENABLE */

#if CAN3_ENABLE
        case can3_selected:
            return &can3_handle;
#endif /* CAN3_ENABLE */

        default:
            return NULL;
    }
}

/**
 * @brief CAN send message.
 *
 * @param can_selected Specific which CAN to send message.
 * @param can_ide Specific standard ID or Extend ID.
 * @param id Specific message id.
 * @param len Specific message length.
 * @param msg Specific message content.
 * @return Send status.
 *  @retval - 0: Success.
 *  @retval - 1: Send error.
 *  @retval - 2: Timeout.
 *  @retval - 3: Parameter invalid.
 *  @retval - 4: This CAN is not initialized.
 */
uint8_t can_send_message(can_selected_t can_selected, uint32_t can_ide,
                         uint32_t id, uint8_t len, const uint8_t *msg) {
    CAN_HandleTypeDef *can_handle = can_get_handle(can_selected);
    if (can_handle == NULL) {
        return 3;
    }

    if (len > 8) {
        return 3;
    }

    if (HAL_CAN_GetState(can_handle) == HAL_CAN_STATE_RESET) {
        return 4;
    }

    uint16_t wait_time = 0;
    uint32_t tx_mail_box = CAN_TX_MAILBOX0;

    CAN_TxHeaderTypeDef tx_header = {0};
    tx_header.IDE = can_ide;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    if (can_ide == CAN_ID_STD) {
        tx_header.StdId = id;
    } else {
        tx_header.ExtId = id;
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(can_handle) == 0) {
        /* Wait to all mailbox is empty. */
        ++wait_time;
        if (wait_time > CAN_SEND_TIMEOUT) {
            return 2;
        }
    }

    if (HAL_CAN_AddTxMessage(can_handle, &tx_header, msg, &tx_mail_box) !=
        HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief CAN send remote message.
 *
 * @param can_selected Specific which CAN to send message.
 * @param can_ide Specific standard ID or Extend ID.
 * @param id Specific message id.
 * @param len Specific message length.
 * @param msg Specific message content.
 * @return Send status.
 *  @retval - 0: Success.
 *  @retval - 1: Send error.
 *  @retval - 2: Timeout.
 *  @retval - 3: Parameter invalid.
 *  @retval - 4: This CAN is not initialized.
 */
uint8_t can_send_remote(can_selected_t can_selected, uint32_t can_ide,
                        uint32_t id, uint8_t len, const uint8_t *msg) {
    CAN_HandleTypeDef *can_handle = can_get_handle(can_selected);
    if (can_handle == NULL) {
        return 3;
    }

    if (len > 8) {
        return 3;
    }

    if (HAL_CAN_GetState(can_handle) == HAL_CAN_STATE_RESET) {
        return 4;
    }

    uint16_t wait_time = 0;
    uint32_t tx_mail_box = CAN_TX_MAILBOX0;

    CAN_TxHeaderTypeDef tx_header = {0};
    tx_header.IDE = can_ide;
    tx_header.RTR = CAN_RTR_REMOTE;
    tx_header.DLC = len;
    if (can_ide == CAN_ID_STD) {
        tx_header.StdId = id;
    } else {
        tx_header.ExtId = id;
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(can_handle) == 0) {
        /* Wait to all mailbox is empty. */
        ++wait_time;
        if (wait_time > CAN_SEND_TIMEOUT) {
            return 2;
        }
    }

    if (HAL_CAN_AddTxMessage(can_handle, &tx_header, msg, &tx_mail_box) !=
        HAL_OK) {
        return 1;
    }

    return 0;
}

/**
 * @}
 */

