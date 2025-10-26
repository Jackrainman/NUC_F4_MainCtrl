/**
 * @file    I2C_STM32F4xx.c
 * @author  Deadline039
 * @brief   Chip Support Package of I2C on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 * @note    Generate Automatically. 
 */

#include <CSP_Config.h>

#include "I2C_STM32F4xx.h"

/*****************************************************************************
 * @defgroup I2C1 Functions
 * @{
 */

#if I2C1_ENABLE

I2C_HandleTypeDef i2c1_handle = {
    .Instance = I2C1,
    .Init = {.DualAddressMode = I2C_DUALADDRESS_DISABLE,
             .GeneralCallMode = I2C_GENERALCALL_DISABLE,
             .NoStretchMode = I2C_NOSTRETCH_DISABLE}};

#if I2C1_RX_DMA
static DMA_HandleTypeDef i2c1_dmarx_handle = {
    .Instance = CSP_DMA_STREAM(I2C1_RX_DMA_NUMBER, I2C1_RX_DMA_STREAM),
    .Init = {.Channel = CSP_DMA_CHANNEL(I2C1_RX_DMA_CHANNEL),
             .FIFOMode = DMA_FIFOMODE_DISABLE,
             .Direction = DMA_PERIPH_TO_MEMORY,
             .PeriphInc = DMA_PINC_DISABLE,
             .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
             .MemInc = DMA_MINC_ENABLE,
             .MemDataAlignment = DMA_MDATAALIGN_BYTE,
             .Mode = DMA_NORMAL,
             .Priority = I2C1_RX_DMA_PRIORITY}};
#endif /* I2C1_RX_DMA */

#if I2C1_TX_DMA
static DMA_HandleTypeDef i2c1_dmatx_handle = {
    .Instance = CSP_DMA_STREAM(I2C1_TX_DMA_NUMBER, I2C1_TX_DMA_STREAM),
    .Init = {.Channel = CSP_DMA_CHANNEL(I2C1_TX_DMA_CHANNEL),
             .FIFOMode = DMA_FIFOMODE_DISABLE,
             .Direction = DMA_MEMORY_TO_PERIPH,
             .PeriphInc = DMA_PINC_DISABLE,
             .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
             .MemInc = DMA_MINC_ENABLE,
             .MemDataAlignment = DMA_MDATAALIGN_BYTE,
             .Mode = DMA_NORMAL,
             .Priority = I2C1_TX_DMA_PRIORITY}};
#endif /* I2C1_TX_DMA */

/**
 * @brief I2C1 initialization.
 *
 * @param clock_speed The speed of I2C communication. Unit: kHz.
 * @param address The address of this device, 7-bit or 10-bit.
 * @param address_mode Specific the address length, this parameter can ref
 *                    `I2C_addressing_mode`.
 * @return I2C init status.
 *  @retval - 0: `I2C_INIT_OK`:       Success.
 *  @retval - 1: `I2C_INIT_FAIL`:     I2C init failed.
 *  @retval - 2: `I2C_INIT_DMA_FAIL`: I2C DMA init failed.
 *  @retval - 3: `I2C_INITED`:        I2C is inited.
 */
uint8_t i2c1_init(uint32_t clock_speed, uint32_t address,
                  uint32_t address_mode) {
    if (HAL_I2C_GetState(&i2c1_handle) != HAL_I2C_STATE_RESET) {
        return I2C_INITED;
    }

    GPIO_InitTypeDef gpio_init_struct = {.Mode = GPIO_MODE_AF_OD,
                                         .Pull = GPIO_PULLUP,
                                         .Speed = GPIO_SPEED_FREQ_VERY_HIGH};

    i2c1_handle.Init.ClockSpeed = clock_speed * 1000;
    i2c1_handle.Init.AddressingMode = address_mode;
    i2c1_handle.Init.OwnAddress1 = address;


    CSP_GPIO_CLK_ENABLE(I2C1_SCL_PORT);
    gpio_init_struct.Pin = I2C1_SCL_PIN;
    gpio_init_struct.Alternate = I2C1_SCL_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(I2C1_SCL_PORT), &gpio_init_struct);

    CSP_GPIO_CLK_ENABLE(I2C1_SDA_PORT);
    gpio_init_struct.Pin = I2C1_SDA_PIN;
    gpio_init_struct.Alternate = I2C1_SDA_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(I2C1_SDA_PORT), &gpio_init_struct);

    __HAL_RCC_I2C1_CLK_ENABLE();

#if I2C1_IT_ENABLE
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, I2C1_IT_PRIORITY, I2C1_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
#endif /* I2C1_IT_ENABLE */

#if I2C1_RX_DMA

    CSP_DMA_CLK_ENABLE(I2C1_RX_DMA_NUMBER);

    if (HAL_DMA_Init(&i2c1_dmarx_handle) != HAL_OK) {
        return I2C_INIT_DMA_FAIL;
    }

    __HAL_LINKDMA(&i2c1_handle, hdmarx, i2c1_dmarx_handle);

    HAL_NVIC_SetPriority(I2C1_RX_DMA_IRQn, I2C1_RX_DMA_IT_PRIORITY, I2C1_RX_DMA_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C1_RX_DMA_IRQn);

#endif /* I2C1_RX_DMA */

#if I2C1_TX_DMA

    CSP_DMA_CLK_ENABLE(I2C1_TX_DMA_NUMBER);

    if (HAL_DMA_Init(&i2c1_dmatx_handle) != HAL_OK) {
        return I2C_INIT_DMA_FAIL;
    }

    __HAL_LINKDMA(&i2c1_handle, hdmatx, i2c1_dmatx_handle);

    HAL_NVIC_SetPriority(I2C1_TX_DMA_IRQn, I2C1_TX_DMA_IT_PRIORITY, I2C1_TX_DMA_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C1_TX_DMA_IRQn);

#endif /* I2C1_TX_DMA */

    if (HAL_I2C_Init(&i2c1_handle) != HAL_OK) {
        return I2C_INIT_FAIL;
    }

    return I2C_INIT_OK;
}

#if I2C1_IT_ENABLE

/**
 * @brief I2C1 Event ISR.
 *
 */
void I2C1_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&i2c1_handle);
}

#endif /* I2C1_IT_ENABLE */

#if I2C1_RX_DMA

/**
 * @brief I2C1 RX DMA ISR.
 *
 */
void I2C1_RX_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(&i2c1_dmarx_handle);
}

#endif /* I2C1_RX_DMA */

#if I2C1_TX_DMA

/**
 * @brief I2C1 TX DMA ISR.
 *
 */
void I2C1_TX_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(&i2c1_dmatx_handle);
}

#endif /* I2C1_TX_DMA */

/**
 * @brief I2C1 deinitialization.
 *
 * @return I2C1 deinit status.
 *  @retval - 0: `I2C_DEINIT_OK`:       Success.
 *  @retval - 1: `I2C_DEINIT_FAIL`:     I2C deinit failed.
 *  @retval - 2: `I2C_DEINIT_DMA_FAIL`: I2C DMA deinit failed.
 *  @retval - 3: `I2C_NO_INIT`:         I2C is not init.
 */
uint8_t i2c1_deinit(void) {
    if (HAL_I2C_GetState(&i2c1_handle) == HAL_I2C_STATE_RESET) {
        return I2C_NO_INIT;
    }

    __HAL_RCC_I2C1_CLK_DISABLE();

    HAL_GPIO_DeInit(CSP_GPIO_PORT(I2C1_SCL_PORT), I2C1_SCL_PIN);
    HAL_GPIO_DeInit(CSP_GPIO_PORT(I2C1_SDA_PORT), I2C1_SDA_PIN);


#if I2C1_IT_ENABLE
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
#endif /* I2C1_IT_ENABLE */

#if I2C1_RX_DMA
    HAL_DMA_Abort(&i2c1_dmarx_handle);
    if (HAL_DMA_DeInit(&i2c1_dmarx_handle) != HAL_OK) {
        return I2C_DEINIT_DMA_FAIL;
    }

    HAL_NVIC_DisableIRQ(I2C1_RX_DMA_IRQn);
    i2c1_handle.hdmarx = NULL;
#endif /* I2C1_RX_DMA */

#if I2C1_TX_DMA
    HAL_DMA_Abort(&i2c1_dmatx_handle);
    if (HAL_DMA_DeInit(&i2c1_dmatx_handle) != HAL_OK) {
        return I2C_DEINIT_DMA_FAIL;
    }

    HAL_NVIC_DisableIRQ(I2C1_TX_DMA_IRQn);
    i2c1_handle.hdmatx = NULL;
#endif /* I2C1_TX_DMA */

    if (HAL_I2C_DeInit(&i2c1_handle) != HAL_OK) {
        return I2C_DEINIT_FAIL;
    }

    return I2C_DEINIT_OK;
}

#endif /* I2C1_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup I2C2 Functions
 * @{
 */

#if I2C2_ENABLE

I2C_HandleTypeDef i2c2_handle = {
    .Instance = I2C2,
    .Init = {.DualAddressMode = I2C_DUALADDRESS_DISABLE,
             .GeneralCallMode = I2C_GENERALCALL_DISABLE,
             .NoStretchMode = I2C_NOSTRETCH_DISABLE}};

#if I2C2_RX_DMA
static DMA_HandleTypeDef i2c2_dmarx_handle = {
    .Instance = CSP_DMA_STREAM(I2C2_RX_DMA_NUMBER, I2C2_RX_DMA_STREAM),
    .Init = {.Channel = CSP_DMA_CHANNEL(I2C2_RX_DMA_CHANNEL),
             .FIFOMode = DMA_FIFOMODE_DISABLE,
             .Direction = DMA_PERIPH_TO_MEMORY,
             .PeriphInc = DMA_PINC_DISABLE,
             .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
             .MemInc = DMA_MINC_ENABLE,
             .MemDataAlignment = DMA_MDATAALIGN_BYTE,
             .Mode = DMA_NORMAL,
             .Priority = I2C2_RX_DMA_PRIORITY}};
#endif /* I2C2_RX_DMA */

#if I2C2_TX_DMA
static DMA_HandleTypeDef i2c2_dmatx_handle = {
    .Instance = CSP_DMA_STREAM(I2C2_TX_DMA_NUMBER, I2C2_TX_DMA_STREAM),
    .Init = {.Channel = CSP_DMA_CHANNEL(I2C2_TX_DMA_CHANNEL),
             .FIFOMode = DMA_FIFOMODE_DISABLE,
             .Direction = DMA_MEMORY_TO_PERIPH,
             .PeriphInc = DMA_PINC_DISABLE,
             .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
             .MemInc = DMA_MINC_ENABLE,
             .MemDataAlignment = DMA_MDATAALIGN_BYTE,
             .Mode = DMA_NORMAL,
             .Priority = I2C2_TX_DMA_PRIORITY}};
#endif /* I2C2_TX_DMA */

/**
 * @brief I2C2 initialization.
 *
 * @param clock_speed The speed of I2C communication. Unit: kHz.
 * @param address The address of this device, 7-bit or 10-bit.
 * @param address_mode Specific the address length, this parameter can ref
 *                    `I2C_addressing_mode`.
 * @return I2C init status.
 *  @retval - 0: `I2C_INIT_OK`:       Success.
 *  @retval - 1: `I2C_INIT_FAIL`:     I2C init failed.
 *  @retval - 2: `I2C_INIT_DMA_FAIL`: I2C DMA init failed.
 *  @retval - 3: `I2C_INITED`:        I2C is inited.
 */
uint8_t i2c2_init(uint32_t clock_speed, uint32_t address,
                  uint32_t address_mode) {
    if (HAL_I2C_GetState(&i2c2_handle) != HAL_I2C_STATE_RESET) {
        return I2C_INITED;
    }

    GPIO_InitTypeDef gpio_init_struct = {.Mode = GPIO_MODE_AF_OD,
                                         .Pull = GPIO_PULLUP,
                                         .Speed = GPIO_SPEED_FREQ_VERY_HIGH};

    i2c2_handle.Init.ClockSpeed = clock_speed * 1000;
    i2c2_handle.Init.AddressingMode = address_mode;
    i2c2_handle.Init.OwnAddress1 = address;


    CSP_GPIO_CLK_ENABLE(I2C2_SCL_PORT);
    gpio_init_struct.Pin = I2C2_SCL_PIN;
    gpio_init_struct.Alternate = I2C2_SCL_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(I2C2_SCL_PORT), &gpio_init_struct);

    CSP_GPIO_CLK_ENABLE(I2C2_SDA_PORT);
    gpio_init_struct.Pin = I2C2_SDA_PIN;
    gpio_init_struct.Alternate = I2C2_SDA_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(I2C2_SDA_PORT), &gpio_init_struct);

    __HAL_RCC_I2C2_CLK_ENABLE();

#if I2C2_IT_ENABLE
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, I2C2_IT_PRIORITY, I2C2_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
#endif /* I2C2_IT_ENABLE */

#if I2C2_RX_DMA

    CSP_DMA_CLK_ENABLE(I2C2_RX_DMA_NUMBER);

    if (HAL_DMA_Init(&i2c2_dmarx_handle) != HAL_OK) {
        return I2C_INIT_DMA_FAIL;
    }

    __HAL_LINKDMA(&i2c2_handle, hdmarx, i2c2_dmarx_handle);

    HAL_NVIC_SetPriority(I2C2_RX_DMA_IRQn, I2C2_RX_DMA_IT_PRIORITY, I2C2_RX_DMA_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C2_RX_DMA_IRQn);

#endif /* I2C2_RX_DMA */

#if I2C2_TX_DMA

    CSP_DMA_CLK_ENABLE(I2C2_TX_DMA_NUMBER);

    if (HAL_DMA_Init(&i2c2_dmatx_handle) != HAL_OK) {
        return I2C_INIT_DMA_FAIL;
    }

    __HAL_LINKDMA(&i2c2_handle, hdmatx, i2c2_dmatx_handle);

    HAL_NVIC_SetPriority(I2C2_TX_DMA_IRQn, I2C2_TX_DMA_IT_PRIORITY, I2C2_TX_DMA_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C2_TX_DMA_IRQn);

#endif /* I2C2_TX_DMA */

    if (HAL_I2C_Init(&i2c2_handle) != HAL_OK) {
        return I2C_INIT_FAIL;
    }

    return I2C_INIT_OK;
}

#if I2C2_IT_ENABLE

/**
 * @brief I2C2 Event ISR.
 *
 */
void I2C2_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&i2c2_handle);
}

#endif /* I2C2_IT_ENABLE */

#if I2C2_RX_DMA

/**
 * @brief I2C2 RX DMA ISR.
 *
 */
void I2C2_RX_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(&i2c2_dmarx_handle);
}

#endif /* I2C2_RX_DMA */

#if I2C2_TX_DMA

/**
 * @brief I2C2 TX DMA ISR.
 *
 */
void I2C2_TX_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(&i2c2_dmatx_handle);
}

#endif /* I2C2_TX_DMA */

/**
 * @brief I2C2 deinitialization.
 *
 * @return I2C2 deinit status.
 *  @retval - 0: `I2C_DEINIT_OK`:       Success.
 *  @retval - 1: `I2C_DEINIT_FAIL`:     I2C deinit failed.
 *  @retval - 2: `I2C_DEINIT_DMA_FAIL`: I2C DMA deinit failed.
 *  @retval - 3: `I2C_NO_INIT`:         I2C is not init.
 */
uint8_t i2c2_deinit(void) {
    if (HAL_I2C_GetState(&i2c2_handle) == HAL_I2C_STATE_RESET) {
        return I2C_NO_INIT;
    }

    __HAL_RCC_I2C2_CLK_DISABLE();

    HAL_GPIO_DeInit(CSP_GPIO_PORT(I2C2_SCL_PORT), I2C2_SCL_PIN);
    HAL_GPIO_DeInit(CSP_GPIO_PORT(I2C2_SDA_PORT), I2C2_SDA_PIN);


#if I2C2_IT_ENABLE
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
#endif /* I2C2_IT_ENABLE */

#if I2C2_RX_DMA
    HAL_DMA_Abort(&i2c2_dmarx_handle);
    if (HAL_DMA_DeInit(&i2c2_dmarx_handle) != HAL_OK) {
        return I2C_DEINIT_DMA_FAIL;
    }

    HAL_NVIC_DisableIRQ(I2C2_RX_DMA_IRQn);
    i2c2_handle.hdmarx = NULL;
#endif /* I2C2_RX_DMA */

#if I2C2_TX_DMA
    HAL_DMA_Abort(&i2c2_dmatx_handle);
    if (HAL_DMA_DeInit(&i2c2_dmatx_handle) != HAL_OK) {
        return I2C_DEINIT_DMA_FAIL;
    }

    HAL_NVIC_DisableIRQ(I2C2_TX_DMA_IRQn);
    i2c2_handle.hdmatx = NULL;
#endif /* I2C2_TX_DMA */

    if (HAL_I2C_DeInit(&i2c2_handle) != HAL_OK) {
        return I2C_DEINIT_FAIL;
    }

    return I2C_DEINIT_OK;
}

#endif /* I2C2_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup I2C3 Functions
 * @{
 */

#if I2C3_ENABLE

I2C_HandleTypeDef i2c3_handle = {
    .Instance = I2C3,
    .Init = {.DualAddressMode = I2C_DUALADDRESS_DISABLE,
             .GeneralCallMode = I2C_GENERALCALL_DISABLE,
             .NoStretchMode = I2C_NOSTRETCH_DISABLE}};

#if I2C3_RX_DMA
static DMA_HandleTypeDef i2c3_dmarx_handle = {
    .Instance = CSP_DMA_STREAM(I2C3_RX_DMA_NUMBER, I2C3_RX_DMA_STREAM),
    .Init = {.Channel = CSP_DMA_CHANNEL(I2C3_RX_DMA_CHANNEL),
             .FIFOMode = DMA_FIFOMODE_DISABLE,
             .Direction = DMA_PERIPH_TO_MEMORY,
             .PeriphInc = DMA_PINC_DISABLE,
             .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
             .MemInc = DMA_MINC_ENABLE,
             .MemDataAlignment = DMA_MDATAALIGN_BYTE,
             .Mode = DMA_NORMAL,
             .Priority = I2C3_RX_DMA_PRIORITY}};
#endif /* I2C3_RX_DMA */

#if I2C3_TX_DMA
static DMA_HandleTypeDef i2c3_dmatx_handle = {
    .Instance = CSP_DMA_STREAM(I2C3_TX_DMA_NUMBER, I2C3_TX_DMA_STREAM),
    .Init = {.Channel = CSP_DMA_CHANNEL(I2C3_TX_DMA_CHANNEL),
             .FIFOMode = DMA_FIFOMODE_DISABLE,
             .Direction = DMA_MEMORY_TO_PERIPH,
             .PeriphInc = DMA_PINC_DISABLE,
             .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
             .MemInc = DMA_MINC_ENABLE,
             .MemDataAlignment = DMA_MDATAALIGN_BYTE,
             .Mode = DMA_NORMAL,
             .Priority = I2C3_TX_DMA_PRIORITY}};
#endif /* I2C3_TX_DMA */

/**
 * @brief I2C3 initialization.
 *
 * @param clock_speed The speed of I2C communication. Unit: kHz.
 * @param address The address of this device, 7-bit or 10-bit.
 * @param address_mode Specific the address length, this parameter can ref
 *                    `I2C_addressing_mode`.
 * @return I2C init status.
 *  @retval - 0: `I2C_INIT_OK`:       Success.
 *  @retval - 1: `I2C_INIT_FAIL`:     I2C init failed.
 *  @retval - 2: `I2C_INIT_DMA_FAIL`: I2C DMA init failed.
 *  @retval - 3: `I2C_INITED`:        I2C is inited.
 */
uint8_t i2c3_init(uint32_t clock_speed, uint32_t address,
                  uint32_t address_mode) {
    if (HAL_I2C_GetState(&i2c3_handle) != HAL_I2C_STATE_RESET) {
        return I2C_INITED;
    }

    GPIO_InitTypeDef gpio_init_struct = {.Mode = GPIO_MODE_AF_OD,
                                         .Pull = GPIO_PULLUP,
                                         .Speed = GPIO_SPEED_FREQ_VERY_HIGH};

    i2c3_handle.Init.ClockSpeed = clock_speed * 1000;
    i2c3_handle.Init.AddressingMode = address_mode;
    i2c3_handle.Init.OwnAddress1 = address;


    CSP_GPIO_CLK_ENABLE(I2C3_SCL_PORT);
    gpio_init_struct.Pin = I2C3_SCL_PIN;
    gpio_init_struct.Alternate = I2C3_SCL_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(I2C3_SCL_PORT), &gpio_init_struct);

    CSP_GPIO_CLK_ENABLE(I2C3_SDA_PORT);
    gpio_init_struct.Pin = I2C3_SDA_PIN;
    gpio_init_struct.Alternate = I2C3_SDA_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(I2C3_SDA_PORT), &gpio_init_struct);

    __HAL_RCC_I2C3_CLK_ENABLE();

#if I2C3_IT_ENABLE
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, I2C3_IT_PRIORITY, I2C3_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
#endif /* I2C3_IT_ENABLE */

#if I2C3_RX_DMA

    CSP_DMA_CLK_ENABLE(I2C3_RX_DMA_NUMBER);

    if (HAL_DMA_Init(&i2c3_dmarx_handle) != HAL_OK) {
        return I2C_INIT_DMA_FAIL;
    }

    __HAL_LINKDMA(&i2c3_handle, hdmarx, i2c3_dmarx_handle);

    HAL_NVIC_SetPriority(I2C3_RX_DMA_IRQn, I2C3_RX_DMA_IT_PRIORITY, I2C3_RX_DMA_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C3_RX_DMA_IRQn);

#endif /* I2C3_RX_DMA */

#if I2C3_TX_DMA

    CSP_DMA_CLK_ENABLE(I2C3_TX_DMA_NUMBER);

    if (HAL_DMA_Init(&i2c3_dmatx_handle) != HAL_OK) {
        return I2C_INIT_DMA_FAIL;
    }

    __HAL_LINKDMA(&i2c3_handle, hdmatx, i2c3_dmatx_handle);

    HAL_NVIC_SetPriority(I2C3_TX_DMA_IRQn, I2C3_TX_DMA_IT_PRIORITY, I2C3_TX_DMA_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C3_TX_DMA_IRQn);

#endif /* I2C3_TX_DMA */

    if (HAL_I2C_Init(&i2c3_handle) != HAL_OK) {
        return I2C_INIT_FAIL;
    }

    return I2C_INIT_OK;
}

#if I2C3_IT_ENABLE

/**
 * @brief I2C3 Event ISR.
 *
 */
void I2C3_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&i2c3_handle);
}

#endif /* I2C3_IT_ENABLE */

#if I2C3_RX_DMA

/**
 * @brief I2C3 RX DMA ISR.
 *
 */
void I2C3_RX_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(&i2c3_dmarx_handle);
}

#endif /* I2C3_RX_DMA */

#if I2C3_TX_DMA

/**
 * @brief I2C3 TX DMA ISR.
 *
 */
void I2C3_TX_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(&i2c3_dmatx_handle);
}

#endif /* I2C3_TX_DMA */

/**
 * @brief I2C3 deinitialization.
 *
 * @return I2C3 deinit status.
 *  @retval - 0: `I2C_DEINIT_OK`:       Success.
 *  @retval - 1: `I2C_DEINIT_FAIL`:     I2C deinit failed.
 *  @retval - 2: `I2C_DEINIT_DMA_FAIL`: I2C DMA deinit failed.
 *  @retval - 3: `I2C_NO_INIT`:         I2C is not init.
 */
uint8_t i2c3_deinit(void) {
    if (HAL_I2C_GetState(&i2c3_handle) == HAL_I2C_STATE_RESET) {
        return I2C_NO_INIT;
    }

    __HAL_RCC_I2C3_CLK_DISABLE();

    HAL_GPIO_DeInit(CSP_GPIO_PORT(I2C3_SCL_PORT), I2C3_SCL_PIN);
    HAL_GPIO_DeInit(CSP_GPIO_PORT(I2C3_SDA_PORT), I2C3_SDA_PIN);


#if I2C3_IT_ENABLE
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
#endif /* I2C3_IT_ENABLE */

#if I2C3_RX_DMA
    HAL_DMA_Abort(&i2c3_dmarx_handle);
    if (HAL_DMA_DeInit(&i2c3_dmarx_handle) != HAL_OK) {
        return I2C_DEINIT_DMA_FAIL;
    }

    HAL_NVIC_DisableIRQ(I2C3_RX_DMA_IRQn);
    i2c3_handle.hdmarx = NULL;
#endif /* I2C3_RX_DMA */

#if I2C3_TX_DMA
    HAL_DMA_Abort(&i2c3_dmatx_handle);
    if (HAL_DMA_DeInit(&i2c3_dmatx_handle) != HAL_OK) {
        return I2C_DEINIT_DMA_FAIL;
    }

    HAL_NVIC_DisableIRQ(I2C3_TX_DMA_IRQn);
    i2c3_handle.hdmatx = NULL;
#endif /* I2C3_TX_DMA */

    if (HAL_I2C_DeInit(&i2c3_handle) != HAL_OK) {
        return I2C_DEINIT_FAIL;
    }

    return I2C_DEINIT_OK;
}

#endif /* I2C3_ENABLE */

/**
 * @}
 */

/*****************************************************************************
 * @defgroup I2C4 Functions
 * @{
 */

#if I2C4_ENABLE

I2C_HandleTypeDef i2c4_handle = {
    .Instance = I2C4,
    .Init = {.DualAddressMode = I2C_DUALADDRESS_DISABLE,
             .GeneralCallMode = I2C_GENERALCALL_DISABLE,
             .NoStretchMode = I2C_NOSTRETCH_DISABLE}};

#if I2C4_RX_DMA
static DMA_HandleTypeDef i2c4_dmarx_handle = {
    .Instance = CSP_DMA_STREAM(I2C4_RX_DMA_NUMBER, I2C4_RX_DMA_STREAM),
    .Init = {.Channel = CSP_DMA_CHANNEL(I2C4_RX_DMA_CHANNEL),
             .FIFOMode = DMA_FIFOMODE_DISABLE,
             .Direction = DMA_PERIPH_TO_MEMORY,
             .PeriphInc = DMA_PINC_DISABLE,
             .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
             .MemInc = DMA_MINC_ENABLE,
             .MemDataAlignment = DMA_MDATAALIGN_BYTE,
             .Mode = DMA_NORMAL,
             .Priority = I2C4_RX_DMA_PRIORITY}};
#endif /* I2C4_RX_DMA */

#if I2C4_TX_DMA
static DMA_HandleTypeDef i2c4_dmatx_handle = {
    .Instance = CSP_DMA_STREAM(I2C4_TX_DMA_NUMBER, I2C4_TX_DMA_STREAM),
    .Init = {.Channel = CSP_DMA_CHANNEL(I2C4_TX_DMA_CHANNEL),
             .FIFOMode = DMA_FIFOMODE_DISABLE,
             .Direction = DMA_MEMORY_TO_PERIPH,
             .PeriphInc = DMA_PINC_DISABLE,
             .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
             .MemInc = DMA_MINC_ENABLE,
             .MemDataAlignment = DMA_MDATAALIGN_BYTE,
             .Mode = DMA_NORMAL,
             .Priority = I2C4_TX_DMA_PRIORITY}};
#endif /* I2C4_TX_DMA */

/**
 * @brief I2C4 initialization.
 *
 * @param clock_speed The speed of I2C communication. Unit: kHz.
 * @param address The address of this device, 7-bit or 10-bit.
 * @param address_mode Specific the address length, this parameter can ref
 *                    `I2C_addressing_mode`.
 * @return I2C init status.
 *  @retval - 0: `I2C_INIT_OK`:       Success.
 *  @retval - 1: `I2C_INIT_FAIL`:     I2C init failed.
 *  @retval - 2: `I2C_INIT_DMA_FAIL`: I2C DMA init failed.
 *  @retval - 3: `I2C_INITED`:        I2C is inited.
 */
uint8_t i2c4_init(uint32_t clock_speed, uint32_t address,
                  uint32_t address_mode) {
    if (HAL_I2C_GetState(&i2c4_handle) != HAL_I2C_STATE_RESET) {
        return I2C_INITED;
    }

    GPIO_InitTypeDef gpio_init_struct = {.Mode = GPIO_MODE_AF_OD,
                                         .Pull = GPIO_PULLUP,
                                         .Speed = GPIO_SPEED_FREQ_VERY_HIGH};

    i2c4_handle.Init.ClockSpeed = clock_speed * 1000;
    i2c4_handle.Init.AddressingMode = address_mode;
    i2c4_handle.Init.OwnAddress1 = address;


    CSP_GPIO_CLK_ENABLE(I2C4_SCL_PORT);
    gpio_init_struct.Pin = I2C4_SCL_PIN;
    gpio_init_struct.Alternate = I2C4_SCL_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(I2C4_SCL_PORT), &gpio_init_struct);

    CSP_GPIO_CLK_ENABLE(I2C4_SDA_PORT);
    gpio_init_struct.Pin = I2C4_SDA_PIN;
    gpio_init_struct.Alternate = I2C4_SDA_GPIO_AF;
    HAL_GPIO_Init(CSP_GPIO_PORT(I2C4_SDA_PORT), &gpio_init_struct);

    __HAL_RCC_I2C4_CLK_ENABLE();

#if I2C4_IT_ENABLE
    HAL_NVIC_SetPriority(I2C4_EV_IRQn, I2C4_IT_PRIORITY, I2C4_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);
#endif /* I2C4_IT_ENABLE */

#if I2C4_RX_DMA

    CSP_DMA_CLK_ENABLE(I2C4_RX_DMA_NUMBER);

    if (HAL_DMA_Init(&i2c4_dmarx_handle) != HAL_OK) {
        return I2C_INIT_DMA_FAIL;
    }

    __HAL_LINKDMA(&i2c4_handle, hdmarx, i2c4_dmarx_handle);

    HAL_NVIC_SetPriority(I2C4_RX_DMA_IRQn, I2C4_RX_DMA_IT_PRIORITY, I2C4_RX_DMA_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C4_RX_DMA_IRQn);

#endif /* I2C4_RX_DMA */

#if I2C4_TX_DMA

    CSP_DMA_CLK_ENABLE(I2C4_TX_DMA_NUMBER);

    if (HAL_DMA_Init(&i2c4_dmatx_handle) != HAL_OK) {
        return I2C_INIT_DMA_FAIL;
    }

    __HAL_LINKDMA(&i2c4_handle, hdmatx, i2c4_dmatx_handle);

    HAL_NVIC_SetPriority(I2C4_TX_DMA_IRQn, I2C4_TX_DMA_IT_PRIORITY, I2C4_TX_DMA_IT_SUB);
    HAL_NVIC_EnableIRQ(I2C4_TX_DMA_IRQn);

#endif /* I2C4_TX_DMA */

    if (HAL_I2C_Init(&i2c4_handle) != HAL_OK) {
        return I2C_INIT_FAIL;
    }

    return I2C_INIT_OK;
}

#if I2C4_IT_ENABLE

/**
 * @brief I2C4 Event ISR.
 *
 */
void I2C4_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&i2c4_handle);
}

#endif /* I2C4_IT_ENABLE */

#if I2C4_RX_DMA

/**
 * @brief I2C4 RX DMA ISR.
 *
 */
void I2C4_RX_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(&i2c4_dmarx_handle);
}

#endif /* I2C4_RX_DMA */

#if I2C4_TX_DMA

/**
 * @brief I2C4 TX DMA ISR.
 *
 */
void I2C4_TX_DMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(&i2c4_dmatx_handle);
}

#endif /* I2C4_TX_DMA */

/**
 * @brief I2C4 deinitialization.
 *
 * @return I2C4 deinit status.
 *  @retval - 0: `I2C_DEINIT_OK`:       Success.
 *  @retval - 1: `I2C_DEINIT_FAIL`:     I2C deinit failed.
 *  @retval - 2: `I2C_DEINIT_DMA_FAIL`: I2C DMA deinit failed.
 *  @retval - 3: `I2C_NO_INIT`:         I2C is not init.
 */
uint8_t i2c4_deinit(void) {
    if (HAL_I2C_GetState(&i2c4_handle) == HAL_I2C_STATE_RESET) {
        return I2C_NO_INIT;
    }

    __HAL_RCC_I2C4_CLK_DISABLE();

    HAL_GPIO_DeInit(CSP_GPIO_PORT(I2C4_SCL_PORT), I2C4_SCL_PIN);
    HAL_GPIO_DeInit(CSP_GPIO_PORT(I2C4_SDA_PORT), I2C4_SDA_PIN);


#if I2C4_IT_ENABLE
    HAL_NVIC_DisableIRQ(I2C4_EV_IRQn);
#endif /* I2C4_IT_ENABLE */

#if I2C4_RX_DMA
    HAL_DMA_Abort(&i2c4_dmarx_handle);
    if (HAL_DMA_DeInit(&i2c4_dmarx_handle) != HAL_OK) {
        return I2C_DEINIT_DMA_FAIL;
    }

    HAL_NVIC_DisableIRQ(I2C4_RX_DMA_IRQn);
    i2c4_handle.hdmarx = NULL;
#endif /* I2C4_RX_DMA */

#if I2C4_TX_DMA
    HAL_DMA_Abort(&i2c4_dmatx_handle);
    if (HAL_DMA_DeInit(&i2c4_dmatx_handle) != HAL_OK) {
        return I2C_DEINIT_DMA_FAIL;
    }

    HAL_NVIC_DisableIRQ(I2C4_TX_DMA_IRQn);
    i2c4_handle.hdmatx = NULL;
#endif /* I2C4_TX_DMA */

    if (HAL_I2C_DeInit(&i2c4_handle) != HAL_OK) {
        return I2C_DEINIT_FAIL;
    }

    return I2C_DEINIT_OK;
}

#endif /* I2C4_ENABLE */

/**
 * @}
 */
