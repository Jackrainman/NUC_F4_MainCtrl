/**
 * @file    beep.h
 * @author  Deadline039
 * @brief   On board beep.
 * @version 1.0
 * @date    2025-05-01
 */

#include "beep.h"

/**
 * @brief Beep initialization.
 *
 */
void beep_init(void) {
    GPIO_InitTypeDef gpio_initure = {.Mode = GPIO_MODE_OUTPUT_PP,
                                     .Pull = GPIO_PULLUP,
                                     .Speed = GPIO_SPEED_FREQ_HIGH,
                                     .Pin = BEEP_GPIO_PIN};

    CSP_GPIO_CLK_ENABLE(BEEP_GPIO_PORT);
    HAL_GPIO_Init(CSP_GPIO_PORT(BEEP_GPIO_PORT), &gpio_initure);
    BEEP_OFF();
}
