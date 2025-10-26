/**
 * @file    led.c
 * @author  Deadline039
 * @brief   LEDs function.
 * @version 1.1
 * @date    2023-10-20
 */

#include <CSP_Config.h>

#include "led.h"

/**
 * @brief LEDs initialization.
 *
 */
void led_init(void) {
    GPIO_InitTypeDef gpio_initure = {.Mode = GPIO_MODE_OUTPUT_PP,
                                     .Pull = GPIO_PULLUP,
                                     .Speed = GPIO_SPEED_FREQ_HIGH};

    CSP_GPIO_CLK_ENABLE(LED0_GPIO_PORT);
    gpio_initure.Pin = LED0_GPIO_PIN;
    HAL_GPIO_Init(CSP_GPIO_PORT(LED0_GPIO_PORT), &gpio_initure);
    LED0_OFF();

    CSP_GPIO_CLK_ENABLE(LED1_GPIO_PORT);
    gpio_initure.Pin = LED1_GPIO_PIN;
    HAL_GPIO_Init(CSP_GPIO_PORT(LED1_GPIO_PORT), &gpio_initure);
    LED1_OFF();

    CSP_GPIO_CLK_ENABLE(LED2_GPIO_PORT);
    gpio_initure.Pin = LED2_GPIO_PIN;
    HAL_GPIO_Init(CSP_GPIO_PORT(LED2_GPIO_PORT), &gpio_initure);
    LED2_OFF();

    CSP_GPIO_CLK_ENABLE(LED3_GPIO_PORT);
    gpio_initure.Pin = LED3_GPIO_PIN;
    HAL_GPIO_Init(CSP_GPIO_PORT(LED3_GPIO_PORT), &gpio_initure);
    LED3_OFF();
}
