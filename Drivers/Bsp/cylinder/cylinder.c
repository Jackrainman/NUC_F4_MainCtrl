/**
 * @file cylinder.c
 * @author CV-Engineer-Chen
 * @brief 气缸初始化
 * @version 0.1
 * @date 2025-03-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "cylinder.h"

/**
 * @brief cylinder initialization 
 * 
 */
void cylinder_init(void) {
    GPIO_InitTypeDef gpio_initure = {0};

    /**
     * @todo 电平上拉or下拉需要确定
     * 
     */
    CSP_GPIO_CLK_ENABLE(CYLINDER_CLAMP_PORT);
    gpio_initure.Pin = CYLINDER_CLAMP_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initure.Pull = GPIO_PULLDOWN;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CSP_GPIO_PORT(CYLINDER_CLAMP_PORT), &gpio_initure);

    CSP_GPIO_CLK_ENABLE(CYLINDER_PUSH_PORT);
    gpio_initure.Pin = CYLINDER_PUSH_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initure.Pull = GPIO_PULLDOWN;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CSP_GPIO_PORT(CYLINDER_PUSH_PORT), &gpio_initure);

    CSP_GPIO_CLK_ENABLE(CYLINDER_TOP_PORT);
    gpio_initure.Pin = CYLINDER_TOP_PIN;
    gpio_initure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_initure.Pull = GPIO_PULLDOWN;
    gpio_initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CSP_GPIO_PORT(CYLINDER_TOP_PORT), &gpio_initure);

    CYLINDER_TOP_OFF();
    CYLINDER_CLAMP_OFF();

    // CYLINDER_PUSH_ON();
    CYLINDER_PUSH_OFF();
}

