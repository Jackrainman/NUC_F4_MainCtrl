/**
 * @file npn_switch.h
 * @author CV-Engineer-Chen
 * @brief npn型接近开关
 * @version 0.1
 * @date 2025-03-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __NPN_SWITCH_H
#define __NPN_SWITCH_H

#include <CSP_Config.h>

#define NPN_SWITCH_PORT D
#define NPN_SWITCH_PIN  GPIO_PIN_5
#define NPN_SWTICH_PULL GPIO_NOPULL

#define NPN_SWITCH()                                                           \
    HAL_GPIO_ReadPin(CSP_GPIO_PORT(NPN_SWITCH_PORT), NPN_SWITCH_PIN)

#define NPN_SWITCH_PORT_2 D
#define NPN_SWITCH_PIN_2  GPIO_PIN_6
#define NPN_SWTICH_PULL_2 GPIO_NOPULL

#define NPN_SWITCH_2()                                                         \
    HAL_GPIO_ReadPin(CSP_GPIO_PORT(NPN_SWITCH_PORT_2), NPN_SWITCH_PIN_2)

#define NPN_SWITCH_PORT_3 D
#define NPN_SWITCH_PIN_3  GPIO_PIN_7
#define NPN_SWTICH_PULL_3 GPIO_NOPULL

#define NPN_SWITCH_3()                                                         \
    HAL_GPIO_ReadPin(CSP_GPIO_PORT(NPN_SWITCH_PORT_3), NPN_SWITCH_PIN_3)

/* 推出开关 */
#define PROXIMITY_SWITCH_OUT_PORT B
#define PROXIMITY_SWITCH_OUT_PIN  GPIO_PIN_12
#define PROXIMITY_SWITCH_OUT_PULL GPIO_PULLDOWN

#define PROXIMITY_OUT_SWITCH()                                                 \
    HAL_GPIO_ReadPin(CSP_GPIO_PORT(PROXIMITY_SWITCH_OUT_PORT),                 \
                     PROXIMITY_SWITCH_OUT_PIN)

/* 收缩开关 */
#define PROXIMITY_SWITCH_IN_PORT A
#define PROXIMITY_SWITCH_IN_PIN  GPIO_PIN_5
#define PROXIMITY_SWITCH_IN_PULL GPIO_PULLDOWN

#define PROXIMITY_IN_SWITCH()                                                  \
    HAL_GPIO_ReadPin(CSP_GPIO_PORT(PROXIMITY_SWITCH_IN_PORT),                  \
                     PROXIMITY_SWITCH_IN_PIN)

/* 触发光电IO的电平 */
#define NPN_SWITCH_TOUCHED           GPIO_PIN_SET
#define NPN_SWITCH2_TOUCHED          GPIO_PIN_SET
#define NPN_SWITCH3_TOUCHED          GPIO_PIN_SET
/* 触发接近开关的电平 */
#define PROXIMITY_OUT_SWITCH_TOUCHED GPIO_PIN_SET
#define PROXIMITY_IN_SWITCH_TOUCHED  GPIO_PIN_SET

void npn_switch_init(void);

#endif // !__NPN_SWITCH_H
