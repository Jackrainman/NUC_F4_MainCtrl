/**
 * @file cylinder.h
 * @author CV-Engineer-Chen
 * @brief On board 气缸gpio控制
 * @version 0.1
 * @date 2025-03-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <CSP_Config.h>

#ifndef __CYLINDER_H
#define __CYLINDER_H

/* top ************************************************** */
#define CYLINDER_TOP_PORT        E
#define CYLINDER_TOP_PIN         GPIO_PIN_2
/* 气缸初始状态的电平(即气缸关闭状态的gpio电平) */ 
// #define CYLINDER_TOP_INIT_STATU  GPIO_PIN_SET
#define CYLINDER_TOP_INIT_STATU  GPIO_PIN_RESET

/* CLAMP ************************************************* */
#define CYLINDER_CLAMP_PORT        E
#define CYLINDER_CLAMP_PIN         GPIO_PIN_4
/* 气缸初始状态的电平 */
#define CYLINDER_CLAMP_INIT_STATU  GPIO_PIN_RESET
// #define CYLINDER_CLAMP_INIT_STATU  GPIO_PIN_SET

/* PUSH ************************************************ */
#define CYLINDER_PUSH_PORT       E
#define CYLINDER_PUSH_PIN        GPIO_PIN_3
/* 设置气缸初始状态的电平 */
#define CYLINDER_PUSH_INIT_STATU GPIO_PIN_RESET

/**
 * @todo 不确定on与off是高电平还是低电平 
 */

/* TOP */
#define CYLINDER_TOP_ON()                                                     \
    HAL_GPIO_WritePin(CSP_GPIO_PORT(CYLINDER_TOP_PORT), CYLINDER_TOP_PIN,    \
                      !(CYLINDER_TOP_INIT_STATU))
#define CYLINDER_TOP_OFF()                                                    \
    HAL_GPIO_WritePin(CSP_GPIO_PORT(CYLINDER_TOP_PORT), CYLINDER_TOP_PIN,    \
                      CYLINDER_TOP_INIT_STATU)
#define CYLINFER_TOP_TOGGLE()                                                 \
    HAL_GPIO_TogglePin(CSP_GPIO_PORT(CYLINDER_TOP_PORT), CYLINDER_TOP_PIN)

/* LEFT */
#define CYLINDER_CLAMP_ON()                                                     \
    HAL_GPIO_WritePin(CSP_GPIO_PORT(CYLINDER_CLAMP_PORT), CYLINDER_CLAMP_PIN,    \
                      !(CYLINDER_CLAMP_INIT_STATU))
#define CYLINDER_CLAMP_OFF()                                                    \
    HAL_GPIO_WritePin(CSP_GPIO_PORT(CYLINDER_CLAMP_PORT), CYLINDER_CLAMP_PIN,    \
                      CYLINDER_CLAMP_INIT_STATU)
#define CYLINFER_CLAMP_TOGGLE()                                                 \
    HAL_GPIO_TogglePin(CSP_GPIO_PORT(CYLINDER_CLAMP_PORT), CYLINDER_CLAMP_PIN)

/* RIGHT */
#define CYLINDER_PUSH_ON()                                                    \
    HAL_GPIO_WritePin(CSP_GPIO_PORT(CYLINDER_PUSH_PORT), CYLINDER_PUSH_PIN,  \
                      !(CYLINDER_PUSH_INIT_STATU))
#define CYLINDER_PUSH_OFF()                                                   \
    HAL_GPIO_WritePin(CSP_GPIO_PORT(CYLINDER_PUSH_PORT), CYLINDER_PUSH_PIN,  \
                      CYLINDER_PUSH_INIT_STATU)
#define CYLINFER_PUSH_TOGGLE()                                                \
    HAL_GPIO_TogglePin(CSP_GPIO_PORT(CYLINDER_PUSH_PORT), CYLINDER_PUSH_PIN)

void cylinder_init(void);

#endif // !__CYLINDER_H
