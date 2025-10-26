/**
 * @file    action_position.h
 * @author  Deadline039
 * @brief   东大全场定位解析代码
 * @version 0.1
 * @date    2023-11-11
 */

#ifndef __ACTION_POSITION_H
#define __ACTION_POSITION_H

#include <CSP_Config.h>

#include <stdbool.h>

/**
 * @brief 全场定位数据
 */
typedef struct {
    float x;         /*!< x 坐标 */
    float y;         /*!< y 坐标 */
    float roll;      /*!< 横滚角 */
    float pitch;     /*!< 俯仰角 */
    float yaw;       /*!< 偏航角 */
    float yaw_speed; /*!< 偏航角速度 */
    bool recv_cplt;  /*!< 接收完整的一帧数据 */
} act_pos_data_t;

extern act_pos_data_t g_action_pos_data;

void act_position_register_send_uart(UART_HandleTypeDef *huart);
void act_position_parse_data(uint8_t *data, uint32_t len);
void act_position_update_x(float new_x);
void act_position_update_y(float new_y);
void act_position_update_yaw(float new_yaw);
void act_position_reset_data(void);

#endif /* __ACTION_POSITION_H */