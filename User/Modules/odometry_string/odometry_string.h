/**
 * @file odometry_string.h
 * @author PickingChip
 * @brief 里程计解析代码
 * @version 0.1
 * @date 2025-05-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ODOMETRY_STRING_H
#define __ODOMETRY_STRING_H
#include <CSP_Config.h>

typedef struct {
    float pos_x;
    float pos_y;
    float pos_yaw;
    float pos_x_l;
    float pos_y_l;
} odometry_data_t;

extern odometry_data_t g_odometry_data; /*!<里程计数据*/
extern float g_delta_x;
extern float g_delta_y;

void odometry_register_recv_uart(UART_HandleTypeDef *huart);
void odometry_parse_data(uint8_t *data, uint32_t len);
void odometry_correct_data(void);
void odometry_delta_correct(void);

#endif //__ODOMETRY_STRING_H