/**
 * @file odometry_string.c
 * @author PickingChip 
 * @brief 里程计数据接收处理
 * @version 0.1
 * @date 2025-05-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "odometry_string.h"
#include "action_position/action_position.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *recv_uart; /*!<串口句柄*/

odometry_data_t g_odometry_data; /*!<里程计数据*/
float g_delta_x;
float g_delta_y;

/**
 * @brief 里程计解析初始化
 * 
 * @param huart 消息接收串口
 */
void odometry_register_recv_uart(UART_HandleTypeDef *huart) {
    recv_uart = huart;
}

/**
  * @brief 数据解析函数
  * 
  * @param data 
  * @param len 
  */
void odometry_parse_data(uint8_t *data, uint32_t len) {
    /* 解析数据 */
    static char char_buf[59];
    static float odo_data[3];
    static uint32_t recv_len;

    for (uint8_t i = 0; i < len; ++i) {
        if (data[i] == '\n') {
            recv_len = 0;
            sscanf(char_buf, "%f,%f,%f", &odo_data[0], &odo_data[1],
                   &odo_data[2]);
            g_odometry_data.pos_x = 1000.0f * odo_data[0];
            g_odometry_data.pos_y = 1000.0f * odo_data[1];
            g_odometry_data.pos_yaw = odo_data[2];
            /* 计算差值 */
            g_delta_x = g_odometry_data.pos_x - g_action_pos_data.x;
            g_delta_y = g_odometry_data.pos_y - g_action_pos_data.y;
            continue;
        }
        char_buf[recv_len] = data[i];
        ++recv_len;
    }
    memset(data, 0, len); /* 清空数据 */
}
