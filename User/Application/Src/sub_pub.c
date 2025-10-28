/**
 * @file sub_pub.c
 * @author CV-Engineer-Chen
 * @brief sub-订阅变量值;pub-发布参数.
 *        - 订阅遥控器的值并发布给main-ctrl
 *        - 订阅底盘参数:speedx\speedy\speedz\halt
 *        - 订阅摩擦带参数:5065-speed\云台angle\俯仰angle\shoot-flag
 *        - 订阅运球参数:dribblling_flag
 * @version 0.1
 * @date 2025-04-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "includes.h"
#include "message-protocol/msg_protocol.h"
#include "remote_ctrl/remote_ctrl.h"
#include "action_position/action_position.h"
#include "odometry_string/odometry_string.h"
#include "logger/logger.h"
#include "my_math/my_math.h"

#define ACT_POS_USART_HANDLE &usart6_handle
#define NUC_UART_HANDLE      &uart5_handle
#define REMOTE_UART_HANDLE   &uart4_handle

TaskHandle_t sub_pub_task_handle;
TaskHandle_t msg_polling_task_handle;

float *world_yaw; /* 指向全场定位的坐标 */

float g_basket_radius;
static struct __packed {
    float chassis_speedx;    /* 底盘speedx */
    float chassis_speedy;    /* 底盘speedy */
    float chassis_speedyaw;  /* 底盘speedyaw */
    float chassis_world_yaw; /* 世界坐标yaw */
    bool chassis_halt;       /* 底盘停止标志 */

    float friction_speed_5065; /* 摩擦带速度 */
    uint8_t shoot_flag;        /* 射击标志 */
} pub_to_slave_data;

// nuc_pos_data_t g_nuc_pos_data;

/**
 * @brief 订阅底盘速度
 * 
 * @param speedx x方向速度 
 * @param speedy y方向速度
 * @param speedw w方向速度
 */
void sub_chassis_speed(float speedx, float speedy, float speedw) {
    pub_to_slave_data.chassis_speedx = speedx;
    pub_to_slave_data.chassis_speedy = speedy;
    pub_to_slave_data.chassis_speedyaw = speedw;
}

/**
 * @brief 订阅底盘驻停状态 
 * 
 * @param halt 驻停状态
 */
void sub_chassis_halt(bool halt) {
    pub_to_slave_data.chassis_halt = halt ? 1 : 0;
}

/**
 * @brief 订阅世界坐标指针 
 * 
 * @note 在底盘init处指向指定位置
 * @param yaw_angle 世界坐标下的位置
 */
void sub_chassis_world_yaw(float *yaw_angle) {
    world_yaw = yaw_angle;
}

/**
 * @brief 订阅摩擦轮参数
 * 
 * @param friction_speed 摩擦轮shoot的速度
 * @return * void 
 */
void sub_friction_data(float friction_speed) {
    pub_to_slave_data.friction_speed_5065 = friction_speed;
}

/**
 * @brief 订阅摩擦轮发射状态
 * 
 * @param shoot_flag 标志位shoot：0
 * @return * void 
 */
void sub_friction_flag(uint8_t shoot_flag) {
    pub_to_slave_data.shoot_flag = shoot_flag;
}

typedef enum __attribute((packed)) {
    SERIAL_RELOCALIZATION_STOP,  /* 重定位停止为0 */
    SERIAL_RELOCALIZATION_START, /* 重定位启动为1 */
    SERIAL_START_SERVICE,        /* 启动服务 */
    SERIAL_STOP_SERVICE,         /* 停止服务 */
    SERIAL_RESTART_SERVICE,      /* 重启服务 */
} serial_flag_t;

/**
 * @brief sub-pub一直轮询的任务
 * 
 * @note - 主负板发送任务
 * @param pvParameters 
 */
void sub_pub_task(void *pvParameters) {
    UNUSED(pvParameters);
    serial_flag_t send_nuc_data = SERIAL_STOP_SERVICE;

    /* 初始化pub_to_slave_data,防止被编译器优化*/
    memset(&pub_to_slave_data, 0, sizeof(pub_to_slave_data));
    pub_to_slave_data.chassis_halt = 1;
    /* 主板发送给从板 */
    message_register_send_uart(MSG_TO_SLAVE, &usart2_handle, 128);
    /* F4-小电脑 */
    message_register_send_uart(MSG_NUC, NUC_UART_HANDLE, 32);

    while (1) {
        /* 更新world—yaw数据 */
        pub_to_slave_data.chassis_world_yaw = *world_yaw;
        /* 串口发送数据 */
        message_send_data(MSG_TO_SLAVE, MSG_DATA_CUSTOM,
                          (uint8_t *)&pub_to_slave_data,
                          sizeof(pub_to_slave_data));

        vTaskDelay(2);
    }
}
nuc_pos_data_t g_nuc_pos_data;
/**
 * @brief 小电脑接收回调函数
 * 
 * @param msg_length 消息帧长度
 * @param msg_id_type 消息 ID 和数据类型 (高四位为 ID, 低四位为数据类型)
 * @param[in] msg_data 消息数据接收区
 */
static void nuc_msg_callback(uint32_t msg_length, uint8_t msg_id_type,
                             uint8_t *msg_data) {
    UNUSED(msg_length);
    UNUSED(msg_id_type);
    nuc_pos_data_t temp_data;
    memcpy(&temp_data, msg_data, sizeof(nuc_pos_data_t));

    /* 更新全局变量 */
    g_nuc_pos_data.x = 1000.0f * temp_data.x;
    g_nuc_pos_data.y = 1000.0f * temp_data.y;
    g_nuc_pos_data.yaw = temp_data.yaw;

    LED2_TOGGLE();
}

void msg_polling_task(void *pvParameters) {
    UNUSED(pvParameters);

    /* 注册遥控器接收 */
    message_register_polling_uart(MSG_REMOTE, REMOTE_UART_HANDLE, 32, 32);
    /* 注册遥控器接收callback */
    message_register_recv_callback(MSG_REMOTE, remote_receive_callback);

    /* 注册action接收 */
    // act_position_register_send_uart(ACT_POS_USART_HANDLE);

    /* 注册小电脑接收 */
    message_register_polling_uart(MSG_NUC, NUC_UART_HANDLE, 512, 512);
    message_register_recv_callback(MSG_NUC, nuc_msg_callback);

    /* 遥控器上报数据类型 */
    // remote_report_data_t report_data = REMOTE_REPORT_POSITION;

    while (1) {
        message_polling_data();
        // xQueueSend(remote_report_data_queue, &report_data, 1);
        float delat_x = BASKET_POINT_X - g_nuc_pos_data.x;
        float delat_y = BASKET_POINT_Y - g_nuc_pos_data.y;

        g_basket_radius = sqrtf(delat_x * delat_x + delat_y * delat_y);

        vTaskDelay(2);
    }
}