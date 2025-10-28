/**
 * @file    remote_report.c
 * @author  Deadline039
 * @brief   遥控器数据上报
 * @version 1.0
 * @date    2025-05-04
 */

#include "includes.h"
#include "message-protocol/msg_protocol.h"
#include "action_position/action_position.h"
#include "logger/logger.h"

/**
 * @brief 位置信息上报
 */
static struct __packed {
    remote_report_data_t type; /*!< 上报的数据类型: 位置 */
    int16_t x;                 /*!< x 坐标 */
    int16_t y;                 /*!< y 坐标 */
    int16_t yaw;               /*!< yaw 坐标 */
    uint8_t point_index;          /*!< 目标点序列号 */

    /*!< 底盘状态 
     * bit[7:3] 保留
     * bit[2]   是否自锁
     * bit[1]   是否在自瞄状态
     * bit[0]   是否世界坐标系
     */
    uint8_t chassis_status;
} report_chassis = {.type = REMOTE_REPORT_POSITION};

/**
 * @brief 发射信息上报
 */
static struct __packed {
    remote_report_data_t type; /*!< 上报的数据类型: 位置 */
    float shoot_spd;           /*!< 发射速度 */

    /*!< 发射状态 
     * bit[7:2] 保留
     * bit[1]   2006是否推出
     * bit[0]   是否在发射状态
     */
    uint8_t shoot_status;
} report_shoot = {.type = REMOTE_REPORT_SHOOT};

#define PUSH_POS  1
#define PUSH_MSK  (1U << PUSH_POS)

#define SHOOT_POS 0
#define SHOOT_MSK (1U << SHOOT_POS)

#define HALT_POS  2
#define HALT_MSK  (1U << HALT_POS)

#define FOCUS_POS 1
#define FOCUS_MSK (1U << FOCUS_POS)

#define WORLD_POS 0
#define WORLD_MSK (1U << WORLD_POS)

TaskHandle_t remote_report_task_handle;
/* 要上报的数据类型, 值为`remote_report_data_t`的值 */
QueueHandle_t remote_report_data_queue;

/**
 * @brief 上报数据任务, 需要给`remote_report_data_queue`发数据才会上报
 * 
 * @param pvParameters 
 */
void remote_report_task(void *pvParameters) {
    UNUSED(pvParameters);
    remote_report_data_queue = xQueueCreate(1, sizeof(remote_report_data_t));
    message_register_send_uart(MSG_REMOTE, &uart4_handle, 50);

    remote_report_data_t report_data;

    while (1) {
        xQueueReceive(remote_report_data_queue, &report_data, portMAX_DELAY);

        switch (report_data) {
            case REMOTE_REPORT_POSITION: {
                /* 更新位置信息 */
                report_chassis.x = (int16_t)g_nuc_pos_data.x;
                report_chassis.y = (int16_t)g_nuc_pos_data.y;
                report_chassis.yaw = (int16_t)g_nuc_pos_data.yaw;
                // report_chassis.x = (int16_t)(HAL_GetTick()/8);
                // report_chassis.y = (int16_t)(HAL_GetTick()/2);
                // report_chassis.yaw = (int16_t)(HAL_GetTick()/100);
                report_chassis.point_index = chassis_state.point_index;
                if (chassis_state.collimation_flag) {
                    SET_BIT(report_chassis.chassis_status, FOCUS_MSK);
                } else {
                    CLEAR_BIT(report_chassis.chassis_status, FOCUS_MSK);
                }
                if (chassis_state.yaw_flag) {
                    SET_BIT(report_chassis.chassis_status, WORLD_MSK);
                } else {
                    CLEAR_BIT(report_chassis.chassis_status, WORLD_MSK);
                }
                if (chassis_state.halt_flag) {
                    SET_BIT(report_chassis.chassis_status, HALT_MSK);
                } else {
                    CLEAR_BIT(report_chassis.chassis_status, HALT_MSK);
                }
                message_send_data(MSG_REMOTE, MSG_DATA_CUSTOM,
                                  (uint8_t *)&report_chassis,
                                  sizeof(report_chassis));
            } break;

            case REMOTE_REPORT_SHOOT: {
                /* 更新发射信息 */
                report_shoot.shoot_spd = shoot_sub.shoot_speed;
                if (shoot_sub.flag == 1) {
                    SET_BIT(report_shoot.shoot_status, PUSH_MSK);
                } else {
                    CLEAR_BIT(report_shoot.shoot_status, PUSH_MSK);
                }
                if (shoot_sub.flag) {
                    SET_BIT(report_shoot.shoot_status, SHOOT_MSK);
                } else {
                    CLEAR_BIT(report_shoot.shoot_status, SHOOT_MSK);
                }
                message_send_data(MSG_REMOTE, MSG_DATA_CUSTOM,
                                  (uint8_t *)&report_shoot,
                                  sizeof(report_shoot));
            } break;

            default: {
                log_message(LOG_ERROR,
                            "[Remote report] Unknow data to report. ");
            } break;
        }
    }
}
