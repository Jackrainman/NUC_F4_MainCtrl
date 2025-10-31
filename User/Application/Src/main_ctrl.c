/**
 * @file main_ctrl.c
 * @author DIDI
 * @brief 主要控制逻辑,用于给不同模块发布消息,控制总流程
 * @version 0.1
 * @date 2025-04-27
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "includes.h"
#include "remote_ctrl/remote_ctrl.h"
#include "logger/logger.h"
#include "../Utils/my_math/my_math.h"
#define RADIUM_CTRL              0

#define MAIN_CTRL_MIN_RADIUM_KEY 17 /* 自动跑环按键，暂时不用 */
#define MAIN_CTRL_AUTO_POINT_RUN 16 /* 自动跑点运行 */

typedef enum {
    MAIN_CTRL_RADIUM,     /* 自动跑环信号 */
    MAIN_CTRL_POINT_RUN   /* 自动跑点信号 */
} main_ctrl_queue_t;

TaskHandle_t main_ctrl_task_handle;
QueueHandle_t main_ctrl_queue;


/* [半径][转速] 数组，半径小于3100为三分线内的点 */
/* 7.11比赛场地测试的参数 */
const float radium_speed[LOOP_NUM][2] = {
    {2000, 13600 /**/}, {2100, 13450 /**/}, {2200, 13600 /**/},
    {2400, 13800 /**/}, {2550, 14100 /**/}, {2700, 14300},
    {2850, 14500 /**/}, {3000, 14900 /**/}, {3400, 15400},
    {3600, 15800},      {3900, 16300 /**/}, {4200, 17200},
    {4500, 17500 /**/}, {4800, 18200},      {5100, 18500 /**/},
    {5400, 19300},      {5700, 19700 /**/}, {6000, 20200}};


// /* [半径][转速] 数组 */
// const float radium_speed[LOOP_NUM][2] = {
//     {2400, 14800}, {2700, 14300 /**/}, {3000, 14800}, {3400, 15800},
//     {3600, 16200}, {3900, 16400 /**/}, {4200, 17000}, {4500, 17200 /**/},
//     {4800, 18200}, {5100, 18500 /**/}, {5400, 19500}, {5700, 19800 /**/},
//     {6000, 20800}};

uint8_t min_index_return(float radium) {
    float min_abs_radium = my_fabs(radium - radium_speed[0][0]);
    uint8_t min_index = 0;

    for (uint8_t i = 1; i < LOOP_NUM; i++) {
        float current_abs_radium = my_fabs(radium - radium_speed[i][0]);
        if (current_abs_radium < min_abs_radium) {
            min_abs_radium = current_abs_radium;
            min_index = i;
        }
    }

#if RADIUM_CTRL /* 这个宏打开的时候右手遥感y轴可以控制半径的选择,默认选择后面一环 */
    if (g_basket_radius >= min_abs_radium) {
        if (g_remote_ctrl_data.rs[3] > 5) {
            min_index = min_index;
        } else {
            min_index++;
        }
    } else {
        if (g_remote_ctrl_data.rs[3] > 5) {
            min_index--;
        } else {
            min_index = min_index;
        }
    }
#endif
    if (min_index >= (LOOP_NUM - 1)) {
        min_index = (LOOP_NUM - 1); /* 确保索引不越界 */
    }
    if ((int8_t)min_index < 0) {
        /* 因为min_index是uint8_t类型的数据,当数据为-1时是255,应当强转为int8进行判定 */
        min_index = 0; /* 确保索引不越界 */
    }
    return min_index;
}

void main_ctrl_task(void *pvParameters) {
    UNUSED(pvParameters);
    main_ctrl_queue_t received_message;
    static uint8_t index;
    static uint8_t point_index = 0;  // 静态变量，记录当前点位索引
    while (1) {
        if (xQueueReceive(main_ctrl_queue, &received_message, portMAX_DELAY) ==
            pdPASS) {
            switch (received_message) {
                case MAIN_CTRL_RADIUM:
                    /* 运球回收篮球 */
                    // dribble_set_ctrl(DRIBBLE_HANDLEOVER_BALL);
                    /* 计算最小半径 */
                    index = min_index_return(g_basket_radius);
                    /* 设置目标半径 */
                    index = chassis_overwrite_pointarray(index);
                    /* 摩擦轮同时准备转动 */
                    shoot_machine_set_ctrl(radium_speed[index][1],
                                           SHOOT_MACHINE_EVENT_FRIBELT_DIRECT);
                    /* 设置底盘控制任务  */
                    chassis_set_ctrl(CHASSIS_SET_MIN_RADIUM);
                    break;

                case MAIN_CTRL_POINT_RUN:                     //修改部分
                    point_index = (point_index + 1) % 4;
                    chassis_state.point_index = point_index;  // 直接设置索引
                    chassis_set_ctrl(CHASSIS_SET_POINT);      // 触发跑点
                    break;

                default:
                    break;
            }
        }
        vTaskDelay(1);
    }
}

void key_main_ctrl(uint8_t key, remote_key_event_t event) {
    UNUSED(event);

    main_ctrl_queue_t send_msg;
    switch (key) {
        case MAIN_CTRL_MIN_RADIUM_KEY:
            send_msg = MAIN_CTRL_RADIUM;
            break;
        case MAIN_CTRL_AUTO_POINT_RUN:
            send_msg = MAIN_CTRL_POINT_RUN;
            break;
        default:
            break;
    }
    xQueueOverwrite(main_ctrl_queue, &send_msg);
}
void main_ctrl_init(void) {
    main_ctrl_queue = xQueueCreate(1, sizeof(main_ctrl_queue_t));
    if (main_ctrl_queue == NULL) {
        log_message(LOG_ERROR, "main_ctrl_queue create failed");
        return;
    }

    xTaskCreate(main_ctrl_task, "main_ctrl_task", 512, NULL, 4,
                &main_ctrl_task_handle);

    /* 表演时注释掉跑环，仅跑点 */
    // remote_register_key_callback(MAIN_CTRL_MIN_RADIUM_KEY,
    //                              REMOTE_KEY_PRESS_DOWN, key_main_ctrl);

    remote_register_key_callback(MAIN_CTRL_AUTO_POINT_RUN,
                                  REMOTE_KEY_PRESS_DOWN, key_main_ctrl);
}
