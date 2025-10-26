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
#define RADIUM_CTRL               1 /* 这个宏打开的时候右手遥感y轴可以控制半径的选择 */

#define MAIN_CTRL_MIN_RADIUM_KEY  4  /* 自动跑环按键 */
#define MAIN_CTRL_RIGHT_POINT_KEY 18 /* 右侧自动装球跑点 */
#define MAIN_CTRL_LEFT_POINT_KEY  17 /* 左侧自动装球跑点 */
#define MAIN_CTRL_SMALL_LOOP_KEY  5  /* 切换小环按键 */
#define MAIN_CTRL_BIG_LOOP_KEY    11 /* 切换大环按键 */
//#define MAIN_CTRL_AUTO_POINT_KEY  4

typedef enum {
    MAIN_CTRL_RADIUM,     /* 自动跑环信号 */
    MAIN_CTRL_LEFT_BALL,  /* 右侧自动装球跑点信号 */
    MAIN_CTRL_RIGHT_BALL, /* 左侧自动装球跑点信号 */
    MAIN_CTRL_SMALL_LOOP, /* 切换小环信号 */
    MAIN_CTRL_BIG_LOOP    /* 切换大环信号 */
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


/* RED2 172-4 */
/* [半径][转速] 数组 */ /* 巨匠里我们的场测试的参数 */
// const float radium_speed[LOOP_NUM][2] = {
//     {2000, 14500 /**/}, {2100, 14450 /**/}, {2200, 14100 /**/},
//     {2400, 14800 /**/}, {2550, 14100 /**/}, {2700, 14300},
//     {2850, 14500 /**/}, {3000, 14800 /**/}, {3400, 15800},
//     {3600, 16200},      {3900, 16400 /**/}, {4200, 17000},
//     {4500, 17200 /**/}, {4800, 18200},      {5100, 18500 /**/},
//     {5400, 19500},      {5700, 19800 /**/}, {6000, 20800}};

/**
 * @brief 圆环索引返回函数
 *
 * @param radium 距离目标点的半径
 * @return 圆环在数组中的索引
 */
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

#if RADIUM_CTRL /* 遥感控制大小环的选择 */
    if (g_basket_radius >= min_abs_radium) {
        if (g_remote_ctrl_data.rs[3] > 5) {
            min_index = min_index;
        } else if (g_remote_ctrl_data.rs[3] < -5) {
            min_index++;
        }
    } else {
        if (g_remote_ctrl_data.rs[3] > 5) {
            min_index--;
        } else if (g_remote_ctrl_data.rs[3] < -5) {
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

/**
 * @brief 主控任务
 * @note 该任务用于接收主控队列中的消息,并根据消息类型进行相应的处理
 *
 * @param pvParameters
 */
void main_ctrl_task(void *pvParameters) {
    UNUSED(pvParameters);
    main_ctrl_queue_t received_message;
    static uint8_t index;
    while (1) {
        if (xQueueReceive(main_ctrl_queue, &received_message, portMAX_DELAY) ==
            pdPASS) {
            switch (received_message) {
                case MAIN_CTRL_RADIUM:
                    /* 计算最小半径 */
                    index = min_index_return(g_basket_radius);
                    /* 设置目标半径 */
                    index = chassis_overwrite_pointarray(index);
                    /* 摩擦轮同时准备转动 */
                    shoot_machine_set_ctrl(radium_speed[index][1],
                                           SHOOT_MACHINE_EVENT_FRIBELT_PRE);
                    /* 设置底盘控制任务  */
                    chassis_set_ctrl(CHASSIS_SET_MIN_RADIUM);

                    break;
                case MAIN_CTRL_LEFT_BALL:
                    /* 设置底盘控制任务  */
                    chassis_set_ctrl(CHASSIS_SET_MANUAL);
                    chassis_set_ctrl(CHASSIS_LEFT_AIMING);
                    // dribble_set_ctrl(DRIBBLE_PUSH_OUT);
                    shoot_machine_set_ctrl(0.0f, SHOOT_MACHINE_EVENT_DISABLE);
                    vTaskDelay(500);
                    /* 清除自瞄表示位防止到点后剧烈转动 */
                    chassis_set_ctrl(CHASSIS_RESET_AIMING);
                    // dribble_set_ctrl(DRIBBLE_PUSH_IN);
                    /* 反转装球 */
                    shoot_machine_set_ctrl(-2800.0f,
                                           SHOOT_MACHINE_EVENT_LOAD_BALL);
                    break;
                case MAIN_CTRL_RIGHT_BALL:
                    /* 设置底盘控制任务  */
                    chassis_set_ctrl(CHASSIS_SET_MANUAL);
                    chassis_set_ctrl(CHASSIS_RIGHT_AIMING);
                    // dribble_set_ctrl(DRIBBLE_PUSH_OUT);
                    shoot_machine_set_ctrl(0.0f, SHOOT_MACHINE_EVENT_DISABLE);
                    vTaskDelay(500);
                    /* 清除自瞄表示位防止到点后剧烈转动 */
                    chassis_set_ctrl(CHASSIS_RESET_AIMING);
                    vTaskDelay(500);
                    // dribble_set_ctrl(DRIBBLE_PUSH_IN);
                    /* 反转装球 */
                    shoot_machine_set_ctrl(-2800.0f,
                                           SHOOT_MACHINE_EVENT_LOAD_BALL);
                    break;
                case MAIN_CTRL_BIG_LOOP:
                    index++;
                    if (index >= (LOOP_NUM - 1)) {
                        index = (LOOP_NUM - 1);
                    }
                    /* 设置目标半径 */
                    index = chassis_overwrite_pointarray(index);
                    /* 摩擦轮同时准备转动 */
                    shoot_machine_set_ctrl(radium_speed[index][1],
                                           SHOOT_MACHINE_EVENT_FRIBELT_DIRECT);
                    /* 设置底盘控制任务  */
                    chassis_set_ctrl(CHASSIS_SET_MIN_RADIUM);
                    break;
                case MAIN_CTRL_SMALL_LOOP:
                    index--;
                    if ((int8_t)index <= 0) {
                        index = 0;
                    }
                    /* 设置目标半径 */
                    index = chassis_overwrite_pointarray(index);
                    /* 摩擦轮同时准备转动 */
                    shoot_machine_set_ctrl(radium_speed[index][1],
                                           SHOOT_MACHINE_EVENT_FRIBELT_DIRECT);
                    /* 设置底盘控制任务  */
                    chassis_set_ctrl(CHASSIS_SET_MIN_RADIUM);
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(1);
    }
}

/**
 * @brief 设置主控按键函数
 * @note 该函数用于处理遥控器按键事件,并将相应的消息发送到主控队列中
 *
 * @param key
 * @param event
 */
void set_main_ctrl_key(uint8_t key, remote_key_event_t event) {
    UNUSED(event);

    main_ctrl_queue_t send_msg;
    switch (key) {
        case MAIN_CTRL_MIN_RADIUM_KEY:
            send_msg = MAIN_CTRL_RADIUM;
            break;
        case MAIN_CTRL_RIGHT_POINT_KEY:
            send_msg = MAIN_CTRL_RIGHT_BALL;
            break;
        case MAIN_CTRL_LEFT_POINT_KEY:
            send_msg = MAIN_CTRL_LEFT_BALL;
            break;
        case MAIN_CTRL_SMALL_LOOP_KEY:
            send_msg = MAIN_CTRL_SMALL_LOOP;
            break;
        case MAIN_CTRL_BIG_LOOP_KEY:
            send_msg = MAIN_CTRL_BIG_LOOP;
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

    xTaskCreate(main_ctrl_task, "main_ctrl_task", 256, NULL, 4,
                &main_ctrl_task_handle);
    remote_register_key_callback(MAIN_CTRL_MIN_RADIUM_KEY,
                                 REMOTE_KEY_PRESS_DOWN, set_main_ctrl_key);
    remote_register_key_callback(MAIN_CTRL_LEFT_POINT_KEY, REMOTE_KEY_PRESS_UP,
                                 set_main_ctrl_key);
    remote_register_key_callback(MAIN_CTRL_RIGHT_POINT_KEY, REMOTE_KEY_PRESS_UP,
                                 set_main_ctrl_key);
    remote_register_key_callback(MAIN_CTRL_SMALL_LOOP_KEY, REMOTE_KEY_PRESS_UP,
                                 set_main_ctrl_key);
    remote_register_key_callback(MAIN_CTRL_BIG_LOOP_KEY, REMOTE_KEY_PRESS_UP,
                                 set_main_ctrl_key);
}
