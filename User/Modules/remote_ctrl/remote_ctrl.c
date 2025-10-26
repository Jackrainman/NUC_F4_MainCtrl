/**
 * @file    remote_ctrl.c
 * @author  Deadline039
 * @brief   遥控器
 * @version 0.1
 * @date    2024-04-19
 */

#include "message-protocol/msg_protocol.h"

#include "remote_ctrl.h"
#include <string.h>

/* 遥控器数据 */
remote_ctrl_data_t g_remote_ctrl_data;

/* 18 个按键, 3 个事件 */
static remote_key_callback_t key_callback[18][REMOTE_KEY_EVENT_NUM];

#define KEY_EVENT_CB(key, event)                                               \
    do {                                                                       \
        if ((key <= 18) && key_callback[key - 1][event]) {                     \
            key_callback[key - 1][event](key, event);                          \
        }                                                                      \
    } while (0)

/**
 * @brief 串口遥控器回调
 *
 * @param msg_length 数据长度
 * @param msg_type 数据类型
 * @param msg_data 数据区内容
 */
void remote_receive_callback(uint32_t msg_length, uint8_t msg_type,
                             uint8_t *msg_data) {
    static uint8_t last_key = 0;
    if (msg_length != 5 || msg_type != MSG_DATA_UINT8) {
        return;
    }
    remote_ctrl_data_t *p_msg_data;
    p_msg_data = (remote_ctrl_data_t *)msg_data;
    memcpy(&g_remote_ctrl_data, p_msg_data, sizeof(g_remote_ctrl_data));

    static uint32_t start_time = 0;
    if (HAL_GetTick() - start_time > 500) {
        start_time = HAL_GetTick();
        LED1_TOGGLE();
    }

    if (last_key != 0) {
        if (g_remote_ctrl_data.key == 0) {
            /* 按键抬起 */
            KEY_EVENT_CB(last_key, REMOTE_KEY_PRESS_UP);
        } else if (g_remote_ctrl_data.key == last_key) {
            /* 长按 */
            KEY_EVENT_CB(g_remote_ctrl_data.key, REMOTE_KEY_PRESSING);
        } else {
            /* 抬起一个按键, 按下另一个按键 */
            KEY_EVENT_CB(last_key, REMOTE_KEY_PRESS_UP);
            KEY_EVENT_CB(g_remote_ctrl_data.key, REMOTE_KEY_PRESS_DOWN);
        }
    } else {
        if (g_remote_ctrl_data.key != 0) {
            /* 按键按下 */
            KEY_EVENT_CB(g_remote_ctrl_data.key, REMOTE_KEY_PRESS_DOWN);
        }
    }

    last_key = g_remote_ctrl_data.key;
}

/**
 * @brief 遥控器键盘注册回调函数
 * 
 * @param key 按键
 * @param event 事件
 * @param callback 事件回调函数
 */
void remote_register_key_callback(uint8_t key, remote_key_event_t event,
                                  remote_key_callback_t callback) {
    if (key > 18) {
        return;
    }

    if (event >= REMOTE_KEY_EVENT_NUM) {
        return;
    }

    key_callback[key - 1][event] = callback;
}

/**
 * @brief 遥控器键盘注册回调函数
 * 
 * @param key 按键
 * @param event 事件
 * @param callback 事件回调函数
 */
void remote_unregister_key_callback(uint8_t key, remote_key_event_t event) {
    if (key > 18) {
        return;
    }

    if (event >= REMOTE_KEY_EVENT_NUM) {
        return;
    }

    key_callback[key - 1][event] = NULL;
}