/**
 * @file    remote_ctrl.h
 * @author  Deadline039
 * @brief   遥控器接收处理
 * @version 1.0
 * @date    2024-04-20
 */

#ifndef __REMOTE_CTRL_H
#define __REMOTE_CTRL_H

#include <bsp.h>
#include "message-protocol/msg_protocol.h"

/**
 * @brief 遥控器键盘事件
 */
typedef enum {
    REMOTE_KEY_PRESS_DOWN, /*!< 按键按下 */
    REMOTE_KEY_PRESSING,   /*!< 按键长按 */
    REMOTE_KEY_PRESS_UP,   /*!< 按键抬起 */
    REMOTE_KEY_EVENT_NUM   /*!< 保留长度 */
} remote_key_event_t;

/**
 * @brief 遥控器键盘回调函数
 * 
 * @param key 按键
 * @param event 事件
 */
typedef void (*remote_key_callback_t)(uint8_t key, remote_key_event_t event);

/**
 * @brief 遥控器数据
 */
typedef struct __packed {
    uint8_t key;  /*!< 按键值 */
    int8_t rs[4]; /*!< 摇杆, 左 x, 左 y, 右 x, 右 y */
} remote_ctrl_data_t;

extern remote_ctrl_data_t g_remote_ctrl_data;

void remote_receive_callback(uint32_t msg_length, uint8_t msg_type,
                             uint8_t *msg_data);
void remote_register_key_callback(uint8_t key, remote_key_event_t event,
                                  remote_key_callback_t callback);
void remote_unregister_key_callback(uint8_t key, remote_key_event_t event);

#endif /* __REMOTE_CTRL_H */