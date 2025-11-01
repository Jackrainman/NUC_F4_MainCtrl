/**
 * @file    includes.h
 * @author  Deadline039
 * @brief   Include files
 * @version 1.0
 * @date    2024-04-03
 */

#ifndef __INCLUDES_H
#define __INCLUDES_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <bsp.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

/* 投篮赛点位 */
#define BASKET_OFFSET_X -16.0f
#define BASKET_OFFSET_Y +20.0f

#define BASKET_POINT_X  (3624.3744f + BASKET_OFFSET_X)
#define BASKET_POINT_Y  (13439.3975f + BASKET_OFFSET_Y)

#define SITH_WIDTH      7400.0f
#define LOOP_NUM        18

extern const float radium_speed[LOOP_NUM][2];
extern float g_basket_radius;

/** **********************************************************************************************
 * @defgroup rtos_task
 * @{
 */

void freertos_start(void);

/*****************************************************************************
 * @defgroup remote_report
 */
typedef enum __packed {
    REMOTE_REPORT_POSITION, /*!< 上报位置信息 */
    REMOTE_REPORT_SHOOT,    /*!< 上报位置信息 */
} remote_report_data_t;

extern TaskHandle_t remote_report_task_handle;
extern QueueHandle_t remote_report_data_queue;
void remote_report_task(void *pvParameters);

/**
 * @} remopet_report
 */

/** **********************************************************************************************
 * @defgroup sub_pub
 * @{
 */
typedef struct {
    float x;
    float y;
    float yaw;
} nuc_pos_data_t;
extern nuc_pos_data_t g_nuc_pos_data;

extern TaskHandle_t sub_pub_task_handle;
extern TaskHandle_t action_position_recv_task_handle;
extern TaskHandle_t msg_polling_task_handle;
void sub_pub_task(void *pvParameters);
void msg_polling_task(void *pvParameters);

void sub_chassis_speed(float speedx, float speedy, float speedw);
void sub_chassis_halt(bool halt);
void sub_chassis_world_yaw(float *yaw_angle);
void sub_friction_flag(uint8_t shoot_flag);
void sub_friction_data(float friction_speed);

/**
 * @} sub_pub
 */

/** **********************************************************************************************
 * @defgroup chassis
 * @{
 */
/* 底盘控制消息队列 */
typedef enum {
    CHASSIS_SET_HALT,       /* 自锁 */
    CHASSIS_SET_UNHALT,     /* 解自锁 */
    CHASSIS_SET_POINT,      /* 跑点 */
    CHASSIS_SET_MANUAL,     /* 手控 */
    CHASSIS_SET_MIN_RADIUM, /* 计算并去往最近的点位,且恢复手控 */

    CHASSIS_SET_NO_TASK
} chassis_event_t;

typedef enum {
    CHASSIS_STATUS_HALT,          /* 底盘自锁 */
    CHASSIS_STATUS_AUTO,          /* 底盘自动跑点 */
    CHASSIS_STATUS_POINT_ARRIVED, /* 底盘跑到 */
    CHASSIS_STATUS_MANUAL,        /* 底盘手控 */

    CHASSIS_STATUS_NUM
} chassis_status_t;

typedef struct {
    uint32_t timestap;     /* 时间戳 */
    chassis_event_t event; /* 底盘控制时间 */
} chassis_ctrl_queue_t;

typedef struct {
    uint32_t timestap;       /* 时间戳 */
    chassis_status_t status; /* 底盘状态 */
} chassis_status_queue_t;

typedef struct chassis_state {
    bool halt_flag;        /*!< 底盘自锁标志位*/
    bool collimation_flag; /*!< 自瞄标志位 */
    bool yaw_flag;         /*!< 坐标系切换标志位 */
    uint8_t point_index;   /*!< 目标点序列号 */
} chassis_state_t;
extern QueueHandle_t chassis_ctrl_queue;   /* 底盘控制队列 */
extern chassis_state_t chassis_state;      /* 底盘状态 */
extern QueueHandle_t chassis_status_queue; /* 底盘状态消息队列 */

void chassis_init(void);
void chassis_set_halt(bool);
// void chassis_ctrl_queue_reset(void);
void chassis_set_ctrl(chassis_event_t event);
uint8_t chassis_overwrite_pointarray(uint8_t target_index);
/**
 * @} chassis
 */

/** **********************************************************************************************
 * @defgroup friction
 * @{
 */

/**
 * @brief  发射运行状态
 * @note 此处的值主要通过`shoot_mahine_status_queue`传递
 */
typedef enum {
    SHOOT_MACHINE_STATUS_PASS_DONE = 1, /* 接球成功 */
    SHOOT_MACHINE_STATUS_PUSH_DONE,     /* 推球成功 */
    SHOOT_MACHINE_STATUS_FRIBELT_DONE,  /* 摩擦带转动 */
    SHOOT_MACHINE_STATUS_FRIBELT_ZERO,  /* 摩擦带静止 */
    SHOOT_MACHINE_STATUS_ENABLE,        /* 使能成功 */
    SHOOT_MACHINE_STATUS_DISABLE,       /* 失能成功 */
} shoot_machine_status_t;

/**
 * @brief 发射状态消息内容 (队列实际传输的内容)
 */
typedef struct {
    uint32_t timestamp;                    /* 时间戳 */
    shoot_machine_status_t fribelt_status; /* 摩擦轮状态 */
    shoot_machine_status_t push_status;    /* 2006状态 */
    shoot_machine_status_t able_status;    /* 使能状态 */
} shoot_machine_status_msg_t;

extern QueueHandle_t shoot_machine_status_queue;

/**
 * @brief  发射事件, 要执行的操作
 * @note 此处的值主要通过`shoot_machine_evnet_queue`传递
 */
typedef enum {
    SHOOT_MACHINE_EVENT_ENABLE = 1, /* 使能事件 */
    SHOOT_MACHINE_EVENT_DISABLE,    /* 失能事件 */
    SHOOT_MACHINE_EVENT_READY,      /* 接球事件 */
    SHOOT_MACHINE_EVENT_PUSH,       /* 推球事件 */
    SHOOT_MACHINE_EVENT_LOAD_BALL,  /* 装球事件 */

    SHOOT_MACHINE_EVENT_FRIBELT_PRE,    /* 预备转速事件 */
    SHOOT_MACHINE_EVENT_FRIBELT_DIRECT, /* 直接设置转速事件，一定要谨慎！ */
    SHOOT_MACHINE_EVENT_FRIBELT_ZERO,   /* 转速清零事件 */
    SHOOT_MACHINE_EVENT_FRIBELT_ADD,    /* 转速增加事件 */
    SHOOT_MACHINE_EVENT_FRIBELT_DEC,    /* 转速减少事件 */
    SHOOT_MACHINE_EVENT_CALCULATE,      /* 计算转速 */
} shoot_machine_event_t;

/**
 * @brief 发射事件消息内容 (队列实际传输的内容)
 */
typedef struct {
    uint32_t timestamp;          /* 时间戳 */
    shoot_machine_event_t event; /* 事件 */
    float shoot_speed;           /* 摩擦带速度 */
} shoot_machine_event_msg_t;

typedef struct shoot_sub {
    float shoot_speed; /* 摩擦轮速度 */
    uint8_t flag;      /* 发射标志位 */
} shoot_sub_t;

extern QueueHandle_t shoot_machine_event_queue;
extern shoot_sub_t shoot_sub;
void shoot_ctrl_init(void);
void shoot_machine_set_ctrl(float speed, shoot_machine_event_t event);
/**
 * @} friction
 */

/** **********************************************************************************************
 * @defgroup dribble
 * @{
 */
typedef enum {
    DRIBBLE_PROCESS,       /* 纯运球流程 */
    DRIBBLE_CLAMP_AND_HIT,  /* 交接球流程 */

    DRIBBLE_OPEN_CLAMP,    /* 张开夹子 */
    DRIBBLE_CLOSE_CLAMP,   /* 关闭夹子 */
    DRIBBLE_HIT_BALL,      /* 单击打球 */
    DRIBBLE_PUSH_OUT,      /* 接球抽屉推出 */
    DRIBBLE_PUSH_IN,       /* 接球抽屉缩回 */
    DRIBBLE_CLAMP_PUSH_OUT, /* 夹子推出 */
    DRIBBLE_CLAMP_PUSH_IN,  /* 夹子收回 */
    DRIBBLE_GET_STATUES,   /* 获得夹子中球的状态 */

    DRIBBLE_MOVE_TO_CATCH,   /* 伸出交接装置到运球下 */
    DRIBBLE_MOVE_TO_SHOOT,   /* 交接送球到shoot装置*/
    DRIBBLE_HANDLEOVER_BALL, /* 交接球流程 */

    DRIBBLE_NO_TASK /* 保留位-无任务 */
} dribble_event_t;

/* 运球反馈 */
typedef enum {
    DRIBBLE_SUCCESSFUL,   /* 运球成功 */
    DRIBBLE_UNSUCCESSFUL, /* 运球失败 */
    DRIBBLE_NO_BALL,      /* 夹子中没有球 */
    DRIBBLE_HAVE_BALL,    /* 夹子中有球 */
    DRIBBLE_CLAMP_OPENED  /* 夹子已经合上 */
} dribble_status_t;

/* 运球控制队列结构体 */
typedef struct {
    uint32_t timestap;     /* 时间戳 */
    dribble_event_t event; /* 触发事件 */
} dribble_ctrl_queue_t;

/* 运球状态反馈结构体 */
typedef struct {
    uint32_t timestap;       /* 时间戳 */
    dribble_status_t status; /* 状态 */
} dribble_status_queue_t;

extern QueueHandle_t dribble_ctrl_queue;
extern QueueHandle_t dribble_status_queue;
void dribble_init(void);
void dribble_set_ctrl(dribble_event_t event);
/**
 * @} dribble
 */

void main_ctrl_init(void);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDES_H */
