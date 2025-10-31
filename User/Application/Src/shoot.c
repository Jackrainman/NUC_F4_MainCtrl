/**
 * @file shoot.c
 * @author meiwenhuaqingnian
 * @brief 发射控制相关函数
 * @version 2.0
 * @date 2025-05-15
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "includes.h"
#include "remote_ctrl/remote_ctrl.h"
#include "action_position/action_position.h"
#include "my_math/my_math.h"
#include "logger/logger.h"
#include "odometry_string/odometry_string.h"

#define SPEED_ADD_KEY      5  /* 速度增加按键 */
#define SPEED_DEC_KEY      11 /* 速度减小按键 */
#define SPEED_PRE_KEY      38  /* 预订转速按键 */
#define ENABLE_KEY         6 /* 发射使能按键 */
#define DISABLE_KEY        12 /* 发射失能按键 */
#define SANCTION_KEY       0 /* 制裁按键 */
#define SMALL_SPEED_KEY    35  /*!<低速模式按键 */

// #define SPEED_CALCULATE_KEY 16 /* 速度拟合按键 */

#define SPEED_ZERO_KEY     0xFF

#define SHOOT_PRE_SPEED    17000
#define SHOOT_DIVIDE_SPEED 100
#define SANCTION_SPD       30000

/* 赛场拟合出的数据 */
#define FIT_A              -9.1348e-05f /* 三分外的速度曲线拟合参数 */
#define FIT_B              2.7080f
#define FIT_C              7.2400e+03f

#define FIT_A2             9.9838e-04f /* 三分内的速度曲线拟合参数 */
#define FIT_B2             -3.6070f
#define FIT_C2             1.6728e+04f

/**
 * @brief 速度计算曲线函数
 *
 * @param radium
 * @return float
 */
float get_friction_speed(float radium) {
    if (radium < 3125.0f) {
        /* 三分内 */
        return FIT_A2 * radium * radium + FIT_B2 * radium + FIT_C2;
    } else {
        /* 三分外 */
        return FIT_A * radium * radium + FIT_B * radium + FIT_C;
    }
}

TaskHandle_t shoot_machine_task_handle;
void shoot_machine_task(void *pvParameters);

/* 发射状态队列, 发射 --> 其他模块 */
QueueHandle_t shoot_machine_status_queue;
/* 发射事件队列, 其他模块 --> 发射 */
QueueHandle_t shoot_machine_event_queue;

shoot_sub_t shoot_sub = {0};

/**
 * @brief 摩擦带速度控制
 *
 * @param key
 * @param event
 */
void fribelt_speed_ctrl(uint8_t key, remote_key_event_t event) {
    UNUSED(event);

    shoot_machine_event_msg_t shoot_mach_event = {0};
    switch (key) {
        case SPEED_PRE_KEY:
            shoot_mach_event.event = SHOOT_MACHINE_EVENT_FRIBELT_DIRECT;
            shoot_mach_event.shoot_speed = get_friction_speed(g_basket_radius);
            break;
        case SPEED_ZERO_KEY:
            shoot_mach_event.event = SHOOT_MACHINE_EVENT_FRIBELT_ZERO;
            break;
        case SPEED_ADD_KEY:
            shoot_mach_event.event = SHOOT_MACHINE_EVENT_FRIBELT_ADD;
            break;
        case SPEED_DEC_KEY:
            shoot_mach_event.event = SHOOT_MACHINE_EVENT_FRIBELT_DEC;
            break;
        case SANCTION_KEY:
            shoot_mach_event.event = SHOOT_MACHINE_EVENT_FRIBELT_PRE;
            shoot_mach_event.shoot_speed = SANCTION_SPD;
            break;
        case SMALL_SPEED_KEY:
            shoot_mach_event.event = SHOOT_MACHINE_EVENT_FRIBELT_PRE;
            shoot_mach_event.shoot_speed = 3500;
            break;
        default:
            break;
    }

    UNUSED(shoot_mach_event.shoot_speed);
    shoot_mach_event.timestamp = HAL_GetTick();
    if (xQueueSend(shoot_machine_event_queue, &shoot_mach_event, 5) != pdPASS) {
        log_message(LOG_ERROR, "[Shoot machine] Shoot machine FRIBELT event "
                               "send failed, timeout. ");
    }
}

/**
 * @brief 推球控制
 *
 * @param key
 * @param event
 */
void push_ball_ctrl(uint8_t key, remote_key_event_t event) {
    UNUSED(event);
    shoot_machine_event_msg_t shoot_mach_event = {0};

    switch (key) {
        case ENABLE_KEY:
            if (shoot_sub.flag == 0) {
                // shoot_mach_event.event = SHOOT_MACHINE_EVENT_ENABLE; /* 使能 */
                shoot_sub.flag = 2;
                sub_friction_flag(shoot_sub.flag);
                log_message(LOG_INFO, "发射结构使能!\n");
                break;
            } else if (shoot_sub.flag == 1) {
                // shoot_mach_event.event =
                //     SHOOT_MACHINE_EVENT_READY; /* 切换状态:推球态->准备态 */
                shoot_sub.flag = 2;
                sub_friction_flag(shoot_sub.flag);
                log_message(LOG_INFO, "发射结构推球 !\n");
            } else if (shoot_sub.flag == 2) {
                shoot_sub.flag = 1;
                sub_friction_flag(shoot_sub.flag);
                // shoot_mach_event.event =
                //     SHOOT_MACHINE_EVENT_PUSH; /* 切换状态:推球态<-准备态 */
                log_message(LOG_INFO, "发射结构接球 !\n");
            }
            break;
        case DISABLE_KEY:
            shoot_mach_event.event = SHOOT_MACHINE_EVENT_DISABLE; /* 失能 */
            shoot_sub.flag = 0;
            shoot_sub.shoot_speed = 0;
            sub_friction_data(shoot_sub.shoot_speed);
            sub_friction_flag(shoot_sub.flag);

            log_message(LOG_INFO, "发射结构失能！\n");
            break;
        default:
            log_message(LOG_INFO, "[Shoot machine] Push ball err, timetap: %u",
                        shoot_mach_event.timestamp);
            break;
    }
    shoot_mach_event.timestamp = HAL_GetTick();
    if (xQueueSend(shoot_machine_event_queue, &shoot_mach_event, 5) != pdPASS) {
        log_message(LOG_ERROR, "[Shoot machine] Shoot machine event "
                               "send failed, timeout. ");
    }
}

/**
 * @brief 初始化按键
 *
 */
void shoot_ctrl_init(void) {
    /* 注册遥控器按键 */

    remote_register_key_callback(SPEED_ADD_KEY, REMOTE_KEY_PRESS_DOWN,
                                 fribelt_speed_ctrl); /* 加速 */
    remote_register_key_callback(SPEED_PRE_KEY, REMOTE_KEY_PRESS_DOWN,
                                 fribelt_speed_ctrl); /* 预备转速 */
    remote_register_key_callback(SPEED_DEC_KEY, REMOTE_KEY_PRESS_DOWN,
                                 fribelt_speed_ctrl); /* 减速 */

    // remote_register_key_callback(SPEED_ZERO_KEY, REMOTE_KEY_PRESS_DOWN,
    //                              fribelt_speed_ctrl); /* 置零 */

    remote_register_key_callback(ENABLE_KEY, REMOTE_KEY_PRESS_DOWN,
                                 push_ball_ctrl); /* 推接球 + 使能 */
    remote_register_key_callback(DISABLE_KEY, REMOTE_KEY_PRESS_DOWN,
                                 push_ball_ctrl); /* 失能 */
    remote_register_key_callback(SMALL_SPEED_KEY, REMOTE_KEY_PRESS_DOWN,
                                 fribelt_speed_ctrl);

    xTaskCreate(shoot_machine_task, "shoot_machine_task", 256, NULL, 4,
                &shoot_machine_task_handle);
    shoot_machine_event_queue =
        xQueueCreate(5, sizeof(shoot_machine_event_msg_t));
    shoot_machine_status_queue =
        xQueueCreate(5, sizeof(shoot_machine_status_msg_t));
}
/**
 * @brief 摩擦带速度计算
 *
 */
float fribelt_speed_cal(float radius) {
    float erpm;
    // erpm = 1.83 * radius + 10376.45; /* 还不错，有几个点是ok的 */
    // erpm = 1.98 * radius + 9817.82;  /* 不好，非常差 */
    // erpm = 0.000188 * radius * radius - 0.8304 * radius + 18900.0f;
    erpm = 1.8877 * (radius - 100) + 10406.43;
    return erpm;
}

void shoot_machine_set_ctrl(float speed, shoot_machine_event_t event) {
    static shoot_machine_event_msg_t shoot_machine_event_msg;

    shoot_machine_event_msg.shoot_speed = speed;
    shoot_machine_event_msg.timestamp = HAL_GetTick();
    shoot_machine_event_msg.event = event;
    xQueueSend(shoot_machine_event_queue, &shoot_machine_event_msg, 5);
}

/**
 * @brief 发射任务
 *
 * @param pvParameters
 */
void shoot_machine_task(void *pvParameters) {
    UNUSED(pvParameters);
    shoot_machine_event_msg_t event = {0};
    uint32_t timegap = 0;
    shoot_machine_status_msg_t msg = {0};
    msg.timestamp = HAL_GetTick();

    while (1) {
        xQueueReceive(shoot_machine_event_queue, &event, portMAX_DELAY);
        timegap = HAL_GetTick() - event.timestamp;

        if (timegap > 1000) {
            log_message(
                LOG_WARNING,
                "[Shoot Machine] Received message timeout, timegap: %u, "
                "event: %d. ",
                timegap, event.event);
            continue;
        }

        switch (event.event) {
            case SHOOT_MACHINE_EVENT_DISABLE: {
                /* 失能，虽然按键回调中失能按键已经将标志位置0，但是为了main_ctrl中控制还是再次将标志位置零 */
                shoot_sub.flag = 0;
                shoot_sub.shoot_speed = 0;
                msg.able_status = SHOOT_MACHINE_STATUS_DISABLE;
            } break;

            case SHOOT_MACHINE_EVENT_ENABLE: {
                /* 始能:可以给转速-抬起球-不推球 */
                shoot_sub.flag = 2;
                msg.able_status = SHOOT_MACHINE_STATUS_ENABLE;
            } break;

            case SHOOT_MACHINE_EVENT_PUSH: {
                /* 推球 */
                shoot_sub.flag = 1;
                msg.push_status = SHOOT_MACHINE_STATUS_PUSH_DONE;
            } break;

            case SHOOT_MACHINE_EVENT_READY: {
                /* 预备转速 */
                shoot_sub.flag = 2;
                msg.push_status = SHOOT_MACHINE_STATUS_PASS_DONE;
            } break;

            case SHOOT_MACHINE_EVENT_FRIBELT_PRE: {
                shoot_sub.flag = 2;
                sub_friction_flag(shoot_sub.flag);
                /* 预备转速 */
                for (int8_t i = 0; i < 10; i++) {
                    shoot_sub.shoot_speed = (i + 1) * event.shoot_speed / 10;
                    sub_friction_data(shoot_sub.shoot_speed);
                    vTaskDelay(200);
                }
                msg.fribelt_status = SHOOT_MACHINE_STATUS_FRIBELT_DONE;
            } break;

            case SHOOT_MACHINE_EVENT_FRIBELT_ZERO: {
                /* 转速清零 */
                shoot_sub.shoot_speed = 0;
                msg.fribelt_status = SHOOT_MACHINE_STATUS_FRIBELT_ZERO;
            } break;

            case SHOOT_MACHINE_EVENT_FRIBELT_DIRECT: {
                /* 直接设置转速事件 */
                shoot_sub.shoot_speed = event.shoot_speed;
                sub_friction_data(shoot_sub.shoot_speed);
            } break;

            case SHOOT_MACHINE_EVENT_FRIBELT_ADD: {
                /* 转速增加 */
                shoot_sub.shoot_speed += SHOOT_DIVIDE_SPEED;
                msg.fribelt_status = SHOOT_MACHINE_STATUS_FRIBELT_DONE;
            } break;

            case SHOOT_MACHINE_EVENT_FRIBELT_DEC: {
                /* 转速减小 */
                shoot_sub.shoot_speed -= SHOOT_DIVIDE_SPEED;
                msg.fribelt_status = SHOOT_MACHINE_STATUS_FRIBELT_DONE;
            } break;

            case SHOOT_MACHINE_EVENT_CALCULATE: {
                /* 自计算转速 */
                shoot_sub.flag = 2;
                sub_friction_flag(shoot_sub.flag); /* 进入始能模式 */

                /* 递增速度到13000 */
                for (uint8_t i = 1; i < 11; i++) {
                    shoot_sub.shoot_speed = 1000 * i;
                    sub_friction_data(shoot_sub.shoot_speed);
                    vTaskDelay(150);
                }
                shoot_sub.shoot_speed = 13000;
                sub_friction_data(shoot_sub.shoot_speed);
                vTaskDelay(300);
                /* 目标速度 */
                shoot_sub.shoot_speed = fribelt_speed_cal(g_basket_radius);
                sub_friction_data(shoot_sub.shoot_speed);

                /* 等待到达目标转速 */
                vTaskDelay(5000);
                shoot_sub.flag = 1; /* 推球上去 */
            } break;

            default: {
            } break;
        }
        sub_friction_data(shoot_sub.shoot_speed);
        sub_friction_flag(shoot_sub.flag);
        xQueueSend(shoot_machine_status_queue, &msg, 5);

        /* 上报遥控器数据 */
        remote_report_data_t shoot_report = REMOTE_REPORT_SHOOT;
        xQueueOverwrite(remote_report_data_queue, &shoot_report);
        vTaskDelay(5);
    }
}
