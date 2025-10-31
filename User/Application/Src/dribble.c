/**
 * @file    dribble.c
 * @author  Didi
 * @brief 运球模块控制代码
 * @version 0.1
 * @date    2025-05-03
 */
#include "includes.h"
#include "remote_ctrl/remote_ctrl.h"
#include "logger/logger.h"
#include "pid/pid.h"
#include "my_math/my_math.h"

/**
 *     按键布局
 *     6  7
 *     12 13
 */
#define DRIBBLE_WHOLE_PROCESS_KEY 8
#define DRIBBLE_CLAMP_KEY         2
#define DRIBBLE_PUSH_KEY          27
#define DRIBBLE_HANDOVER_KEY      9
#define DRIBBLE_PART_PROCESS_KEY  10
#define DRIBBLE_PUSH_IN_KEY       3
#define DRIBBLE_PUSH_OUT_KEY      4

#define HANDOVER_DEBUG            0

TaskHandle_t dribble_ctrl_task_handle;
void dribble_ctrl_task(void *pvParameters);
QueueHandle_t dribble_ctrl_queue;
QueueHandle_t dribble_status_queue;

void push_out(void);

void dribble_set_ctrl(dribble_event_t event) {
    static dribble_ctrl_queue_t dribble_ctrl_msg;
    dribble_ctrl_msg.event = event;
    dribble_ctrl_msg.timestap = HAL_GetTick();
    xQueueSend(dribble_ctrl_queue, &dribble_ctrl_msg, 5);
}

/**
 * @brief 运球按键回调函数
 *
 * @param key 按键
 * @param event 按下按键发送运球信号,松开清空发送
 * @return null
 */

void key_dribble_ball(uint8_t key, remote_key_event_t key_event) {
    UNUSED(key_event);
    dribble_ctrl_queue_t dribble_ctrl_msg;
    dribble_ctrl_msg.event = DRIBBLE_NO_TASK;
    static bool push_flag = false;  /* true-机构推出；false-机构回缩 */
    static bool clamp_flag = false; /* true-夹子张开；false-夹子闭合 */

    switch (key) {
        case DRIBBLE_WHOLE_PROCESS_KEY:
            /* 纯运球流程 */
            dribble_ctrl_msg.event = DRIBBLE_WHOLE_PROCESS;
            break;
        case DRIBBLE_PART_PROCESS_KEY:
            /* 交接球流程 */
            dribble_ctrl_msg.event = DRIBBLE_PART_PROCESS;
            break;
        case DRIBBLE_CLAMP_KEY:
            /* 夹子张开-闭合 */
            if (clamp_flag) {
                dribble_ctrl_msg.event = DRIBBLE_CLOSE_CLAMP;
                clamp_flag = false;
            } else {
                dribble_ctrl_msg.event = DRIBBLE_OPEN_CLAMP;
                clamp_flag = true;
            }
            break;
        case DRIBBLE_PUSH_KEY:
            /* 推出-收回 */
            if (push_flag) {
                dribble_ctrl_msg.event = DRIBBLE_PUSH_IN;
                push_flag = false;
            } else {
                dribble_ctrl_msg.event = DRIBBLE_PUSH_OUT;
                push_flag = true;
            }
            break;
        case DRIBBLE_PUSH_IN_KEY:
            dribble_ctrl_msg.event = DRIBBLE_MOVE_TO_SHOOT;
            break;
        case DRIBBLE_PUSH_OUT_KEY:
            dribble_ctrl_msg.event = DRIBBLE_MOVE_TO_CATCH;
            break;
        case DRIBBLE_HANDOVER_KEY: {
#if HANDOVER_DEBUG
            static bool handover_flag = false;
            if (handover_flag) {
                /* 如果状态为伸出:发送信号量->回缩 */
                dribble_ctrl_msg.event = DRIBBLE_MOVE_TO_SHOOT;
            } else {
                /* 如果状态为回缩:发送信号量->伸出 */
                dribble_ctrl_msg.event = DRIBBLE_MOVE_TO_CATCH;
            }
            handover_flag = !handover_flag;
#else
            dribble_ctrl_msg.event = DRIBBLE_HANDLEOVER_BALL;
            shoot_machine_set_ctrl(16000, SHOOT_MACHINE_EVENT_FRIBELT_PRE);
#endif
        } break;
        default:
            break;
    }
    dribble_ctrl_msg.timestap = HAL_GetTick();
    xQueueReset(dribble_ctrl_queue);
    xQueueSend(dribble_ctrl_queue, &dribble_ctrl_msg, 0);
}

/**
 * @brief 夹子张开
 *
 */
void clamp_open(void) {
    push_out();
    vTaskDelay(500);
    CYLINDER_CLAMP_ON();
}

/**
 * @brief 夹子闭合
 *
 */
void clamp_close(void) {
    CYLINDER_CLAMP_OFF();
}

/**
 * @brief 击打球机构
 *
 */
void hit_ball(void) {
    CYLINDER_TOP_ON();
    vTaskDelay(100);
    CYLINDER_TOP_OFF();
}

/**
 * @brief 将气缸推出
 *
 */
void push_out(void) {
    CYLINDER_PUSH_ON();
}

/**
 * @brief 将气缸收回
 *
 */
void push_in(void) {
    CYLINDER_PUSH_OFF();
    clamp_close();
}

/**
 * @defgroup handover
 * @{
 * **************************************************************************
 */
TaskHandle_t catch_motor_ctrl_task_handle;
#define CATCH_TARGET_SPEED 5000

typedef enum {
    CATCH_STATUS_TO_SHOOT = 0, /* 回缩机构->发球 */
    CATCH_STATUS_TO_CATCH,     /* 伸出机构->接球 */

    CATCH_STATUS_NUM
} catch_status_t;
catch_status_t catch_status = CATCH_STATUS_TO_SHOOT; /* 状态量：默认回收 */

dji_motor_handle_t catch_motor_handle; /* 电机控制句柄 */
pid_t catch_motor_speed_pid = {0};     /* dji速度pid */
pid_t catch_motor_angle_pid = {0};     /* dji角度pid */

/**
 * @brief 设置任务状态
 *
 * @param status 状态量:
 *  @val  CATCH_STATUS_TO_SHOOT
 *  @val  CATCH_STATUS_TO_CATCH
 */
void set_catch_motor_statue(catch_status_t status) {
    catch_status = status;
}

/**
 * @brief catch to shoot
 *
 */
void catch_to_shoot(void) {
    /* 伸出接球装置*/
    set_catch_motor_statue(CATCH_STATUS_TO_CATCH);
    do {
        vTaskDelay(1);
    } while (PROXIMITY_OUT_SWITCH() != PROXIMITY_OUT_SWITCH_TOUCHED);
    clamp_open(); /* 夹子张开 */
    /* 等待球落下 */
    uint32_t start_times = HAL_GetTick();
    do {
        vTaskDelay(1);
        if (HAL_GetTick() - start_times > 2000) {
            break;
        }
    } while (NPN_SWITCH() == NPN_SWITCH_TOUCHED);
    vTaskDelay(500);
    clamp_close(); /* 夹子合拢 */

    /**
     * @todo 等待球落下
     *
     */
    push_in(); /* 收回交接机构 */
    set_catch_motor_statue(CATCH_STATUS_TO_SHOOT);
}

/**
 * @brief catch to shoot
 *
 */
void catch_to_shoot_push_in(void) {
    /* 伸出接球装置*/
    set_catch_motor_statue(CATCH_STATUS_TO_CATCH);
}

void catch_to_shoot_push_out(void) {
    /* 收回接球装置*/
    set_catch_motor_statue(CATCH_STATUS_TO_SHOOT);
}

/**
 * @brief 接球装置伸缩任务
 *
 * @param pvParameters
 */
void catch_motor_ctrl_task(void *pvParameters) {
    UNUSED(pvParameters);
    /* 初始化电机 */
    dji_motor_init(&catch_motor_handle, DJI_M2006, CAN_Motor1_ID,
                   can1_selected);
    /* 初始化pid */
    // pid_init(&catch_motor_speed_pid, 16384, 5000, 10, 16384, POSITION_PID, 9.0f,
    //          0.01f, 1.0f);
    pid_init(&catch_motor_speed_pid, 16384, 5000, 10, 16384, POSITION_PID,
             15.0f, 0.01f, 1.0f);
    pid_init(&catch_motor_angle_pid, 8192, 8192, 10, 16384, POSITION_PID, 20.0f,
             0.001f, 11.0f);
    static float target_rpm = 0;
    static float rpm_out = 0;
    static float break_angle = 0;

    while (1) {
        switch (catch_status) {
            case CATCH_STATUS_TO_CATCH: {
                /* 判定接近开关状态 */
                if (PROXIMITY_OUT_SWITCH() != PROXIMITY_OUT_SWITCH_TOUCHED) {
                    /* 开关没被触发,恒定速度 */
                    target_rpm = CATCH_TARGET_SPEED;
                    break_angle = 0.0f;
                } else if (break_angle == 0.0f) {
                    /* 开关触发且角度为0->设置角度为当前角度 */
                    break_angle = catch_motor_handle.rotor_degree;
                } else {
                    /* 当前角度自锁 */
                    target_rpm = pid_calc(&catch_motor_angle_pid, break_angle,
                                          catch_motor_handle.rotor_degree);
                }
            } break;
            case CATCH_STATUS_TO_SHOOT: {
                /* 判定接近开关状态 */
                if (PROXIMITY_IN_SWITCH() != PROXIMITY_IN_SWITCH_TOUCHED) {
                    /* 开关没被触发,恒定速度 */
                    target_rpm = -CATCH_TARGET_SPEED;
                    break_angle = 0.0f;
                } else if (break_angle == 0.0f) {
                    break_angle = catch_motor_handle.rotor_degree;
                } else {
                    target_rpm = pid_calc(&catch_motor_angle_pid, break_angle,
                                          catch_motor_handle.rotor_degree);
                }
            } break;
            default:
                break;
        }
        rpm_out = pid_calc(&catch_motor_speed_pid, target_rpm,
                           catch_motor_handle.speed_rpm);


        dji_motor_set_current(can1_selected, DJI_MOTOR_GROUP1, (int16_t)rpm_out,
                              0, 0, 0);
        vTaskDelay(10);
    }
}

/**
 * **************************************************************************
 * @}
 */

/**
 * @brief 运球
 *
 */
void whole_process(void) {
    static int8_t times_flag = 0;
    /* 夹子张开 */
    clamp_open();
    vTaskDelay(35);
    /* 顶部击打 */
    hit_ball();
    /* 等待球落下 */
    do {
        vTaskDelay(1);
    } while (NPN_SWITCH() == NPN_SWITCH_TOUCHED);
    vTaskDelay(30);
    /* 等待球反弹回来 */
    static uint32_t start_time;
    start_time = HAL_GetTick();
    do {
        if (HAL_GetTick() - start_time > 3000) {
            break;
        }
        vTaskDelay(1);
    } while ((NPN_SWITCH() != NPN_SWITCH_TOUCHED) &&
             (NPN_SWITCH_2() != NPN_SWITCH2_TOUCHED) &&
             (NPN_SWITCH_3() != NPN_SWITCH3_TOUCHED));
    /* 夹子合拢 */
    clamp_close();
    // times_flag++;
    // if (times_flag == 2) {
    //     times_flag = 0;
    //     catch_to_shoot();
    //     shoot_machine_set_ctrl(16000.0f, SHOOT_MACHINE_EVENT_FRIBELT_PRE);
    // }

    //     clamp_open();
    //     vTaskDelay(35);
    //     /* 顶部击打 */
    //     hit_ball();
    //     /* 等待球落下 */
    //     do {
    //         vTaskDelay(1);
    //     } while (NPN_SWITCH() == NPN_SWITCH_TOUCHED);
    //     vTaskDelay(30);
    //     /* 等待球反弹回来 */
    //     start_time = HAL_GetTick();
    //     do {
    //         if (HAL_GetTick() - start_time > 3000) {
    //             break;
    //         }
    //         vTaskDelay(1);
    //     } while ((NPN_SWITCH() != NPN_SWITCH_TOUCHED) &&
    //              (NPN_SWITCH_2() != NPN_SWITCH2_TOUCHED) &&
    //              (NPN_SWITCH_3() != NPN_SWITCH3_TOUCHED));
    //     /* 夹子合拢 */
    //     clamp_close();
    //     /* 交接机构伸出 */
    // #if !(HANDOVER_DEBUG) /* 非调试模式每次全流程运球两次 */
    //     // set_catch_motor_statue(CATCH_STATUS_TO_CATCH);
    //     catch_to_shoot();
    //     shoot_machine_set_ctrl(16000, SHOOT_MACHINE_EVENT_FRIBELT_PRE);
    // #endif // HANDOVER_DEBUG
}

/**
 * @brief 控制运球任务
 *
 * @param pvParameters
 */
void dribble_ctrl_task(void *pvParameters) {
    UNUSED(pvParameters);
    dribble_ctrl_queue_t dribble_ctrl;
    static dribble_status_queue_t dribble_status;

    while (1) {
        // if (g_nuc_pos_data.x > 500.0f || g_nuc_pos_data.y > 500.0f) {
        //     /* 准备区区外气缸伸出：再次回来时机构不缩回*/
        //     CYLINDER_PUSH_ON();
        // }

        if (xQueueReceive(dribble_ctrl_queue, &dribble_ctrl, 5000) == pdFALSE) {
            /* 避免接收失败后消息仍然运行下面的语句 */
            continue;
        }
        if ((HAL_GetTick() - dribble_ctrl.timestap) > 2000) {
            /* 超时处理 */
            log_message(LOG_WARNING,
                        "[Dribble] Received message timeout,timegap : % u,"
                        "event: %d. ",
                        dribble_ctrl.timestap, dribble_ctrl.event);
            continue;
        }

        switch (dribble_ctrl.event) {
            case DRIBBLE_WHOLE_PROCESS: {
                /* 全流程 */
                whole_process();
                if (NPN_SWITCH() == NPN_SWITCH_TOUCHED) {
                    /* 判定有球->运球成功 */
                    dribble_status.status = DRIBBLE_SUCCESSFUL;
                } else {
                    /* 无球->运球失败 */
                    dribble_status.status = DRIBBLE_UNSUCCESSFUL;
                }
                dribble_status.timestap = HAL_GetTick();
                xQueueSend(dribble_status_queue, &dribble_status, 1);
            } break;
            case DRIBBLE_PART_PROCESS: {
                /* 交接球流程 */
                catch_to_shoot();
                shoot_machine_set_ctrl(16000.0f, SHOOT_MACHINE_EVENT_FRIBELT_PRE);
            } break;
            case DRIBBLE_OPEN_CLAMP: {
                /* 夹子关闭 */
                clamp_open();
            } break;
            case DRIBBLE_CLOSE_CLAMP: {
                /* 夹子关闭 */
                clamp_close();
            } break;
            case DRIBBLE_HIT_BALL: {
                /* 击打球 */
                hit_ball();
            } break;
            case DRIBBLE_PUSH_OUT: {
                /* 伸出 */
                push_out();
            } break;
            case DRIBBLE_PUSH_IN: {
                /* 收回 */
                push_in();
            } break;
            case DRIBBLE_GET_STATUES: {
                if (NPN_SWITCH() == NPN_SWITCH_TOUCHED) {
                    /* 判定夹子中有球 */
                    dribble_status.status = DRIBBLE_HAVE_BALL;
                } else {
                    /* 判定夹子中无球 */
                    dribble_status.status = DRIBBLE_CLAMP_OPENED;
                }
                dribble_status.timestap = HAL_GetTick();
                xQueueSend(dribble_status_queue, &dribble_status, 0);
            } break;
            case DRIBBLE_MOVE_TO_CATCH: {
                /* 将2006伸出 */
                set_catch_motor_statue(CATCH_STATUS_TO_CATCH);
            } break;
            case DRIBBLE_MOVE_TO_SHOOT: {
                /* 将2006收回接球装置 */
                set_catch_motor_statue(CATCH_STATUS_TO_SHOOT);
                shoot_machine_set_ctrl(16000.0f, SHOOT_MACHINE_EVENT_FRIBELT_PRE);

            } break;
            case DRIBBLE_HANDLEOVER_BALL: {
                /* 将2006接球装置收回 */
                catch_to_shoot();
            } break;

            default:
                continue; /* 打断此次循环(不要发送消息) */
        }
        dribble_ctrl.event = DRIBBLE_NO_TASK;
    }
}

/**
 * @brief 运球初始化
 *
 * @return
 */
void dribble_init(void) {
    cylinder_init();   /* 初始化气缸 */
    npn_switch_init(); /* 初始化npn开关 */

    /* 设置上电运球初始状态 */
    push_in(); /* 函数内有对于夹子状态的保护 */
               // clamp_open(); /* 夹子默认上电张开:方便20s中放球 */
    clamp_close();
    set_catch_motor_statue(CATCH_STATUS_TO_SHOOT); /* 默认状态下抽屉回收 */

    /* 运球按键注册 */
    remote_register_key_callback(DRIBBLE_WHOLE_PROCESS_KEY,
                                 REMOTE_KEY_PRESS_DOWN, key_dribble_ball);
    remote_register_key_callback(DRIBBLE_PART_PROCESS_KEY, REMOTE_KEY_PRESS_DOWN,
                                 key_dribble_ball);
    remote_register_key_callback(DRIBBLE_CLAMP_KEY, REMOTE_KEY_PRESS_DOWN,
                                 key_dribble_ball);
    // remote_register_key_callback(DRIBBLE_CLAMP_KEY, REMOTE_KEY_PRESS_UP,
    //                              key_dribble_ball);
    remote_register_key_callback(DRIBBLE_PUSH_KEY, REMOTE_KEY_PRESS_DOWN,
                                 key_dribble_ball);
    remote_register_key_callback(DRIBBLE_HANDOVER_KEY, REMOTE_KEY_PRESS_DOWN,
                                 key_dribble_ball);
    remote_register_key_callback(DRIBBLE_PUSH_IN_KEY, REMOTE_KEY_PRESS_DOWN,
                                 key_dribble_ball);
    remote_register_key_callback(DRIBBLE_PUSH_OUT_KEY, REMOTE_KEY_PRESS_DOWN,
                                 key_dribble_ball);

    /* 创建运球任务 */
    xTaskCreate(dribble_ctrl_task, "dribble—ctrl-task", 192, NULL, 4,
                &dribble_ctrl_task_handle);
    /* 创建交接球任务 */
    xTaskCreate(catch_motor_ctrl_task, "catch_motor_ctrl_task", 192, NULL, 4,
                &catch_motor_ctrl_task_handle);
    /* 创建运球消息队列 */
    dribble_ctrl_queue = xQueueCreate(1, sizeof(dribble_ctrl_queue_t));
    /* 创建状态返回消息队列 */
    dribble_status_queue = xQueueCreate(1, sizeof(dribble_status_queue_t));
}
