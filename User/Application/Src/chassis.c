/**
 * @file chassis.c
 * @author DIDI
 * @brief 底盘控制相关任务
 * @version 0.1
 * @date 2025-04-25
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "includes.h"
#include "remote_ctrl/remote_ctrl.h"
#include "my_math/my_math.h"
#include "go_path/go_path.h"
#include "action_position/action_position.h"
#include "logger/logger.h"

#define POS_NUM               5 /*!< 点位数量 */

/* 按键宏定义 */
#define CHASSIS_AIMING_KEY    3 /*!< 底盘自瞄开启 */
#define CHASSIS_SET_HALT_KEY  1 /*!< 底盘是否自锁 */
#define CHASSIS_WORLD_KEY     2 /*!< 底盘切手控 */
#define CHASSIS_RUN_POINT_KEY 4 /*!< 底盘跑点按键 */
#define CHASSIS_SPDZ_CTR_KEY  6 /*!< 调整底盘自转速度点按键 */

/* FreeRTOS-句柄 & 函数声明 */
TaskHandle_t chassis_manual_ctrl_task_handle; /*!< 手动任务句柄 */
void chassis_manual_ctrl_task(void *pvParametes);
TaskHandle_t chassis_auto_ctrl_task_handle; /*!< 自动任务句柄 */
void chassis_auto_ctrl_task(void *pvParametes);
TaskHandle_t chassis_ctrl_task_handle; /*!< 底盘控制任务句柄 */
void chassis_ctrl_task(void *pvParametes);

QueueHandle_t chassis_ctrl_queue;   /*!< 底盘控制队列 */
QueueHandle_t chassis_status_queue; /*!< 底盘状态队列 */

/* 底盘控制函数 */
go_path_chassis_func_t chassis_wheel_ctrl;

static float self_yaw = 0.0f; /*! 自身坐标系yaw角，恒为零 */

/* 底盘状态结构体 */
chassis_state_t chassis_state = {
    .halt_flag = true,
    .point_index = 0,
    .yaw_flag = true,
    .collimation_flag = false,
    .spd_z = false,
};

float spdz_ctr_flag = 1;

/* go_path中相关参数 */
pid_t nuc_flat_speed_pid;
pid_t nuc_flat_angle_pid;

/* 固定朝向参数 */
pid_t orientation_angle_pid; /*!< 固定朝向自转速度pid */

static float target_point_x = 0.0f; /*!< 目标朝向点x坐标 */
static float target_point_y = 0.0f; /*!< 目标朝向点y坐标 */

/* 定点结构体 */
typedef struct pos_node {
    float pos_x;
    float pos_y;
    float pos_yaw;
    go_path_point_type_t pos_type;
} pos_node_t;

enum {
    EX_NODE_TARGET_RADIUM, /* 目标半径下的点位 */
    EX_NODE_LEFT,          /* 场地左边的装球点 */
    EX_NODE_RIGHT,         /* 场地右边的装球点 */

    EX_NODE_NUM
} ex_node;

/* 固定点位+可变点位函数 */
pos_node_t pos_array[POS_NUM + EX_NODE_NUM] = {
    [0] = {-0.12f, 0.59f, 0.0f, POINT_TYPE_NUC_FLAT},
    [1] = {3656.11f, 3314.07f, 0.0f, POINT_TYPE_NUC_FLAT},
    [2] = {2614.78f, 3600.57f, 88.09f, POINT_TYPE_NUC_FLAT},
    [3] = {0, 0, 0, POINT_TYPE_NUC_FLAT}, /*!< 固定点位信息 */

    /* 圆环上的点位 */
    [POS_NUM +
        EX_NODE_TARGET_RADIUM] = {0.0, 0.1, 0.2, POINT_TYPE_TARGET_RADIUM},
    /* 场地左边装球点 */
    [POS_NUM + EX_NODE_LEFT] = {-7000.0f, 0.0, 0.0f, POINT_TYPE_LOAD_BALL},
    /* 场地右边装球点 */
    [POS_NUM + EX_NODE_RIGHT] = {-800.0f, 0.0, 0.0f, POINT_TYPE_LOAD_BALL}};

/**
 * @brief 底盘设置自锁函数
 *
 * @param halt
 *        -[val] 1-自锁
 *        -[val] 0-取消自锁
 * */
void chassis_set_halt(bool halt) {
    sub_chassis_halt(halt);
    if (halt) {
        sub_chassis_speed(0, 0, 0);
    }
}

/**
 * @brief 写入底盘状态函数
 *
 * @param status
 * @return
 */
void chassis_set_status(chassis_status_t status) {
    static chassis_status_queue_t chassis_statue_msg;
    xQueueReset(chassis_status_queue); /* 清除消息队列状态 */
    chassis_statue_msg.timestap = HAL_GetTick();
    chassis_statue_msg.status = status;
    xQueueSend(chassis_status_queue, &chassis_statue_msg, 5);
}

/**
 * @brief 控制底盘状态函数
 *
 * @param event
 * @return
 */
void chassis_set_ctrl(chassis_event_t event) {
    static chassis_ctrl_queue_t chassis_ctrl_msg;
    xQueueReset(chassis_ctrl_queue); /* 清除消息队列状态 */
    chassis_ctrl_msg.timestap = HAL_GetTick();
    chassis_ctrl_msg.event = event;
    xQueueSend(chassis_ctrl_queue, &chassis_ctrl_msg, 5);
}

void chassis_ctrl_queue_reset(void) {
    xQueueReset(chassis_ctrl_queue);
}

/**
 * @brief 切換手动函数
 *
 * @return
 */
void chassis_set_manual_ctrl(void) {
    static chassis_ctrl_queue_t chassis_ctrl_msg;

    chassis_ctrl_queue_reset();
    chassis_ctrl_msg.event = CHASSIS_SET_MANUAL;
    chassis_ctrl_msg.timestap = HAL_GetTick();
    xQueueSend(chassis_ctrl_queue, &chassis_ctrl_msg, 5);
}

/**
 * @brief 自动跑点
 *
 */
void chassis_set_point_run(uint8_t index) {
    chassis_state.point_index = index;
    static chassis_ctrl_queue_t chassis_ctrl_msg;

    chassis_ctrl_queue_reset();
    chassis_ctrl_msg.event = CHASSIS_SET_POINT;
    chassis_ctrl_msg.timestap = HAL_GetTick();
    xQueueSend(chassis_ctrl_queue, &chassis_ctrl_msg, 5);
}

/**
 * @brief 自动瞄准调整函数
 *
 */
void chassis_set_aiming(chassis_event_t event) {
    switch (event) {
        case CHASSIS_SHOOT_AIMING: {
            chassis_state.collimation_flag = !chassis_state.collimation_flag;
            target_point_x = BASKET_POINT_X;
            target_point_y = BASKET_POINT_Y;

        } break;
        case CHASSIS_LEFT_AIMING: {
            chassis_state.collimation_flag = true;
            target_point_x = -8000.0f;
            target_point_y = g_nuc_pos_data.y;
        } break;
        case CHASSIS_RIGHT_AIMING: {
            chassis_state.collimation_flag = true;
            target_point_x = -10.0f;
            target_point_y = g_nuc_pos_data.y;
        } break;
        case CHASSIS_RESET_AIMING: {
            chassis_state.collimation_flag = false;
        } break;
        default:
            break;
    }
}

static float orientation_aim_angle; /* 目标点夹角，单位RAD */

/**
 * @brief 固定朝向转动速度解算
 *
 * @param pos_x 目标点在世界坐标系下的x轴坐标
 * @param pos_y 目标点在世界坐标系下的y轴坐标
 * @return 返回转动速度
 */
float constant_orientation_resolve(float pos_x, float pos_y) {
    float speedw = 0.0f;
    orientation_aim_angle =
        -atanf((pos_x - g_nuc_pos_data.x) / (pos_y - g_nuc_pos_data.y));
    /* 超域扩展 */
    if (pos_y - g_nuc_pos_data.y < 0) {
        orientation_aim_angle > 0 ? (orientation_aim_angle -= PI)
                                  : (orientation_aim_angle += PI);
    }

    /* 计算pid */
    float delta_angle =
        angle_trans(g_nuc_pos_data.yaw, RAD2DEG(orientation_aim_angle));
    speedw = pid_calc(&orientation_angle_pid, delta_angle, 0);
    return speedw;
}

/**
 * @brief 通过目标半径计算目标点位-生成跑点目标
 *
 * @param target_radium 目标半径
 * @return
 */
uint8_t chassis_overwrite_pointarray(int target_index) {
    float target_radium;
    float aim_x = 0.0f, aim_y = 0.0f;
label:
    target_radium = radium_speed[target_index][0];
    constant_orientation_resolve(BASKET_POINT_X, BASKET_POINT_Y);
    if (orientation_aim_angle >= 0 && orientation_aim_angle < PI / 2) {
        /* 第二象限 */
        aim_x = (target_radium - g_basket_radius) * sinf(orientation_aim_angle);
        aim_y =
            -(target_radium - g_basket_radius) * cosf(orientation_aim_angle);
    } else if (orientation_aim_angle >= PI / 2 && orientation_aim_angle < PI) {
        /* 第三象限 */
        aim_x = (target_radium - g_basket_radius) *
                sinf(PI - orientation_aim_angle);
        aim_y = (target_radium - g_basket_radius) *
                cosf(PI - orientation_aim_angle);
    } else if (orientation_aim_angle >= -PI &&
               orientation_aim_angle < -PI / 2) {
        /* 第四象限 */
        aim_x = -(target_radium - g_basket_radius) *
                sinf(PI - orientation_aim_angle);
        aim_y = (target_radium - g_basket_radius) *
                cosf(PI - orientation_aim_angle);
    } else {
        /* 第一象限 */
        aim_x =
            -(target_radium - g_basket_radius) * sinf(-orientation_aim_angle);
        aim_y =
            -(target_radium - g_basket_radius) * cosf(-orientation_aim_angle);
    }

    pos_array[POS_NUM + EX_NODE_TARGET_RADIUM].pos_x = g_nuc_pos_data.x + aim_x;
    pos_array[POS_NUM + EX_NODE_TARGET_RADIUM].pos_y = g_nuc_pos_data.y + aim_y;
    pos_array[POS_NUM + EX_NODE_TARGET_RADIUM].pos_yaw =
        RAD2DEG(orientation_aim_angle);
    /* 如果大圆环上的目标点位与场地边界相切，则选择较小圆环，这里的判断条件可能需要根据车启动的方位进行调整 */
    if ((pos_array[POS_NUM + EX_NODE_TARGET_RADIUM].pos_x <
         -1 * (SITH_WIDTH)) ||
        (pos_array[POS_NUM + EX_NODE_TARGET_RADIUM].pos_x > 400)) {
        target_index = (target_index > 0) ? target_index - 1 : 0;
        goto label;
    }
    return target_index;
}
/*****************************************************************************
 * @defgroup 底盘注册遥控函数组
 * @{
 */

/**
 * @brief 遥控器按键注册函数
 *
 * @param key 键值
 * @param key_event 事件
 */
void chassis_remote_key(uint8_t key, remote_key_event_t key_event) {
    static chassis_ctrl_queue_t msg;
    switch (key) {
        case CHASSIS_SET_HALT_KEY: {
            chassis_state.halt_flag = !chassis_state.halt_flag;
            chassis_set_halt(chassis_state.halt_flag);
        } break;
        case CHASSIS_AIMING_KEY: {
            chassis_set_aiming(CHASSIS_SHOOT_AIMING); /* 投篮点自瞄 */
        } break;
        case CHASSIS_RUN_POINT_KEY: {
#if 1
            switch (key_event) {
                case REMOTE_KEY_PRESS_DOWN:
                    /* 按下按键自动跑点 */
                    msg.timestap = HAL_GetTick();
                    msg.event = CHASSIS_SET_POINT;
                    xQueueSend(chassis_ctrl_queue, &msg, 0);
                    break;
                case REMOTE_KEY_PRESS_UP:
                    /* 松开按键切换手动 */
                    msg.timestap = HAL_GetTick();
                    msg.event = CHASSIS_SET_MANUAL;
                    xQueueSend(chassis_ctrl_queue, &msg, 0);
                    break;
                default:
                    break;
            }
#endif
        } break;
        case CHASSIS_WORLD_KEY: {
#if 0
            if (chassis_state.yaw_flag) {
                sub_chassis_world_yaw(&self_yaw);
            } else {
                sub_chassis_world_yaw(&g_action_pos_data.yaw);
            }
            chassis_state.yaw_flag = !chassis_state.yaw_flag;
#endif
            chassis_set_manual_ctrl();
        } break;
        case CHASSIS_SPDZ_CTR_KEY: {
            chassis_set_halt(false);
            /* 清除自瞄表示位防止到点后剧烈转动 */
            chassis_set_ctrl(CHASSIS_RESET_AIMING);
            if (chassis_state.spd_z == true) {
                spdz_ctr_flag = 1;
                chassis_state.spd_z = false;
            } else {
                spdz_ctr_flag = 0.12f;
                chassis_state.spd_z = true;
            }

        } break;
        default:
            break;
    }
    remote_report_data_t remote_report = REMOTE_REPORT_POSITION;
    xQueueOverwrite(remote_report_data_queue, &remote_report);
}

/**
 * @}底盘注册遥控函数组
 */

/*****************************************************************************
 * @defgroup 底盘控制任务
 * @{
 */

/**
 * @brief 底盘控制任务：用于处理底盘控制队列消息
 *        (给其他模块控制底盘的接口，主要用于接收main-ctrl的消息)
 * @param pvParametes
 */
void chassis_ctrl_task(void *pvParametes) {
    UNUSED(pvParametes);
    chassis_ctrl_queue_t chassis_ctrl;

    while (1) {
        if (xQueueReceive(chassis_ctrl_queue, &chassis_ctrl, portMAX_DELAY) ==
            pdFALSE) {
            continue;
        }
        if ((HAL_GetTick() - chassis_ctrl.timestap) > 100) {
            log_message(LOG_WARNING,
                        "[Chassis] Received message timeout, timegap: %u, "
                        "event: %d. ",
                        chassis_ctrl.timestap, chassis_ctrl.event);
            continue;
        }
        switch (chassis_ctrl.event) {
            case CHASSIS_SET_POINT: {
                /* 底盘自动控制 */
                vTaskSuspend(chassis_manual_ctrl_task_handle);
                vTaskResume(chassis_auto_ctrl_task_handle);
            } break;
            case CHASSIS_SET_MANUAL: {
                /* 底盘手控 */
                vTaskSuspend(chassis_auto_ctrl_task_handle);
                vTaskResume(chassis_manual_ctrl_task_handle);
            } break;
            case CHASSIS_SET_HALT:
                chassis_set_halt(1);
                break;
            case CHASSIS_SET_UNHALT:
                chassis_set_halt(0);
                break;

            case CHASSIS_SET_NO_TASK: {
                vTaskSuspend(chassis_auto_ctrl_task_handle);
                vTaskSuspend(chassis_manual_ctrl_task_handle);
                sub_chassis_speed(0, 0, 0);
            } break;

            case CHASSIS_SET_MIN_RADIUM: {
                chassis_set_point_run(
                    POS_NUM +
                    EX_NODE_TARGET_RADIUM); /* 启动自动跑点-设置跑点点位 */
            } break;
            case CHASSIS_LEFT_AIMING: {
                // chassis_state.point_index =
                //     POS_NUM + EX_NODE_LEFT; /* 将定点索引指向右侧装球点 */
                // pos_array[POS_NUM + EX_NODE_LEFT].pos_y = g_nuc_pos_data.y;

                // chassis_set_point_run(
                //     POS_NUM + EX_NODE_LEFT); /* 启动自动跑点-设置跑点点位 */
                chassis_set_aiming(
                    CHASSIS_LEFT_AIMING); /* 设置左侧装球点自瞄 */
            } break;
            case CHASSIS_RIGHT_AIMING: {
                // chassis_state.point_index =
                //     POS_NUM + EX_NODE_RIGHT; /* 将定点索引指向左侧装球点 */
                // pos_array[POS_NUM + EX_NODE_RIGHT].pos_y = g_nuc_pos_data.y;
                // chassis_set_point_run(
                //     POS_NUM + EX_NODE_RIGHT); /* 启动自动跑点-设置跑点点位 */
                chassis_set_aiming(
                    CHASSIS_RIGHT_AIMING); /* 设置右侧装球点自瞄 */
            } break;
            case CHASSIS_RESET_AIMING: {
                chassis_set_aiming(
                    CHASSIS_RESET_AIMING); /* 用于定点时关闭自瞄 */
            } break;
            default:
                break;
        }
    }
}

/**
 * @brief 底盘手动控制任务
 *
 * @param pvParametes UNUSED
 */
void chassis_manual_ctrl_task(void *pvParametes) {
    UNUSED(pvParametes);

    float speedx = 0.0, speedy = 0.0, speedz = 0.0;
    pid_init(&orientation_angle_pid, 300, 8, 0.0f, 500.0f, POSITION_PID, 3.2f,
             0.1f, 2.0f);

    while (1) {
        /* 处理遥控器遥感0点飘动 */
        speedy = (my_abs(g_remote_ctrl_data.rs[1]) < 3) ? 0
                 : (my_abs(g_remote_ctrl_data.rs[1]) < 15)
                     ? g_remote_ctrl_data.rs[1] * 200.0f
                     : g_remote_ctrl_data.rs[1] * 800 -
                           my_sign(g_remote_ctrl_data.rs[1]) * 9000.0f;

        speedx = (my_abs(g_remote_ctrl_data.rs[0]) < 3) ? 0
                 : (my_abs(g_remote_ctrl_data.rs[0]) < 15)
                     ? g_remote_ctrl_data.rs[0] * 200.0f
                     : g_remote_ctrl_data.rs[0] * 800 -
                           my_sign(g_remote_ctrl_data.rs[0]) * 9000.0f;

        speedz =
            chassis_state.collimation_flag
                ? constant_orientation_resolve(target_point_x, target_point_y) *
                      12
            : my_abs(g_remote_ctrl_data.rs[2]) < 3
                ? 0
                : g_remote_ctrl_data.rs[2] * 100 * spdz_ctr_flag;

        chassis_wheel_ctrl(speedx, speedy, speedz);

        vTaskDelay(1);
    }
}

pid_t radium_speed_pid;
pid_t radium_angle_pid;
pid_t load_ball_speed_pid;
pid_t load_ball_angle_pid;
/**
 * @brief 底盘自动控制任务(定点)
 *
 * @param pvParameters
 */
void chassis_auto_ctrl_task(void *pvParameters) {
    UNUSED(pvParameters);
    static uint32_t timeouts = 0;

    /* 初始化go_path相关变量 */
    go_path_chassis_ctrl_init(chassis_wheel_ctrl);
    go_path_location_init(LOCATION_TYPE_ACTION, &g_action_pos_data.x,
                          &g_action_pos_data.y, &g_action_pos_data.yaw);
    go_path_location_init(LOCATION_TYPE_NUC, &g_nuc_pos_data.x,
                          &g_nuc_pos_data.y, &g_nuc_pos_data.yaw);

    /* go_path中pid点位类型初始化 */
    pid_init(&nuc_flat_speed_pid, 3000, 1000, 0.0f, 50000.0f, POSITION_PID,
             1.5f, 0.1f, 0.0f);
    pid_init(&nuc_flat_angle_pid, 500, 15, 0.0f, 180.0f, POSITION_PID, 1.5f,
             0.01f, 0.5f);
    go_path_pidpoint_init(&nuc_flat_speed_pid, &nuc_flat_angle_pid, 3.0, 0.5,
                          POINT_TYPE_NUC_FLAT, LOCATION_TYPE_NUC);
    /* 跑环的pid*/
    pid_init(&radium_speed_pid, 500, 500 / 2, 0.0f, 50000.0f, POSITION_PID,
             1.5f / 5.0f, 0.1f, 0.0f);
    // pid_init(&radium_speed_pid, 2000, 500, 0.0f, 50000.0f, POSITION_PID, 3.0f,
    //          1.0f, 0.0f);
    pid_init(&radium_angle_pid, 200, 8, 0.0f, 500.0f, POSITION_PID, 3.2 * 10.0f,
             0.0f, 2.0 * 10.0f);
    go_path_pidpoint_init(&radium_speed_pid, &radium_angle_pid, 20.0, 0.5,
                          POINT_TYPE_TARGET_RADIUM, LOCATION_TYPE_NUC);
    /* 装球跑点pid初始化 */
    // pid_init(&load_ball_speed_pid, 5000, 5000 / 2, 0.0f, 50000.0f, POSITION_PID,
    //          15.0f, 5.0f, 0.0f);
    pid_init(&load_ball_speed_pid, 5000, 500 / 2, 0.0f, 50000.0f, POSITION_PID,
             15.0f, 5.0f, 0.0f);
    pid_init(&load_ball_angle_pid, 1000, 500 / 2, 0.0f, 50000.0f, POSITION_PID,
             5.0f, 0.5f, 0.0f);
    go_path_pidpoint_init(&load_ball_speed_pid, &load_ball_angle_pid, 20.0, 0.5,
                          POINT_TYPE_LOAD_BALL, LOCATION_TYPE_NUC);

    /* 默认挂起自动任务 */
    vTaskSuspend(chassis_auto_ctrl_task_handle);
    while (1) {
        constant_orientation_resolve(BASKET_POINT_X, BASKET_POINT_Y);
        pos_array[POS_NUM + EX_NODE_TARGET_RADIUM].pos_yaw =
            RAD2DEG(orientation_aim_angle);
        if (pos_array[chassis_state.point_index].pos_type ==
                POINT_TYPE_LOAD_BALL &&
            (g_nuc_pos_data.x < -7000 || g_nuc_pos_data.x > -800)) {
            chassis_set_status(CHASSIS_STATUS_POINT_ARRIVED);
            float speedx = 0.0, speedy = 0.0, speedz = 0.0;
            chassis_wheel_ctrl(speedx, speedy, speedz);
            chassis_set_manual_ctrl(); /* 切换手控 */
        }
        if (go_path_by_point(pos_array[chassis_state.point_index].pos_x,
                             pos_array[chassis_state.point_index].pos_y,
                             pos_array[chassis_state.point_index].pos_yaw,
                             pos_array[chassis_state.point_index].pos_type) ==
            GO_PATH_TARGET_ARRIVE) {
            timeouts++;
            if (timeouts >= 5) {
                timeouts = 0;
                /* 发送信号量更新底盘状态 */
                chassis_set_status(CHASSIS_STATUS_POINT_ARRIVED);
                chassis_set_manual_ctrl(); /* 切换手控 */
            }
        }
        vTaskDelay(1);
    }
}

/**
 * @} 底盘控制任务
 */

/**
 * @brief 初始化底盘相关函数
 *
 * @return * void
 */
void chassis_init(void) {
    /* 定义底盘控制函数 */
    chassis_wheel_ctrl = sub_chassis_speed;

    /* 创建底盘控制消息队列 */
    chassis_ctrl_queue = xQueueCreate(1, sizeof(chassis_ctrl_queue_t));
    if (chassis_ctrl_queue == NULL) {
        log_message(LOG_ERROR, "chassis_ctrl_queue faild!");
        return;
    }

    /* 创建底盘状态消息队列 */
    chassis_status_queue = xQueueCreate(1, sizeof(chassis_status_t));
    if (chassis_status_queue == NULL) {
        log_message(LOG_ERROR, "chassis_status_queue faild!");
        return;
    }

    BaseType_t task_create_res = pdFAIL;
    /* 创建底盘手控任务 */
    task_create_res =
        xTaskCreate(chassis_manual_ctrl_task, "chassis_manual_ctrl_task", 128,
                    NULL, 4, &chassis_manual_ctrl_task_handle);
    if (task_create_res != pdPASS) {
        log_message(LOG_ERROR, "chassis_manual_ctrl_task creat faild!");
        return;
    }
    /* 创建底盘自动控制任务 */
    task_create_res =
        xTaskCreate(chassis_auto_ctrl_task, "chassis_auto_ctrl_task", 128, NULL,
                    4, &chassis_auto_ctrl_task_handle);
    if (task_create_res != pdPASS) {
        log_message(LOG_ERROR, "chassis_auto_ctrl_task creat faild!");
        return;
    }
    /* 创建底盘控制任务 */
    task_create_res = xTaskCreate(chassis_ctrl_task, "chassis_ctrl_task", 128,
                                  NULL, 4, &chassis_ctrl_task_handle);
    if (task_create_res != pdPASS) {
        log_message(LOG_ERROR, "chassis_ctrl_task creat faild!");
        return;
    }

    /* 上电默认自锁 */
    chassis_set_halt(1);
    /* 默认开启世界坐标 */
    sub_chassis_world_yaw(&g_nuc_pos_data.yaw);
    /* 注册按键 */
    remote_register_key_callback(CHASSIS_AIMING_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_remote_key);
    remote_register_key_callback(CHASSIS_SET_HALT_KEY, REMOTE_KEY_PRESS_DOWN,
                                 chassis_remote_key);

#if 1 /* 固定点位跑点现在需要 */
    /*按键按下自动跑点松开自动切回手动任务*/
    remote_register_key_callback(CHASSIS_RUN_POINT_KEY, REMOTE_KEY_PRESS_DOWN,
                                 chassis_remote_key);
    remote_register_key_callback(CHASSIS_RUN_POINT_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_remote_key);
#endif

    remote_register_key_callback(CHASSIS_WORLD_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_remote_key);
    remote_register_key_callback(CHASSIS_SPDZ_CTR_KEY, REMOTE_KEY_PRESS_UP,
                                 chassis_remote_key);
    log_message(LOG_INFO, "chassis init OK!");
    printf("%f", g_basket_radius);
}
