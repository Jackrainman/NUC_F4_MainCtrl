/**
 * @file go_path.c
 * @author PickingChip
 * @brief 跑点算法
 * @version 0.1
 * @date 2025-04-18
 *
 */
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "my_math/my_math.h"
#include "go_path.h"

/* pid类型 */
enum {
    SPEED_PID = 0,
    ANGLE_PID,
    pid_num,
};

/* 底盘位置数据 */
typedef struct {
    float *chassis_x;   /*!< 车身x轴坐标 */
    float *chassis_y;   /*!< 车身y轴坐标 */
    float *chassis_yaw; /*!< 车身yaw角 */
} pos_point_t;

/* 跑点状态数据 */
static struct {
    float target_x;                        /*!< 目标点x轴坐标 */
    float target_y;                        /*!< 目标点y轴坐标 */
    float target_yaw;                      /*!< 目标点yaw角 */
    go_path_location_type_t location_type; /*!< 定位类型 */
    float distance_deadband;               /*!< 平动死区 */
    float angle_deadband;                  /*!< 角度死区 */
    pos_point_t chassis_position;          /*!< 当前底盘点位坐标 */
    pid_t *running_pid[pid_num];           /*!< 跑点pid组 */
} go_path_state[POINT_TYPE_NUM];

/* 底盘在一个机器人上只有一个，不需要每个点注册一次 */
go_path_chassis_func_t go_path_chassis_ctrl;

/* 定位解算对应的类型位置存储 */
pos_point_t location_pos[LOCATION_TYPE_NUM];

/* 解算结果 */
go_path_result_t go_path_result = {0};

/**
 * @brief 角度优化计算,选择劣弧转动
 *
 * @param x 当前角度
 * @param y 目标角度
 * @return 选的的角度
 */
float angle_trans(float self_angle, float target_angle) {
    float tmp, res1, res2;
    tmp = self_angle - target_angle;
    res1 = tmp > 0 ? tmp - 360.0f : tmp + 360.0f;
    res2 = tmp;
    tmp = my_fabs(res1) < my_fabs(res2) ? res1 : res2;
    return tmp;
}

/**
 * @brief go_path_chassis_ctrl_init 注册底盘控制函数
 * 
 * @param chassis_ctrl_func 控制底盘的函数指针
 * @return * void 
 */
void go_path_chassis_ctrl_init(go_path_chassis_func_t chassis_ctrl_func) {
    go_path_chassis_ctrl = chassis_ctrl_func;
}

/**
 * @brief 初始化坐标系定位
 * 
 * @param location_type 定位方案和对应位置指针
 * @param pos_x x方位pos
 * @param pos_y y方位pos
 * @param pos_z angle方位pos
 */
void go_path_location_init(go_path_location_type_t location_type, float *pos_x,
                           float *pos_y, float *pos_z) {
    if (location_type >= LOCATION_TYPE_NUM) {
        return;
    }
    location_pos[location_type].chassis_x = pos_x;
    location_pos[location_type].chassis_y = pos_y;
    location_pos[location_type].chassis_yaw = pos_z;
}

/**
 * @brief 跑点初始化函数
 * 
 * @param speed_pid 平动pid
 * @param angle_pid 转动pid
 * @param distance_deadband 平动死区 
 * @param angle_deadband 角度死区
 * @param point_type 点位类型
 * @param location_type 定位类型
 */
void go_path_pidpoint_init(pid_t *speed_pid, pid_t *angle_pid,
                           float distance_deadband, float angle_deadband,
                           go_path_point_type_t point_type,
                           go_path_location_type_t location_type) {
    if (point_type >= POINT_TYPE_NUM) {
        /* 超域 */
        return;
    }
    if (location_type >= LOCATION_TYPE_NUM) {
        /* 超域 */
        return;
    }
    go_path_state[point_type].target_x = 0.0f;
    go_path_state[point_type].target_y = 0.0f;
    go_path_state[point_type].target_yaw = 0.0f;
    go_path_state[point_type].distance_deadband = distance_deadband;
    go_path_state[point_type].angle_deadband = angle_deadband;
    go_path_state[point_type].location_type = location_type;
    go_path_state[point_type].running_pid[0] = speed_pid;
    go_path_state[point_type].running_pid[1] = angle_pid;

    go_path_state[point_type].chassis_position.chassis_x =
        location_pos[location_type].chassis_x;
    go_path_state[point_type].chassis_position.chassis_y =
        location_pos[location_type].chassis_y;
    go_path_state[point_type].chassis_position.chassis_yaw =
        location_pos[location_type].chassis_yaw;
}

/**
 * @brief action定位控制函数。
 *
 * @param point_type
 */
static void action_pid_control(go_path_point_type_t point_type) {
    bool arrive_xy = false;  /*!< 到达目标点坐标标志位 */
    bool arrive_yaw = false; /*!< 到达目标点角度标志位 */
    /* 平动速度解算 */
    float delta_distance = two_dimensions(
        *go_path_state[point_type].chassis_position.chassis_x,
        *go_path_state[point_type].chassis_position.chassis_y,
        go_path_state[point_type].target_x, go_path_state[point_type].target_y);

    if (delta_distance > go_path_state[point_type].distance_deadband) {
        go_path_result.moving_velocity =
            pid_calc(go_path_state[point_type].running_pid[SPEED_PID],
                     delta_distance, 0);
        /* 反三角函数输出的范围仅有0~pi,需要映射到-pi~pi */
        if ((go_path_state[point_type].target_y -
             *go_path_state[point_type].chassis_position.chassis_y) > 0) {
            go_path_result.speed_angle =
                acosf((go_path_state[point_type].target_x -
                       *go_path_state[point_type].chassis_position.chassis_x) /
                      delta_distance);
        } else {
            go_path_result.speed_angle =
                -acosf((go_path_state[point_type].target_x -
                        *go_path_state[point_type].chassis_position.chassis_x) /
                       delta_distance);
        }
        arrive_xy = false;
    } else {
        go_path_result.moving_velocity = 0.0f;
        go_path_result.speed_angle = 0.0f;
        go_path_state[point_type].running_pid[SPEED_PID]->iout = 0;
        arrive_xy = true;
    }
    /* 转动速度解算 */
    float delta_angle =
        angle_trans(*go_path_state[point_type].chassis_position.chassis_yaw,
                    go_path_state[point_type].target_yaw);
    if (math_compare_float(my_fabs(delta_angle),
                           go_path_state[point_type].angle_deadband) ==
        MATH_FP_MORETHAN) {
        go_path_result.turning_velocity =
             pid_calc(go_path_state[point_type].running_pid[ANGLE_PID],
                          delta_angle, 0);
        arrive_yaw = false;
    } else {
        go_path_result.turning_velocity = 0.0f;
        go_path_state[point_type].running_pid[ANGLE_PID]->iout = 0;
        arrive_yaw = true;
    }
    /* 运动状态更新 */
    if (arrive_xy && arrive_yaw) {
        go_path_result.arrived = GO_PATH_TARGET_ARRIVE;
    } else {
        go_path_result.arrived = GO_PATH_TARGET_NO_ARRIVE;
    }
}

/**
 * @brief dt35定位控制函数。
 *
 * @param point_type
 */
void dt35_pid_control(go_path_point_type_t point_type) {}

/**
 * @brief 跑点单点函数，将计算结果通过go_path_result结构体传递出去
 *
 * @param target_x 目标点x轴坐标
 * @param target_y 目标点y轴坐标
 * @param target_yaw 目标点车身角度
 * @param point_type 目标点的类型
 * @return 底盘跑点状态
 */
go_path_arrive_status_t go_path_by_point(float target_x, float target_y,
                                         float target_yaw,
                                         go_path_point_type_t point_type) {
    if (point_type >= POINT_TYPE_NUM) {
        /* 超域 */
        return GO_PATH_TARGET_POINT_TYPE_ERR;
    }
    float speedx = 0.0f, speedy = 0.0f, speedw = 0.0f;

    go_path_state[point_type].target_x = target_x;
    go_path_state[point_type].target_y = target_y;
    go_path_state[point_type].target_yaw = target_yaw;
    switch (go_path_state[point_type].location_type) {
        case LOCATION_TYPE_ACTION:
            action_pid_control(point_type);
            break;
        case LOCATION_TYPE_DT35:
            dt35_pid_control(point_type);
            break;
        case LOCATION_TYPE_NUC:
            action_pid_control(point_type);
            break;
        default:
            break;
    }
    speedx = go_path_result.moving_velocity * cosf(go_path_result.speed_angle);
    speedy = go_path_result.moving_velocity * sinf(go_path_result.speed_angle);
    speedw = go_path_result.turning_velocity;

    go_path_chassis_ctrl(speedx, speedy, speedw);

    return go_path_result.arrived;
}