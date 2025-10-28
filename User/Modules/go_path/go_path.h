/**
 * @file go_path.h
 * @author PickingChip 
 * @brief 跑点算法
 * @version 0.1
 * @date 2025-04-18
 * 
 */
#ifndef __GO_PATH_H
#define __GO_PATH_H

#include "pid/pid.h"

/* 解算结果处理函数指针 */
typedef void (*go_path_chassis_func_t)(float, float, float);

/* 跑点状态 */
typedef enum {
    GO_PATH_TARGET_NO_ARRIVE = 0,
    GO_PATH_TARGET_ARRIVE,
    GO_PATH_TARGET_POINT_TYPE_ERR,

    GO_PATH_RESERVE_STATUS_NUM
} go_path_arrive_status_t;

/* 点位类型 */
typedef enum {
    POINT_TYPE_NUC_FLAT = 0,
    POINT_TYPE_DT35,
    POINT_TYPE_TARGET_RADIUM,

    POINT_TYPE_NUM
} go_path_point_type_t;

/* 定位类型 */
typedef enum {
    LOCATION_TYPE_ACTION = 0,
    LOCATION_TYPE_DT35,
    LOCATION_TYPE_NUC,

    LOCATION_TYPE_NUM
} go_path_location_type_t;

/* 解算结果 */
typedef struct {
    float moving_velocity;           /*!< 平动速度 */
    float turning_velocity;          /*!< 旋转速度 */
    float speed_angle;               /*!< 平动速度与世界坐标轴之间的夹角 */
    go_path_arrive_status_t arrived; /*!< 底盘是否到达位置 */
} go_path_result_t;

float angle_trans(float self_angle, float target_angle);
void go_path_chassis_ctrl_init(go_path_chassis_func_t chassis_ctrl_func);
void go_path_location_init(go_path_location_type_t location_type, float *pos_x,
                           float *pos_y, float *pos_z);
void go_path_pidpoint_init(pid_t *speed_pid, pid_t *angle_pid,
                           float distance_deadband, float angle_deadband,
                           go_path_point_type_t point_type,
                           go_path_location_type_t location_type);

go_path_arrive_status_t go_path_by_point(float target_x, float target_y,
                                         float target_yaw,
                                         go_path_point_type_t point_type);

#endif /* __GO_PATH_H */