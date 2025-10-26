/**
 * @file    logger.h
 * @author  Deadline039
 * @brief   日志记录系统
 * @version 1.2
 * @date    2024-03-25
 * @see     https://www.bilibili.com/video/av1250963900/?p=89
 *
 ******************************************************************************
 *    Date    | Version |   Author    | Version Info
 * -----------+---------+-------------+----------------------------------------
 * 2024-04-01 |   1.0   | Deadline039 | 初版
 * 2024-04-04 |   1.1   | Deadline039 | 更改 LOG_ENABLE 宏, 如果关闭此宏开关, 
 *            |         |             | 日志函数会变为空函数
 * 2525-03-14 |   1.2   | Deadline039 | 修改输出接口和获取时间接口, 缓冲区加信号量保护, 
 *            |         |             | 修改部分宏定义, 移除 RTC 与 FATFS. 
 */

#ifndef __LOGGER_H
#define __LOGGER_H

/** 日志系统配置 *************************************************************/

/* 是否使用日志, 关闭后函数会变为空函数 */
#define LOG_ENABLE                    1

/******************************************************************************
 * @defgroup 输出相关配置
 * @{
 */

/* 输出流头文件 */
// #define LOG_OUTPUT_STREAM_HEADER_FILE <stdio.h>
// /* 输出函数 */
// #define LOG_OUTPUT_STREAM_FUNCTION(str, len)                                   \
//     do {                                                                       \
//         while (len--) {                                                        \
//             putchar(*str++);                                                   \
//         }                                                                      \
//     } while (0)

/* 输出流头文件 */
#define LOG_OUTPUT_STREAM_HEADER_FILE <bsp.h>
/* 输出函数 */
#define LOG_OUTPUT_STREAM_FUNCTION(str, len)                                   \
    do {                                                                       \
        while (len--) {                                                        \
            ITM_SendChar(*str++);                                              \
        }                                                                      \
    } while (0)

/* 输出缓冲区大小 */
#define LOG_BUFFER_SIZE       128
/* 输出末尾换行符 */
#define LOG_OUTPUT_NEWLINE    "\n"

/* 输出缓冲区是一个全局变量, 需要加信号量保护, 需要保证日志在调度器启动后初始化 */
#define LOG_USE_RTOS          1
/* 使用互斥信号量, 需要保证日志不在中断中使用 */
#define LOG_USE_MUTEX         1

/**
 * @}
 */

/******************************************************************************
 * @defgroup 时间显示相关配置
 * @{
 */

/* 是否显示运行时间 */
#define LOG_SHOW_RUNNING_TIME 1

#if LOG_SHOW_RUNNING_TIME
/* 获取时间函数头文件 */
#define LOG_GET_RUNNING_TIME_HEADER_FILE <bsp.h>
/* 获取时间函数, 返回值需要是微秒 */
#define LOG_GET_RUNNING_TIME()           HAL_GetTick()
#endif /* LOG_SHOW_RUNNING_TIME */

/**
 * @}
 */

/**
 * @brief 日志级别
 */
typedef enum {
    LOG_DEBUG,   /*!< 调试信息 */
    LOG_INFO,    /*!< 一般信息 */
    LOG_WARNING, /*!< 警告信息 */
    LOG_ERROR,   /*!< 错误信息 */
    LOG_FATAL,   /*!< 致命错误 */
    LOG_UNKNOW   /*!< 未知信息 */
} log_level_t;

void log_init(log_level_t init_level);
void log_message(log_level_t level, char *format, ...);
void log_set_level(log_level_t level);

log_level_t log_get_level(void);

#endif /* __LOGGER_H */
