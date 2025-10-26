/**
 * @file    logger.c
 * @author  Deadline039
 * @brief   日志记录系统
 * @version 1.2
 * @date    2024-03-25
 * @see     https://www.bilibili.com/video/av1250963900/?p=89
 */

#include "logger.h"

#include LOG_OUTPUT_STREAM_HEADER_FILE

#if LOG_SHOW_RUNNING_TIME
#include LOG_GET_RUNNING_TIME_HEADER_FILE
#endif /* LOG_SHOW_RUNNING_TIME */

#if LOG_USE_RTOS
#include "FreeRTOS.h"
#include "semphr.h"
static SemaphoreHandle_t buf_semp;
#endif /* LOG_USE_RTOS */

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static log_level_t current_level;
static char log_buffer[LOG_BUFFER_SIZE];

static const char log_level_str[6][11] = {
    "[DEBUG]   ", "[INFO]    ", "[WARNING] ",
    "[ERROR]   ", "[FATAL]   ", "[UNKNOW]  ",
};

/**
 * @brief 初始化日志系统
 * 
 * @param init_level 初始等级
 */
void log_init(log_level_t init_level) {
    if (init_level < LOG_UNKNOW) {
        current_level = init_level;
    }
#if LOG_USE_RTOS
#if LOG_USE_MUTEX
    buf_semp = xSemaphoreCreateMutex();
#else  /* LOG_USE_MUTEX */
    buf_semp = xSemaphoreCreateBinary();
#endif /* LOG_USE_MUTEX */
#endif /* LOG_USE_RTOS */
}

/**
 * @brief 日志消息
 *
 * @param level 日志等级
 * @param format 输出字符串
 * @note 如果日志级别低于设置的级别, 将不会输出
 */
void log_message(log_level_t level, char *format, ...) {
#if LOG_ENABLE

    unsigned int string_length = 0;
    if (level < current_level) {
        /* 日志等级低 */
        return;
    }

#if LOG_USE_RTOS
    xSemaphoreTake(buf_semp, portMAX_DELAY);
#endif /* LOG_USE_RTOS */

#if LOG_SHOW_RUNNING_TIME
    unsigned int milliseconds = LOG_GET_RUNNING_TIME();
    int hours = (int)((float)milliseconds / 1000.0f / 3600.0f);
    int minute = (int)((float)milliseconds / 1000.0f / 60.0f) % 60;
    float second = (float)milliseconds / 1000.0f - (float)hours * 3600 -
                   (float)minute * 60;

    snprintf((char *)log_buffer, sizeof(log_buffer), "[%d:%02d:%02.3f] ", hours,
             minute, second);
    string_length = strlen((char *)log_buffer);
#endif /* LOG_SHOW_RUNNING_TIME */

    if (level > LOG_UNKNOW) {
        level = LOG_UNKNOW;
    }

    strncpy((char *)&log_buffer[string_length], log_level_str[level],
            sizeof(log_buffer) - string_length);

    string_length = strlen((char *)log_buffer);

    va_list args;
    va_start(args, format);
    vsnprintf((char *)&log_buffer[string_length],
              sizeof(log_buffer) - string_length, (const char *)format, args);
    va_end(args);

    strncat((char *)log_buffer, LOG_OUTPUT_NEWLINE,
            sizeof(log_buffer) - strlen(log_buffer) - 1);
    string_length = strlen((char *)log_buffer);
    char *log_output_buf = log_buffer;

    LOG_OUTPUT_STREAM_FUNCTION(log_output_buf, string_length);

#if LOG_USE_RTOS
    xSemaphoreGive(buf_semp);
#endif /* LOG_USE_RTOS */

#endif /* LOG_ENABLE */
}

/**
 * @brief 设置当前日志级别
 *
 * @param level 日志等级
 */
void log_set_level(log_level_t level) {
    current_level = level;
}

/**
 * @brief 获取当前日志级别
 *
 * @return log_level_t 获取到的日志级别
 */
log_level_t log_get_level(void) {
    return current_level;
}
