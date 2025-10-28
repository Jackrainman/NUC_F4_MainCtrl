/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"
#include "logger/logger.h"
#include "message-protocol/msg_protocol.h"

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task1_handle;
void task1(void *pvParameters);

/*****************************************************************************/

/**
 * @brief FreeRTOS start up.
 *
 */
void freertos_start(void) {
    xTaskCreate(start_task, "start_task", 256, NULL, 2, &start_task_handle);
    vTaskStartScheduler();
}

/**
 * @brief Start up task.
 *
 * @param pvParameters Start parameters.
 */
void start_task(void *pvParameters) {
    UNUSED(pvParameters);
    taskENTER_CRITICAL();
    /* 初始化log */
    log_init(LOG_INFO);

    /* 初始化Can-List */
    can_list_add_can(can1_selected, 4, 4);
    can1_init(1000, 350);

    /* 创建遥控器数据上报任务 */
    xTaskCreate(remote_report_task, "remote_report_task", 256, NULL, 3,
                &remote_report_task_handle);

    xTaskCreate(msg_polling_task, "msg_polling_task", 256, NULL, 4,
                &msg_polling_task_handle);

    /* 初始化底盘 */
    chassis_init();

    /* 初始化运球 */
    dribble_init();

    /* 初始化发射控制 */
    shoot_ctrl_init();

    main_ctrl_init();

    /* 创建led流水灯任务(偶尔可以检测板子任务正不正常) */
    xTaskCreate(task1, "task1", 128, NULL, 2, &task1_handle);
    /* 创建后台任务 */
    xTaskCreate(sub_pub_task, "sub_pub_task", 256, NULL, 4,
                &sub_pub_task_handle);

    vTaskDelete(start_task_handle);
    taskEXIT_CRITICAL();
}

/**
 * @brief Task1: Blink.
 *
 * @param pvParameters Start parameters.
 */
void task1(void *pvParameters) {
    UNUSED(pvParameters);
    LED0_OFF();

    while (1) {
        LED0_TOGGLE(); /* LED0闪烁 */

        vTaskDelay(1000);
    }
}

#ifdef configASSERT

/**
 * @brief FreeRTOS assert failed function.
 *
 * @param pcFile File name
 * @param ulLine File line
 */
void vAssertCalled(const char *pcFile, unsigned int ulLine) {
    fprintf(stderr, "FreeRTOS assert failed. File: %s, line: %u. \n", pcFile,
            ulLine);
    __BKPT();
}
#endif /* configASSERT */

#if configCHECK_FOR_STACK_OVERFLOW

/**
 * @brief The application stack overflow hook is called when a stack overflow is detected for a task.
 *
 * @param xTask the task that just exceeded its stack boundaries.
 * @param pcTaskName A character string containing the name of the offending task.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    UNUSED(xTask);
    fprintf(stderr, "Stack overflow! Taskname: %s. \n", pcTaskName);
    __BKPT();
}

#endif /* configCHECK_FOR_STACK_OVERFLOW */

#if configUSE_MALLOC_FAILED_HOOK

/**
 * @brief This hook function is called when allocation failed.
 *
 */
void vApplicationMallocFailedHook(void) {
    fprintf(stderr, "FreeRTOS malloc failed! \n");
    __BKPT();
}

#endif /* configUSE_MALLOC_FAILED_HOOK */
