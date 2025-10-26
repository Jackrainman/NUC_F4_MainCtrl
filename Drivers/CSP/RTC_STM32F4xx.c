/**
 * @file    RTC_STM32F4xx.c
 * @author  Deadline039
 * @brief   Chip Support Package of RTC on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 */

#include <CSP_Config.h>

#include "RTC_STM32F4xx.h"

RTC_HandleTypeDef rtc_handle;

#define RTC_USE_LSE 0x8800 /* Configuration to use external clock. */
#define RTC_USE_LSI 0x8801 /* Configuration to use internal clock. */

/**
 * @brief RTC initialization.
 *
 * @return RTC init status:
 *  @retval - 0: `RTC_INIT_OK`:      Success.
 *  @retval - 1: `RTC_INIT_FAIL`:    RTC init failed.
 *  @retval - 2: `RTC_INIT_RESET`:   RTC clock is reseted.
 *  @retval - 3: `RTC_INITED`:       RTC is inited.
 */
uint8_t rtc_init(void) {
    if (HAL_RTC_GetState(&rtc_handle) != HAL_RTC_STATE_RESET) {
        return RTC_INITED;
    }

    uint16_t bkp_flag = 0;

    rtc_handle.Instance = RTC;
    rtc_handle.Init.HourFormat = RTC_HOURFORMAT_24;
    rtc_handle.Init.AsynchPrediv = 0x7F;
    rtc_handle.Init.SynchPrediv = 0xFF;
    rtc_handle.Init.OutPut = RTC_OUTPUT_DISABLE;
    rtc_handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    rtc_handle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    /* Check the backup flag. */
    bkp_flag = HAL_RTCEx_BKUPRead(&rtc_handle, RTC_BKP_DR0);

    if (HAL_RTC_Init(&rtc_handle) != HAL_OK) {
        return RTC_INIT_FAIL;
    }

    if ((bkp_flag != RTC_USE_LSE) && (bkp_flag != RTC_USE_LSI)) {
        time_t init_time = 946684800;
        rtc_set_time_t(&init_time);
        return RTC_INIT_RESET;
    }

    return RTC_INIT_OK;
}

/**
 * @brief RTC deinit
 *
 * @return RTC deinit status:
 *  @retval - 0: `RTC_DEINIT_OK`:    Success.
 *  @retval - 1: `RTC_DEINIT_FAIL`:  RTC deinit failed.
 *  @retval - 3: `RTC_NO_INIT`:      RTC is not init.
 */
uint8_t rtc_deinit(void) {
    if (HAL_RTC_GetState(&rtc_handle) == HAL_RTC_STATE_RESET) {
        return RTC_NO_INIT;
    }

    if (HAL_RTC_DeInit(&rtc_handle) != HAL_OK) {
        return RTC_DEINIT_FAIL;
    }

    return RTC_DEINIT_OK;
}

/**
 * @brief RTC low level initialization.
 *
 * @param hrtc The handle of RTC.
 */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc) {
    uint16_t retry = 200;

    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTC_ENABLE();

    RCC_OscInitTypeDef rcc_osc_initstruct;
    RCC_PeriphCLKInitTypeDef rcc_periphclk_initstruct;

    RCC->BDCR |= 1 << 0; /* Open external low speed clock. */

    while (retry && ((RCC->BDCR & 0X02) == 0)) {
        /* Wait LSE ready. */
        retry--;
        HAL_Delay(5);
    }

    if (retry == 0) {
        /* External clock startup failed, use interal clock. */
        rcc_osc_initstruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
        rcc_osc_initstruct.LSEState = RCC_LSI_ON;
        rcc_osc_initstruct.PLL.PLLState = RCC_PLL_NONE;
        HAL_RCC_OscConfig(&rcc_osc_initstruct);

        rcc_periphclk_initstruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
        rcc_periphclk_initstruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
        HAL_RCCEx_PeriphCLKConfig(&rcc_periphclk_initstruct);
        HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR0, RTC_USE_LSI);
    } else {
        rcc_osc_initstruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
        rcc_osc_initstruct.LSEState = RCC_LSE_ON;
        rcc_osc_initstruct.PLL.PLLState = RCC_PLL_NONE;
        HAL_RCC_OscConfig(&rcc_osc_initstruct);

        rcc_periphclk_initstruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
        rcc_periphclk_initstruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
        HAL_RCCEx_PeriphCLKConfig(&rcc_periphclk_initstruct);
        HAL_RTCEx_BKUPWrite(hrtc, RTC_BKP_DR0, RTC_USE_LSE);
    }

#if RTC_WKUP_IT_ENABLE
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, RTC_WKUP_IT_PRIORITY, RTC_WKUP_IT_SUB);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
#endif /* RTC_WKUP_IT_ENABLE */

#if RTC_ALARM_IT_ENABLE
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, RTC_ALARM_IT_PRIORITY,
                         RTC_ALARM_IT_SUB);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
#endif /* RTC_ALARM_IT_ENABLE */
}

/**
 * @brief RTC low level deinitialization.
 *
 * @param hrtc The handle of RTC.
 */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc) {
    UNUSED(hrtc);

    __HAL_RCC_RTC_DISABLE();

#if RTC_WKUP_IT_ENABLE
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
#endif /* RTC_WKUP_IT_ENABLE */

#if RTC_ALARM_IT_ENABLE
    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
#endif /* RTC_ALARM_IT_ENABLE */
}

#if RTC_WKUP_IT_ENABLE

/**
 * @brief RTC wake up ISR.
 *
 */
void RTC_WKUP_IRQHandler(void) {
    HAL_RTCEx_WakeUpTimerIRQHandler(&rtc_handle);
}

#endif /* RTC_WKUP_IT_ENABLE */

#if RTC_ALARM_IT_ENABLE

/**
 * @brief RTC alarm ISR.
 *
 */
void RTC_Alarm_IRQHandler(void) {
    HAL_RTC_AlarmIRQHandler(&rtc_handle);
}

#endif /* RTC_ALARM_IT_ENABLE */

static const uint8_t month_day_table[12] = {31, 28, 31, 30, 31, 30,
                                            31, 31, 30, 31, 30, 31};

/**
 * @brief Get the week day of specific date.
 *
 * @param year Year
 * @param month Month
 * @param day Day
 * @return Weekday:
 *  @retval - 0:     Sunday
 *  @retval - 1 ~ 6: Monday to Saturday.
 * @note Use Kim Larsen calculation formula. You can
 *       see: https://www.cnblogs.com/fengbohello/p/3264300.html
 */
uint8_t rtc_get_week(uint16_t year, uint8_t month, uint8_t day) {
    uint8_t week = 0;

    if (month < 3) {
        month += 12;
        --year;
    }

    week = (day + 1 + 2 * month + 3 * (month + 1) / 5 + year + (year >> 2) -
            year / 100 + year / 400) %
           7;
    return week;
}

/**
 * @brief Get now timestamp (Unix timestamp).
 *
 * @return Unix timestamp.
 */
time_t rtc_get_time_t(void) {
    return time(NULL);
}

/**
 * @brief Get now time with time structure.
 *
 * @return The now time.
 */
struct tm *rtc_get_time(void) {
    static struct tm now_time;
    RTC_TimeTypeDef rtc_time;
    RTC_DateTypeDef rtc_date;
    HAL_RTC_GetTime(&rtc_handle, &rtc_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&rtc_handle, &rtc_date, RTC_FORMAT_BIN);

    /* In STM32, the Year range is 0 - 99, in other words 2000 - 2099. But
     * `tm_year` range is now year before 1900, so add 100 there. */
    now_time.tm_year = rtc_date.Year + 100;
    now_time.tm_mon = rtc_date.Month - 1;
    now_time.tm_mday = rtc_date.Date;
    now_time.tm_wday = rtc_date.WeekDay - 1;
    /* Calculate today's day of the year. */
    for (int i = 0; i < now_time.tm_mon; ++i) {
        now_time.tm_yday += month_day_table[i];
    }

    /* If it is a leap year and the month is greater than 2, add 1 day. */
    if (now_time.tm_mon >= 2 && IS_LEAP_YEAR(1900 + now_time.tm_year)) {
        now_time.tm_yday += 1;
    }

    now_time.tm_isdst = HAL_RTC_DST_ReadStoreOperation(&rtc_handle);

    now_time.tm_hour = rtc_time.Hours;
    now_time.tm_min = rtc_time.Minutes;
    now_time.tm_sec = rtc_time.Seconds;

    return &now_time;
}

/**
 * @brief Set time by timestamp.
 *
 * @param _time Now time timestamp.
 */
void rtc_set_time_t(const time_t *_time) {
    struct tm *time_struct = localtime(_time);
    rtc_set_time(time_struct);
}

/**
 * @brief Set time by time structure.
 *
 * @param _tm Now time structure.
 */
void rtc_set_time(struct tm *_tm) {
    RTC_DateTypeDef rtc_date = {0};
    RTC_TimeTypeDef rtc_time = {0};

    rtc_date.Year = _tm->tm_year - 100;
    rtc_date.Month = _tm->tm_mon + 1;
    rtc_date.Date = _tm->tm_mday;
    rtc_date.WeekDay = _tm->tm_wday + 1;

    rtc_time.Hours = _tm->tm_hour;
    rtc_time.Minutes = _tm->tm_min;
    rtc_time.Seconds = _tm->tm_sec;

    /* Process the leap second, substrcut 1 second, and delay 1 second. */
    if (_tm->tm_sec == 60) {
        rtc_time.Seconds = 59;
        HAL_Delay(1000);
    }

    HAL_RTC_SetDate(&rtc_handle, &rtc_date, RTC_FORMAT_BIN);
    HAL_RTC_SetTime(&rtc_handle, &rtc_time, RTC_FORMAT_BIN);

    if (_tm->tm_isdst) {
        /* Daylight Saving Time. */
        HAL_RTC_DST_SetStoreOperation(&rtc_handle);
    } else {
        HAL_RTC_DST_ClearStoreOperation(&rtc_handle);
    }
}

/**
 * @brief Redefine the C library time funciton.
 *
 * @param[out] _time The timestamp to receive.
 * @return Now timestamp.
 */
time_t time(time_t *_time) {
    time_t now_time = mktime(rtc_get_time());
    if (_time != NULL) {
        *_time = now_time;
    }
    return now_time;
}
