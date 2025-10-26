/**
 * @file    RTC_STM32F4xx.h
 * @author  Deadline039
 * @brief   Chip Support Package of RTC on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 * @note    We used C time library to process the time. The hardware
 *          configuration reference the ST: AN3371.
 *          You should attention the time zone if you want to set time by NTP.
 */

#ifndef __RTC_STM32F4XX_H
#define __RTC_STM32F4XX_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <time.h>

#if RTC_ENABLE

extern RTC_HandleTypeDef rtc_handle;

#define RTC_INIT_OK     0
#define RTC_INIT_FAIL   1
#define RTC_INIT_RESET  2
#define RTC_INITED      3

#define RTC_DEINIT_OK   0
#define RTC_DEINIT_FAIL 1
#define RTC_NO_INIT     2

#define IS_LEAP_YEAR(YEAR)                                                     \
    (((((YEAR) % 4 == 0) && ((YEAR) % 100) != 0) || ((YEAR) % 400 == 0)) ? 1   \
                                                                         : 0)
uint8_t rtc_init(void);
uint8_t rtc_deinit(void);

uint8_t rtc_get_week(uint16_t year, uint8_t month, uint8_t day);

time_t rtc_get_time_t(void);
struct tm *rtc_get_time(void);

void rtc_set_time_t(const time_t *_time);
void rtc_set_time(struct tm *_tm);

#endif /* RTC_ENABLE */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __RTC_STM32F4XX_H */
