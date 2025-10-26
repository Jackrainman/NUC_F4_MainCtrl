/**
 * @file    ETH_STM32F4xx.h
 * @author  Deadline039
 * @brief   Chip Support Package of Ethernet on STM32F4xx
 * @version 3.3.3
 * @date    2025-05-19
 * @ref     https://github.com/libdriver/lan8720/
 */

#ifndef __ETH_STM32F4XX_H
#define __ETH_STM32F4XX_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup ETH
 * @{
 */

#if ETH_ENABLE

#define ETH_CHIP_ADDR ((uint16_t)0x0000U)

extern ETH_HandleTypeDef eth_handle;

uint8_t eth_init(uint8_t mac[6]);
uint8_t eth_deinit(void);

uint32_t eth_read_phy(uint16_t reg);
void eth_write_phy(uint16_t reg, uint16_t value);

#endif /* ETH_ENABLE */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ETH_STM32F4XX_H */
