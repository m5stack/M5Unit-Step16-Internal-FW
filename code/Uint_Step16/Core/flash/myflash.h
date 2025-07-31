/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __MYFLASH_H
#define __MYFLASH_H

#ifdef __cplusplus

extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include "stm32g0xx_hal_flash_ex.h"

#define STM32G0xx_PAGE_SIZE              (0x800)      // Page size: 2048 bytes (2 KB)
#define STM32G0xx_FLASH_PAGE0_STARTADDR  (0x8000000)  // Start address of flash page 0
#define STM32G0xx_FLASH_PAGE13_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR + 13 * STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE14_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR + 14 * STM32G0xx_PAGE_SIZE)
#define STM32G0xx_FLASH_PAGE15_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR + 15 * STM32G0xx_PAGE_SIZE)
#define STEP16_LED_STATUS_ADDR           (STM32G0xx_FLASH_PAGE13_STARTADDR + 0)
#define STEP16_LED_LIGHT_ADDR            (STM32G0xx_FLASH_PAGE13_STARTADDR + 1)
#define STEP16_SWITH_ADDR                (STM32G0xx_FLASH_PAGE13_STARTADDR + 2)
#define RGB_STATUS_ADDR                  (STM32G0xx_FLASH_PAGE14_STARTADDR + 0)
#define RGB_BRIGHTNESS_ADDR              (STM32G0xx_FLASH_PAGE14_STARTADDR + 1)
#define I2C_ADDR                         (STM32G0xx_FLASH_PAGE15_STARTADDR + 0)

uint8_t get_step16_led_status(void);
uint8_t get_step16_led_light(void);
uint8_t get_step16_switch(void);
uint8_t get_step16_sensitivity(void);
bool write_step16_flash(uint8_t step16_led_status, uint8_t step16_led_light, uint8_t step16_switch);

uint8_t get_rgb_status(void);
uint8_t get_rgb_brightness(void);
bool write_rgb_flash(uint8_t rgb_status, uint8_t rgb_brightness);

uint8_t get_i2c_addr(void);
bool set_i2c_addr(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __MYFLASH_H */
