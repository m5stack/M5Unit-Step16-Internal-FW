/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __RGB_H
#define __RGB_H

#ifdef __cplusplus

extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"

#define RGB_NUM (1)

void rgb_init(void);
void rgb_show(void);
void rgb_off(void);
void update_rgb_buffer(void);
void update_rgb_status(void);

#ifdef __cplusplus
}
#endif

#endif /*__ RGB_H */
