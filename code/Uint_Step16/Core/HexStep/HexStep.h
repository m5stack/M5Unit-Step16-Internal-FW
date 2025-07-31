/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HEXSTEP_HEXSTEP_H_
#define HEXSTEP_HEXSTEP_H_

#ifdef __cplusplus

extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "time.h"

#define FILTER_WINDOW_DEFAULT_SIZE (10)
#define SAMPLE_INTERVAL_MS         (10)

typedef struct {
    uint8_t buffer[FILTER_WINDOW_DEFAULT_SIZE];
    uint8_t index;
    uint8_t stable_value;
} filter_t;

void filter_init(void);
void sample_data(void);
void update_step16_value(void);
void displayHex(void);

#ifdef __cplusplus
}
#endif

#endif /* HEXSTEP_HEXSTEP_H_ */
