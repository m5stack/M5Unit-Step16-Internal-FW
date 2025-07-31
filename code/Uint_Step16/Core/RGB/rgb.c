#include "rgb.h"
#include "stdlib.h"
#include <stdbool.h>

#define gpio_low()  GPIOB->BRR = GPIO_PIN_5
#define gpio_high() GPIOB->BSRR = GPIO_PIN_5

#define delay_150ns() \
    __asm("NOP");     \
    __asm("NOP")

#define delay_300ns() \
    delay_150ns();    \
    delay_150ns();

#define delay_600ns() \
    delay_300ns();    \
    delay_150ns();    \
    __asm("NOP");      \
    __asm("NOP");      \
    __asm("NOP");

#define out_bit_low() \
    gpio_high();      \
    delay_300ns();    \
    gpio_low()

#define out_bit_high() \
    gpio_high();       \
    delay_600ns();     \
    gpio_low()

#define restart()                           \
    do {                                    \
        for (uint8_t i = 0; i < 255; i++) { \
            delay_600ns();                  \
        }                                   \
    } while (0)

// External RGB control registers (DO NOT MODIFY)
extern __IO uint8_t rgb_status_reg;
extern __IO uint8_t rgb_brightness_reg;
extern __IO uint8_t rgb_value_reg[3];
extern __IO uint8_t rgb_value_update;

uint8_t last_rgb_status     = 0xFF;
uint8_t last_rgb_brightness = 0xFF;

__IO uint32_t color_buf = 0;

__IO uint32_t rgb_off_buf = 0;

void gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};                     // Structure to hold GPIO initialization parameters
    GPIO_InitStruct.Pin              = GPIO_PIN_5;              // Specify the GPIO pin to configure
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;     // Set pin mode to output push-pull
    GPIO_InitStruct.Pull             = GPIO_NOPULL;             // No internal pull-up or pull-down
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_MEDIUM;  // Set speed to medium frequency
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                     // Initialize GPIO with the specified settings

    if (rgb_status_reg == 1) {
        HAL_GPIO_WritePin(RGB_VDD_EN_GPIO_Port, RGB_VDD_EN_Pin, GPIO_PIN_SET);
    } else if (rgb_status_reg == 0) {
        HAL_GPIO_WritePin(RGB_VDD_EN_GPIO_Port, RGB_VDD_EN_Pin, GPIO_PIN_RESET);
    }
}

static void rgb_send_data(uint32_t color)
{
    for (uint8_t i = 0; i < 24; i++)  // Loop through each bit of the 24-bit color value
    {
        if (color & (1 << (23 - i)))  // Check if the current bit is set
        {
            out_bit_high();  // Set the output high if the bit is 1
        } else {
            out_bit_low();  // Set the output low if the bit is 0
        }
    }
}

void rgb_init(void)
{
    gpio_init();  // Initialize GPIO settings
    rgb_off();
}

void rgb_show(void)
{
    __disable_irq();  // Disable interrupts for safe data transmission

    rgb_send_data(color_buf);
    restart();  // Refresh LED display

    __enable_irq();  // Re-enable interrupts
}

void rgb_off(void)
{
    __disable_irq();  // Disable interrupts for safe data transmission

    rgb_send_data(rgb_off_buf);
    restart();  // Refresh LED display

    __enable_irq();  // Re-enable interrupts
}

void update_rgb_buffer(void)
{
    // Scale brightness properly by using fixed-point multiplication
    uint8_t rgb_r = (uint32_t)(rgb_value_reg[0] * rgb_brightness_reg) / 100;
    uint8_t rgb_g = (uint32_t)(rgb_value_reg[1] * rgb_brightness_reg) / 100;
    uint8_t rgb_b = (uint32_t)(rgb_value_reg[2] * rgb_brightness_reg) / 100;
    // Combine the RGB components into a single 24-bit value (0x00RRGGBB)
    color_buf = ((uint32_t)rgb_g << 16) | ((uint32_t)rgb_r << 8) | (uint32_t)rgb_b;
}

void update_rgb_status(void)
{
    static bool rgb_delay_flag        = false;  // Delay flag after enabling power
    static uint32_t rgb_delay_timeout = 0;      // Delay start tick

    bool rgb_status_changed     = (last_rgb_status != rgb_status_reg);
    bool rgb_brightness_changed = (last_rgb_brightness != rgb_brightness_reg);
    bool need_update            = rgb_status_changed || rgb_brightness_changed || rgb_value_update;

    last_rgb_status     = rgb_status_reg;
    last_rgb_brightness = rgb_brightness_reg;

    switch (rgb_status_reg) {
        case 0:  // Power off
            if (HAL_GPIO_ReadPin(RGB_VDD_EN_GPIO_Port, RGB_VDD_EN_Pin) != GPIO_PIN_RESET) {
                HAL_GPIO_WritePin(RGB_VDD_EN_GPIO_Port, RGB_VDD_EN_Pin, GPIO_PIN_RESET);
            }

            if (need_update) {
                update_rgb_buffer();
                rgb_value_update = 0;
            }
            break;

        case 1:  // Power on
            if (HAL_GPIO_ReadPin(RGB_VDD_EN_GPIO_Port, RGB_VDD_EN_Pin) != GPIO_PIN_SET) {
                HAL_GPIO_WritePin(RGB_VDD_EN_GPIO_Port, RGB_VDD_EN_Pin, GPIO_PIN_SET);
                rgb_delay_flag    = true;
                rgb_delay_timeout = HAL_GetTick();
                return;  // Delay before update
            }

            if (rgb_delay_flag) {
                if ((HAL_GetTick() - rgb_delay_timeout) < 50) {
                    return;  // Wait for power stabilization
                }

                update_rgb_buffer();
                rgb_show();
                rgb_value_update  = 0;
                rgb_delay_flag    = false;
                rgb_delay_timeout = 0;
                break;
            }

            if (need_update) {
                update_rgb_buffer();
                rgb_show();
                rgb_value_update = 0;
            }
            break;

        default:
            break;
    }

}

void rgb_test(void){
	while(1){
		rgb_show();
		rgb_show();
		rgb_show();
		HAL_Delay(1000);
		LL_IWDG_ReloadCounter(IWDG);
	}
}


