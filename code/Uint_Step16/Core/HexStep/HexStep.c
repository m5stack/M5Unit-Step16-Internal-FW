#include "HexStep.h"
#include <stdbool.h>

extern __IO uint8_t step16_value_reg;
extern __IO uint8_t step16_led_config_reg;
extern __IO uint8_t step16_switch_reg;
extern __IO uint8_t step16_led_light_reg;
extern __IO uint32_t rgb_turn_off_timeout;
extern __IO uint32_t step_led_timeout_delay;

static filter_t filter           = {0};
static uint32_t last_sample_time = 0;

const uint8_t s_hex_to_segment[16] = {
    0b1111110,  // 0: A, B, C, D, E, F
    0b0110000,  // 1: B, C
    0b1101101,  // 2: A, B, G, E, D
    0b1111001,  // 3: A, B, G, C, D
    0b0110011,  // 4: F, G, B, C
    0b1011011,  // 5: A, F, G, C, D
    0b1011111,  // 6: A, F, G, E, C, D
    0b1110000,  // 7: A, B, C
    0b1111111,  // 8: A, B, C, D, E, F, G
    0b1111011,  // 9: A, B, C, D, F, G
    0b1110111,  // A: A, B, C, E, F, G
    0b0011111,  // B: E, F, G, C, D
    0b1001110,  // C: A, F, E, D
    0b0111101,  // D: A, B, C, D, E
    0b1001111,  // E: A, F, G, E, D
    0b1000111   // F: A, F, G, E
};

const uint8_t lookup_table[16] = {0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

static void setSegmentBrightness(char segment, uint8_t step16_led_light_reg)
{
    uint32_t pwmValue = step16_led_light_reg;

    switch (segment) {
        case 'A':
            LL_TIM_OC_SetCompareCH1(TIM14, pwmValue);
            break;
        case 'B':
            LL_TIM_OC_SetCompareCH1(TIM2, pwmValue);
            break;
        case 'C':
            LL_TIM_OC_SetCompareCH1(TIM3, pwmValue);
            break;
        case 'D':
            LL_TIM_OC_SetCompareCH2(TIM3, pwmValue);
            break;
        case 'E':
            LL_TIM_OC_SetCompareCH3(TIM3, pwmValue);
            break;
        case 'F':
            LL_TIM_OC_SetCompareCH4(TIM3, pwmValue);
            break;
        case 'G':
            LL_TIM_OC_SetCompareCH1(TIM1, pwmValue);
            break;
        default:
            break;
    }
}

static void led_display(void)
{
    uint8_t segments = s_hex_to_segment[step16_value_reg];
    // A 段，最高位
    setSegmentBrightness('A', (segments & 0x40) ? step16_led_light_reg : 0);
    // B 段
    setSegmentBrightness('B', (segments & 0x20) ? step16_led_light_reg : 0);
    // C 段
    setSegmentBrightness('C', (segments & 0x10) ? step16_led_light_reg : 0);
    // D 段
    setSegmentBrightness('D', (segments & 0x08) ? step16_led_light_reg : 0);
    // E 段
    setSegmentBrightness('E', (segments & 0x04) ? step16_led_light_reg : 0);
    // F 段
    setSegmentBrightness('F', (segments & 0x02) ? step16_led_light_reg : 0);
    // G 段，最低位
    setSegmentBrightness('G', (segments & 0x01) ? step16_led_light_reg : 0);
}

static void led_off(void)
{
    // A 段，最高位
    setSegmentBrightness('A', 0);
    // B 段
    setSegmentBrightness('B', 0);
    // C 段
    setSegmentBrightness('C', 0);
    // D 段
    setSegmentBrightness('D', 0);
    // E 段
    setSegmentBrightness('E', 0);
    // F 段
    setSegmentBrightness('F', 0);
    // G 段，最低位
    setSegmentBrightness('G', 0);
}

static uint8_t get_hexstep_position(void)
{
    uint8_t position = ((~LL_GPIO_IsInputPinSet(BCD_8_GPIO_Port, BCD_8_Pin) & 0x01) << 3) |  // 8 (PA3)
                       ((~LL_GPIO_IsInputPinSet(BCD_4_GPIO_Port, BCD_4_Pin) & 0x01) << 2) |  // 4 (PA2)
                       ((~LL_GPIO_IsInputPinSet(BCD_2_GPIO_Port, BCD_2_Pin) & 0x01) << 1) |  // 2 (PA1)
                       ((~LL_GPIO_IsInputPinSet(BCD_1_GPIO_Port, BCD_1_Pin)) & 0x01);        // 1 (PA0)
    return position;
}

static void window_stable_update(void)
{
    uint8_t first_value = filter.buffer[0];
    for (uint8_t i = 1; i < FILTER_WINDOW_DEFAULT_SIZE; i++) {
        if (filter.buffer[i] != first_value) {
            return;
        }
    }
    filter.stable_value = first_value;
}

void filter_init(void)
{
    filter.index   = 0;
    uint8_t sample = get_hexstep_position();
    for (uint8_t i = 0; i < FILTER_WINDOW_DEFAULT_SIZE; i++) {
        filter.buffer[i] = sample;
    }
    filter.stable_value = sample;
    last_sample_time    = HAL_GetTick();
}

void sample_data(void)
{
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_sample_time >= SAMPLE_INTERVAL_MS) {
        uint8_t sample              = get_hexstep_position();
        filter.buffer[filter.index] = sample;
        filter.index                = (filter.index + 1) % FILTER_WINDOW_DEFAULT_SIZE;
        last_sample_time            = current_time;
    }
}

void update_step16_value(void)
{
    window_stable_update();
    if (step16_switch_reg == 0) {
        step16_value_reg = lookup_table[filter.stable_value];
    }else{
    	step16_value_reg = filter.stable_value;
    }
}

void displayHex(void)
{
    static uint8_t last_hex_value  = 0xFF;
    static uint8_t last_led_status = 0xFF;
    static uint8_t last_brightness = 0xFF;
    static uint8_t timeout_flag    = 0;

    bool hex_value_update  = (last_hex_value != step16_value_reg);
    bool led_status_update = (last_led_status != step16_led_config_reg);
    bool brightness_update = (last_brightness != step16_led_light_reg);

    last_hex_value  = step16_value_reg;
    last_led_status = step16_led_config_reg;
    last_brightness = step16_led_light_reg;

    switch (step16_led_config_reg) {
        case 0x00:
            led_off();
            timeout_flag = 0;
            break;

        case 0xFE:
            led_display();
            timeout_flag = 0;
            break;

        default:
            if (hex_value_update || led_status_update || brightness_update) {
                led_display();
                rgb_turn_off_timeout = HAL_GetTick();
                timeout_flag         = 1;
            } else if (timeout_flag && (HAL_GetTick() - rgb_turn_off_timeout > step_led_timeout_delay)) {
                led_off();
                timeout_flag = 0;
            }
            break;
    }
}
