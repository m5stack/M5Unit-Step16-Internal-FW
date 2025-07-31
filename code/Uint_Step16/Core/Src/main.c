/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "HexStep.h"
#include "i2c_ex.h"
#include "myflash.h"
#include "rgb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FIRMWARE_VERSION       (0x01)
#define I2C1_ADDR_BASE         (0x48)
#define APPLICATION_ADDRESS    (0x08001800)  // APP的起始地址
#define RGB_BRIGHTNESS_MAX     (0x64)
#define RGB_BRIGHTNESS_DEFAULT (0x32)
#define STEP16_LED_STATUS_BASE (0xFE)
#define STEP16_LED_LIGHT_BASE  (0x32)
#define STEP16_SWITCH_BASE     (0x01)
#define RGB_STATUS_BASE        (0x01)
#define RGB_BRINHTNESS_MAX     (0x64)
#define RGB_BRIGHTNESS_BASE    (0x32)
#define STEP_SENSITIVITY_BASE  (0x64)

#define STEP16_VALUE_REG_ADDR      (0x00)
#define STEP16_LED_CONFIG_REG_ADDR (0x10)
#define STEP16_LED_LIGHT_REG_ADDR  (0x20)
#define STEP16_SWITCH_REG_ADDR     (0x30)
#define RGB_CONFIGURATON_REG_ADDR  (0x40)
#define RGB_BRIGHTNESS_REG_ADDR    (0x41)
#define RGB_VALUE_REG_ADDR         (0x50)
#define FLASH_WRITE_BACK           (0xF0)
#define IAP_UPDATE_REG_ADDR        (0xFD)
#define FIRMWARE_VERSION_REG_ADDR  (0xFE)
#define I2C_ADDRESS_REG_ADDR       (0xFF)

__IO uint8_t step16_value_reg      = 0;
__IO uint8_t step16_led_config_reg = 0;
__IO uint8_t step16_led_light_reg  = 0;
__IO uint8_t step16_switch_reg     = 0;
__IO uint8_t rgb_status_reg        = 0;
__IO uint8_t rgb_brightness_reg    = 100;
__IO uint8_t rgb_value_reg[3]      = {0};
__IO uint8_t firmware_version_reg  = FIRMWARE_VERSION;
__IO uint8_t i2c_addr_reg          = I2C1_ADDR_BASE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__IO uint32_t i2c_stop_timeout_delay = 0;
__IO uint32_t rgb_turn_off_timeout   = 0;
__IO uint8_t rgb_value_update        = 0;
__IO uint32_t step_led_timeout_delay = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void iap_set()
{
    uint8_t i;
    uint32_t *pVecTab = (uint32_t *)(0x20000000);
    for (i = 0; i < 48; i++) {
        *(pVecTab++) = *(__IO uint32_t *)(APPLICATION_ADDRESS + (i << 2));
    }
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
}

void step16_init(void)
{
    bool need_write_step16 = false;
    bool need_write_rgb    = false;

    // 读取 step16_led_config_reg
    uint8_t led_status = get_step16_led_status();
    if (led_status == 0xFF) {
        need_write_step16     = true;
        step16_led_config_reg = STEP16_LED_STATUS_BASE;
    } else {
        step16_led_config_reg = led_status;
    }

    // 读取 step16_led_light_reg
    uint8_t led_light = get_step16_led_light();
    if (led_light == 0xFF) {
        need_write_step16    = true;
        step16_led_light_reg = STEP16_LED_LIGHT_BASE;
    } else {
        step16_led_light_reg = led_light;
    }

    // 读取 step16_switch_reg
    uint8_t step_switch = get_step16_switch();
    if (step_switch == 0xFF) {
        need_write_step16 = true;
        step16_switch_reg = STEP16_SWITCH_BASE;
    } else {
        step16_switch_reg = step_switch;
    }

    // 读取 rgb_status_reg
    uint8_t rgb_status = get_rgb_status();
    if (rgb_status == 0xFF) {
        need_write_rgb = true;
        rgb_status_reg = RGB_STATUS_BASE;
    } else {
        rgb_status_reg = rgb_status;
    }

    // 读取 rgb_brightness_reg
    uint8_t rgb_brightness = get_rgb_brightness();
    if (rgb_brightness == 0xFF) {
        need_write_rgb     = true;
        rgb_brightness_reg = RGB_BRIGHTNESS_BASE;
    } else {
        rgb_brightness_reg = rgb_brightness;
    }

    // 只有在必要时才写入 Flash，减少擦写次数
    if (need_write_step16) {
        write_step16_flash(step16_led_config_reg, step16_led_light_reg, step16_switch_reg);
    }
    if (need_write_rgb) {
        write_rgb_flash(rgb_status_reg, rgb_brightness_reg);
    }

    // 读取 i2c_addr_reg
    uint8_t i2c_addr = get_i2c_addr();
    if (i2c_addr == 0xFF) {
        i2c_addr_reg = I2C1_ADDR_BASE;
        set_i2c_addr(I2C1_ADDR_BASE);
    } else {
        i2c_addr_reg = i2c_addr;
    }

    step_led_timeout_delay = step16_led_config_reg * 1000;
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len)
{
    uint8_t rx_buf[16];
    uint8_t tx_buf[32];
    uint8_t rx_mark[16] = {0};
    if (len > 1) {
        if (rx_data[0] == STEP16_LED_CONFIG_REG_ADDR && len == 2) {
            if (step16_led_config_reg != rx_data[1]) {
                step16_led_config_reg  = rx_data[1];
                step_led_timeout_delay = step16_led_config_reg * 1000;
            }
        } else if (rx_data[0] == STEP16_LED_LIGHT_REG_ADDR && len == 2) {
            if (rx_data[1] >= 0 && rx_data[1] <= 100) {
                if (step16_led_light_reg != rx_data[1]) {
                    step16_led_light_reg = rx_data[1];
                }
            }
        } else if (rx_data[0] == STEP16_SWITCH_REG_ADDR && len == 2) {
            if (rx_data[1] == 0 || rx_data[1] == 1) {
                if (step16_switch_reg != rx_data[1]) {
                    step16_switch_reg = rx_data[1];
                }
            }
        } else if (rx_data[0] == I2C_ADDRESS_REG_ADDR && len == 2) {
            if (rx_data[1] >= 0x08 && rx_data[1] <= 0x77) {
                if (i2c_addr_reg != rx_data[1]) {
                    i2c_addr_reg = rx_data[1];
                    set_i2c_addr(rx_data[1]);
                    user_i2c_init();
                }
            }
        } else if (rx_data[0] >= RGB_CONFIGURATON_REG_ADDR && rx_data[0] <= RGB_BRIGHTNESS_REG_ADDR && len <= 3) {
            uint8_t offset = rx_data[0] - RGB_CONFIGURATON_REG_ADDR;
            uint8_t limit  = len - 1;
            for (uint8_t i = 0; i < limit; i++) {
                uint8_t index  = offset + i;
                rx_buf[index]  = rx_data[i + 1];
                rx_mark[index] = 1;
            }
            if (rx_mark[0] && rgb_status_reg != rx_buf[0]) {
                rgb_status_reg = rx_buf[0];
            }
            if (rx_mark[1] && rgb_brightness_reg != rx_buf[1] && rx_buf[1] <= 100) {
                rgb_brightness_reg = rx_buf[1];
            }

        } else if (rx_data[0] >= RGB_VALUE_REG_ADDR && rx_data[0] <= 0x52 && len <= 4) {
            uint8_t offset = rx_data[0] - RGB_VALUE_REG_ADDR;
            uint8_t limit  = len - 1;
            for (uint8_t i = 0; i < limit; i++) {
                uint8_t index  = offset + i;
                rx_buf[index]  = rx_data[i + 1];
                rx_mark[index] = 1;
            }
            for (uint8_t i = 0; i < 3; i++) {
                if (rx_mark[i] && rgb_value_reg[i] != rx_buf[i]) {
                    rgb_value_reg[i] = rx_buf[i];
                    rgb_value_update = 1;
                }
            }
        } else if (rx_data[0] == IAP_UPDATE_REG_ADDR && len == 2) {
            if (rx_data[1] > 0) {
                uint8_t pwmValue = 100;
                LL_TIM_OC_SetCompareCH1(TIM14, pwmValue);
                LL_TIM_OC_SetCompareCH1(TIM2, pwmValue);
                LL_TIM_OC_SetCompareCH1(TIM3, pwmValue);
                LL_TIM_OC_SetCompareCH2(TIM3, pwmValue);
                LL_TIM_OC_SetCompareCH3(TIM3, pwmValue);
                LL_TIM_OC_SetCompareCH4(TIM3, pwmValue);
                LL_TIM_OC_SetCompareCH1(TIM1, pwmValue);
                NVIC_SystemReset();  // 复位单片机触发升级
            }
        } else if (rx_data[0] == FLASH_WRITE_BACK && len == 2) {
            if (rx_data[1] == 1) {
                if (step16_led_config_reg != get_step16_led_status() ||
                    step16_led_light_reg != get_step16_led_light() || step16_switch_reg != get_step16_switch()) {
                    write_step16_flash(step16_led_config_reg, step16_led_light_reg, step16_switch_reg);
                }
            } else if (rx_data[1] == 2) {
                if (rgb_status_reg != get_rgb_status() || rgb_brightness_reg != get_rgb_brightness()) {
                    write_rgb_flash(rgb_status_reg, rgb_brightness_reg);
                }
            }
        }
    }
    if (len == 1) {
        if (rx_data[0] == STEP16_VALUE_REG_ADDR) {
            tx_buf[0] = step16_value_reg;
            i2c2_set_send_data(tx_buf, 1);
        } else if (rx_data[0] == STEP16_LED_CONFIG_REG_ADDR) {
            tx_buf[0] = step16_led_config_reg;
            i2c2_set_send_data(tx_buf, 1);
        } else if (rx_data[0] == STEP16_LED_LIGHT_REG_ADDR) {
            tx_buf[0] = step16_led_light_reg;
            i2c2_set_send_data(tx_buf, 1);
        } else if (rx_data[0] == STEP16_SWITCH_REG_ADDR) {
            tx_buf[0] = step16_switch_reg;
            i2c2_set_send_data(tx_buf, 1);
        } else if (rx_data[0] == RGB_CONFIGURATON_REG_ADDR) {
            tx_buf[0] = rgb_status_reg;
            tx_buf[1] = rgb_brightness_reg;
            i2c2_set_send_data(tx_buf, 2);
        } else if (rx_data[0] == RGB_BRIGHTNESS_REG_ADDR) {
            tx_buf[0] = rgb_brightness_reg;
            i2c2_set_send_data(tx_buf, 1);
        } else if (rx_data[0] >= RGB_VALUE_REG_ADDR && rx_data[0] <= 0x52) {
            uint8_t *src_ptr = (uint8_t *)(rgb_value_reg + rx_data[0] - 0x50);
            uint8_t data_len = sizeof(rgb_value_reg) + 0x50 - rx_data[0];
            memcpy(tx_buf, src_ptr, data_len);
            i2c2_set_send_data(tx_buf, data_len);
        } else if (rx_data[0] == FIRMWARE_VERSION_REG_ADDR) {
            tx_buf[0] = firmware_version_reg;
            i2c2_set_send_data(tx_buf, 1);
        } else if (rx_data[0] == I2C_ADDRESS_REG_ADDR) {
            tx_buf[0] = i2c_addr_reg;
            i2c2_set_send_data(tx_buf, 1);
        }
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    iap_set();
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM14_Init();
    MX_IWDG_Init();
    /* USER CODE BEGIN 2 */
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);  // 启动通道输出
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);  // 启动通道输出
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);  // 启动通道输出
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);  // 启动通道输出
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);  // 启动通道输出
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);  // 启动通道输出
    LL_TIM_EnableCounter(TIM14);
    LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);  // 启动通道输出
    filter_init();
    update_step16_value();
    step16_init();
    rgb_init();
    user_i2c_init();
    i2c2_it_enable();
//    rgb_test();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        i2c_timeout_counter = 0;
        if (i2c_stop_timeout_flag) {
            if (i2c_stop_timeout_delay < HAL_GetTick()) {
                i2c_stop_timeout_counter++;
                i2c_stop_timeout_delay = HAL_GetTick() + 10;
            }
        }
        if (i2c_stop_timeout_counter > 50) {
            LL_I2C_DeInit(I2C2);
            LL_I2C_DisableAutoEndMode(I2C2);
            LL_I2C_Disable(I2C2);
            LL_I2C_DisableIT_ADDR(I2C2);
            user_i2c_init();
            i2c2_it_enable();
            HAL_Delay(500);
        }
        sample_data();
        update_step16_value();
        displayHex();
        update_rgb_status();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        LL_IWDG_ReloadCounter(IWDG);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN            = 8;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV4;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV4;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
