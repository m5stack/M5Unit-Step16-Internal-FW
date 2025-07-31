#include "myflash.h"

/**
 * @brief  根据地址获得内存的页
 * @param  addr 内存地址
 * @retval 返回页数
 */
static uint32_t GetPage(uint32_t addr)
{
    return (addr - STM32G0xx_FLASH_PAGE0_STARTADDR) / FLASH_PAGE_SIZE;
}

/**
 * @brief  修改8byte的某一字节
 * @param  data 要修改的数据
 * @param  byte_index 数据下标
 * @param  new_value 修改值
 * @retval 修改之后的数
 */
static void set_byte_in_uint64(uint64_t *data, uint8_t byte_index, uint8_t new_value)
{
    *data &= ~((uint64_t)(0xFF) << (byte_index * 8));
    // 然后设置新的字节值
    *data |= (uint64_t)new_value << (byte_index * 8);
}

/**
 * @brief  读取一字节
 * @param  address 内存地址
 * @retval 返回值
 */
uint8_t my_flash_read_byte(uint32_t address)
{
    return *((__IO uint8_t *)(address));
}

/**
 * @brief  读取半字
 * @param  address 内存地址
 * @retval 返回值
 */
uint16_t my_flash_read_half_word(uint32_t address)
{
    return *((__IO uint16_t *)(address));
}

/**
 * @brief  读取字
 * @param  address 内存地址
 * @retval 返回值
 */
uint32_t my_flash_read_word(uint32_t address)
{
    return *((__IO uint32_t *)(address));
}

/**
 * @brief  读取双字
 * @param  address 内存地址
 * @retval 返回值
 */
uint64_t my_flash_read_double_word(uint32_t address)
{
    return *((__IO uint64_t *)(address));
}

/**
 * @brief  页擦除
 * @param  address 内存地址
 * @retval 是否成功
 */
bool my_flash_earse_pages(uint32_t page_address)
{
    uint32_t page_error = 0;                     // 设置Page_error,如果出现错误这个变量会被设置为出错的FLASH地址
    FLASH_EraseInitTypeDef my_flash;             // 页擦除所在的位置
    my_flash.TypeErase = FLASH_TYPEERASE_PAGES;  // 标明Flash执行页面只做擦除操作
    my_flash.Page      = GetPage(page_address);
    my_flash.NbPages   = 1;  // 说明要擦除的页数
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&my_flash, &page_error);
    HAL_FLASH_Lock();
    if (status == HAL_OK) {
        return true;
    }
    return false;
}

/**
 * @brief  写半字
 * @param  address 内存地址
 * @param  data 要写入的数据
 * @retval 是否成功
 */
bool my_flash_write_half_word(uint32_t address, uint16_t data)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    HAL_FLASH_Lock();
    if (status == HAL_OK) {
        return true;
    }
    return false;
}

/**
 * @brief  写双字
 * @param  address 内存地址
 * @param  data 要写入的数据
 * @retval 是否成功
 */
static bool my_flash_write_double_word(uint32_t address, uint64_t data)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    HAL_FLASH_Lock();
    if (status == HAL_OK) {
        return true;
    }
    return false;
}

uint8_t get_step16_led_status(void)
{
    return *((__IO uint8_t *)(STEP16_LED_STATUS_ADDR));
}

uint8_t get_step16_led_light(void)
{
    return *((__IO uint8_t *)(STEP16_LED_LIGHT_ADDR));
}

uint8_t get_step16_switch(void)
{
    return *((__IO uint8_t *)(STEP16_SWITH_ADDR));
}

bool write_step16_flash(uint8_t step16_led_status, uint8_t step16_led_light, uint8_t step16_switch)
{
    __disable_irq();
    uint64_t temp = my_flash_read_double_word(STM32G0xx_FLASH_PAGE13_STARTADDR);
    set_byte_in_uint64(&temp, 0, step16_led_status);
    set_byte_in_uint64(&temp, 1, step16_led_light);
    set_byte_in_uint64(&temp, 2, step16_switch);
    my_flash_earse_pages(STM32G0xx_FLASH_PAGE13_STARTADDR);
    my_flash_write_double_word(STM32G0xx_FLASH_PAGE13_STARTADDR, temp);
    __enable_irq();
    if (step16_led_status == get_step16_led_status() && get_step16_led_light == get_step16_led_light() &&
        step16_switch == get_step16_switch()) {
        return true;
    }
    return false;
}

uint8_t get_rgb_status(void)
{
    return *((__IO uint8_t *)(RGB_STATUS_ADDR));
}

uint8_t get_rgb_brightness(void)
{
    return *((__IO uint8_t *)(RGB_BRIGHTNESS_ADDR));
}

bool write_rgb_flash(uint8_t rgb_status, uint8_t rgb_brightness)
{
    __disable_irq();
    uint64_t temp = my_flash_read_double_word(STM32G0xx_FLASH_PAGE14_STARTADDR);
    set_byte_in_uint64(&temp, 0, rgb_status);
    set_byte_in_uint64(&temp, 1, rgb_brightness);
    my_flash_earse_pages(STM32G0xx_FLASH_PAGE14_STARTADDR);
    my_flash_write_double_word(STM32G0xx_FLASH_PAGE14_STARTADDR, temp);
    __enable_irq();
    if (rgb_status == get_rgb_status() && rgb_brightness == get_rgb_brightness()) {
        return true;
    }
    return false;
}

uint8_t get_i2c_addr(void)
{
    return *((__IO uint8_t *)(I2C_ADDR));
}

bool set_i2c_addr(uint8_t data)
{
    __disable_irq();
    uint64_t temp = my_flash_read_double_word(STM32G0xx_FLASH_PAGE15_STARTADDR);
    set_byte_in_uint64(&temp, 0, data);
    my_flash_earse_pages(STM32G0xx_FLASH_PAGE15_STARTADDR);
    my_flash_write_double_word(STM32G0xx_FLASH_PAGE15_STARTADDR, temp);
    __enable_irq();
    uint8_t dat = get_i2c_addr();
    if (dat == data) {
        return true;
    }
    return false;
}
