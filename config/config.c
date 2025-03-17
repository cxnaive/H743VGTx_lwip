#include "config.h"
#include "stm32h7xx_hal.h"
#include "main.h"
#include "string.h"
#define FLASH_BUFFER_SIZE       9600
#define FLASH_USER_START_ADDR   0x8100000      // 起始地址（需32字节对齐）
#define FLASH_SECTOR            FLASH_SECTOR_0  // 扇区号

struct config_t config;
uint8_t flash_write_buffer[FLASH_BUFFER_SIZE];
extern CRC_HandleTypeDef hcrc;

void init_config(){
    memset(flash_write_buffer, 0, FLASH_BUFFER_SIZE);
    memset(&config, 0, sizeof(config));
}

void Flash_EraseSector() {
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = FLASH_BANK_1;          // 选择BANK1
    erase_init.Sector = FLASH_SECTOR;         // 扇区号
    erase_init.NbSectors = 1;                 // 擦除1个扇区
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 电压范围（参考手册）

    HAL_FLASHEx_Erase(&erase_init, &sector_error); // 执行擦除
}

// uint32_t align
// 写入Flash
void Flash_WriteData(uint32_t addr, uint8_t *data, uint32_t size) {
    // 检查地址是否32字节对齐
    if (addr % 32 != 0) {
        Error_Handler(); // 处理地址错误
    }

    // 解锁Flash
    if (HAL_FLASH_Unlock() != HAL_OK) {
        Error_Handler(); // 解锁失败
    }

    // 擦除目标扇区
    Flash_EraseSector();

    // 写入数据（每次32字节）
    for (uint32_t i = 0; i < size; i += 8) { // 每次写入8个uint32（32字节）
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + i, (uint32_t)(data + i)) != HAL_OK) {
            HAL_FLASH_Lock(); // 写入失败后上锁
            Error_Handler();
        }
    }

    // 上锁Flash
    HAL_FLASH_Lock();
}

void save_flash_config(struct flash_config_t * flash_config){
    flash_config->version = FLASH_VERSION;
    uint32_t config_size = sizeof(struct flash_config_t);
    flash_config->crc_16 = HAL_CRC_Calculate(&hcrc, (uint32_t*)flash_config, config_size - 2);
    memcpy(flash_write_buffer, flash_config, config_size);
    Flash_WriteData(FLASH_USER_START_ADDR, flash_write_buffer, config_size);
}

void read_flash_config(struct flash_config_t * flash_config){
    uint32_t config_size = sizeof(struct flash_config_t);
    memcpy(flash_config, (void*)FLASH_USER_START_ADDR, config_size);
    uint16_t crc_validate = HAL_CRC_Calculate(&hcrc, (uint32_t*)flash_config, config_size - 2);
    if (flash_config->version != FLASH_VERSION || crc_validate != flash_config->crc_16){
        memset(flash_config, 0, config_size);
    }
}