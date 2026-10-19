/*
 * nvs.c
 *
 *  Created on: Sep 8, 2025
 *      Author: Luke 1281772125@qq.com
 */
#include <stdio.h>
#include <string.h>
#include "main.h"

#define CONFIG_VERSION_BYTES	4

/* STM32F401RCT6
MEMORY
{
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 64K
NVS_FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 16K
FLASH(xrw) :ORIGIN = 0x8008000, LENGTH = 224K
}

SECTIONS
{
  // The startup code goes first into FLASH 
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) // Startup code 
    . = ALIGN(4);
  } >NVS_FLASH
*/

// NVS起始地址要避开ISR向量表所在的0x08000000-0x08003FFF区域
#define NVS_ADDR 0x8004000
// 扇区定义（根据地址归属确定）
#define NVS_SECTOR           FLASH_SECTOR_1  // NVS所在的扇区
#define NVS_SECTORS_COUNT    1               // 只需要擦除1个扇区
#define NVS_BYTES			 16384		     // 只使用16KB
#define NVS_CACHE_BYTES		 1024

static uint16_t _flash_cache[NVS_CACHE_BYTES / sizeof(uint16_t)];

int16_t init_nvs() {
	//printf("Initializing NVS ...\n");
	//Read Flash data
	uint16_t *ptr = (uint16_t*) (NVS_ADDR);
	uint16_t i;
	for(i=0; i<NVS_CACHE_BYTES/2; i++)
		_flash_cache[i] = *ptr++;
	return 0;
} 

size_t nvs_get_data(uint32_t version, void* buffer, size_t size) {
	//Check version number
	uint32_t *verp = (uint32_t*) _flash_cache;
	if(*verp != version) {
		//printf("nvs_get_data(): %08lxh is NOT %08lxh!\n", *verp, version);
		return 0;
	}
	if(size > NVS_BYTES - CONFIG_VERSION_BYTES)
		size = NVS_BYTES - CONFIG_VERSION_BYTES;
	char *src_ptr = (char*)_flash_cache;
    src_ptr += CONFIG_VERSION_BYTES;
	memcpy(buffer, src_ptr, size);
	return size;
}

int16_t nvs_set_data(uint32_t version, void *buffer, size_t size) {
    // 检查数据大小是否超过NVS可用空间
    if(size > NVS_BYTES - CONFIG_VERSION_BYTES) {
        //printf(".nvs_set_data(): size=%u > %d Bytes.\n",size, NVS_BYTES - CONFIG_VERSION_BYTES);
        return -1;
    }

    // 将数据复制到缓存（版本号 + 实际数据）
    uint32_t *verp = (uint32_t*) _flash_cache;
    *verp = version;
    char *dst_ptr = (char*)_flash_cache;
    dst_ptr += CONFIG_VERSION_BYTES;
    memcpy(dst_ptr, buffer, size);

    // FLASH操作变量定义
    FLASH_EraseInitTypeDef flash;
    HAL_StatusTypeDef sta;
    uint32_t sectorError = 0;

    // 解锁FLASH
    sta = HAL_FLASH_Unlock();
    if (sta != HAL_OK) {
        __enable_irq();
        //printf("!nvs_set_data(): HAL_FLASH_Unlock failed => %d\n", sta);
        return -1;
    }

    // 配置扇区擦除参数（STM32F407使用扇区擦除）
    flash.TypeErase = FLASH_TYPEERASE_SECTORS;       // 改为扇区擦除
    flash.Sector = NVS_SECTOR;                       // NVS所在的起始扇区（需定义）
    flash.NbSectors = NVS_SECTORS_COUNT;             // 需要擦除的扇区数量（需定义）
    flash.VoltageRange = FLASH_VOLTAGE_RANGE_3;      // F407工作电压范围(2.7V-3.6V)
    #if defined(FLASH_BANK_1)
    flash.Banks = FLASH_BANK_1;                      // STM32F407VET6只有Bank1
    #endif

    // 执行扇区擦除
    sta = HAL_FLASHEx_Erase(&flash, &sectorError);
    if (sta != HAL_OK) {
        //printf("!nvs_set_data(): HAL_FLASHEx_Erase failed, Try again...\n");
        if(HAL_FLASHEx_Erase(&flash, &sectorError) != HAL_OK) {
            HAL_FLASH_Lock();
            //printf("!nvs_set_data(): HAL_FLASHEx_Erase DOUBLE failed!\n");
            return -1;
        }
    }

    // 编程FLASH（按半字写入，F407最小编程单位为16位）
    uint16_t off;
    uint32_t addr;
    for (off = 0; off < NVS_CACHE_BYTES / 2; off++) {
        addr = NVS_ADDR + off * 2;
        // 检查地址是否在已配置的NVS扇区内
        if (addr >= (NVS_ADDR + NVS_BYTES)) {
            break;
        }
        sta = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, _flash_cache[off]);
        if (sta != HAL_OK) {
            //printf("!nvs_set_data(): HAL_FLASH_Program failed => %d at address: 0x%lX\n",sta, addr);
            break;
        }
    }

    // 锁定FLASH
    HAL_FLASH_Lock();
    __enable_irq();
    //printf(".nvs_set_date(): version = %lx0xh, OK=%d\n", version, (sta == HAL_OK) ? 0 : -1);
    return (sta == HAL_OK) ? 0 : -1;
}

