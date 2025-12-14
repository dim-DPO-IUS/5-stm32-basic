/*
 * memory_access.h
 *
 *  Created on: Dec 14, 2025
 *      Author: dim0k
 */

#ifndef MEMORY_ACCESS_H
#define MEMORY_ACCESS_H

#include <stdint.h>
#include <stdbool.h>

// Границы памяти для STM32F411RE
#define FLASH_START   0x08000000U
#define FLASH_END     0x0807FFFFU  // 512KB

#define SRAM1_START   0x20000000U
#define SRAM1_END     0x2001FFFFU  // 128KB

// Проверка, можно ли читать этот адрес
bool Memory_IsAddressReadable(uint32_t address);

// Безопасное чтение байта
uint8_t Memory_ReadByte(uint32_t address);

// Безопасное чтение блока
bool Memory_ReadBlock(uint32_t address, uint8_t* buffer, uint32_t size);

#endif
