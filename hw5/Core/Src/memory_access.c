/*
 * memory_access.c
 *
 *  Created on: Dec 14, 2025
 *      Author: dim0k
 */


#include "memory_access.h"

// Проверка адреса
bool Memory_IsAddressReadable(uint32_t address) {
    // Flash память
    if (address >= FLASH_START && address <= FLASH_END) {
        return true;
    }

    // SRAM память
    if (address >= SRAM1_START && address <= SRAM1_END) {
        return true;
    }

    // Можно добавить другие области (CCM RAM, Backup RAM и т.д.)
    return false;
}

// Чтение одного байта
uint8_t Memory_ReadByte(uint32_t address) {
    if (!Memory_IsAddressReadable(address)) {
        return 0xFF;  // Значение по умолчанию для ошибки
    }

    return *(volatile uint8_t*)address;
}

// Чтение блока памяти
bool Memory_ReadBlock(uint32_t address, uint8_t* buffer, uint32_t size) {
    if (!Memory_IsAddressReadable(address) ||
        !Memory_IsAddressReadable(address + size - 1)) {
        return false;
    }

    for (uint32_t i = 0; i < size; i++) {
        buffer[i] = *(volatile uint8_t*)(address + i);
    }

    return true;
}
