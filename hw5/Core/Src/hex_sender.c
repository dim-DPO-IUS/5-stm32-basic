#include "hex_sender.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

static const char hex_digits[] = "0123456789ABCDEF";

// Вспомогательная функция для добавления HEX байта
static void add_hex_byte(char** ptr, uint8_t byte) {
    *(*ptr)++ = hex_digits[byte >> 4];
    *(*ptr)++ = hex_digits[byte & 0x0F];
}

// БЕЗОПАСНОЕ чтение байта из памяти
static uint8_t safe_read_byte(uint32_t address) {
    // Flash память: 0x08000000 - 0x0807FFFF (512KB)
    if (address >= 0x08000000 && address <= 0x0807FFFF) {
        return *(volatile uint8_t*)address;
    }

    // RAM память: 0x20000000 - 0x2001FFFF (128KB)
    if (address >= 0x20000000 && address <= 0x2001FFFF) {
        return *(volatile uint8_t*)address;
    }

    // Backup RAM: 0x40024000 - 0x40024FFF (4KB)
    if (address >= 0x40024000 && address <= 0x40024FFF) {
        return *(volatile uint8_t*)address;
    }

    // System memory (Bootloader, Option bytes)
    if (address >= 0x1FFF0000 && address <= 0x1FFF7A0F) {
        return *(volatile uint8_t*)address;
    }

    // Для всех остальных адресов возвращаем 0xFF
    return 0xFF;
}

// Основная функция обработки команды get_hex
void HEX_ProcessCommand(const char* cmd) {
    // Проверяем, что это команда get_hex
    if (strncmp(cmd, "get_hex ", 8) != 0) {
        const char* error = "ERROR: Unknown command. Use: get_hex <address> <length>\r\n";
        CDC_Transmit_FS((uint8_t*)error, strlen(error));
        return;
    }

    // Парсим адрес и длину
    uint32_t address = strtoul(cmd + 8, NULL, 16);
    const char* len_ptr = strchr(cmd + 8, ' ');
    if (!len_ptr) {
        CDC_Transmit_FS((uint8_t*)"ERROR: Invalid format. Use: get_hex <address> <length>\r\n", 57);
        return;
    }

    uint32_t length = strtoul(len_ptr + 1, NULL, 10);
    if (length == 0) {
        CDC_Transmit_FS((uint8_t*)"ERROR: Length must be > 0\r\n", 28);
        return;
    }

    // Ограничиваем максимальную длину
    if (length > 4096) {
        length = 4096;
        char msg[64];
        snprintf(msg, sizeof(msg), "INFO: Length limited to 4096 bytes\r\n");
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    }

    // Буфер для всего ответа
    uint8_t buffer[8192];  // Увеличенный буфер
    char* ptr = (char*)buffer;

    // 1. Extended Linear Address Record
    uint8_t ela_data[2] = {
        (uint8_t)((address >> 24) & 0xFF),
        (uint8_t)((address >> 16) & 0xFF)
    };

    *ptr++ = ':';
    add_hex_byte(&ptr, 0x02);           // Length = 2
    add_hex_byte(&ptr, 0x00);           // Address high = 0x00
    add_hex_byte(&ptr, 0x00);           // Address low = 0x00
    add_hex_byte(&ptr, 0x04);           // Record type = Extended Linear Address
    add_hex_byte(&ptr, ela_data[0]);    // Data byte 1
    add_hex_byte(&ptr, ela_data[1]);    // Data byte 2

    // Контрольная сумма для ELA
    uint8_t checksum = 0x100 - (0x02 + 0x00 + 0x00 + 0x04 + ela_data[0] + ela_data[1]);
    add_hex_byte(&ptr, checksum);
    *ptr++ = '\r'; *ptr++ = '\n';

    // 2. Data Records
    uint32_t remaining = length;
    uint32_t current_addr = address;

    while (remaining > 0) {
        uint8_t chunk = (remaining > 16) ? 16 : remaining;

        // Начало строки
        *ptr++ = ':';
        add_hex_byte(&ptr, chunk);  // Length

        // Адрес (младшие 2 байта)
        add_hex_byte(&ptr, (current_addr >> 8) & 0xFF);
        add_hex_byte(&ptr, current_addr & 0xFF);

        add_hex_byte(&ptr, 0x00);  // Record type = Data

        uint8_t sum = chunk + ((current_addr >> 8) & 0xFF) + (current_addr & 0xFF);

        // БЕЗОПАСНОЕ чтение данных
        for (int i = 0; i < chunk; i++) {
            uint8_t data_byte = safe_read_byte(current_addr + i);
            add_hex_byte(&ptr, data_byte);
            sum += data_byte;
        }

        // Контрольная сумма
        checksum = 0x100 - sum;
        add_hex_byte(&ptr, checksum);

        *ptr++ = '\r'; *ptr++ = '\n';

        current_addr += chunk;
        remaining -= chunk;
    }

    // 3. End Of File Record
    *ptr++ = ':';
    add_hex_byte(&ptr, 0x00);  // Length = 0
    add_hex_byte(&ptr, 0x00);  // Address high = 0x00
    add_hex_byte(&ptr, 0x00);  // Address low = 0x00
    add_hex_byte(&ptr, 0x01);  // Record type = End Of File

    // Контрольная сумма для EOF
    checksum = 0x100 - (0x00 + 0x00 + 0x00 + 0x01);
    add_hex_byte(&ptr, checksum);

    *ptr++ = '\r'; *ptr++ = '\n';

    // Отправка всего буфера
    CDC_Transmit_FS(buffer, ptr - (char*)buffer);
}

// Функция для проверки команды
uint8_t HEX_IsGetHexCommand(const char* cmd) {
    return (strncmp(cmd, "get_hex ", 8) == 0);
}

// Инициализация
void HEX_Init(void) {
}
