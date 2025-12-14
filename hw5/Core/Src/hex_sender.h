/*
 * hex_sender.h
 *
 *  Created on: Dec 14, 2025
 *      Author: dim0k
 */

#ifndef HEX_SENDER_H
#define HEX_SENDER_H

#include <stdint.h>

// Инициализация
void HEX_Init(void);

// Обработка команды
void HEX_ProcessCommand(const char* cmd);

// Проверка, является ли команда get_hex
uint8_t HEX_IsGetHexCommand(const char* cmd);

#endif
