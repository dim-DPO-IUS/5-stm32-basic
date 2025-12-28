#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

// Добавляем заголовки FATFS
#include "fatfs.h"
#include "ff.h"

#define CSV_BUFFER_SIZE 12000
#define CSV_FILENAME "data.csv"

// Объявляем глобальные переменные
extern char csv_buffer[CSV_BUFFER_SIZE];
extern bool csv_file_initialized;
extern uint32_t data_counter;
extern uint32_t buffer_index;
extern FIL csv_file;

//
void csv_logger_init(void);
void csv_logger_write(uint16_t pwm_mv, uint16_t adc_mv);
void csv_logger_flush(void);
uint32_t csv_logger_get_count(void);

// ФУНКЦИЯ для формата с timestamp и delta
void csv_logger_write_timestamp(uint32_t timestamp, uint16_t adc_mv,
		uint32_t delta);

#endif
