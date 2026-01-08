#ifndef DISPLAY_TESTS_H
#define DISPLAY_TESTS_H

#include "main.h"
#include <stdio.h>
#include <string.h>
//#include "st7789.h"

//----------------------------------------------------------------------------
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define GRAY        0X8430
#define BRED        0XF81F
#define GRED        0XFFE0
#define GBLUE       0X07FF
#define BROWN       0XBC40
#define BRRED       0XFC07
#define DARKBLUE    0X01CF
#define LIGHTBLUE   0X7D7C
#define GRAYBLUE    0X5458

#define LIGHTGREEN  0X841F
#define LGRAY       0XC618
#define LGRAYBLUE   0XA651
#define LBBLUE      0X2B12
//----------------------------------------------------------------------------
/* Control Registers and constant codes */
#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36
//----------------------------------------------------------------------------
void Full_ST7789_UART_Init(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void Full_ST7789_Init(SPI_HandleTypeDef *hspi);
void Minimal_ST7789_Init(SPI_HandleTypeDef *hspi);
void Fixed_ST7789_Init(SPI_HandleTypeDef *hspi);
void Optimal_ST7789_Init(SPI_HandleTypeDef *hspi);
void Stable_ST7789_Init(SPI_HandleTypeDef *hspi);
void TestA_ST7789_Init(SPI_HandleTypeDef *hspi);
void Force_Clear_Display(SPI_HandleTypeDef *hspi);
//----------------------------------------------------------------------------
void Simple_Fill_Color(SPI_HandleTypeDef *hspi, uint16_t color);
void Monitor_SPI_Data(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void Diagnostic_Fill_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi,
		volatile uint8_t *spi_error_flag);
void Test_Different_Configs(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void Ultimate_Hardware_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void Physical_MOSI_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void Test_Alternative_MOSI_Pin(UART_HandleTypeDef *huart,
		SPI_HandleTypeDef *hspi);
void Quick_SPI_Diagnostic(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void Optimized_Fill_Color(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi,
		uint16_t color);
void Debug_DC_Signal(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void Test_SPI_Modes(UART_HandleTypeDef *huart);
void Test_With_CS_Control(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void CS_Simulation_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void ST7789_Simple_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);
void ST7789_Arduino_Style_Test(UART_HandleTypeDef *huart,
		SPI_HandleTypeDef *hspi);
void Hardware_Verification_Test(UART_HandleTypeDef *huart,
		SPI_HandleTypeDef *hspi);
void Test_Full_Screen(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi);

#endif
