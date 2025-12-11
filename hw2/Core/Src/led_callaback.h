/*
 * led_init.h
 *
 *  Created on: Dec 7, 2025
 *      Author: dim0k
 */

#ifndef SRC_LED_CALLABACK_H_
#define SRC_LED_CALLABACK_H_

#include "stm32f4xx_hal.h"

void init_led(GPIO_TypeDef* port, uint16_t pin);

#endif /* SRC_LED_CALLABACK_H_ */
