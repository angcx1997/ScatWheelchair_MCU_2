/*
 * peripheral_init.h
 *  - Configuration of all peripheral needed during initialization
 *
 *  Created on: Feb 17, 2022
 *      Author: ray
 */

#ifndef PERIPHERAL_INIT_H_
#define PERIPHERAL_INIT_H_

#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void Peripheral_Init(void);

#endif /* PERIPHERAL_INIT_H_ */
